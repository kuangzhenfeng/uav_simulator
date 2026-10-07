// Copyright Epic Games, Inc. All Rights Reserved.

#include "ScenarioLoader.h"
#include "../uav_simulator.h"
#include "../Planning/ObstacleManager.h"
#include "../Planning/DynamicObstacleActor.h"
#include "../Core/UAVTypes.h"
#include "../Core/UAVPawn.h"
#include "../Mission/MissionComponent.h"
#include "../Environment/WindField.h"
#include "Engine/World.h"
#include "Engine/Engine.h"

DEFINE_LOG_CATEGORY_STATIC(LogScenarioLoader, Log, All);

UScenarioLoader::UScenarioLoader()
{
}

int32 UScenarioLoader::AssembleObstacles(const UScenario* Scenario, UObstacleManager* ObstacleManager, UObject* WorldContext)
{
	if (!Scenario || !ObstacleManager)
	{
		UE_LOG(LogScenarioLoader, Warning, TEXT("[Scenario] AssembleObstacles: null Scenario or ObstacleManager"));
		return 0;
	}

	// 加载软引用的障碍布局
	const UObstacleLayout* Layout = Scenario->ObstacleLayout.LoadSynchronous();
	if (!Layout)
	{
		UE_LOG(LogScenarioLoader, Log, TEXT("[Scenario] No ObstacleLayout on scenario '%s'"), *Scenario->Name);
		return 0;
	}

	// 装配目标 World（可选，用于 Spawn 可视化）
	// 兼容多种上下文：直接传 UWorld*、或传含 GetWorld() 的对象（Actor/Component）。
	UWorld* World = nullptr;
	if (WorldContext)
	{
		World = Cast<UWorld>(WorldContext);
		if (!World && GEngine)
		{
			World = GEngine->GetWorldFromContextObject(WorldContext, EGetWorldErrorMode::ReturnNull);
		}
	}

	int32 Registered = 0;
	for (const FScenarioObstacleEntry& Entry : Layout->Obstacles)
	{
		FObstacleInfo Obstacle;
		Obstacle.Type = Entry.Type;
		Obstacle.Center = Entry.Center;
		Obstacle.Extents = Entry.Extents;
		Obstacle.Rotation = Entry.Rotation;
		Obstacle.SafetyMargin = Entry.SafetyMargin;
		// Scenario 层用 MovementType 表达运动模型，运行时层用 bIsDynamic + Velocity。
		// 非 Static 即动态；LinearVelocity 模式取声明速度，PatrolLoop/PingPong 的速度由
		// 驱动 Actor 每帧驱动，ObstacleManager 会从 LinkedActor 反算（此处初值置零）。
		Obstacle.bIsDynamic = (Entry.MovementType != EObstacleMovementType::Static);
		Obstacle.Velocity = (Entry.MovementType == EObstacleMovementType::LinearVelocity)
			? Entry.Velocity : FVector::ZeroVector;

		// 所有场景障碍共用声明驱动的 Actor，避免静态裸 Actor 和动态尺寸反推。
		if (World)
		{
			FActorSpawnParameters SpawnParams;
			SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
			ADynamicObstacleActor* Actor = World->SpawnActor<ADynamicObstacleActor>(Entry.Center, Entry.Rotation, SpawnParams);
			if (Actor)
			{
				Actor->Configure(Entry);
				Obstacle = Actor->GetObstacleSnapshot();
				ObstacleManager->AddTickPrerequisiteActor(Actor);
			}
		}

		ObstacleManager->RegisterObstacle(Obstacle);
		++Registered;
	}

	UE_LOG(LogScenarioLoader, Log, TEXT("[Scenario] Obstacles assembled: %d (scenario='%s')"),
		Registered, *Scenario->Name);
	return Registered;
}

int32 UScenarioLoader::AssembleFleetAndMission(
	const UScenario* Scenario,
	UObject* WorldContext,
	TArray<AUAVPawn*>& OutFleet,
	TSubclassOf<AUAVPawn> DefaultUAVClass)
{
	OutFleet.Reset();

	if (!Scenario || !WorldContext)
	{
		UE_LOG(LogScenarioLoader, Warning, TEXT("[Scenario] AssembleFleet: null Scenario or WorldContext"));
		return 0;
	}

	UWorld* World = Cast<UWorld>(WorldContext);
	if (!World && GEngine)
	{
		World = GEngine->GetWorldFromContextObject(WorldContext, EGetWorldErrorMode::ReturnNull);
	}
	if (!World)
	{
		UE_LOG(LogScenarioLoader, Warning, TEXT("[Scenario] AssembleFleet: no World in context"));
		return 0;
	}

	const UFleetSetup* Fleet = Scenario->FleetSetup.LoadSynchronous();
	if (!Fleet || Fleet->Agents.Num() == 0)
	{
		UE_LOG(LogScenarioLoader, Log, TEXT("[Scenario] No FleetSetup on scenario '%s'"), *Scenario->Name);
		return 0;
	}

	for (const FScenarioAgentEntry& Entry : Fleet->Agents)
	{
		// Agent 未指定蓝图时回退到默认 UAV 类
		TSubclassOf<AUAVPawn> ClassToSpawn = Entry.UAVClass ? Entry.UAVClass : DefaultUAVClass;
		if (!ClassToSpawn)
		{
			UE_LOG(LogScenarioLoader, Warning, TEXT("[Scenario] Agent has no UAVClass and no default provided"));
			continue;
		}

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		FRotator SpawnRot(0.0f, Entry.InitialYaw, 0.0f);
		const FTransform SpawnTransform(SpawnRot,Entry.InitialPosition);
		AUAVPawn* UAV = World->SpawnActorDeferred<AUAVPawn>(ClassToSpawn,SpawnTransform,nullptr,nullptr,ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
		if (UAV)
		{
			UAV->SetModelID(Entry.ModelID);
			UAV->FinishSpawning(SpawnTransform);
			UAV->SetActorLocation(Entry.InitialPosition, false, nullptr, ETeleportType::ResetPhysics);
			UAV->SetActorRotation(SpawnRot, ETeleportType::ResetPhysics);
			OutFleet.Add(UAV);
		}
	}

	// 全局任务档案（作为每机内联航线为空时的回退）。
	const UMissionProfile* GlobalMissionProfile = Scenario->MissionProfile.LoadSynchronous();

	// 逐机装配航线：优先用 Agent 内联航点，为空则回退全局 MissionProfile。
	// 实现每机独立航线（修复历史"只装配到 Fleet[0]"的限制）。
	for (int32 Idx = 0; Idx < OutFleet.Num(); ++Idx)
	{
		const FScenarioAgentEntry& Entry = Fleet->Agents[Idx];
		AUAVPawn* UAV = OutFleet[Idx];
		if (!UAV)
		{
			continue;
		}

		UMissionComponent* Mission = UAV->FindComponentByClass<UMissionComponent>();
		if (!Mission)
		{
			continue;
		}

		// 内联航线非空 -> 该机独立航线；否则回退全局（若无全局则该机无任务，靠编队/行为树跟随）。
		const bool bHasInlineMission = Entry.Waypoints.Num() > 0;
		if (bHasInlineMission)
		{
			Mission->SetMissionMode(Entry.MissionMode);
			Mission->SetMissionWaypoints(Entry.Waypoints);
			Mission->StartMission();
		}
		else if (GlobalMissionProfile)
		{
			// 向后兼容：无内联航线时下发全局任务（含历史 Leader 行为）。
			Mission->SetMissionMode(GlobalMissionProfile->Mode);
			Mission->SetMissionWaypoints(GlobalMissionProfile->Waypoints);
			Mission->StartMission();
		}
	}

	UE_LOG(LogScenarioLoader, Log, TEXT("[Scenario] Fleet assembled: %d UAV(s) (scenario='%s')"),
		OutFleet.Num(), *Scenario->Name);
	return OutFleet.Num();
}

bool UScenarioLoader::AssembleWind(const UScenario* Scenario, UWindField* WindField)
{
	if (!Scenario || !WindField)
	{
		UE_LOG(LogScenarioLoader, Warning, TEXT("[Scenario] AssembleWind: null Scenario or WindField"));
		return false;
	}

	const UWindProfile* Profile = Scenario->WindProfile.LoadSynchronous();
	if (!Profile)
	{
		UE_LOG(LogScenarioLoader, Log, TEXT("[Scenario] No WindProfile on scenario '%s' (keep default)"), *Scenario->Name);
		return false;
	}

	WindField->SetWindConfig(Profile->Config);
	UE_LOG(LogScenarioLoader, Log, TEXT("[Scenario] WindProfile applied (scenario='%s', WindType=%d)"),
		*Scenario->Name, (int32)Profile->Config.WindType);
	return true;
}
