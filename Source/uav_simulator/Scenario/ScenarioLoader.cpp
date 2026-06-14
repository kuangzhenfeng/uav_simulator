// Copyright Epic Games, Inc. All Rights Reserved.

#include "ScenarioLoader.h"
#include "../uav_simulator.h"
#include "../Planning/ObstacleManager.h"
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
		Obstacle.bIsDynamic = Entry.bIsDynamic;
		Obstacle.Velocity = Entry.Velocity;

		// Spawn 可视化 Actor 并建立关联（逻辑几何与可视化表现同源）。
		// SpawnActor 对裸 AActor 基类不会自动落地 Location（无 RootComponent），
		// 真实场景使用 BP_Obstacle_Default 时其 RootComponent 会处理位姿；
		// 此处显式 SetActorLocation/Rotation 保证裸 Actor 也落到声明位姿。
		if (World)
		{
			FActorSpawnParameters SpawnParams;
			SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
			AActor* VisualActor = World->SpawnActor<AActor>(AActor::StaticClass(), Entry.Center, Entry.Rotation, SpawnParams);
			if (VisualActor)
			{
				Obstacle.LinkedActor = VisualActor;
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
		AUAVPawn* UAV = World->SpawnActor<AUAVPawn>(ClassToSpawn, Entry.InitialPosition, SpawnRot, SpawnParams);
		if (UAV)
		{
			UAV->SetModelID(Entry.ModelID);
			UAV->SetActorLocation(Entry.InitialPosition, false, nullptr, ETeleportType::ResetPhysics);
			UAV->SetActorRotation(SpawnRot, ETeleportType::ResetPhysics);
			OutFleet.Add(UAV);
		}
	}

	// 把任务档案下发给首个 UAV 的 MissionComponent 并启动任务。
	const UMissionProfile* MissionProfile = Scenario->MissionProfile.LoadSynchronous();
	if (MissionProfile && OutFleet.Num() > 0)
	{
		AUAVPawn* Lead = OutFleet[0];
		if (UMissionComponent* Mission = Lead->FindComponentByClass<UMissionComponent>())
		{
			Mission->SetMissionMode(MissionProfile->Mode);
			Mission->SetMissionWaypoints(MissionProfile->Waypoints);
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
