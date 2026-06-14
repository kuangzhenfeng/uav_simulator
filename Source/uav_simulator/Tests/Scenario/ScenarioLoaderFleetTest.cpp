// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Scenario/ScenarioLoader.h"
#include "../../Scenario/ScenarioTypes.h"
#include "../../Core/UAVPawn.h"
#include "../../Mission/MissionComponent.h"
#include "../../Mission/MissionTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	// 合成 World helper 已集中到 UAVTestCommon.h，避免跨 TU 重定义。

	// 构造一个单 Agent + 单航点的场景。
	UScenario* MakeSingleAgentScenario(UObject* Outer,
		const FVector& SpawnPos, const FVector& WaypointPos)
	{
		UScenario* Scenario = NewObject<UScenario>(Outer);
		Scenario->Name = TEXT("FleetMissionTest");

		UFleetSetup* Fleet = NewObject<UFleetSetup>(Scenario);
		FScenarioAgentEntry& Agent = Fleet->Agents.AddDefaulted_GetRef();
		Agent.ModelID = EUAVModelID::Agri_AG20;
		Agent.InitialPosition = SpawnPos;
		Scenario->FleetSetup = Fleet;

		UMissionProfile* Mission = NewObject<UMissionProfile>(Scenario);
		Mission->Mode = EMissionMode::Once;
		FMissionWaypoint& WP = Mission->Waypoints.AddDefaulted_GetRef();
		WP.Position = WaypointPos;
		Scenario->MissionProfile = Mission;

		return Scenario;
	}
}

// ==================== 装配机队：Spawn 出声明的 UAV ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioFleetSpawnsUAVTest,
	"UAVSimulator.Scenario.Fleet.SpawnsUAV",
	UAV_TEST_FLAGS)

bool FScenarioFleetSpawnsUAVTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("ScenarioFleetSpawn"));
	if (!TestNotNull(TEXT("合成 World 创建成功"), World))
	{
		return false;
	}

	UScenarioLoader* Loader = NewObject<UScenarioLoader>();
	UScenario* Scenario = MakeSingleAgentScenario(GetTransientPackage(),
		FVector(0.0f, 0.0f, 200.0f), FVector(5000.0f, 0.0f, 200.0f));

	TArray<AUAVPawn*> Fleet;
	const int32 Count = Loader->AssembleFleetAndMission(Scenario, World, Fleet, AUAVPawn::StaticClass());

	TestEqual(TEXT("Spawn 出 1 架 UAV"), Count, 1);
	TestEqual(TEXT("Fleet 数组含 1 架"), Fleet.Num(), 1);
	if (Fleet.Num() == 1)
	{
		TestNotNull(TEXT("UAV 实例非空"), Fleet[0]);
	}

	DestroyScenarioTestWorld(World);
	return true;
}

// ==================== 装配机队：型号与位姿正确 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioFleetModelAndPoseTest,
	"UAVSimulator.Scenario.Fleet.ModelAndPose",
	UAV_TEST_FLAGS)

bool FScenarioFleetModelAndPoseTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("ScenarioFleetPose"));
	if (!TestNotNull(TEXT("合成 World 创建成功"), World))
	{
		return false;
	}

	UScenarioLoader* Loader = NewObject<UScenarioLoader>();
	const FVector SpawnPos(1000.0f, -2000.0f, 500.0f);
	UScenario* Scenario = MakeSingleAgentScenario(GetTransientPackage(), SpawnPos, FVector(6000.0f, 0.0f, 500.0f));

	TArray<AUAVPawn*> Fleet;
	Loader->AssembleFleetAndMission(Scenario, World, Fleet, AUAVPawn::StaticClass());

	if (!TestTrue(TEXT("Fleet 含 1 架"), Fleet.Num() == 1))
	{
		DestroyScenarioTestWorld(World);
		return false;
	}

	// 型号与位姿应是装配后的外部可观察状态
	TestEqual(TEXT("型号一致"), (int32)Fleet[0]->GetModelID(), (int32)EUAVModelID::Agri_AG20);
	UAV_TEST_VECTOR_EQUAL(Fleet[0]->GetActorLocation(), SpawnPos, 1.0f);

	DestroyScenarioTestWorld(World);
	return true;
}

// ==================== 装配任务：航点已下发 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioFleetMissionWaypointsTest,
	"UAVSimulator.Scenario.Fleet.MissionWaypoints",
	UAV_TEST_FLAGS)

bool FScenarioFleetMissionWaypointsTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("ScenarioFleetMission"));
	if (!TestNotNull(TEXT("合成 World 创建成功"), World))
	{
		return false;
	}

	UScenarioLoader* Loader = NewObject<UScenarioLoader>();
	const FVector WP(8000.0f, 1000.0f, 300.0f);
	UScenario* Scenario = MakeSingleAgentScenario(GetTransientPackage(), FVector::ZeroVector, WP);

	TArray<AUAVPawn*> Fleet;
	Loader->AssembleFleetAndMission(Scenario, World, Fleet, AUAVPawn::StaticClass());

	if (!TestTrue(TEXT("Fleet 含 1 架"), Fleet.Num() == 1))
	{
		DestroyScenarioTestWorld(World);
		return false;
	}

	UMissionComponent* Mission = Fleet[0]->FindComponentByClass<UMissionComponent>();
	if (!TestNotNull(TEXT("UAV 有 MissionComponent"), Mission))
	{
		DestroyScenarioTestWorld(World);
		return false;
	}

	const TArray<FMissionWaypoint>& WPs = Mission->GetMissionWaypoints();
	TestEqual(TEXT("航点数 = 1"), WPs.Num(), 1);
	if (WPs.Num() == 1)
	{
		UAV_TEST_VECTOR_EQUAL(WPs[0].Position, WP, 1.0f);
	}

	DestroyScenarioTestWorld(World);
	return true;
}

// ==================== 装配任务：任务已启动（状态非 Idle） ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioFleetMissionStartedTest,
	"UAVSimulator.Scenario.Fleet.MissionStarted",
	UAV_TEST_FLAGS)

bool FScenarioFleetMissionStartedTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("ScenarioFleetStart"));
	if (!TestNotNull(TEXT("合成 World 创建成功"), World))
	{
		return false;
	}

	UScenarioLoader* Loader = NewObject<UScenarioLoader>();
	UScenario* Scenario = MakeSingleAgentScenario(GetTransientPackage(),
		FVector::ZeroVector, FVector(5000.0f, 0.0f, 200.0f));

	TArray<AUAVPawn*> Fleet;
	Loader->AssembleFleetAndMission(Scenario, World, Fleet, AUAVPawn::StaticClass());

	if (!TestTrue(TEXT("Fleet 含 1 架"), Fleet.Num() == 1))
	{
		DestroyScenarioTestWorld(World);
		return false;
	}

	UMissionComponent* Mission = Fleet[0]->FindComponentByClass<UMissionComponent>();
	if (!TestNotNull(TEXT("UAV 有 MissionComponent"), Mission))
	{
		DestroyScenarioTestWorld(World);
		return false;
	}

	// 装配后任务应已启动：状态不再是 Idle
	const EMissionState State = Mission->GetMissionState();
	TestFalse(TEXT("任务已启动(非 Idle)"), State == EMissionState::Idle);

	DestroyScenarioTestWorld(World);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
