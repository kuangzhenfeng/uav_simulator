// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Scenario/ScenarioLoader.h"
#include "../../Scenario/ScenarioTypes.h"
#include "../../Scenario/ScenarioEvaluator.h"
#include "../../Core/UAVPawn.h"
#include "../../Mission/MissionComponent.h"
#include "../../Mission/MissionTypes.h"
#include "../../Environment/WindField.h"
#include "../../Environment/EnvironmentTypes.h"
#include "../../Planning/ObstacleManager.h"
#include "../../Core/UAVTypes.h"

#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	// 合成 World 与场景构造 helper 已集中到 UAVTestCommon.h，避免跨 TU 重定义。

	// 构造"StraightThroughObstacles"示例场景（内存版）：
	// 单架 UAV 从原点出发，穿越两个静态障碍到达远处航点。
	UScenario* MakeStraightThroughScenario(UObject* Outer)
	{
		UScenario* Scenario = NewObject<UScenario>(Outer);
		Scenario->Name = TEXT("StraightThroughObstacles");
		Scenario->RandomSeed = 42;

		// 障碍布局：路径上两个静态球障碍
		UObstacleLayout* Layout = NewObject<UObstacleLayout>(Scenario);
		FScenarioObstacleEntry& O1 = Layout->Obstacles.AddDefaulted_GetRef();
		O1.Type = EObstacleType::Sphere;
		O1.Center = FVector(3000.0f, 800.0f, 0.0f);
		O1.Extents = FVector(300.0f);
		O1.SafetyMargin = 100.0f;

		FScenarioObstacleEntry& O2 = Layout->Obstacles.AddDefaulted_GetRef();
		O2.Type = EObstacleType::Sphere;
		O2.Center = FVector(6000.0f, -800.0f, 0.0f);
		O2.Extents = FVector(300.0f);
		O2.SafetyMargin = 100.0f;
		Scenario->ObstacleLayout = Layout;

		// 风场档案：恒定微风
		UWindProfile* Wind = NewObject<UWindProfile>(Scenario);
		Wind->Config.WindType = EWindFieldType::Constant;
		Wind->Config.SteadyWindVelocity = FVector(200.0f, 0.0f, 0.0f);
		Wind->Config.AirDensity = 1.225f;
		Scenario->WindProfile = Wind;

		// 机队：单 Agent 从原点出发
		UFleetSetup* Fleet = NewObject<UFleetSetup>(Scenario);
		FScenarioAgentEntry& Agent = Fleet->Agents.AddDefaulted_GetRef();
		Agent.ModelID = EUAVModelID::Agri_AG20;
		Agent.InitialPosition = FVector(0.0f, 0.0f, 200.0f);
		Scenario->FleetSetup = Fleet;

		// 任务：直线到 9000cm 处
		UMissionProfile* Mission = NewObject<UMissionProfile>(Scenario);
		Mission->Mode = EMissionMode::Once;
		FMissionWaypoint& WP = Mission->Waypoints.AddDefaulted_GetRef();
		WP.Position = FVector(9000.0f, 0.0f, 200.0f);
		Scenario->MissionProfile = Mission;

		// 验收标准
		UAcceptanceCriteria* Criteria = NewObject<UAcceptanceCriteria>(Scenario);
		Criteria->bRequireAllWaypoints = true;
		Criteria->MinClearanceCm = 100.0f;
		Criteria->MaxLateralDeviationCm = 300.0f;
		Criteria->TimeoutSeconds = 120.0f;
		Scenario->AcceptanceCriteria = Criteria;

		return Scenario;
	}
}

// ==================== 端到端：装配链路打通（风场→障碍→机队/任务） ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioE2EAssemblyPipelineTest,
	"UAVSimulator.Scenario.E2E.AssemblyPipeline",
	UAV_TEST_FLAGS)

bool FScenarioE2EAssemblyPipelineTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("ScenarioE2EAssembly"));
	if (!TestNotNull(TEXT("合成 World 创建成功"), World))
	{
		return false;
	}

	UScenario* Scenario = MakeStraightThroughScenario(GetTransientPackage());

	// 场景级 WindField（模拟 GameMode 持有）
	UWindField* SceneWindField = NewObject<UWindField>();
	UScenarioLoader* Loader = NewObject<UScenarioLoader>();

	// 装配顺序：风场 → 机队/任务 → 障碍（机队 Spawn 后才有 ObstacleManager）
	TestTrue(TEXT("风场装配成功"), Loader->AssembleWind(Scenario, SceneWindField));

	TArray<AUAVPawn*> Fleet;
	Loader->AssembleFleetAndMission(Scenario, World, Fleet, AUAVPawn::StaticClass());
	TestEqual(TEXT("Spawn 出 1 架 UAV"), Fleet.Num(), 1);

	int32 ObstaclesRegistered = 0;
	if (Fleet.Num() > 0 && Fleet[0])
	{
		UObstacleManager* Obs = Fleet[0]->FindComponentByClass<UObstacleManager>();
		if (TestNotNull(TEXT("UAV 有 ObstacleManager"), Obs))
		{
			ObstaclesRegistered = Loader->AssembleObstacles(Scenario, Obs, World);
		}
	}
	TestEqual(TEXT("注册了 2 个障碍"), ObstaclesRegistered, 2);

	DestroyScenarioTestWorld(World);
	return true;
}

// ==================== 端到端：风场配置已下发到场景级单例 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioE2EWindAppliedTest,
	"UAVSimulator.Scenario.E2E.WindApplied",
	UAV_TEST_FLAGS)

bool FScenarioE2EWindAppliedTest::RunTest(const FString& Parameters)
{
	UScenario* Scenario = MakeStraightThroughScenario(GetTransientPackage());
	UWindField* SceneWindField = NewObject<UWindField>();
	UScenarioLoader* Loader = NewObject<UScenarioLoader>();

	Loader->AssembleWind(Scenario, SceneWindField);

	FWindConfig Applied = SceneWindField->GetWindConfig();
	TestEqual(TEXT("风场类型为 Constant"), (int32)Applied.WindType, (int32)EWindFieldType::Constant);
	UAV_TEST_VECTOR_EQUAL(Applied.SteadyWindVelocity, FVector(200.0f, 0.0f, 0.0f), 1.0f);

	return true;
}

// ==================== 端到端：验收链路（装配 → 指标 → 判决 → JSON） ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioE2EVerdictPipelineTest,
	"UAVSimulator.Scenario.E2E.VerdictPipeline",
	UAV_TEST_FLAGS)

bool FScenarioE2EVerdictPipelineTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("ScenarioE2EVerdict"));
	if (!TestNotNull(TEXT("合成 World 创建成功"), World))
	{
		return false;
	}

	UScenario* Scenario = MakeStraightThroughScenario(GetTransientPackage());
	UScenarioLoader* Loader = NewObject<UScenarioLoader>();

	TArray<AUAVPawn*> Fleet;
	Loader->AssembleFleetAndMission(Scenario, World, Fleet, AUAVPawn::StaticClass());
	UAcceptanceCriteria* Criteria = Scenario->AcceptanceCriteria.LoadSynchronous();

	// 模拟"全部航点到达、无碰撞、净空充足"的成功终态
	FScenarioMetrics Metrics;
	Metrics.WaypointsReached = 1;
	Metrics.WaypointsTotal = 1;
	Metrics.MinClearanceCm = 250.0f;     // > 100
	Metrics.MaxLateralDeviationCm = 180.0f; // < 300
	Metrics.ElapsedSec = 15.0f;           // < 120
	Metrics.bCollided = false;

	const FScenarioVerdict Verdict = UScenarioEvaluator::Evaluate(Metrics, Criteria);
	TestTrue(TEXT("模拟成功终态 -> verdict PASS"), Verdict.bPassed);

	// 写 JSON 到临时路径，验证可读回
	const FString TempPath = FPaths::CreateTempFilename(*FPaths::ProjectIntermediateDir(), TEXT("E2EVerdict"), TEXT(".json"));
	TestTrue(TEXT("JSON 写入成功"), UScenarioEvaluator::WriteResultJson(Scenario->Name, Scenario->RandomSeed, Verdict, TempPath));

	FString Content;
	TestTrue(TEXT("JSON 可读回"), FFileHelper::LoadFileToString(Content, *TempPath));
	TestTrue(TEXT("JSON 含场景名 StraightThroughObstacles"), Content.Contains(TEXT("StraightThroughObstacles")));
	TestTrue(TEXT("JSON 含 verdict PASS"), Content.Contains(TEXT("\"verdict\": \"PASS\"")));

	IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
	PF.DeleteFile(*TempPath);

	DestroyScenarioTestWorld(World);
	return true;
}

// ==================== 端到端：FAIL 终态正确写出（覆盖 pkill 强杀兜底场景） ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioE2EFailSnapshotTest,
	"UAVSimulator.Scenario.E2E.FailSnapshot",
	UAV_TEST_FLAGS)

bool FScenarioE2EFailSnapshotTest::RunTest(const FString& Parameters)
{
	UScenario* Scenario = MakeStraightThroughScenario(GetTransientPackage());
	UAcceptanceCriteria* Criteria = Scenario->AcceptanceCriteria.LoadSynchronous();

	// 模拟"任务未完成、发生碰撞"的失败终态（这正是 sim.sh pkill 强杀时
	// 靠周期快照兜底的场景——Evaluator 应已写出最近一次 FAIL 快照）
	FScenarioMetrics Metrics;
	Metrics.WaypointsReached = 0;
	Metrics.WaypointsTotal = 1;
	Metrics.bCollided = true;
	Metrics.MinClearanceCm = 50.0f; // < 100

	const FScenarioVerdict Verdict = UScenarioEvaluator::Evaluate(Metrics, Criteria);
	TestFalse(TEXT("碰撞+未到达 -> verdict FAIL"), Verdict.bPassed);
	TestTrue(TEXT("失败项含碰撞"), Verdict.Failures.ContainsByPredicate(
		[](const FString& F) { return F.Contains(TEXT("Collision")); }));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
