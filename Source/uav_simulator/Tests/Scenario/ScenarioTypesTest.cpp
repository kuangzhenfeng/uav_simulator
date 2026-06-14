// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Scenario/ScenarioTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * 场景资产类型层烟雾测试。
 * 验证 UScenario 及五个子资产可实例化、默认值合理。
 * 这是类型层的存在性验证；装配行为由 ScenarioLoader 测试覆盖，
 * 验收判定行为由 ScenarioEvaluator 测试覆盖。
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FScenarioTypesSmokeTest,
	"UAVSimulator.Scenario.TypesSmoke",
	UAV_TEST_FLAGS)

bool FScenarioTypesSmokeTest::RunTest(const FString& Parameters)
{
	// UScenario 默认状态：无名称、随机种子为 0、所有子资产引用为空、算法覆盖默认关闭
	UScenario* Scenario = NewObject<UScenario>();
	TestNotNull(TEXT("UScenario 可实例化"), Scenario);
	TestEqual(TEXT("默认随机种子为 0"), Scenario->RandomSeed, 0);
	TestTrue(TEXT("默认算法覆盖关闭(控制模式)"), !Scenario->AlgorithmOverride.bOverrideControlMode);
	TestTrue(TEXT("默认算法覆盖关闭(MPC类型)"), !Scenario->AlgorithmOverride.bOverrideMPCType);
	TestNull(TEXT("默认无障碍布局引用"), Scenario->ObstacleLayout.Get());
	TestNull(TEXT("默认无风场档案引用"), Scenario->WindProfile.Get());
	TestNull(TEXT("默认无机队配置引用"), Scenario->FleetSetup.Get());
	TestNull(TEXT("默认无任务档案引用"), Scenario->MissionProfile.Get());
	TestNull(TEXT("默认无验收标准引用"), Scenario->AcceptanceCriteria.Get());

	// 五个子资产可实例化
	TestNotNull(TEXT("UObstacleLayout 可实例化"), NewObject<UObstacleLayout>());
	TestNotNull(TEXT("UWindProfile 可实例化"), NewObject<UWindProfile>());
	TestNotNull(TEXT("UFleetSetup 可实例化"), NewObject<UFleetSetup>());
	TestNotNull(TEXT("UMissionProfile 可实例化"), NewObject<UMissionProfile>());
	TestNotNull(TEXT("UAcceptanceCriteria 可实例化"), NewObject<UAcceptanceCriteria>());

	// UAcceptanceCriteria 默认值：要求到达全部航点、有合理阈值
	UAcceptanceCriteria* Criteria = NewObject<UAcceptanceCriteria>();
	TestTrue(TEXT("默认要求到达全部航点"), Criteria->bRequireAllWaypoints);
	TestTrue(TEXT("默认航点到达半径>0"), Criteria->WaypointArrivalRadius > 0.0f);
	TestTrue(TEXT("默认最大横向偏差>0"), Criteria->MaxLateralDeviationCm > 0.0f);
	TestTrue(TEXT("默认超时>0"), Criteria->TimeoutSeconds > 0.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
