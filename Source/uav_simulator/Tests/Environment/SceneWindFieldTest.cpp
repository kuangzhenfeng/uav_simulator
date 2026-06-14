// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Environment/WindField.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * 场景级 WindField characterization 测试。
 *
 * 锁住 WindField 的外部行为（SetWindConfig / ComputeWindDragAcceleration），
 * 使其从 UAVPawn 子组件提升为 GameMode 场景级单例（ADR-0002）后行为不退化。
 *
 * 这些测试只验证 WindField 自身的公开接口，不依赖其挂载位置，
 * 因此重构后无需修改。
 */

// ==================== 配置下发可读回 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneWindFieldConfigRoundTripTest,
	"UAVSimulator.Environment.WindField.ConfigRoundTrip",
	UAV_TEST_FLAGS)

bool FSceneWindFieldConfigRoundTripTest::RunTest(const FString& Parameters)
{
	UWindField* WindField = NewObject<UWindField>();

	FWindConfig Config;
	Config.WindType = EWindFieldType::Constant;
	Config.SteadyWindVelocity = FVector(500.0f, 0.0f, 0.0f);
	Config.AirDensity = 1.3f;
	Config.DragArea = 0.05f;
	WindField->SetWindConfig(Config);

	FWindConfig ReadBack = WindField->GetWindConfig();
	UAV_TEST_VECTOR_EQUAL(ReadBack.SteadyWindVelocity, Config.SteadyWindVelocity, 1.0f);
	UAV_TEST_FLOAT_EQUAL(ReadBack.AirDensity, Config.AirDensity, 1e-4f);

	return true;
}

// ==================== 阻力计算与 UAV 运动方向相反 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneWindFieldDragOpposesMotionTest,
	"UAVSimulator.Environment.WindField.DragOpposesMotion",
	UAV_TEST_FLAGS)

bool FSceneWindFieldDragOpposesMotionTest::RunTest(const FString& Parameters)
{
	UWindField* WindField = NewObject<UWindField>();

	// 恒定风场，零稳态风速
	FWindConfig Config;
	Config.WindType = EWindFieldType::Constant;
	Config.SteadyWindVelocity = FVector::ZeroVector;
	Config.AirDensity = 1.225f;
	Config.DragArea = 0.04f;
	Config.DragCoefficient = 1.0f;
	WindField->SetWindConfig(Config);

	// UAV 向 +X 运动，应产生 -X 方向的阻力加速度
	FVector Drag = WindField->ComputeWindDragAcceleration(
		FVector(1000.0f, 0.0f, 0.0f), // UAV 速度
		FVector::ZeroVector,           // 位置
		1.0f);                          // 质量

	TestTrue(TEXT("阻力 X 分量应为负（与运动方向相反）"), Drag.X < 0.0f);
	TestTrue(TEXT("阻力非零"), Drag.Size() > 0.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
