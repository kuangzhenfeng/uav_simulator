// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Environment/WindField.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== DragAcceleration 禁用风场测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWindFieldDragDisabledTest,
	"UAVSimulator.Environment.WindField.DragAcceleration_Disabled",
	UAV_TEST_FLAGS)

bool FWindFieldDragDisabledTest::RunTest(const FString& Parameters)
{
	UWindField* WindField = NewObject<UWindField>();

	// 禁用风场
	FWindConfig Config;
	Config.bEnabled = false;
	WindField->SetWindConfig(Config);

	// 即使有风速和 UAV 速度，也应返回零加速度
	FVector Drag = WindField->ComputeWindDragAcceleration(
		FVector(1000, 0, 0), FVector(0, 0, 0), 1.0f);

	UAV_TEST_VECTOR_EQUAL(Drag, FVector::ZeroVector, 0.01f);

	return true;
}

// ==================== DragAcceleration 零质量测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWindFieldDragZeroMassTest,
	"UAVSimulator.Environment.WindField.DragAcceleration_ZeroMass",
	UAV_TEST_FLAGS)

bool FWindFieldDragZeroMassTest::RunTest(const FString& Parameters)
{
	UWindField* WindField = NewObject<UWindField>();

	// 启用风场，设置风速
	FWindConfig Config;
	Config.bEnabled = true;
	Config.SteadyWindVelocity = FVector(1000, 0, 0);
	WindField->SetWindConfig(Config);

	// 手动设置当前风速（TickComponent 不会运行）
	// 通过 SetWindConfig 设置后，风场内部状态可能需要通过其他方式设置
	// 使用零质量 → 应返回零加速度（防止除零）
	FVector Drag = WindField->ComputeWindDragAcceleration(
		FVector(0, 0, 0), FVector(0, 0, 0), 0.0f);

	// 零质量时不应崩溃，应返回零或合理值
	TestTrue(TEXT("Zero mass should not crash and return small value"),
		Drag.Size() < 1e10f);

	return true;
}

// ==================== DragAcceleration 无相对风速测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWindFieldDragNoRelativeTest,
	"UAVSimulator.Environment.WindField.DragAcceleration_NoRelativeWind",
	UAV_TEST_FLAGS)

bool FWindFieldDragNoRelativeTest::RunTest(const FString& Parameters)
{
	UWindField* WindField = NewObject<UWindField>();

	FWindConfig Config;
	Config.bEnabled = true;
	// 显式清零风速，FWindConfig 默认 SteadyWindVelocity = (0, 100, 0)
	Config.SteadyWindVelocity = FVector::ZeroVector;
	// UAV 速度也为 0 → 无相对风速 → 零阻力
	WindField->SetWindConfig(Config);

	FVector Drag = WindField->ComputeWindDragAcceleration(
		FVector(0, 0, 0), FVector(0, 0, 0), 1.0f);

	// 无相对风速时阻力应为零
	UAV_TEST_VECTOR_EQUAL(Drag, FVector::ZeroVector, 0.01f);

	return true;
}

// ==================== DragAcceleration 逆风测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWindFieldDragHeadwindTest,
	"UAVSimulator.Environment.WindField.DragAcceleration_Headwind",
	UAV_TEST_FLAGS)

bool FWindFieldDragHeadwindTest::RunTest(const FString& Parameters)
{
	UWindField* WindField = NewObject<UWindField>();

	// 无风，UAV 向前飞 → 逆风阻力
	FWindConfig Config;
	Config.bEnabled = true;
	Config.SteadyWindVelocity = FVector::ZeroVector;
	Config.AirDensity = 1.225f;
	Config.DragArea = 0.04f;
	Config.DragCoefficient = 1.0f;
	WindField->SetWindConfig(Config);

	// UAV 速度 (1000, 0, 0)，质量 1.0 kg
	FVector Drag = WindField->ComputeWindDragAcceleration(
		FVector(1000, 0, 0), FVector(0, 0, 0), 1.0f);

	// 阻力方向应与速度方向相反（-X）
	TestTrue(TEXT("Drag should oppose velocity direction"),
		Drag.X < 0.0f);

	// 阻力量级检查: F = 0.5 * rho * V² * Cd * A
	// V = 1000 cm/s = 10 m/s
	// F = 0.5 * 1.225 * 100 * 1.0 * 0.04 = 2.45 N
	// a = F/m = 2.45 m/s² = 245 cm/s²
	TestTrue(TEXT("Drag magnitude should be reasonable"),
		Drag.Size() > 10.0f && Drag.Size() < 1000.0f);

	return true;
}

// ==================== DragAcceleration 平方比例测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWindFieldDragProportionalTest,
	"UAVSimulator.Environment.WindField.DragAcceleration_Proportional",
	UAV_TEST_FLAGS)

bool FWindFieldDragProportionalTest::RunTest(const FString& Parameters)
{
	UWindField* WindField = NewObject<UWindField>();

	FWindConfig Config;
	Config.bEnabled = true;
	Config.SteadyWindVelocity = FVector::ZeroVector;
	Config.AirDensity = 1.225f;
	Config.DragArea = 0.04f;
	Config.DragCoefficient = 1.0f;
	WindField->SetWindConfig(Config);

	float Mass = 1.0f;

	// 速度 V
	FVector Drag1 = WindField->ComputeWindDragAcceleration(
		FVector(500, 0, 0), FVector(0, 0, 0), Mass);

	// 速度 2V
	FVector Drag2 = WindField->ComputeWindDragAcceleration(
		FVector(1000, 0, 0), FVector(0, 0, 0), Mass);

	// 气动阻力 ∝ V² → 速度翻倍 → 阻力 4 倍
	float Ratio = Drag2.Size() / FMath::Max(Drag1.Size(), 0.001f);
	UAV_TEST_FLOAT_EQUAL(Ratio, 4.0f, 0.5f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
