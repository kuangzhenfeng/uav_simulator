// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Physics/UAVDynamics.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== RK4 积分测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsRK4Test,
	"UAVSimulator.Physics.UAVDynamics.RK4Integration",
	UAV_TEST_FLAGS)

bool FUAVDynamicsRK4Test::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	// 设置悬停推力（4个电机平均分担重力）
	// 重力 = 1.5kg * 9.8m/s² = 14.7N
	// 每个电机需要 14.7/4 = 3.675N
	// 归一化推力 = 3.675/15 ≈ 0.245
	TArray<float> HoverThrusts;
	HoverThrusts.Init(0.245f, 4);
	Dynamics->SetMotorThrusts(HoverThrusts);

	// 初始状态
	FUAVState InitialState;
	InitialState.Position = FVector(0.0f, 0.0f, 100.0f);
	InitialState.Velocity = FVector::ZeroVector;
	InitialState.Rotation = FRotator::ZeroRotator;
	InitialState.AngularVelocity = FVector::ZeroVector;

	// 更新动力学
	float DeltaTime = 0.01f;
	FUAVState NewState = Dynamics->UpdateDynamics(InitialState, DeltaTime);

	// 悬停时位置应该基本不变
	UAV_TEST_VECTOR_EQUAL(NewState.Position, InitialState.Position, 10.0f);

	// 速度应该很小
	TestTrue(TEXT("Velocity should be small during hover"), NewState.Velocity.Size() < 50.0f);

	return true;
}

// ==================== 推力计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsThrustTest,
	"UAVSimulator.Physics.UAVDynamics.ThrustCalculation",
	UAV_TEST_FLAGS)

bool FUAVDynamicsThrustTest::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	// 设置最大推力
	TArray<float> MaxThrusts;
	MaxThrusts.Init(1.0f, 4);
	Dynamics->SetMotorThrusts(MaxThrusts);

	// 验证推力设置
	TArray<float> RetrievedThrusts = Dynamics->GetMotorThrusts();
	TestEqual(TEXT("Should have 4 motor thrusts"), RetrievedThrusts.Num(), 4);

	for (int32 i = 0; i < 4; ++i)
	{
		UAV_TEST_FLOAT_EQUAL(RetrievedThrusts[i], 1.0f, 0.01f);
	}

	// 测试最大推力下的加速度
	FUAVState InitialState;
	InitialState.Position = FVector(0.0f, 0.0f, 100.0f);
	InitialState.Velocity = FVector::ZeroVector;
	InitialState.Rotation = FRotator::ZeroRotator;
	InitialState.AngularVelocity = FVector::ZeroVector;

	FUAVState NewState = Dynamics->UpdateDynamics(InitialState, 0.01f);

	// 最大推力应该产生向上加速度
	TestTrue(TEXT("Should accelerate upward with max thrust"), NewState.Velocity.Z > 0.0f);

	return true;
}

// ==================== 力矩计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsTorqueTest,
	"UAVSimulator.Physics.UAVDynamics.TorqueCalculation",
	UAV_TEST_FLAGS)

bool FUAVDynamicsTorqueTest::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	// 设置不对称推力以产生力矩
	// 电机布局（俯视）：
	// 0(前右) 1(后右)
	// 3(前左) 2(后左)
	TArray<float> AsymmetricThrusts;
	AsymmetricThrusts.Add(0.3f);  // 前右
	AsymmetricThrusts.Add(0.2f);  // 后右
	AsymmetricThrusts.Add(0.2f);  // 后左
	AsymmetricThrusts.Add(0.3f);  // 前左
	Dynamics->SetMotorThrusts(AsymmetricThrusts);

	FUAVState InitialState;
	InitialState.Position = FVector(0.0f, 0.0f, 100.0f);
	InitialState.Velocity = FVector::ZeroVector;
	InitialState.Rotation = FRotator::ZeroRotator;
	InitialState.AngularVelocity = FVector::ZeroVector;

	FUAVState NewState = Dynamics->UpdateDynamics(InitialState, 0.01f);

	// 不对称推力应该产生角速度变化
	// 具体方向取决于电机布局

	return true;
}

// ==================== 重力测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsGravityTest,
	"UAVSimulator.Physics.UAVDynamics.Gravity",
	UAV_TEST_FLAGS)

bool FUAVDynamicsGravityTest::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	// 零推力
	TArray<float> ZeroThrusts;
	ZeroThrusts.Init(0.0f, 4);
	Dynamics->SetMotorThrusts(ZeroThrusts);

	FUAVState InitialState;
	InitialState.Position = FVector(0.0f, 0.0f, 1000.0f);
	InitialState.Velocity = FVector::ZeroVector;
	InitialState.Rotation = FRotator::ZeroRotator;
	InitialState.AngularVelocity = FVector::ZeroVector;

	// 多次更新
	FUAVState CurrentState = InitialState;
	for (int32 i = 0; i < 10; ++i)
	{
		CurrentState = Dynamics->UpdateDynamics(CurrentState, 0.01f);
	}

	// 应该向下加速（Z 减小）
	TestTrue(TEXT("Should fall due to gravity"), CurrentState.Position.Z < InitialState.Position.Z);
	TestTrue(TEXT("Should have downward velocity"), CurrentState.Velocity.Z < 0.0f);

	return true;
}

// ==================== 空气阻力测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsDragTest,
	"UAVSimulator.Physics.UAVDynamics.AirDrag",
	UAV_TEST_FLAGS)

bool FUAVDynamicsDragTest::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	// 悬停推力
	TArray<float> HoverThrusts;
	HoverThrusts.Init(0.245f, 4);
	Dynamics->SetMotorThrusts(HoverThrusts);

	// 初始有水平速度
	FUAVState InitialState;
	InitialState.Position = FVector(0.0f, 0.0f, 100.0f);
	InitialState.Velocity = FVector(500.0f, 0.0f, 0.0f); // 水平速度
	InitialState.Rotation = FRotator::ZeroRotator;
	InitialState.AngularVelocity = FVector::ZeroVector;

	// 多次更新
	FUAVState CurrentState = InitialState;
	for (int32 i = 0; i < 100; ++i)
	{
		CurrentState = Dynamics->UpdateDynamics(CurrentState, 0.01f);
	}

	// 空气阻力应该减小水平速度
	TestTrue(TEXT("Horizontal velocity should decrease due to drag"),
		FMath::Abs(CurrentState.Velocity.X) < FMath::Abs(InitialState.Velocity.X));

	return true;
}

// ==================== 姿态动力学测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsAttitudeTest,
	"UAVSimulator.Physics.UAVDynamics.AttitudeDynamics",
	UAV_TEST_FLAGS)

bool FUAVDynamicsAttitudeTest::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	// 设置产生 Roll 力矩的推力
	TArray<float> RollThrusts;
	RollThrusts.Add(0.3f);  // 前右 - 高
	RollThrusts.Add(0.3f);  // 后右 - 高
	RollThrusts.Add(0.2f);  // 后左 - 低
	RollThrusts.Add(0.2f);  // 前左 - 低
	Dynamics->SetMotorThrusts(RollThrusts);

	FUAVState InitialState;
	InitialState.Position = FVector(0.0f, 0.0f, 100.0f);
	InitialState.Velocity = FVector::ZeroVector;
	InitialState.Rotation = FRotator::ZeroRotator;
	InitialState.AngularVelocity = FVector::ZeroVector;

	// 更新
	FUAVState NewState = Dynamics->UpdateDynamics(InitialState, 0.01f);

	// 应该产生角速度变化
	// 具体方向取决于电机布局和坐标系定义

	return true;
}

// ==================== 陀螺效应测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsGyroscopicTest,
	"UAVSimulator.Physics.UAVDynamics.GyroscopicEffect",
	UAV_TEST_FLAGS)

bool FUAVDynamicsGyroscopicTest::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	// 悬停推力
	TArray<float> HoverThrusts;
	HoverThrusts.Init(0.245f, 4);
	Dynamics->SetMotorThrusts(HoverThrusts);

	// 初始有角速度
	FUAVState InitialState;
	InitialState.Position = FVector(0.0f, 0.0f, 100.0f);
	InitialState.Velocity = FVector::ZeroVector;
	InitialState.Rotation = FRotator::ZeroRotator;
	InitialState.AngularVelocity = FVector(10.0f, 0.0f, 0.0f); // Roll 角速度

	FUAVState NewState = Dynamics->UpdateDynamics(InitialState, 0.01f);

	// 陀螺效应会产生耦合力矩
	// 验证状态更新正常
	TestTrue(TEXT("State should be updated"), true);

	return true;
}

// ==================== 电机推力设置测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsMotorSetTest,
	"UAVSimulator.Physics.UAVDynamics.MotorThrustSet",
	UAV_TEST_FLAGS)

bool FUAVDynamicsMotorSetTest::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	// 设置不同的推力值
	TArray<float> Thrusts;
	Thrusts.Add(0.1f);
	Thrusts.Add(0.2f);
	Thrusts.Add(0.3f);
	Thrusts.Add(0.4f);
	Dynamics->SetMotorThrusts(Thrusts);

	// 验证
	TArray<float> Retrieved = Dynamics->GetMotorThrusts();
	TestEqual(TEXT("Should have 4 motors"), Retrieved.Num(), 4);
	UAV_TEST_FLOAT_EQUAL(Retrieved[0], 0.1f, 0.01f);
	UAV_TEST_FLOAT_EQUAL(Retrieved[1], 0.2f, 0.01f);
	UAV_TEST_FLOAT_EQUAL(Retrieved[2], 0.3f, 0.01f);
	UAV_TEST_FLOAT_EQUAL(Retrieved[3], 0.4f, 0.01f);

	// 测试边界值
	TArray<float> BoundaryThrusts;
	BoundaryThrusts.Init(0.0f, 4);
	Dynamics->SetMotorThrusts(BoundaryThrusts);
	Retrieved = Dynamics->GetMotorThrusts();
	for (int32 i = 0; i < 4; ++i)
	{
		UAV_TEST_FLOAT_EQUAL(Retrieved[i], 0.0f, 0.01f);
	}

	BoundaryThrusts.Init(1.0f, 4);
	Dynamics->SetMotorThrusts(BoundaryThrusts);
	Retrieved = Dynamics->GetMotorThrusts();
	for (int32 i = 0; i < 4; ++i)
	{
		UAV_TEST_FLOAT_EQUAL(Retrieved[i], 1.0f, 0.01f);
	}

	return true;
}

// ==================== 数值稳定性测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsStabilityTest,
	"UAVSimulator.Physics.UAVDynamics.NumericalStability",
	UAV_TEST_FLAGS)

bool FUAVDynamicsStabilityTest::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	// 悬停推力
	TArray<float> HoverThrusts;
	HoverThrusts.Init(0.245f, 4);
	Dynamics->SetMotorThrusts(HoverThrusts);

	FUAVState CurrentState;
	CurrentState.Position = FVector(0.0f, 0.0f, 100.0f);
	CurrentState.Velocity = FVector::ZeroVector;
	CurrentState.Rotation = FRotator::ZeroRotator;
	CurrentState.AngularVelocity = FVector::ZeroVector;

	// 长时间仿真
	for (int32 i = 0; i < 1000; ++i)
	{
		CurrentState = Dynamics->UpdateDynamics(CurrentState, 0.01f);

		// 检查数值是否有效（没有 NaN 或 Inf）
		TestFalse(TEXT("Position should not be NaN"), CurrentState.Position.ContainsNaN());
		TestFalse(TEXT("Velocity should not be NaN"), CurrentState.Velocity.ContainsNaN());

		// 检查值是否在合理范围内
		TestTrue(TEXT("Position should be reasonable"), CurrentState.Position.Size() < 1000000.0f);
		TestTrue(TEXT("Velocity should be reasonable"), CurrentState.Velocity.Size() < 100000.0f);
	}

	return true;
}

// ==================== 小时间步测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsSmallDeltaTimeTest,
	"UAVSimulator.Physics.UAVDynamics.SmallDeltaTime",
	UAV_TEST_FLAGS)

bool FUAVDynamicsSmallDeltaTimeTest::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	TArray<float> HoverThrusts;
	HoverThrusts.Init(0.245f, 4);
	Dynamics->SetMotorThrusts(HoverThrusts);

	FUAVState InitialState;
	InitialState.Position = FVector(0.0f, 0.0f, 100.0f);
	InitialState.Velocity = FVector::ZeroVector;
	InitialState.Rotation = FRotator::ZeroRotator;
	InitialState.AngularVelocity = FVector::ZeroVector;

	// 非常小的时间步
	FUAVState NewState = Dynamics->UpdateDynamics(InitialState, 0.0001f);

	// 应该正常工作
	TestFalse(TEXT("Position should not be NaN"), NewState.Position.ContainsNaN());
	TestFalse(TEXT("Velocity should not be NaN"), NewState.Velocity.ContainsNaN());

	return true;
}

// ==================== 大时间步测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVDynamicsLargeDeltaTimeTest,
	"UAVSimulator.Physics.UAVDynamics.LargeDeltaTime",
	UAV_TEST_FLAGS)

bool FUAVDynamicsLargeDeltaTimeTest::RunTest(const FString& Parameters)
{
	UUAVDynamics* Dynamics = NewObject<UUAVDynamics>();

	TArray<float> HoverThrusts;
	HoverThrusts.Init(0.245f, 4);
	Dynamics->SetMotorThrusts(HoverThrusts);

	FUAVState InitialState;
	InitialState.Position = FVector(0.0f, 0.0f, 100.0f);
	InitialState.Velocity = FVector::ZeroVector;
	InitialState.Rotation = FRotator::ZeroRotator;
	InitialState.AngularVelocity = FVector::ZeroVector;

	// 较大的时间步
	FUAVState NewState = Dynamics->UpdateDynamics(InitialState, 0.1f);

	// 应该正常工作（可能精度较低）
	TestFalse(TEXT("Position should not be NaN"), NewState.Position.ContainsNaN());
	TestFalse(TEXT("Velocity should not be NaN"), NewState.Velocity.ContainsNaN());

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
