// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Control/PositionController.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 基本控制计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPositionControllerBasicTest,
	"UAVSimulator.Control.PositionController.BasicControl",
	UAV_TEST_FLAGS)

bool FPositionControllerBasicTest::RunTest(const FString& Parameters)
{
	UPositionController* Controller = NewObject<UPositionController>();

	// 设置控制参数
	Controller->Kp_Position = 1.0f;
	Controller->Kp_Velocity = 2.0f;
	Controller->MaxVelocity = 500.0f;
	Controller->MaxTiltAngle = 30.0f;

	// 创建测试状态
	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector(0.0f, 0.0f, 100.0f),
		FVector::ZeroVector,
		FRotator::ZeroRotator,
		FVector::ZeroVector
	);

	// 目标位置
	FVector TargetPosition(500.0f, 0.0f, 100.0f);
	FVector TargetVelocity = FVector::ZeroVector;

	// 计算控制输出
	FRotator DesiredAttitude;
	float Thrust;
	Controller->ComputeControl(CurrentState, TargetPosition, TargetVelocity, DesiredAttitude, Thrust, 0.01f);

	// 验证输出有效
	TestTrue(TEXT("Thrust should be positive"), Thrust > 0.0f);
	TestTrue(TEXT("Thrust should be <= 1"), Thrust <= 1.0f);

	// 目标在 X 正方向，应该产生正 Pitch（向前倾斜）
	// 注意：具体符号取决于坐标系定义

	return true;
}

// ==================== 级联控制测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPositionControllerCascadeTest,
	"UAVSimulator.Control.PositionController.CascadeControl",
	UAV_TEST_FLAGS)

bool FPositionControllerCascadeTest::RunTest(const FString& Parameters)
{
	UPositionController* Controller = NewObject<UPositionController>();

	// 设置级联控制参数
	Controller->Kp_Position = 1.0f;
	Controller->Ki_Position = 0.0f;
	Controller->Kd_Position = 0.0f;
	Controller->Kp_Velocity = 2.0f;
	Controller->Ki_Velocity = 0.0f;
	Controller->Kd_Velocity = 0.0f;

	// 测试位置误差产生速度指令
	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector(0.0f, 0.0f, 100.0f),
		FVector::ZeroVector
	);

	FVector TargetPosition(1000.0f, 0.0f, 100.0f);

	FRotator DesiredAttitude;
	float Thrust;
	Controller->ComputeControl(CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, Thrust, 0.01f);

	// 大位置误差应该产生显著的姿态指令
	TestTrue(TEXT("Should produce attitude command for position error"),
		FMath::Abs(DesiredAttitude.Pitch) > 0.1f || FMath::Abs(DesiredAttitude.Roll) > 0.1f);

	return true;
}

// ==================== 速度限制测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPositionControllerVelocityLimitTest,
	"UAVSimulator.Control.PositionController.VelocityLimit",
	UAV_TEST_FLAGS)

bool FPositionControllerVelocityLimitTest::RunTest(const FString& Parameters)
{
	UPositionController* Controller = NewObject<UPositionController>();

	// 设置较低的速度限制
	Controller->MaxVelocity = 200.0f;
	Controller->Kp_Position = 10.0f; // 高增益以产生大速度指令

	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector(0.0f, 0.0f, 100.0f),
		FVector::ZeroVector
	);

	// 大位置误差
	FVector TargetPosition(5000.0f, 0.0f, 100.0f);

	FRotator DesiredAttitude;
	float Thrust;
	Controller->ComputeControl(CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, Thrust, 0.01f);

	// 输出应该有效（速度被限制）
	TestTrue(TEXT("Thrust should be valid"), Thrust >= 0.0f && Thrust <= 1.0f);

	return true;
}

// ==================== 倾斜角限制测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPositionControllerTiltLimitTest,
	"UAVSimulator.Control.PositionController.TiltLimit",
	UAV_TEST_FLAGS)

bool FPositionControllerTiltLimitTest::RunTest(const FString& Parameters)
{
	UPositionController* Controller = NewObject<UPositionController>();

	// 设置倾斜角限制
	Controller->MaxTiltAngle = 20.0f;
	Controller->Kp_Position = 10.0f;
	Controller->Kp_Velocity = 10.0f;

	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector(0.0f, 0.0f, 100.0f),
		FVector::ZeroVector
	);

	// 大位置误差
	FVector TargetPosition(10000.0f, 10000.0f, 100.0f);

	FRotator DesiredAttitude;
	float Thrust;
	Controller->ComputeControl(CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, Thrust, 0.01f);

	// 验证倾斜角被限制
	TestTrue(TEXT("Pitch should be within limit"),
		FMath::Abs(DesiredAttitude.Pitch) <= Controller->MaxTiltAngle + 1.0f);
	TestTrue(TEXT("Roll should be within limit"),
		FMath::Abs(DesiredAttitude.Roll) <= Controller->MaxTiltAngle + 1.0f);

	return true;
}

// ==================== 加速度前馈测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPositionControllerAccelerationFeedforwardTest,
	"UAVSimulator.Control.PositionController.AccelerationFeedforward",
	UAV_TEST_FLAGS)

bool FPositionControllerAccelerationFeedforwardTest::RunTest(const FString& Parameters)
{
	UPositionController* Controller = NewObject<UPositionController>();

	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector(500.0f, 0.0f, 100.0f),
		FVector(100.0f, 0.0f, 0.0f) // 正在移动
	);

	FVector TargetPosition(500.0f, 0.0f, 100.0f);
	FVector TargetVelocity(100.0f, 0.0f, 0.0f);
	FVector TargetAcceleration(50.0f, 0.0f, 0.0f);

	// 不带加速度前馈
	FRotator DesiredAttitude1;
	float Thrust1;
	Controller->ComputeControl(CurrentState, TargetPosition, TargetVelocity, DesiredAttitude1, Thrust1, 0.01f);

	// 带加速度前馈
	FRotator DesiredAttitude2;
	float Thrust2;
	Controller->ComputeControlWithAcceleration(CurrentState, TargetPosition, TargetVelocity, TargetAcceleration,
		DesiredAttitude2, Thrust2, 0.01f);

	// 两种方法都应该产生有效输出
	TestTrue(TEXT("Thrust without feedforward should be valid"), Thrust1 >= 0.0f && Thrust1 <= 1.0f);
	TestTrue(TEXT("Thrust with feedforward should be valid"), Thrust2 >= 0.0f && Thrust2 <= 1.0f);

	return true;
}

// ==================== 控制器重置测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPositionControllerResetTest,
	"UAVSimulator.Control.PositionController.Reset",
	UAV_TEST_FLAGS)

bool FPositionControllerResetTest::RunTest(const FString& Parameters)
{
	UPositionController* Controller = NewObject<UPositionController>();

	Controller->Ki_Position = 0.1f;
	Controller->Ki_Velocity = 0.1f;

	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector(0.0f, 0.0f, 100.0f),
		FVector::ZeroVector
	);

	FVector TargetPosition(500.0f, 0.0f, 100.0f);

	// 累积积分
	FRotator DesiredAttitude;
	float Thrust;
	for (int32 i = 0; i < 50; ++i)
	{
		Controller->ComputeControl(CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, Thrust, 0.01f);
	}

	// 重置
	Controller->Reset();

	// 重置后计算
	Controller->ComputeControl(CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, Thrust, 0.01f);

	// 创建新控制器比较
	UPositionController* FreshController = NewObject<UPositionController>();
	FreshController->Ki_Position = 0.1f;
	FreshController->Ki_Velocity = 0.1f;

	FRotator FreshAttitude;
	float FreshThrust;
	FreshController->ComputeControl(CurrentState, TargetPosition, FVector::ZeroVector, FreshAttitude, FreshThrust, 0.01f);

	// 重置后应该与新控制器相近
	UAV_TEST_FLOAT_EQUAL(Thrust, FreshThrust, 0.05f);

	return true;
}

// ==================== 推力限制测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPositionControllerThrustLimitTest,
	"UAVSimulator.Control.PositionController.ThrustLimit",
	UAV_TEST_FLAGS)

bool FPositionControllerThrustLimitTest::RunTest(const FString& Parameters)
{
	UPositionController* Controller = NewObject<UPositionController>();

	// 设置推力限制
	Controller->MinThrust = 0.1f;
	Controller->MaxThrust = 0.9f;

	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector(0.0f, 0.0f, 100.0f),
		FVector::ZeroVector
	);

	// 测试向上加速（需要大推力）
	FVector TargetPosition(0.0f, 0.0f, 1000.0f);

	FRotator DesiredAttitude;
	float Thrust;
	Controller->ComputeControl(CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, Thrust, 0.01f);

	// 推力应该在限制范围内
	TestTrue(TEXT("Thrust should be >= MinThrust"), Thrust >= Controller->MinThrust - 0.01f);
	TestTrue(TEXT("Thrust should be <= MaxThrust"), Thrust <= Controller->MaxThrust + 0.01f);

	return true;
}

// ==================== 悬停测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPositionControllerHoverTest,
	"UAVSimulator.Control.PositionController.Hover",
	UAV_TEST_FLAGS)

bool FPositionControllerHoverTest::RunTest(const FString& Parameters)
{
	UPositionController* Controller = NewObject<UPositionController>();

	// 当前位置等于目标位置（悬停）
	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector(500.0f, 500.0f, 100.0f),
		FVector::ZeroVector
	);

	FVector TargetPosition(500.0f, 500.0f, 100.0f);

	FRotator DesiredAttitude;
	float Thrust;
	Controller->ComputeControl(CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, Thrust, 0.01f);

	// 悬停时姿态应该接近水平
	UAV_TEST_FLOAT_EQUAL(DesiredAttitude.Pitch, 0.0f, 5.0f);
	UAV_TEST_FLOAT_EQUAL(DesiredAttitude.Roll, 0.0f, 5.0f);

	// 推力应该接近悬停推力（重力补偿）
	TestTrue(TEXT("Hover thrust should be positive"), Thrust > 0.0f);

	return true;
}

// ==================== 多方向控制测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPositionControllerMultiDirectionTest,
	"UAVSimulator.Control.PositionController.MultiDirection",
	UAV_TEST_FLAGS)

bool FPositionControllerMultiDirectionTest::RunTest(const FString& Parameters)
{
	UPositionController* Controller = NewObject<UPositionController>();

	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector(0.0f, 0.0f, 100.0f),
		FVector::ZeroVector
	);

	// 测试不同方向的目标
	TArray<FVector> Targets = {
		FVector(500.0f, 0.0f, 100.0f),   // +X
		FVector(-500.0f, 0.0f, 100.0f),  // -X
		FVector(0.0f, 500.0f, 100.0f),   // +Y
		FVector(0.0f, -500.0f, 100.0f),  // -Y
		FVector(0.0f, 0.0f, 500.0f),     // +Z
		FVector(0.0f, 0.0f, -50.0f),     // -Z
	};

	for (const FVector& Target : Targets)
	{
		FRotator DesiredAttitude;
		float Thrust;
		Controller->ComputeControl(CurrentState, Target, FVector::ZeroVector, DesiredAttitude, Thrust, 0.01f);

		// 所有方向都应该产生有效输出
		TestTrue(FString::Printf(TEXT("Thrust for target (%f,%f,%f) should be valid"),
			Target.X, Target.Y, Target.Z),
			Thrust >= 0.0f && Thrust <= 1.0f);
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
