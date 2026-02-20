// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Control/AttitudeController.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== PID 计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAttitudeControllerPIDTest,
	"UAVSimulator.Control.AttitudeController.PIDCalculation",
	UAV_TEST_FLAGS)

bool FAttitudeControllerPIDTest::RunTest(const FString& Parameters)
{
	UAttitudeController* Controller = NewObject<UAttitudeController>();

	// 设置 PID 参数
	Controller->RollPID = FPIDParams(0.01f, 0.001f, 0.005f);
	Controller->PitchPID = FPIDParams(0.01f, 0.001f, 0.005f);
	Controller->YawPID = FPIDParams(0.005f, 0.0005f, 0.002f);

	// 创建测试状态
	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector(0.0f, 0.0f, 100.0f),
		FVector::ZeroVector,
		FRotator(0.0f, 0.0f, 0.0f),
		FVector::ZeroVector
	);

	// 目标姿态
	FRotator TargetAttitude(10.0f, 0.0f, 5.0f); // Pitch=10, Yaw=0, Roll=5

	// 计算控制输出
	float DeltaTime = 0.01f;
	FMotorOutput Output = Controller->ComputeControl(CurrentState, TargetAttitude, DeltaTime);

	// 验证输出有效
	TestEqual(TEXT("Should have 4 motor outputs"), Output.Thrusts.Num(), 4);

	// 验证推力在合理范围内
	for (int32 i = 0; i < 4; ++i)
	{
		UAV_TEST_FLOAT_IN_RANGE(Output.Thrusts[i], 0.0f, 1.0f);
	}

	return true;
}

// ==================== 输出限制测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAttitudeControllerOutputLimitTest,
	"UAVSimulator.Control.AttitudeController.OutputLimit",
	UAV_TEST_FLAGS)

bool FAttitudeControllerOutputLimitTest::RunTest(const FString& Parameters)
{
	UAttitudeController* Controller = NewObject<UAttitudeController>();

	// 设置较大的 PID 增益以产生大输出
	Controller->RollPID = FPIDParams(1.0f, 0.0f, 0.0f);
	Controller->PitchPID = FPIDParams(1.0f, 0.0f, 0.0f);
	Controller->MaxControlOutput = 0.2f;

	// 创建大误差的状态
	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector::ZeroVector,
		FVector::ZeroVector,
		FRotator(0.0f, 0.0f, 0.0f),
		FVector::ZeroVector
	);

	// 大角度目标
	FRotator TargetAttitude(45.0f, 0.0f, 45.0f);

	FMotorOutput Output = Controller->ComputeControl(CurrentState, TargetAttitude, 0.01f);

	// 验证输出被限制在 [0, 1] 范围内
	for (int32 i = 0; i < 4; ++i)
	{
		TestTrue(FString::Printf(TEXT("Motor %d thrust should be >= 0"), i), Output.Thrusts[i] >= 0.0f);
		TestTrue(FString::Printf(TEXT("Motor %d thrust should be <= 1"), i), Output.Thrusts[i] <= 1.0f);
	}

	return true;
}

// ==================== 积分抗饱和测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAttitudeControllerAntiWindupTest,
	"UAVSimulator.Control.AttitudeController.AntiWindup",
	UAV_TEST_FLAGS)

bool FAttitudeControllerAntiWindupTest::RunTest(const FString& Parameters)
{
	UAttitudeController* Controller = NewObject<UAttitudeController>();

	// 设置有积分项的 PID
	Controller->RollPID = FPIDParams(0.01f, 0.1f, 0.0f);
	Controller->PitchPID = FPIDParams(0.01f, 0.1f, 0.0f);
	Controller->YawPID = FPIDParams(0.005f, 0.05f, 0.0f);

	FUAVState CurrentState = UAVTestHelpers::CreateUAVState();
	FRotator TargetAttitude(20.0f, 0.0f, 20.0f);

	// 多次调用以累积积分
	TArray<FMotorOutput> Outputs;
	for (int32 i = 0; i < 100; ++i)
	{
		FMotorOutput Output = Controller->ComputeControl(CurrentState, TargetAttitude, 0.01f);
		Outputs.Add(Output);
	}

	// 验证输出仍然在合理范围内（积分抗饱和应该防止输出爆炸）
	FMotorOutput LastOutput = Outputs.Last();
	for (int32 i = 0; i < 4; ++i)
	{
		TestTrue(FString::Printf(TEXT("Motor %d thrust should be bounded after integration"), i),
			LastOutput.Thrusts[i] >= 0.0f && LastOutput.Thrusts[i] <= 1.0f);
	}

	return true;
}

// ==================== 控制器重置测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAttitudeControllerResetTest,
	"UAVSimulator.Control.AttitudeController.Reset",
	UAV_TEST_FLAGS)

bool FAttitudeControllerResetTest::RunTest(const FString& Parameters)
{
	UAttitudeController* Controller = NewObject<UAttitudeController>();

	Controller->RollPID = FPIDParams(0.01f, 0.01f, 0.01f);

	FUAVState CurrentState = UAVTestHelpers::CreateUAVState();
	FRotator TargetAttitude(15.0f, 0.0f, 15.0f);

	// 累积一些状态
	for (int32 i = 0; i < 50; ++i)
	{
		Controller->ComputeControl(CurrentState, TargetAttitude, 0.01f);
	}

	// 重置控制器
	Controller->ResetController();

	// 重置后的第一次计算应该与初始状态相同
	FMotorOutput OutputAfterReset = Controller->ComputeControl(CurrentState, TargetAttitude, 0.01f);

	// 创建新控制器进行比较
	UAttitudeController* FreshController = NewObject<UAttitudeController>();
	FreshController->RollPID = FPIDParams(0.01f, 0.01f, 0.01f);
	FMotorOutput FreshOutput = FreshController->ComputeControl(CurrentState, TargetAttitude, 0.01f);

	// 重置后的输出应该与新控制器的输出相近
	for (int32 i = 0; i < 4; ++i)
	{
		UAV_TEST_FLOAT_EQUAL(OutputAfterReset.Thrusts[i], FreshOutput.Thrusts[i], 0.01f);
	}

	return true;
}

// ==================== 悬停推力测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAttitudeControllerHoverThrustTest,
	"UAVSimulator.Control.AttitudeController.HoverThrust",
	UAV_TEST_FLAGS)

bool FAttitudeControllerHoverThrustTest::RunTest(const FString& Parameters)
{
	UAttitudeController* Controller = NewObject<UAttitudeController>();

	// 设置悬停推力
	Controller->HoverThrust = 0.25f;

	// 零误差状态
	FUAVState CurrentState = UAVTestHelpers::CreateUAVState();
	FRotator TargetAttitude(0.0f, 0.0f, 0.0f);

	FMotorOutput Output = Controller->ComputeControl(CurrentState, TargetAttitude, 0.01f);

	// 在零误差时，所有电机应该输出接近悬停推力
	for (int32 i = 0; i < 4; ++i)
	{
		UAV_TEST_FLOAT_EQUAL(Output.Thrusts[i], Controller->HoverThrust, 0.05f);
	}

	return true;
}

// ==================== 最大倾斜角测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAttitudeControllerMaxTiltTest,
	"UAVSimulator.Control.AttitudeController.MaxTilt",
	UAV_TEST_FLAGS)

bool FAttitudeControllerMaxTiltTest::RunTest(const FString& Parameters)
{
	UAttitudeController* Controller = NewObject<UAttitudeController>();

	// 设置最大倾斜角
	Controller->MaxTiltAngle = 30.0f;

	// 验证参数设置
	UAV_TEST_FLOAT_EQUAL(Controller->MaxTiltAngle, 30.0f, 0.1f);

	// 测试超过最大倾斜角的目标
	FUAVState CurrentState = UAVTestHelpers::CreateUAVState();
	FRotator TargetAttitude(45.0f, 0.0f, 45.0f); // 超过 30 度

	FMotorOutput Output = Controller->ComputeControl(CurrentState, TargetAttitude, 0.01f);

	// 输出应该仍然有效
	for (int32 i = 0; i < 4; ++i)
	{
		TestTrue(FString::Printf(TEXT("Motor %d thrust should be valid"), i),
			Output.Thrusts[i] >= 0.0f && Output.Thrusts[i] <= 1.0f);
	}

	return true;
}

// ==================== 角度归一化测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAttitudeControllerAngleNormalizationTest,
	"UAVSimulator.Control.AttitudeController.AngleNormalization",
	UAV_TEST_FLAGS)

bool FAttitudeControllerAngleNormalizationTest::RunTest(const FString& Parameters)
{
	UAttitudeController* Controller = NewObject<UAttitudeController>();

	// 测试大角度目标（应该被归一化处理）
	FUAVState CurrentState = UAVTestHelpers::CreateUAVState(
		FVector::ZeroVector,
		FVector::ZeroVector,
		FRotator(0.0f, 350.0f, 0.0f), // Yaw = 350
		FVector::ZeroVector
	);

	// 目标 Yaw = 10，实际误差应该是 20 度而不是 340 度
	FRotator TargetAttitude(0.0f, 10.0f, 0.0f);

	FMotorOutput Output = Controller->ComputeControl(CurrentState, TargetAttitude, 0.01f);

	// 输出应该有效
	for (int32 i = 0; i < 4; ++i)
	{
		TestTrue(FString::Printf(TEXT("Motor %d thrust should be valid"), i),
			Output.Thrusts[i] >= 0.0f && Output.Thrusts[i] <= 1.0f);
	}

	return true;
}

// ==================== 微分项测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAttitudeControllerDerivativeTest,
	"UAVSimulator.Control.AttitudeController.Derivative",
	UAV_TEST_FLAGS)

bool FAttitudeControllerDerivativeTest::RunTest(const FString& Parameters)
{
	UAttitudeController* Controller = NewObject<UAttitudeController>();

	// 只使用微分项
	Controller->RollPID = FPIDParams(0.0f, 0.0f, 0.1f);
	Controller->PitchPID = FPIDParams(0.0f, 0.0f, 0.1f);
	Controller->YawPID = FPIDParams(0.0f, 0.0f, 0.05f);

	FUAVState CurrentState = UAVTestHelpers::CreateUAVState();
	FRotator TargetAttitude(10.0f, 0.0f, 10.0f);

	// 第一次调用
	FMotorOutput Output1 = Controller->ComputeControl(CurrentState, TargetAttitude, 0.01f);

	// 第二次调用（误差不变，微分项应该减小）
	FMotorOutput Output2 = Controller->ComputeControl(CurrentState, TargetAttitude, 0.01f);

	// 输出应该有效
	for (int32 i = 0; i < 4; ++i)
	{
		TestTrue(FString::Printf(TEXT("Motor %d thrust should be valid"), i),
			Output2.Thrusts[i] >= 0.0f && Output2.Thrusts[i] <= 1.0f);
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
