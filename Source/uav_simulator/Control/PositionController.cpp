// Copyright Epic Games, Inc. All Rights Reserved.

#include "PositionController.h"

UPositionController::UPositionController()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UPositionController::BeginPlay()
{
	Super::BeginPlay();
	Reset();
}

void UPositionController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UPositionController::ComputeControl(const FUAVState& CurrentState, const FVector& InTargetPosition,
										 const FVector& InTargetVelocity, FRotator& OutDesiredAttitude, float& OutThrust, float DeltaTime)
{
	// 防止除零错误
	if (DeltaTime <= 0.0f) DeltaTime = 0.016f;

	// 位置误差（世界坐标系）
	FVector PositionError = InTargetPosition - CurrentState.Position;

	// 位置PID控制 -> 期望速度
	FVector DesiredVelocity = Kp_Position * PositionError
							+ Ki_Position * PositionErrorIntegral
							+ Kd_Position * (PositionError - PreviousPositionError) / DeltaTime;

	// 添加前馈速度
	DesiredVelocity += InTargetVelocity;

	// 限制期望速度
	float VelocityMag = DesiredVelocity.Size();
	if (VelocityMag > MaxVelocity)
	{
		DesiredVelocity = DesiredVelocity * (MaxVelocity / VelocityMag);
	}

	// 速度误差（世界坐标系）
	FVector VelocityError = DesiredVelocity - CurrentState.Velocity;

	// 速度PID控制 -> 期望加速度
	FVector DesiredAcceleration = Kp_Velocity * VelocityError
								+ Ki_Velocity * VelocityErrorIntegral
								+ Kd_Velocity * (VelocityError - PreviousVelocityError) / DeltaTime;

	// 计算期望推力（使用实际物理参数）
	// 推力需要克服重力并产生期望加速度
	// 当期望加速度向上（正值）时，推力 = m*(g + a)
	// 当期望加速度向下（负值）时，推力 = m*(g + a) = m*(g - |a|)
	// 所以公式是：Thrust = m * (g + a_desired)
	// 但这里 g 是正值980，实际重力是-g，所以：
	// Thrust_needed = m * g (悬停) + m * a_desired (加速度)
	// 当 a_desired 向下（负值）时，推力减小；向上（正值）时，推力增大
	float RequiredThrustN = UAVMass * (GravityAcceleration + DesiredAcceleration.Z) / 100.0f;
	// 归一化到[0,1]范围
	float MaxTotalThrust = NumMotors * SingleMotorMaxThrust;
	float RawThrust = RequiredThrustN / MaxTotalThrust;
	OutThrust = FMath::Clamp(RawThrust, MinThrust, MaxThrust);

	// 调试日志：输出推力计算详情
	UE_LOG(LogTemp, Warning, TEXT("【PositionController, Thrust】 DesiredAccel: (%.1f,%.1f,%.1f) | RequiredThrustN: %.1f | RawThrust: %.3f | ClampedThrust: %.3f"),
		DesiredAcceleration.X, DesiredAcceleration.Y, DesiredAcceleration.Z,
		RequiredThrustN, RawThrust, OutThrust);

	// 计算期望姿态（Roll和Pitch）
	// 推力方向需要同时考虑水平和垂直方向的期望加速度
	// 推力向量 = (a_x, a_y, g + a_z)，其中g是重力加速度，a_z是期望垂直加速度
	FVector ThrustDirection = FVector(DesiredAcceleration.X, DesiredAcceleration.Y, GravityAcceleration + DesiredAcceleration.Z).GetSafeNormal();

	// 从推力方向计算期望姿态
	// Pitch: 前后倾斜（绕Y轴）
	float DesiredPitch = FMath::Asin(-ThrustDirection.X) * (180.0f / PI);

	// Roll: 左右倾斜（绕X轴）
	float DesiredRoll = FMath::Asin(ThrustDirection.Y / FMath::Cos(DesiredPitch * PI / 180.0f)) * (180.0f / PI);

	// 限制倾斜角度
	DesiredPitch = FMath::Clamp(DesiredPitch, -MaxTiltAngle, MaxTiltAngle);
	DesiredRoll = FMath::Clamp(DesiredRoll, -MaxTiltAngle, MaxTiltAngle);

	// Yaw保持当前值（或可以设置为目标Yaw）
	float DesiredYaw = CurrentState.Rotation.Yaw;

	OutDesiredAttitude = FRotator(DesiredPitch, DesiredYaw, DesiredRoll);

	// 调试日志：输出位置控制信息
	UE_LOG(LogTemp, Warning, TEXT("【PositionController, Pos】 Target: (%.1f,%.1f,%.1f) | Current: (%.1f,%.1f,%.1f) | Error: (%.1f,%.1f,%.1f)"),
		InTargetPosition.X, InTargetPosition.Y, InTargetPosition.Z,
		CurrentState.Position.X, CurrentState.Position.Y, CurrentState.Position.Z,
		PositionError.X, PositionError.Y, PositionError.Z);

	UE_LOG(LogTemp, Warning, TEXT("【PositionController, Vel】 Desired: (%.1f,%.1f,%.1f) | Current: (%.1f,%.1f,%.1f) | Error: (%.1f,%.1f,%.1f)"),
		DesiredVelocity.X, DesiredVelocity.Y, DesiredVelocity.Z,
		CurrentState.Velocity.X, CurrentState.Velocity.Y, CurrentState.Velocity.Z,
		VelocityError.X, VelocityError.Y, VelocityError.Z);

	UE_LOG(LogTemp, Warning, TEXT("【PositionController, Output】 Thrust: %.3f | Attitude: R=%.2f P=%.2f Y=%.2f"),
		OutThrust, DesiredRoll, DesiredPitch, DesiredYaw);

	// 更新积分项（抗饱和）
	const float IntegralLimit = 1000.0f;
	PositionErrorIntegral += PositionError * DeltaTime;
	PositionErrorIntegral.X = FMath::Clamp(PositionErrorIntegral.X, -IntegralLimit, IntegralLimit);
	PositionErrorIntegral.Y = FMath::Clamp(PositionErrorIntegral.Y, -IntegralLimit, IntegralLimit);
	PositionErrorIntegral.Z = FMath::Clamp(PositionErrorIntegral.Z, -IntegralLimit, IntegralLimit);

	VelocityErrorIntegral += VelocityError * DeltaTime;
	VelocityErrorIntegral.X = FMath::Clamp(VelocityErrorIntegral.X, -IntegralLimit, IntegralLimit);
	VelocityErrorIntegral.Y = FMath::Clamp(VelocityErrorIntegral.Y, -IntegralLimit, IntegralLimit);
	VelocityErrorIntegral.Z = FMath::Clamp(VelocityErrorIntegral.Z, -IntegralLimit, IntegralLimit);

	// 保存当前误差用于下次微分计算
	PreviousPositionError = PositionError;
	PreviousVelocityError = VelocityError;
}

void UPositionController::Reset()
{
	PositionErrorIntegral = FVector::ZeroVector;
	PreviousPositionError = FVector::ZeroVector;
	VelocityErrorIntegral = FVector::ZeroVector;
	PreviousVelocityError = FVector::ZeroVector;
}
