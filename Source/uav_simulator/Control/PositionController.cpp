// Copyright Epic Games, Inc. All Rights Reserved.

#include "PositionController.h"
#include "../Debug/UAVLogConfig.h"
#include "../Utility/Filter.h"

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

	float VerticalThrustZ = GravityAcceleration + DesiredAcceleration.Z;
	bool bNeedsDescent = VerticalThrustZ <= 0.0f;
	// Z分量钳制到0：多旋翼无法产生向下推力，但保留X/Y分量以维持水平制动能力
	FVector ThrustVector(DesiredAcceleration.X, DesiredAcceleration.Y,
		bNeedsDescent ? 0.0f : VerticalThrustZ);

	float MaxTotalThrust = NumMotors * SingleMotorMaxThrust;
	float RequiredThrustN = UAVMass * ThrustVector.Size() / 100.0f;
	float RawThrust = RequiredThrustN / MaxTotalThrust;
	// 需要下降时，推力上限为悬停推力：确保倾斜后垂直分量 < 重力，UAV 自然下降
	if (bNeedsDescent)
	{
		float HoverThrust = UAVMass * GravityAcceleration / (MaxTotalThrust * 100.0f);
		RawThrust = FMath::Min(RawThrust, HoverThrust);
	}
	OutThrust = FMath::Clamp(RawThrust, MinThrust, MaxThrust);

	// 调试日志：输出推力计算详情（每秒1次）
	UE_LOG_THROTTLE(1.0, LogUAVPosition, Log, TEXT("Thrust: DesiredAccel: (%.1f,%.1f,%.1f) | RequiredThrustN: %.1f | RawThrust: %.3f | ClampedThrust: %.3f"),
		DesiredAcceleration.X, DesiredAcceleration.Y, DesiredAcceleration.Z,
		RequiredThrustN, RawThrust, OutThrust);

	// 计算期望姿态（Roll和Pitch）
	float DesiredPitch = 0.0f;
	float DesiredRoll = 0.0f;
	if (!ThrustVector.IsNearlyZero())
	{
		FVector ThrustDirection = ThrustVector.GetSafeNormal();
		DesiredPitch = FMath::Asin(-ThrustDirection.X) * (180.0f / PI);
		DesiredRoll = FMath::Asin(ThrustDirection.Y / FMath::Cos(DesiredPitch * PI / 180.0f)) * (180.0f / PI);
		DesiredPitch = FMath::Clamp(DesiredPitch, -MaxTiltAngle, MaxTiltAngle);
		DesiredRoll = FMath::Clamp(DesiredRoll, -MaxTiltAngle, MaxTiltAngle);
	}

	// Yaw保持当前值（或可以设置为目标Yaw）
	float DesiredYaw = CurrentState.Rotation.Yaw;

	OutDesiredAttitude = FRotator(DesiredPitch, DesiredYaw, DesiredRoll);

	// 调试日志：输出位置控制信息（每10帧1次）
	UE_LOG_EVERY_N(10, LogUAVPosition, Verbose, TEXT("Target: (%.1f,%.1f,%.1f) | Current: (%.1f,%.1f,%.1f) | Error: (%.1f,%.1f,%.1f)"),
		InTargetPosition.X, InTargetPosition.Y, InTargetPosition.Z,
		CurrentState.Position.X, CurrentState.Position.Y, CurrentState.Position.Z,
		PositionError.X, PositionError.Y, PositionError.Z);

	UE_LOG_EVERY_N(10, LogUAVPosition, Verbose, TEXT("Velocity: Desired: (%.1f,%.1f,%.1f) | Current: (%.1f,%.1f,%.1f) | Error: (%.1f,%.1f,%.1f)"),
		DesiredVelocity.X, DesiredVelocity.Y, DesiredVelocity.Z,
		CurrentState.Velocity.X, CurrentState.Velocity.Y, CurrentState.Velocity.Z,
		VelocityError.X, VelocityError.Y, VelocityError.Z);

	UE_LOG_EVERY_N(10, LogUAVPosition, Verbose, TEXT("Output: Thrust: %.3f | Attitude: R=%.2f P=%.2f Y=%.2f"),
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
	LastDesiredAttitude = FRotator::ZeroRotator;
}

void UPositionController::AccelerationToControl(const FVector& DesiredAcceleration, float CurrentYaw,
	FRotator& OutAttitude, float& OutThrust) const
{
	float VerticalThrustZ = GravityAcceleration + DesiredAcceleration.Z;
	bool bNeedsDescent = VerticalThrustZ <= 0.0f;
	FVector ThrustVector(DesiredAcceleration.X, DesiredAcceleration.Y,
		bNeedsDescent ? 0.0f : VerticalThrustZ);

	float MaxTotalThrust = NumMotors * SingleMotorMaxThrust;
	float RequiredThrustN = UAVMass * ThrustVector.Size() / 100.0f;
	float RawThrust = RequiredThrustN / MaxTotalThrust;
	if (bNeedsDescent)
	{
		float HoverThrust = UAVMass * GravityAcceleration / (MaxTotalThrust * 100.0f);
		RawThrust = FMath::Min(RawThrust, HoverThrust);
	}
	OutThrust = FMath::Clamp(RawThrust, MinThrust, MaxThrust);

	float DesiredPitch = 0.0f;
	float DesiredRoll = 0.0f;
	if (!ThrustVector.IsNearlyZero())
	{
		FVector ThrustDirection = ThrustVector.GetSafeNormal();
		DesiredPitch = FMath::Asin(-ThrustDirection.X) * (180.0f / PI);
		DesiredRoll = FMath::Asin(ThrustDirection.Y / FMath::Cos(DesiredPitch * PI / 180.0f)) * (180.0f / PI);
		DesiredPitch = FMath::Clamp(DesiredPitch, -MaxTiltAngle, MaxTiltAngle);
		DesiredRoll = FMath::Clamp(DesiredRoll, -MaxTiltAngle, MaxTiltAngle);
	}

	// 限制姿态变化率，防止目标姿态突变导致控制饱和
	const float MaxAttitudeRate = 50.0f; // deg/s
	const float DeltaTime = 0.02f; // 假设 50Hz 控制频率
	const float MaxDelta = MaxAttitudeRate * DeltaTime; // 每帧最大变化量

	// 计算姿态变化量
	float RollDelta = DesiredRoll - LastDesiredAttitude.Roll;
	float PitchDelta = DesiredPitch - LastDesiredAttitude.Pitch;

	// 归一化角度差到 [-180, 180]
	while (RollDelta > 180.0f) RollDelta -= 360.0f;
	while (RollDelta < -180.0f) RollDelta += 360.0f;
	while (PitchDelta > 180.0f) PitchDelta -= 360.0f;
	while (PitchDelta < -180.0f) PitchDelta += 360.0f;

	// 限制变化率
	RollDelta = FMath::Clamp(RollDelta, -MaxDelta, MaxDelta);
	PitchDelta = FMath::Clamp(PitchDelta, -MaxDelta, MaxDelta);

	// 应用限制后的姿态
	OutAttitude.Roll = LastDesiredAttitude.Roll + RollDelta;
	OutAttitude.Pitch = LastDesiredAttitude.Pitch + PitchDelta;
	OutAttitude.Yaw = CurrentYaw;

	// 更新上一次姿态
	const_cast<UPositionController*>(this)->LastDesiredAttitude = OutAttitude;
}

void UPositionController::ComputeControlWithAcceleration(const FUAVState& CurrentState, const FVector& InTargetPosition,
	const FVector& InTargetVelocity, const FVector& InTargetAcceleration,
	FRotator& OutDesiredAttitude, float& OutThrust, float DeltaTime)
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

	// 速度PID控制 -> 期望加速度（反馈部分）
	FVector FeedbackAcceleration = Kp_Velocity * VelocityError
								 + Ki_Velocity * VelocityErrorIntegral
								 + Kd_Velocity * (VelocityError - PreviousVelocityError) / DeltaTime;

	// 总期望加速度 = 前馈加速度 + 反馈加速度
	FVector DesiredAcceleration = InTargetAcceleration + FeedbackAcceleration;

	float VerticalThrustZ2 = GravityAcceleration + DesiredAcceleration.Z;
	bool bNeedsDescent2 = VerticalThrustZ2 <= 0.0f;
	FVector ThrustVector(DesiredAcceleration.X, DesiredAcceleration.Y,
		bNeedsDescent2 ? 0.0f : VerticalThrustZ2);

	float MaxTotalThrust = NumMotors * SingleMotorMaxThrust;
	float RequiredThrustN = UAVMass * ThrustVector.Size() / 100.0f;
	float RawThrust = RequiredThrustN / MaxTotalThrust;
	if (bNeedsDescent2)
	{
		float HoverThrust = UAVMass * GravityAcceleration / (MaxTotalThrust * 100.0f);
		RawThrust = FMath::Min(RawThrust, HoverThrust);
	}
	OutThrust = FMath::Clamp(RawThrust, MinThrust, MaxThrust);

	// 计算期望姿态
	float DesiredPitch = 0.0f;
	float DesiredRoll = 0.0f;
	if (!ThrustVector.IsNearlyZero())
	{
		FVector ThrustDirection = ThrustVector.GetSafeNormal();
		DesiredPitch = FMath::Asin(-ThrustDirection.X) * (180.0f / PI);
		DesiredRoll = FMath::Asin(ThrustDirection.Y / FMath::Cos(DesiredPitch * PI / 180.0f)) * (180.0f / PI);
		DesiredPitch = FMath::Clamp(DesiredPitch, -MaxTiltAngle, MaxTiltAngle);
		DesiredRoll = FMath::Clamp(DesiredRoll, -MaxTiltAngle, MaxTiltAngle);
	}

	float DesiredYaw = CurrentState.Rotation.Yaw;

	OutDesiredAttitude = FRotator(DesiredPitch, DesiredYaw, DesiredRoll);

	// 更新积分项
	const float IntegralLimit = 1000.0f;
	PositionErrorIntegral += PositionError * DeltaTime;
	PositionErrorIntegral.X = FMath::Clamp(PositionErrorIntegral.X, -IntegralLimit, IntegralLimit);
	PositionErrorIntegral.Y = FMath::Clamp(PositionErrorIntegral.Y, -IntegralLimit, IntegralLimit);
	PositionErrorIntegral.Z = FMath::Clamp(PositionErrorIntegral.Z, -IntegralLimit, IntegralLimit);

	VelocityErrorIntegral += VelocityError * DeltaTime;
	VelocityErrorIntegral.X = FMath::Clamp(VelocityErrorIntegral.X, -IntegralLimit, IntegralLimit);
	VelocityErrorIntegral.Y = FMath::Clamp(VelocityErrorIntegral.Y, -IntegralLimit, IntegralLimit);
	VelocityErrorIntegral.Z = FMath::Clamp(VelocityErrorIntegral.Z, -IntegralLimit, IntegralLimit);

	PreviousPositionError = PositionError;
	PreviousVelocityError = VelocityError;
}
