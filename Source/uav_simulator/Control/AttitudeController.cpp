// Copyright Epic Games, Inc. All Rights Reserved.

#include "AttitudeController.h"
#include "../Debug/UAVLogConfig.h"

UAttitudeController::UAttitudeController()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UAttitudeController::BeginPlay()
{
	Super::BeginPlay();
	ResetController();
}

void UAttitudeController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

FMotorOutput UAttitudeController::ComputeControl(const FUAVState& CurrentState, const FRotator& TargetAttitude, float DeltaTime)
{
	FMotorOutput Output;

	// 计算姿态误差
	float RollError = NormalizeAngle(TargetAttitude.Roll - CurrentState.Rotation.Roll);
	float PitchError = NormalizeAngle(TargetAttitude.Pitch - CurrentState.Rotation.Pitch);
	float YawError = NormalizeAngle(TargetAttitude.Yaw - CurrentState.Rotation.Yaw);

	// 调试日志：输出姿态信息
	UE_LOG(LogUAVAttitude, Log, TEXT("Target: R=%.2f P=%.2f Y=%.2f | Current: R=%.2f P=%.2f Y=%.2f | Error: R=%.2f P=%.2f Y=%.2f"),
		TargetAttitude.Roll, TargetAttitude.Pitch, TargetAttitude.Yaw,
		CurrentState.Rotation.Roll, CurrentState.Rotation.Pitch, CurrentState.Rotation.Yaw,
		RollError, PitchError, YawError);

	// 限制目标角度
	RollError = FMath::Clamp(RollError, -MaxTiltAngle, MaxTiltAngle);
	PitchError = FMath::Clamp(PitchError, -MaxTiltAngle, MaxTiltAngle);

	// 获取角速度（rad/s转换为deg/s）
	FVector AngularVelDeg = CurrentState.AngularVelocity * (180.0f / PI);

	// 计算比例控制输出
	float RollP = RollPID.Kp * RollError;
	float PitchP = PitchPID.Kp * PitchError;
	float YawP = YawPID.Kp * YawError;

	// 计算角速度阻尼（使用角速度反馈代替误差微分）
	// 减小阻尼系数，确保比例项主导控制方向
	const float AngularDamping = 0.0006f;
	float RollD = -AngularDamping * AngularVelDeg.X;
	float PitchD = -AngularDamping * AngularVelDeg.Y;
	float YawD = -AngularDamping * 0.5f * AngularVelDeg.Z;

	// 合并比例和阻尼项
	float RollControl = RollP + RollD;
	float PitchControl = PitchP + PitchD;
	float YawControl = YawP + YawD;

	// 限制控制输出，防止单个电机推力变化过大
	RollControl = FMath::Clamp(RollControl, -MaxControlOutput, MaxControlOutput);
	PitchControl = FMath::Clamp(PitchControl, -MaxControlOutput, MaxControlOutput);
	YawControl = FMath::Clamp(YawControl, -MaxControlOutput * 0.5f, MaxControlOutput * 0.5f);

	// 调试日志：输出控制量详情
	UE_LOG(LogUAVAttitude, Log, TEXT("PID: P:(%.4f,%.4f) D:(%.4f,%.4f) Out:(%.4f,%.4f) AngVel:(%.1f,%.1f)"),
		RollP, PitchP, RollD, PitchD, RollControl, PitchControl, AngularVelDeg.X, AngularVelDeg.Y);

	// 将控制输出映射到电机推力
	// 四旋翼X型配置:
	//     0(CW)
	//       |
	// 3(CCW)-+-1(CCW)
	//       |
	//     2(CW)

	// 基础推力 + Roll控制 + Pitch控制 + Yaw控制
	Output.Thrusts[0] = HoverThrust - RollControl - PitchControl + YawControl;
	Output.Thrusts[1] = HoverThrust + RollControl - PitchControl - YawControl;
	Output.Thrusts[2] = HoverThrust + RollControl + PitchControl + YawControl;
	Output.Thrusts[3] = HoverThrust - RollControl + PitchControl - YawControl;

	// 限制推力范围 [0, 1]
	for (float& Thrust : Output.Thrusts)
	{
		Thrust = FMath::Clamp(Thrust, 0.0f, 1.0f);
	}

	return Output;
}

void UAttitudeController::ResetController()
{
	RollIntegral = 0.0f;
	PitchIntegral = 0.0f;
	YawIntegral = 0.0f;
	LastRollError = 0.0f;
	LastPitchError = 0.0f;
	LastYawError = 0.0f;
}

float UAttitudeController::ComputePID(float Error, float& Integral, float& LastError, const FPIDParams& Params, float DeltaTime)
{
	// 比例项
	float P = Params.Kp * Error;

	// 积分项（带抗饱和）
	Integral += Error * DeltaTime;
	Integral = FMath::Clamp(Integral, -10.0f, 10.0f); // 积分限幅
	float I = Params.Ki * Integral;

	// 微分项
	float Derivative = (Error - LastError) / DeltaTime;
	float D = Params.Kd * Derivative;

	// 更新上一次误差
	LastError = Error;

	return P + I + D;
}

float UAttitudeController::NormalizeAngle(float Angle) const
{
	while (Angle > 180.0f)
		Angle -= 360.0f;
	while (Angle < -180.0f)
		Angle += 360.0f;
	return Angle;
}
