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
	// 向后兼容：调用带前馈的方法，前馈参数设为零
	return ComputeControlWithFeedforward(CurrentState, TargetAttitude, FRotator::ZeroRotator, DeltaTime);
}

FMotorOutput UAttitudeController::ComputeControlWithFeedforward(
	const FUAVState& CurrentState,
	const FRotator& TargetAttitude,
	const FRotator& DesiredAngularAcceleration,
	float DeltaTime)
{
	FMotorOutput Output;

	// 计算姿态误差
	float RollError = NormalizeAngle(TargetAttitude.Roll - CurrentState.Rotation.Roll);
	float PitchError = NormalizeAngle(TargetAttitude.Pitch - CurrentState.Rotation.Pitch);
	float YawError = NormalizeAngle(TargetAttitude.Yaw - CurrentState.Rotation.Yaw);

	LogAccumTime += DeltaTime;
	const bool bShouldLog = LogAccumTime >= 1.0f;

	// 启动阶段每帧都记录（前 2 秒）
	static float TotalTime = 0.0f;
	TotalTime += DeltaTime;
	const bool bStartupLog = TotalTime <= 2.0f;

	if (bShouldLog)
	{
		LogAccumTime = 0.0f;
		UE_LOG(LogUAVAttitude, Log, TEXT("Target: R=%.2f P=%.2f Y=%.2f | Current: R=%.2f P=%.2f Y=%.2f | Error: R=%.2f P=%.2f Y=%.2f"),
			TargetAttitude.Roll, TargetAttitude.Pitch, TargetAttitude.Yaw,
			CurrentState.Rotation.Roll, CurrentState.Rotation.Pitch, CurrentState.Rotation.Yaw,
			RollError, PitchError, YawError);
	}

	// 限制目标角度
	RollError = FMath::Clamp(RollError, -MaxTiltAngle, MaxTiltAngle);
	PitchError = FMath::Clamp(PitchError, -MaxTiltAngle, MaxTiltAngle);

	// 获取角速度（rad/s转换为deg/s）
	FVector AngularVelDeg = CurrentState.AngularVelocity * (180.0f / PI);

	if (bStartupLog)
	{
		UE_LOG(LogUAVAttitude, Warning, TEXT("[STARTUP] T=%.3f | Cur:(R=%.1f P=%.1f) | Err:(R=%.1f P=%.1f) | AngVel:(%.1f,%.1f)"),
			TotalTime, CurrentState.Rotation.Roll, CurrentState.Rotation.Pitch,
			RollError, PitchError, AngularVelDeg.X, AngularVelDeg.Y);
	}

	// 计算比例控制输出
	float RollP = RollPID.Kp * RollError;
	float PitchP = PitchPID.Kp * PitchError;
	float YawP = YawPID.Kp * YawError;

	// 计算角速度阻尼（使用角速度反馈代替误差微分）
	// 降低阻尼系数，防止高角速度时过度纠正
	const float AngularDamping = 0.0005f;
	float RollD = -AngularDamping * AngularVelDeg.X;
	float PitchD = -AngularDamping * AngularVelDeg.Y;
	float YawD = -AngularDamping * 0.5f * AngularVelDeg.Z;

	// 限制阻尼项，防止高角速度时产生过大控制输出
	RollD = FMath::Clamp(RollD, -0.05f, 0.05f);
	PitchD = FMath::Clamp(PitchD, -0.05f, 0.05f);
	YawD = FMath::Clamp(YawD, -0.025f, 0.025f);

	// 计算前馈力矩
	FRotator FeedforwardTorque = FRotator::ZeroRotator;
	if (ControlConfig.bEnableFeedforward)
	{
		FeedforwardTorque = ComputeFeedforwardTorque(DesiredAngularAcceleration);
	}

	// 更新自适应估计
	if (ControlConfig.bEnableAdaptive)
	{
		FRotator AttitudeError(RollError, PitchError, YawError);
		UpdateAdaptiveEstimate(AttitudeError, DeltaTime);
	}

	// 合并比例、阻尼、前馈和自适应项
	float RollControl = RollP + RollD + ControlConfig.FeedforwardGain * FeedforwardTorque.Roll;
	float PitchControl = PitchP + PitchD + ControlConfig.FeedforwardGain * FeedforwardTorque.Pitch;
	float YawControl = YawP + YawD + ControlConfig.FeedforwardGain * FeedforwardTorque.Yaw;

	// 添加自适应补偿
	if (ControlConfig.bEnableAdaptive)
	{
		// 自适应补偿转换为控制输出
		// 缩放因子需要根据实际系统调整
		const float AdaptiveScale = 0.00005f; // 降低缩放，更温和的补偿
		RollControl += AdaptiveState.DisturbanceEstimate.X * AdaptiveScale;
		PitchControl += AdaptiveState.DisturbanceEstimate.Y * AdaptiveScale;
		YawControl += AdaptiveState.DisturbanceEstimate.Z * AdaptiveScale;
	}

	// 限制控制输出，防止单个电机推力变化过大
	// 启动阶段使用更保守的限制，防止姿态失控
	const float MaxRollPitch = FMath::Min(MaxControlOutput, 0.08f);
	RollControl = FMath::Clamp(RollControl, -MaxRollPitch, MaxRollPitch);
	PitchControl = FMath::Clamp(PitchControl, -MaxRollPitch, MaxRollPitch);
	YawControl = FMath::Clamp(YawControl, -MaxControlOutput * 0.5f, MaxControlOutput * 0.5f);

	// 调试日志：输出控制量详情
	if (bShouldLog)
	{
		UE_LOG(LogUAVAttitude, Log, TEXT("PID: P:(%.4f,%.4f) D:(%.4f,%.4f) Out:(%.4f,%.4f) AngVel:(%.1f,%.1f)"),
			RollP, PitchP, RollD, PitchD, RollControl, PitchControl, AngularVelDeg.X, AngularVelDeg.Y);
	}

	if (bStartupLog)
	{
		UE_LOG(LogUAVAttitude, Warning, TEXT("[STARTUP] PID: P:(%.4f,%.4f) D:(%.4f,%.4f) Final:(%.4f,%.4f) | Kp=%.4f"),
			RollP, PitchP, RollD, PitchD, RollControl, PitchControl, RollPID.Kp);
	}

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

	if (bStartupLog)
	{
		UE_LOG(LogUAVAttitude, Warning, TEXT("[STARTUP] Motors: [%.3f, %.3f, %.3f, %.3f] | Hover=%.3f"),
			Output.Thrusts[0], Output.Thrusts[1], Output.Thrusts[2], Output.Thrusts[3], HoverThrust);
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
	AdaptiveState.Reset();
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

void UAttitudeController::SetControlConfig(const FAttitudeControlConfig& Config)
{
	ControlConfig = Config;
}

FRotator UAttitudeController::ComputeFeedforwardTorque(const FRotator& DesiredAngularAcceleration) const
{
	// 前馈力矩 = 转动惯量 × 期望角加速度
	// 输入: DesiredAngularAcceleration 单位为 deg/s²
	// 物理公式: τ = I × α，其中 α 单位为 rad/s²
	// 因此需要转换: α_rad = α_deg × (π/180)
	constexpr float DegToRad = PI / 180.0f;

	FRotator FeedforwardTorque;
	FeedforwardTorque.Roll = ControlConfig.MomentOfInertia.X * DesiredAngularAcceleration.Roll * DegToRad;
	FeedforwardTorque.Pitch = ControlConfig.MomentOfInertia.Y * DesiredAngularAcceleration.Pitch * DegToRad;
	FeedforwardTorque.Yaw = ControlConfig.MomentOfInertia.Z * DesiredAngularAcceleration.Yaw * DegToRad;
	return FeedforwardTorque;
}

void UAttitudeController::UpdateAdaptiveEstimate(const FRotator& AttitudeError, float DeltaTime)
{
	// MIT规则自适应估计
	// d̂ += γ * e * dt （学习）
	// d̂ *= λ （遗忘）
	// |d̂| ≤ d_max （限幅）

	const float Gamma = ControlConfig.AdaptiveLearningRate;
	const float Lambda = ControlConfig.AdaptiveDecayRate;
	const float DMax = ControlConfig.AdaptiveEstimateLimit;

	// 学习更新
	AdaptiveState.DisturbanceEstimate.X += Gamma * AttitudeError.Roll * DeltaTime;
	AdaptiveState.DisturbanceEstimate.Y += Gamma * AttitudeError.Pitch * DeltaTime;
	AdaptiveState.DisturbanceEstimate.Z += Gamma * AttitudeError.Yaw * DeltaTime;

	// 遗忘因子
	AdaptiveState.DisturbanceEstimate *= Lambda;

	// 限幅保护
	AdaptiveState.DisturbanceEstimate.X = FMath::Clamp(AdaptiveState.DisturbanceEstimate.X, -DMax, DMax);
	AdaptiveState.DisturbanceEstimate.Y = FMath::Clamp(AdaptiveState.DisturbanceEstimate.Y, -DMax, DMax);
	AdaptiveState.DisturbanceEstimate.Z = FMath::Clamp(AdaptiveState.DisturbanceEstimate.Z, -DMax, DMax);
}
