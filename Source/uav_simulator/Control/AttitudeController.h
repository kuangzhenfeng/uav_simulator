// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "AttitudeController.generated.h"

/**
 * PID控制器参数
 */
USTRUCT(BlueprintType)
struct FPIDParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
	float Kp = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
	float Ki = 0.005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
	float Kd = 0.05f;

	FPIDParams() {}
	FPIDParams(float InKp, float InKi, float InKd) : Kp(InKp), Ki(InKi), Kd(InKd) {}
};

/**
 * 姿态控制器配置
 */
USTRUCT(BlueprintType)
struct FAttitudeControlConfig
{
	GENERATED_BODY()

	// 前馈控制开关
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Feedforward")
	bool bEnableFeedforward = true;

	// 前馈增益 (0-1)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Feedforward")
	float FeedforwardGain = 0.7f;

	// 转动惯量 (kg·m²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Feedforward")
	FVector MomentOfInertia = FVector(0.01f, 0.01f, 0.02f);

	// 自适应控制开关
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Adaptive")
	bool bEnableAdaptive = false;  // 默认关闭，需要时手动启用

	// 自适应学习率
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Adaptive")
	float AdaptiveLearningRate = 0.005f;  // 降低学习率，更保守

	// 自适应遗忘因子 (0-1)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Adaptive")
	float AdaptiveDecayRate = 0.99f;  // 提高遗忘因子，更慢遗忘

	// 自适应估计上限 (deg/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Adaptive")
	float AdaptiveEstimateLimit = 30.0f;  // 降低上限，防止过度补偿
};

/**
 * 自适应估计器状态
 */
struct FAdaptiveEstimatorState
{
	// 扰动估计值 (deg/s²)
	FVector DisturbanceEstimate = FVector::ZeroVector;

	// 重置状态
	void Reset()
	{
		DisturbanceEstimate = FVector::ZeroVector;
	}
};

/**
 * 姿态控制器组件
 * 使用PID控制器实现Roll、Pitch、Yaw控制
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UAttitudeController : public UActorComponent
{
	GENERATED_BODY()

public:
	UAttitudeController();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 计算控制输出
	UFUNCTION(BlueprintCallable, Category = "Attitude Controller")
	FMotorOutput ComputeControl(const FUAVState& CurrentState, const FRotator& TargetAttitude, float DeltaTime);

	// 带前馈的控制计算
	UFUNCTION(BlueprintCallable, Category = "Attitude Controller")
	FMotorOutput ComputeControlWithFeedforward(
		const FUAVState& CurrentState,
		const FRotator& TargetAttitude,
		const FRotator& DesiredAngularAcceleration,
		float DeltaTime);

	// 重置控制器状态
	UFUNCTION(BlueprintCallable, Category = "Attitude Controller")
	void ResetController();

	// 设置控制配置
	UFUNCTION(BlueprintCallable, Category = "Attitude Controller")
	void SetControlConfig(const FAttitudeControlConfig& Config);

	// 获取控制配置
	UFUNCTION(BlueprintCallable, Category = "Attitude Controller")
	FAttitudeControlConfig GetControlConfig() const { return ControlConfig; }

	// Roll PID参数 (增大Kp以提高响应速度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Parameters")
	FPIDParams RollPID = FPIDParams(0.012f, 0.0f, 0.0f);

	// Pitch PID参数 (增大Kp以提高响应速度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Parameters")
	FPIDParams PitchPID = FPIDParams(0.012f, 0.0f, 0.0f);

	// Yaw PID参数 (增大Kp以主导控制方向)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Parameters")
	FPIDParams YawPID = FPIDParams(0.005f, 0.0f, 0.0f);

	// 悬停推力 (归一化值 0-1)
	// 调整后的悬停推力值，通过实际测试微调
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Parameters")
	float HoverThrust = 0.245f;

	// 控制输出限制（增大以允许更强的控制响应）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Parameters")
	float MaxControlOutput = 0.20f;

	// 最大倾斜角度 (度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Parameters")
	float MaxTiltAngle = 30.0f;

protected:

	// 控制配置
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Config")
	FAttitudeControlConfig ControlConfig;

	// 自适应估计器状态
	FAdaptiveEstimatorState AdaptiveState;

private:
	// PID积分项
	float RollIntegral = 0.0f;
	float PitchIntegral = 0.0f;
	float YawIntegral = 0.0f;

	// 上一次误差（用于微分项）
	float LastRollError = 0.0f;
	float LastPitchError = 0.0f;
	float LastYawError = 0.0f;

	// 日志降频计时器
	float LogAccumTime = 0.0f;

	// 计算单轴PID控制
	float ComputePID(float Error, float& Integral, float& LastError, const FPIDParams& Params, float DeltaTime);

	// 计算前馈力矩
	FRotator ComputeFeedforwardTorque(const FRotator& DesiredAngularAcceleration) const;

	// 更新自适应估计
	void UpdateAdaptiveEstimate(const FRotator& AttitudeError, float DeltaTime);

	// 限制角度到[-180, 180]
	float NormalizeAngle(float Angle) const;
};
