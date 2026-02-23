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

	// 重置控制器状态
	UFUNCTION(BlueprintCallable, Category = "Attitude Controller")
	void ResetController();

	// Roll PID参数 (增大Kp以主导控制方向)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Parameters")
	FPIDParams RollPID = FPIDParams(0.008f, 0.0f, 0.0f);

	// Pitch PID参数 (增大Kp以主导控制方向)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Parameters")
	FPIDParams PitchPID = FPIDParams(0.008f, 0.0f, 0.0f);

	// Yaw PID参数 (增大Kp以主导控制方向)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Parameters")
	FPIDParams YawPID = FPIDParams(0.005f, 0.0f, 0.0f);

	// 悬停推力 (归一化值 0-1)
	// 调整后的悬停推力值，通过实际测试微调
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Parameters")
	float HoverThrust = 0.245f;

	// 控制输出限制（增大以允许更强的控制响应）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Parameters")
	float MaxControlOutput = 0.12f;

	// 最大倾斜角度 (度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Parameters")
	float MaxTiltAngle = 30.0f;

protected:

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

	// 限制角度到[-180, 180]
	float NormalizeAngle(float Angle) const;
};
