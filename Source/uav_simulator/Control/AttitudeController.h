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
	float Kp = 6.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
	float Ki = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID")
	float Kd = 0.5f;

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

protected:
	// Roll PID参数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Parameters")
	FPIDParams RollPID = FPIDParams(6.0f, 0.1f, 0.5f);

	// Pitch PID参数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Parameters")
	FPIDParams PitchPID = FPIDParams(6.0f, 0.1f, 0.5f);

	// Yaw PID参数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID Parameters")
	FPIDParams YawPID = FPIDParams(4.0f, 0.05f, 0.3f);

	// 悬停推力 (0-1归一化)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Parameters")
	float HoverThrust = 0.6f;

	// 最大倾斜角度 (度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Parameters")
	float MaxTiltAngle = 30.0f;

private:
	// PID积分项
	float RollIntegral = 0.0f;
	float PitchIntegral = 0.0f;
	float YawIntegral = 0.0f;

	// 上一次误差（用于微分项）
	float LastRollError = 0.0f;
	float LastPitchError = 0.0f;
	float LastYawError = 0.0f;

	// 计算单轴PID控制
	float ComputePID(float Error, float& Integral, float& LastError, const FPIDParams& Params, float DeltaTime);

	// 限制角度到[-180, 180]
	float NormalizeAngle(float Angle) const;
};
