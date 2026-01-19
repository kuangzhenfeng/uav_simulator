// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "PositionController.generated.h"

/**
 * 位置控制器组件
 * 实现级联控制架构：位置控制 -> 速度控制 -> 姿态控制
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UPositionController : public UActorComponent
{
	GENERATED_BODY()

public:
	UPositionController();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 计算位置控制输出（期望姿态和推力）
	UFUNCTION(BlueprintCallable, Category = "Position Controller")
	void ComputeControl(const FUAVState& CurrentState, const FVector& TargetPosition, const FVector& TargetVelocity,
						FRotator& OutDesiredAttitude, float& OutThrust, float DeltaTime);

	// 设置目标位置
	UFUNCTION(BlueprintCallable, Category = "Position Controller")
	void SetTargetPosition(const FVector& InTargetPosition) { TargetPosition = InTargetPosition; }

	// 设置目标速度（前馈）
	UFUNCTION(BlueprintCallable, Category = "Position Controller")
	void SetTargetVelocity(const FVector& InTargetVelocity) { TargetVelocity = InTargetVelocity; }

	// 重置控制器状态
	UFUNCTION(BlueprintCallable, Category = "Position Controller")
	void Reset();

	// 位置PID参数 (大幅降低增益以提高稳定性)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Position Control Parameters")
	float Kp_Position = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Position Control Parameters")
	float Ki_Position = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Position Control Parameters")
	float Kd_Position = 0.0f;

	// 速度PID参数 (极低增益，纯P控制)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity Control Parameters")
	float Kp_Velocity = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity Control Parameters")
	float Ki_Velocity = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity Control Parameters")
	float Kd_Velocity = 0.0f;

	// 控制限制
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Limits")
	float MaxVelocity = 500.0f; // cm/s

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Limits")
	float MaxTiltAngle = 30.0f; // 度

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Limits")
	float MaxThrust = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Limits")
	float MinThrust = 0.0f;

	// 物理参数（需要与UAVDynamics保持一致）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Parameters")
	float UAVMass = 1.5f; // kg

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Parameters")
	float SingleMotorMaxThrust = 15.0f; // N

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Parameters")
	int32 NumMotors = 4;

protected:

	// 目标状态
	UPROPERTY(BlueprintReadOnly, Category = "Target State")
	FVector TargetPosition = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Target State")
	FVector TargetVelocity = FVector::ZeroVector;

private:
	// 位置误差积分
	FVector PositionErrorIntegral = FVector::ZeroVector;
	FVector PreviousPositionError = FVector::ZeroVector;

	// 速度误差积分
	FVector VelocityErrorIntegral = FVector::ZeroVector;
	FVector PreviousVelocityError = FVector::ZeroVector;

	// 重力常量
	static constexpr float GravityAcceleration = 980.0f; // cm/s²
};
