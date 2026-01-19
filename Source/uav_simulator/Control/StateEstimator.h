// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "StateEstimator.generated.h"

/**
 * 扩展卡尔曼滤波器（EKF）状态估计器
 * 融合IMU和GPS数据，提供准确的状态估计
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UStateEstimator : public UActorComponent
{
	GENERATED_BODY()

public:
	UStateEstimator();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 预测步骤（使用IMU数据）
	UFUNCTION(BlueprintCallable, Category = "State Estimator")
	void Predict(const FVector& Acceleration, const FVector& AngularVelocity, float DeltaTime);

	// 更新步骤（使用GPS数据）
	UFUNCTION(BlueprintCallable, Category = "State Estimator")
	void UpdateGPS(const FVector& GPSPosition, const FVector& GPSVelocity);

	// 获取估计的状态
	UFUNCTION(BlueprintCallable, Category = "State Estimator")
	FUAVState GetEstimatedState() const { return EstimatedState; }

	// 重置估计器
	UFUNCTION(BlueprintCallable, Category = "State Estimator")
	void Reset(const FUAVState& InitialState);

protected:
	// 估计的状态
	UPROPERTY(BlueprintReadOnly, Category = "State")
	FUAVState EstimatedState;

	// 过程噪声协方差
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EKF Parameters")
	float ProcessNoisePosition = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EKF Parameters")
	float ProcessNoiseVelocity = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EKF Parameters")
	float ProcessNoiseAttitude = 0.01f;

	// 测量噪声协方差
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EKF Parameters")
	float MeasurementNoisePosition = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EKF Parameters")
	float MeasurementNoiseVelocity = 0.1f;

private:
	// 状态协方差矩阵（简化为对角矩阵）
	FVector CovariancePosition = FVector(1.0f, 1.0f, 1.0f);
	FVector CovarianceVelocity = FVector(1.0f, 1.0f, 1.0f);
	FVector CovarianceAttitude = FVector(0.1f, 0.1f, 0.1f);

	// 重力常量
	static constexpr float GravityAcceleration = 980.0f; // cm/s²
};
