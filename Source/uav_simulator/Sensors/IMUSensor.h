// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SensorBase.h"
#include "IMUSensor.generated.h"

/**
 * IMU传感器数据结构
 */
USTRUCT(BlueprintType)
struct FIMUData
{
	GENERATED_BODY()

	// 加速度计数据 (m/s²，机体坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "IMU")
	FVector Accelerometer;

	// 陀螺仪数据 (rad/s，机体坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "IMU")
	FVector Gyroscope;

	FIMUData()
		: Accelerometer(FVector::ZeroVector)
		, Gyroscope(FVector::ZeroVector)
	{}
};

/**
 * IMU传感器组件
 * 模拟加速度计和陀螺仪
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UIMUSensor : public USensorBase
{
	GENERATED_BODY()

public:
	UIMUSensor();

	virtual void UpdateSensor(const FUAVState& TrueState, float DeltaTime) override;

	// 获取IMU数据
	UFUNCTION(BlueprintCallable, Category = "IMU")
	FIMUData GetIMUData() const { return CurrentData; }

protected:
	// 当前IMU数据
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FIMUData CurrentData;

	// 加速度计噪声标准差 (m/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Noise")
	float AccelNoiseStdDev = 0.04f;

	// 陀螺仪噪声标准差 (rad/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Noise")
	float GyroNoiseStdDev = 0.001f;

	// 加速度计偏置 (m/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Bias")
	FVector AccelBias = FVector::ZeroVector;

	// 陀螺仪偏置 (rad/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU Bias")
	FVector GyroBias = FVector::ZeroVector;

private:
	// 添加高斯噪声
	float AddGaussianNoise(float Value, float StdDev) const;
};
