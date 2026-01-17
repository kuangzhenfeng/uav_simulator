// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SensorBase.h"
#include "GPSSensor.generated.h"

/**
 * GPS传感器数据结构
 */
USTRUCT(BlueprintType)
struct FGPSData
{
	GENERATED_BODY()

	// 位置 (世界坐标系，cm)
	UPROPERTY(BlueprintReadWrite, Category = "GPS")
	FVector Position;

	// 速度 (世界坐标系，cm/s)
	UPROPERTY(BlueprintReadWrite, Category = "GPS")
	FVector Velocity;

	// 是否有效
	UPROPERTY(BlueprintReadWrite, Category = "GPS")
	bool bIsValid;

	FGPSData()
		: Position(FVector::ZeroVector)
		, Velocity(FVector::ZeroVector)
		, bIsValid(false)
	{}
};

/**
 * GPS传感器组件
 * 模拟GPS位置和速度测量
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UGPSSensor : public USensorBase
{
	GENERATED_BODY()

public:
	UGPSSensor();

	virtual void UpdateSensor(const FUAVState& TrueState, float DeltaTime) override;

	// 获取GPS数据
	UFUNCTION(BlueprintCallable, Category = "GPS")
	FGPSData GetGPSData() const { return CurrentData; }

protected:
	// 当前GPS数据
	UPROPERTY(BlueprintReadOnly, Category = "GPS")
	FGPSData CurrentData;

	// 位置噪声标准差 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GPS Noise")
	float PositionNoiseStdDev = 100.0f; // 1m

	// 速度噪声标准差 (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GPS Noise")
	float VelocityNoiseStdDev = 10.0f; // 0.1m/s

private:
	// 添加高斯噪声
	float AddGaussianNoise(float Value, float StdDev) const;
};
