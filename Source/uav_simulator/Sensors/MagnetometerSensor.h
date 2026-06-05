// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SensorBase.h"
#include "MagnetometerSensor.generated.h"

/**
 * 磁力计数据结构
 */
USTRUCT(BlueprintType)
struct FMagnetometerData
{
	GENERATED_BODY()

	// 磁场测量值 (μT，机体坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "Magnetometer")
	FVector MagneticField;

	// 磁航向角 (度，0-360，正北=0)
	UPROPERTY(BlueprintReadWrite, Category = "Magnetometer")
	float Heading;

	FMagnetometerData()
		: MagneticField(FVector(0.0f, 0.0f, 50.0f))
		, Heading(0.0f)
	{}
};

/**
 * 磁力计传感器组件
 * 测量地磁场方向，用于航向估计
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UMagnetometerSensor : public USensorBase
{
	GENERATED_BODY()

public:
	UMagnetometerSensor();

	virtual void UpdateSensor(const FUAVState& TrueState, float DeltaTime) override;

	// 获取磁力计数据
	UFUNCTION(BlueprintCallable, Category = "Magnetometer")
	FMagnetometerData GetMagnetometerData() const { return CurrentData; }

protected:
	// 当前磁力计数据
	UPROPERTY(BlueprintReadOnly, Category = "Magnetometer")
	FMagnetometerData CurrentData;

	// 地磁场参考值 (μT，世界坐标系 NED)
	// 典型中纬度地区: 北 ~20μT, 东 ~0μT, 下 ~50μT
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magnetometer Reference")
	FVector ReferenceMagneticField = FVector(20.0f, 0.0f, 50.0f);

	// 磁场噪声标准差 (μT)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magnetometer Noise")
	float MagneticNoiseStdDev = 0.1f;

private:
	// 添加高斯噪声
	float AddGaussianNoise(float Value, float StdDev) const;
};
