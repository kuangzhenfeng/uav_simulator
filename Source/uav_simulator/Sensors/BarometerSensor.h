// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SensorBase.h"
#include "BarometerSensor.generated.h"

/**
 * 气压计数据结构
 */
USTRUCT(BlueprintType)
struct FBarometerData
{
	GENERATED_BODY()

	// 气压测量值 (hPa)
	UPROPERTY(BlueprintReadWrite, Category = "Barometer")
	float Pressure;

	// 气压高度 (cm，相对于参考点)
	UPROPERTY(BlueprintReadWrite, Category = "Barometer")
	float Altitude;

	// 温度测量值 (°C)
	UPROPERTY(BlueprintReadWrite, Category = "Barometer")
	float Temperature;

	FBarometerData()
		: Pressure(1013.25f)
		, Altitude(0.0f)
		, Temperature(20.0f)
	{}
};

/**
 * 气压计传感器组件
 * 基于大气压测量高度，模拟气压高度计
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UBarometerSensor : public USensorBase
{
	GENERATED_BODY()

public:
	UBarometerSensor();

	virtual void UpdateSensor(const FUAVState& TrueState, float DeltaTime) override;

	// 获取气压计数据
	UFUNCTION(BlueprintCallable, Category = "Barometer")
	FBarometerData GetBarometerData() const { return CurrentData; }

	// 设置参考气压 (hPa)
	UFUNCTION(BlueprintCallable, Category = "Barometer")
	void SetReferencePressure(float InPressure) { ReferencePressure = InPressure; }

protected:
	// 当前气压计数据
	UPROPERTY(BlueprintReadOnly, Category = "Barometer")
	FBarometerData CurrentData;

	// 参考气压 (hPa，海平面标准大气压)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Barometer")
	float ReferencePressure = 1013.25f;

	// 气压噪声标准差 (hPa)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Barometer Noise")
	float PressureNoiseStdDev = 0.1f;

	// 温度噪声标准差 (°C)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Barometer Noise")
	float TemperatureNoiseStdDev = 0.5f;

private:
	// 添加高斯噪声
	float AddGaussianNoise(float Value, float StdDev) const;
};
