// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SensorBase.h"
#include "AnemometerSensor.generated.h"

/**
 * 风速计数据结构
 */
USTRUCT(BlueprintType)
struct FAnemometerData
{
	GENERATED_BODY()

	// 测量风速 (cm/s)
	UPROPERTY(BlueprintReadWrite, Category = "Anemometer")
	float WindSpeed;

	// 测量风向 (度，0=X正方向)
	UPROPERTY(BlueprintReadWrite, Category = "Anemometer")
	float WindDirection;

	// 风速向量 (cm/s，世界坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "Anemometer")
	FVector WindVelocity;

	FAnemometerData()
		: WindSpeed(0.0f)
		, WindDirection(0.0f)
		, WindVelocity(FVector::ZeroVector)
	{}
};

// 前向声明
class UWindField;

/**
 * 风速计传感器组件
 * 测量当前位置的风速和风向
 * 需要引用 WindField 组件获取真实风速
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UAnemometerSensor : public USensorBase
{
	GENERATED_BODY()

public:
	UAnemometerSensor();

	virtual void UpdateSensor(const FUAVState& TrueState, float DeltaTime) override;

	// 获取风速计数据
	UFUNCTION(BlueprintCallable, Category = "Anemometer")
	FAnemometerData GetAnemometerData() const { return CurrentData; }

	// 设置风场组件引用
	UFUNCTION(BlueprintCallable, Category = "Anemometer")
	void SetWindField(UWindField* InWindField) { WindFieldRef = InWindField; }

protected:
	// 当前风速计数据
	UPROPERTY(BlueprintReadOnly, Category = "Anemometer")
	FAnemometerData CurrentData;

	// 风速噪声标准差 (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Anemometer Noise")
	float SpeedNoiseStdDev = 5.0f;

	// 风向噪声标准差 (度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Anemometer Noise")
	float DirectionNoiseStdDev = 2.0f;

private:
	// 风场组件引用
	UPROPERTY()
	TObjectPtr<UWindField> WindFieldRef;

	// 添加高斯噪声
	float AddGaussianNoise(float Value, float StdDev) const;
};
