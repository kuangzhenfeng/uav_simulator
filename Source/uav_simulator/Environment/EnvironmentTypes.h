// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "EnvironmentTypes.generated.h"

/**
 * 风场类型枚举
 */
UENUM(BlueprintType)
enum class EWindFieldType : uint8
{
	None			UMETA(DisplayName = "No Wind"),
	Constant		UMETA(DisplayName = "Constant Wind"),
	Gust			UMETA(DisplayName = "Wind with Gusts"),
	Turbulent		UMETA(DisplayName = "Turbulent Wind (Dryden)")
};

/**
 * 天气条件枚举
 */
UENUM(BlueprintType)
enum class EWeatherCondition : uint8
{
	Clear			UMETA(DisplayName = "Clear"),
	Cloudy			UMETA(DisplayName = "Cloudy"),
	Rain			UMETA(DisplayName = "Rain"),
	HeavyRain		UMETA(DisplayName = "Heavy Rain"),
	Snow			UMETA(DisplayName = "Snow"),
	Fog				UMETA(DisplayName = "Fog")
};

/**
 * 风场配置参数
 */
USTRUCT(BlueprintType)
struct FWindConfig
{
	GENERATED_BODY()

	// 风场类型
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind")
	EWindFieldType WindType = EWindFieldType::Constant;

	// 稳态风速 (cm/s，世界坐标系)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind")
	FVector SteadyWindVelocity = FVector(0.0f, 100.0f, 0.0f);

	// 阵风最大幅度 (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Gust")
	float GustAmplitude = 200.0f;

	// 阵风平均持续时间 (s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Gust")
	float GustDuration = 2.0f;

	// 阵风频率（每分钟发生次数）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Gust")
	float GustFrequency = 5.0f;

	// 湍流强度 (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Turbulence")
	float TurbulenceIntensity = 100.0f;

	// 湍流长度尺度 (cm)，影响湍流时间相关性
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Turbulence")
	float TurbulenceLengthScale = 5000.0f;

	// 空气密度 (kg/m³)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Atmosphere")
	float AirDensity = 1.225f;

	// UAV 迎风面积 (m²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Drag")
	float DragArea = 0.04f;

	// 风阻系数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Drag")
	float DragCoefficient = 1.0f;

	// 风场是否启用
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind")
	bool bEnabled = true;

	FWindConfig() = default;
};

/**
 * 风场状态数据（实时测量值）
 */
USTRUCT(BlueprintType)
struct FWindState
{
	GENERATED_BODY()

	// 当前风速 (cm/s，世界坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "Wind")
	FVector WindVelocity;

	// 稳态风分量
	UPROPERTY(BlueprintReadWrite, Category = "Wind")
	FVector SteadyComponent;

	// 阵风分量
	UPROPERTY(BlueprintReadWrite, Category = "Wind")
	FVector GustComponent;

	// 湍流分量
	UPROPERTY(BlueprintReadWrite, Category = "Wind")
	FVector TurbulenceComponent;

	// 风速标量 (cm/s)
	UPROPERTY(BlueprintReadWrite, Category = "Wind")
	float WindSpeed;

	// 风向 (度，0=北/X正方向)
	UPROPERTY(BlueprintReadWrite, Category = "Wind")
	float WindDirection;

	FWindState()
		: WindVelocity(FVector::ZeroVector)
		, SteadyComponent(FVector::ZeroVector)
		, GustComponent(FVector::ZeroVector)
		, TurbulenceComponent(FVector::ZeroVector)
		, WindSpeed(0.0f)
		, WindDirection(0.0f)
	{}
};

/**
 * 天气配置参数
 */
USTRUCT(BlueprintType)
struct FWeatherConfig
{
	GENERATED_BODY()

	// 天气条件
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather")
	EWeatherCondition Condition = EWeatherCondition::Clear;

	// 能见度 (cm)，影响传感器量程
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather")
	float Visibility = 10000000.0f; // 100km

	// 降水量 (mm/h)，影响 IMU/GPS 噪声
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather")
	float PrecipitationRate = 0.0f;

	// 温度 (°C)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather")
	float Temperature = 20.0f;

	// 大气压 (hPa)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather")
	float Pressure = 1013.25f;

	// 湿度 (0-1)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather")
	float Humidity = 0.5f;

	FWeatherConfig() = default;
};
