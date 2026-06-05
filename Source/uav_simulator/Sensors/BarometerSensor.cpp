// Copyright Epic Games, Inc. All Rights Reserved.

#include "BarometerSensor.h"
#include "../Debug/UAVLogConfig.h"

UBarometerSensor::UBarometerSensor()
{
	UpdateRate = 50.0f; // 50Hz 气压计更新率
}

void UBarometerSensor::UpdateSensor(const FUAVState& TrueState, float DeltaTime)
{
	if (!bEnabled)
	{
		return;
	}

	// 控制更新频率
	AccumulatedTime += DeltaTime;
	const float UpdateInterval = 1.0f / FMath::Max(UpdateRate, 1.0f);
	if (AccumulatedTime < UpdateInterval)
	{
		return;
	}
	AccumulatedTime = 0.0f;

	// 使用国际标准大气模型 (ISA) 计算气压
	// P = P0 * (1 - L*h/T0)^(g*M/(R*L))
	// 简化: 使用气压公式 h = 44330 * (1 - (P/P0)^(1/5.255))
	// 逆运算: P = P0 * (1 - h/44330)^5.255

	// 高度单位转换: UE5 cm → m
	float AltitudeM = TrueState.Position.Z * 0.01f;

	// 计算理论气压
	float PressureRatio = FMath::Clamp(1.0f - AltitudeM / 44330.0f, 0.01f, 1.0f);
	float TheoreticalPressure = ReferencePressure * FMath::Pow(PressureRatio, 5.255f);

	// 添加气压噪声
	CurrentData.Pressure = AddGaussianNoise(TheoreticalPressure, PressureNoiseStdDev);

	// 从带噪声的气压反算高度（模拟真实传感器流程）
	CurrentData.Altitude = 44330.0f * (1.0f - FMath::Pow(
		FMath::Clamp(CurrentData.Pressure / ReferencePressure, 0.01f, 1.0f),
		1.0f / 5.255f)) * 100.0f; // m → cm

	// 温度模拟（标准大气递减率: -6.5°C/km）
	CurrentData.Temperature = AddGaussianNoise(20.0f - AltitudeM * 0.0065f, TemperatureNoiseStdDev);
}

float UBarometerSensor::AddGaussianNoise(float Value, float StdDev) const
{
	if (StdDev <= 0.0f)
	{
		return Value;
	}

	// Box-Muller 变换
	float U1 = FMath::FRand();
	float U2 = FMath::FRand();
	if (U1 < KINDA_SMALL_NUMBER) U1 = KINDA_SMALL_NUMBER;
	float Z = FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(2.0f * PI * U2);

	return Value + Z * StdDev;
}
