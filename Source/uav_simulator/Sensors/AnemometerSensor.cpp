// Copyright Epic Games, Inc. All Rights Reserved.

#include "AnemometerSensor.h"
#include "../Environment/WindField.h"
#include "../Debug/UAVLogConfig.h"

UAnemometerSensor::UAnemometerSensor()
{
	UpdateRate = 20.0f; // 20Hz 风速计更新率
}

void UAnemometerSensor::UpdateSensor(const FUAVState& TrueState, float DeltaTime)
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

	// 从风场组件获取真实风速
	FVector TrueWindVelocity = FVector::ZeroVector;
	if (WindFieldRef)
	{
		TrueWindVelocity = WindFieldRef->GetWindAtPosition(TrueState.Position);
	}

	// 添加测量噪声
	float TrueSpeed = TrueWindVelocity.Size();
	CurrentData.WindSpeed = FMath::Max(0.0f, AddGaussianNoise(TrueSpeed, SpeedNoiseStdDev));
	CurrentData.WindVelocity = TrueWindVelocity + FVector(
		AddGaussianNoise(0.0f, SpeedNoiseStdDev),
		AddGaussianNoise(0.0f, SpeedNoiseStdDev),
		AddGaussianNoise(0.0f, SpeedNoiseStdDev * 0.5f) // Z分量噪声较小
	);

	// 计算风向
	if (CurrentData.WindSpeed > KINDA_SMALL_NUMBER)
	{
		float TrueDirection = FMath::Atan2(TrueWindVelocity.Y, TrueWindVelocity.X) * (180.0f / PI);
		if (TrueDirection < 0.0f) TrueDirection += 360.0f;

		// 风向噪声（使用 von Mises 分布近似）
		float DirNoiseRad = AddGaussianNoise(0.0f, DirectionNoiseStdDev) * (PI / 180.0f);
		float NoisyDirRad = TrueDirection * (PI / 180.0f) + DirNoiseRad;
		CurrentData.WindDirection = FMath::RadiansToDegrees(NoisyDirRad);
		if (CurrentData.WindDirection < 0.0f) CurrentData.WindDirection += 360.0f;
		if (CurrentData.WindDirection >= 360.0f) CurrentData.WindDirection -= 360.0f;
	}
	else
	{
		CurrentData.WindDirection = 0.0f;
	}
}

float UAnemometerSensor::AddGaussianNoise(float Value, float StdDev) const
{
	if (StdDev <= 0.0f)
	{
		return Value;
	}

	float U1 = FMath::FRand();
	float U2 = FMath::FRand();
	if (U1 < KINDA_SMALL_NUMBER) U1 = KINDA_SMALL_NUMBER;
	float Z = FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(2.0f * PI * U2);

	return Value + Z * StdDev;
}
