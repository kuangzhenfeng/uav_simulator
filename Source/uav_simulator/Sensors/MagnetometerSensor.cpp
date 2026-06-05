// Copyright Epic Games, Inc. All Rights Reserved.

#include "MagnetometerSensor.h"
#include "../Debug/UAVLogConfig.h"

UMagnetometerSensor::UMagnetometerSensor()
{
	UpdateRate = 100.0f; // 100Hz 磁力计更新率
}

void UMagnetometerSensor::UpdateSensor(const FUAVState& TrueState, float DeltaTime)
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

	// 将地磁场从世界坐标系转换到机体坐标系
	FQuat Orientation = TrueState.Rotation.Quaternion();
	FVector BodyMagneticField = Orientation.Inverse().RotateVector(ReferenceMagneticField);

	// 添加噪声
	CurrentData.MagneticField = FVector(
		AddGaussianNoise(BodyMagneticField.X, MagneticNoiseStdDev),
		AddGaussianNoise(BodyMagneticField.Y, MagneticNoiseStdDev),
		AddGaussianNoise(BodyMagneticField.Z, MagneticNoiseStdDev)
	);

	// 从磁场测量值计算航向角
	// 使用水平分量（X, Y）计算磁航向
	// 注意：UE5 使用 FRD 机体坐标系，X=前，Y=右
	float HeadingRad = FMath::Atan2(
		CurrentData.MagneticField.Y,
		CurrentData.MagneticField.X
	);

	// 转换为度数 [0, 360)
	CurrentData.Heading = FMath::RadiansToDegrees(HeadingRad);
	if (CurrentData.Heading < 0.0f)
	{
		CurrentData.Heading += 360.0f;
	}
}

float UMagnetometerSensor::AddGaussianNoise(float Value, float StdDev) const
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
