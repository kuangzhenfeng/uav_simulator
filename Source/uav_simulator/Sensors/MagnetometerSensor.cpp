// Copyright Epic Games, Inc. All Rights Reserved.

#include "MagnetometerSensor.h"
#include "../Debug/UAVLogConfig.h"
#include "../Utility/Filter.h"

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
	// UE5 Yaw 正方向为绕 Z 轴左手法则（从上往下看顺时针）
	// 当朝东时（Yaw=90），机体系磁场 North 分量沿 -Y 方向
	// 因此航向 = atan2(-BodyY, BodyX)
	float HeadingRad = FMath::Atan2(
		-CurrentData.MagneticField.Y,
		CurrentData.MagneticField.X
	);

	// 转换为度数 [0, 360)
	CurrentData.Heading = FMath::RadiansToDegrees(HeadingRad);
	if (CurrentData.Heading < 0.0f)
	{
		CurrentData.Heading += 360.0f;
	}

	UE_LOG_THROTTLE(1.0, LogUAVSensor, Log, TEXT("[Magnetometer] Heading=%.1f Mag=(%.1f,%.1f,%.1f)"),
		CurrentData.Heading, CurrentData.MagneticField.X, CurrentData.MagneticField.Y, CurrentData.MagneticField.Z);
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
