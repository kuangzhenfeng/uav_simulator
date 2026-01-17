// Copyright Epic Games, Inc. All Rights Reserved.

#include "GPSSensor.h"

UGPSSensor::UGPSSensor()
{
	UpdateRate = 10.0f; // 10Hz (典型GPS更新频率)
}

void UGPSSensor::UpdateSensor(const FUAVState& TrueState, float DeltaTime)
{
	if (!bEnabled)
		return;

	// 控制更新频率
	AccumulatedTime += DeltaTime;
	float UpdateInterval = 1.0f / UpdateRate;

	if (AccumulatedTime < UpdateInterval)
		return;

	AccumulatedTime = 0.0f;

	// 模拟GPS位置测量（添加噪声）
	CurrentData.Position.X = AddGaussianNoise(TrueState.Position.X, PositionNoiseStdDev);
	CurrentData.Position.Y = AddGaussianNoise(TrueState.Position.Y, PositionNoiseStdDev);
	CurrentData.Position.Z = AddGaussianNoise(TrueState.Position.Z, PositionNoiseStdDev);

	// 模拟GPS速度测量（添加噪声）
	CurrentData.Velocity.X = AddGaussianNoise(TrueState.Velocity.X, VelocityNoiseStdDev);
	CurrentData.Velocity.Y = AddGaussianNoise(TrueState.Velocity.Y, VelocityNoiseStdDev);
	CurrentData.Velocity.Z = AddGaussianNoise(TrueState.Velocity.Z, VelocityNoiseStdDev);

	CurrentData.bIsValid = true;
}

float UGPSSensor::AddGaussianNoise(float Value, float StdDev) const
{
	// 使用Box-Muller变换生成高斯噪声
	float U1 = FMath::FRand();
	float U2 = FMath::FRand();
	float Noise = FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(2.0f * PI * U2) * StdDev;
	return Value + Noise;
}
