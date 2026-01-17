// Copyright Epic Games, Inc. All Rights Reserved.

#include "IMUSensor.h"

UIMUSensor::UIMUSensor()
{
	UpdateRate = 1000.0f; // 1000Hz
}

void UIMUSensor::UpdateSensor(const FUAVState& TrueState, float DeltaTime)
{
	if (!bEnabled)
		return;

	// 控制更新频率
	AccumulatedTime += DeltaTime;
	float UpdateInterval = 1.0f / UpdateRate;

	if (AccumulatedTime < UpdateInterval)
		return;

	AccumulatedTime = 0.0f;

	// 计算加速度计数据
	// 加速度计测量的是比力（specific force），包含重力
	// 在机体坐标系中，需要将重力转换到机体坐标系
	FQuat Orientation = TrueState.Rotation.Quaternion();
	FVector GravityWorld(0.0f, 0.0f, -9.8f); // 世界坐标系中的重力 (m/s²)
	FVector GravityBody = Orientation.Inverse().RotateVector(GravityWorld);

	// 线性加速度（从速度变化率计算，简化模型）
	// 实际应该从动力学模型获取
	FVector LinearAccel = FVector::ZeroVector; // 简化：假设匀速运动

	// 加速度计测量 = 线性加速度 - 重力（机体坐标系）
	CurrentData.Accelerometer.X = AddGaussianNoise(LinearAccel.X - GravityBody.X, AccelNoiseStdDev) + AccelBias.X;
	CurrentData.Accelerometer.Y = AddGaussianNoise(LinearAccel.Y - GravityBody.Y, AccelNoiseStdDev) + AccelBias.Y;
	CurrentData.Accelerometer.Z = AddGaussianNoise(LinearAccel.Z - GravityBody.Z, AccelNoiseStdDev) + AccelBias.Z;

	// 计算陀螺仪数据（角速度，机体坐标系）
	CurrentData.Gyroscope.X = AddGaussianNoise(TrueState.AngularVelocity.X, GyroNoiseStdDev) + GyroBias.X;
	CurrentData.Gyroscope.Y = AddGaussianNoise(TrueState.AngularVelocity.Y, GyroNoiseStdDev) + GyroBias.Y;
	CurrentData.Gyroscope.Z = AddGaussianNoise(TrueState.AngularVelocity.Z, GyroNoiseStdDev) + GyroBias.Z;
}

float UIMUSensor::AddGaussianNoise(float Value, float StdDev) const
{
	// 使用Box-Muller变换生成高斯噪声
	float U1 = FMath::FRand();
	float U2 = FMath::FRand();
	float Noise = FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(2.0f * PI * U2) * StdDev;
	return Value + Noise;
}
