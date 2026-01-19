// Copyright Epic Games, Inc. All Rights Reserved.

#include "StateEstimator.h"

UStateEstimator::UStateEstimator()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UStateEstimator::BeginPlay()
{
	Super::BeginPlay();

	// 初始化估计状态
	EstimatedState = FUAVState();
}

void UStateEstimator::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UStateEstimator::Predict(const FVector& Acceleration, const FVector& AngularVelocity, float DeltaTime)
{
	// EKF预测步骤

	// 1. 状态预测
	// 位置预测: p = p + v*dt + 0.5*a*dt^2
	EstimatedState.Position += EstimatedState.Velocity * DeltaTime + 0.5f * Acceleration * DeltaTime * DeltaTime;

	// 速度预测: v = v + a*dt
	EstimatedState.Velocity += Acceleration * DeltaTime;

	// 姿态预测: 使用角速度积分
	FVector AngularVelDeg = AngularVelocity * (180.0f / PI);
	FRotator DeltaRotation(AngularVelDeg.Y * DeltaTime, AngularVelDeg.Z * DeltaTime, AngularVelDeg.X * DeltaTime);
	EstimatedState.Rotation = (EstimatedState.Rotation.Quaternion() * DeltaRotation.Quaternion()).Rotator();

	// 角速度更新
	EstimatedState.AngularVelocity = AngularVelocity;

	// 2. 协方差预测（简化版本）
	// P = P + Q*dt
	CovariancePosition += FVector(ProcessNoisePosition) * DeltaTime;
	CovarianceVelocity += FVector(ProcessNoiseVelocity) * DeltaTime;
	CovarianceAttitude += FVector(ProcessNoiseAttitude) * DeltaTime;
}

void UStateEstimator::UpdateGPS(const FVector& GPSPosition, const FVector& GPSVelocity)
{
	// EKF更新步骤（使用GPS测量）

	// 1. 计算卡尔曼增益（简化版本）
	// K = P / (P + R)
	FVector KalmanGainPosition;
	KalmanGainPosition.X = CovariancePosition.X / (CovariancePosition.X + MeasurementNoisePosition);
	KalmanGainPosition.Y = CovariancePosition.Y / (CovariancePosition.Y + MeasurementNoisePosition);
	KalmanGainPosition.Z = CovariancePosition.Z / (CovariancePosition.Z + MeasurementNoisePosition);

	FVector KalmanGainVelocity;
	KalmanGainVelocity.X = CovarianceVelocity.X / (CovarianceVelocity.X + MeasurementNoiseVelocity);
	KalmanGainVelocity.Y = CovarianceVelocity.Y / (CovarianceVelocity.Y + MeasurementNoiseVelocity);
	KalmanGainVelocity.Z = CovarianceVelocity.Z / (CovarianceVelocity.Z + MeasurementNoiseVelocity);

	// 2. 状态更新
	// x = x + K * (z - x)
	FVector PositionInnovation = GPSPosition - EstimatedState.Position;
	EstimatedState.Position += FVector(
		KalmanGainPosition.X * PositionInnovation.X,
		KalmanGainPosition.Y * PositionInnovation.Y,
		KalmanGainPosition.Z * PositionInnovation.Z
	);

	FVector VelocityInnovation = GPSVelocity - EstimatedState.Velocity;
	EstimatedState.Velocity += FVector(
		KalmanGainVelocity.X * VelocityInnovation.X,
		KalmanGainVelocity.Y * VelocityInnovation.Y,
		KalmanGainVelocity.Z * VelocityInnovation.Z
	);

	// 3. 协方差更新
	// P = (1 - K) * P
	CovariancePosition.X *= (1.0f - KalmanGainPosition.X);
	CovariancePosition.Y *= (1.0f - KalmanGainPosition.Y);
	CovariancePosition.Z *= (1.0f - KalmanGainPosition.Z);

	CovarianceVelocity.X *= (1.0f - KalmanGainVelocity.X);
	CovarianceVelocity.Y *= (1.0f - KalmanGainVelocity.Y);
	CovarianceVelocity.Z *= (1.0f - KalmanGainVelocity.Z);
}

void UStateEstimator::Reset(const FUAVState& InitialState)
{
	EstimatedState = InitialState;

	// 重置协方差
	CovariancePosition = FVector(1.0f, 1.0f, 1.0f);
	CovarianceVelocity = FVector(1.0f, 1.0f, 1.0f);
	CovarianceAttitude = FVector(0.1f, 0.1f, 0.1f);
}
