// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVDynamics.h"

UUAVDynamics::UUAVDynamics()
{
	PrimaryComponentTick.bCanEverTick = true;

	// 初始化4个电机推力为0
	MotorThrusts.Init(0.0f, 4);
}

void UUAVDynamics::BeginPlay()
{
	Super::BeginPlay();
}

void UUAVDynamics::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

FUAVState UUAVDynamics::UpdateDynamics(const FUAVState& CurrentState, float DeltaTime)
{
	FUAVState NewState = CurrentState;

	// 计算总推力和力矩
	FVector TotalForce;
	FVector TotalTorque;
	ComputeForcesAndTorques(TotalForce, TotalTorque);

	// 添加重力 (UE5坐标系: Z轴向上)
	TotalForce.Z -= Mass * GravityAcceleration;

	// 添加空气阻力 (简化模型)
	FVector DragForce = -DragCoefficient * CurrentState.Velocity;
	TotalForce += DragForce;

	// 将力从机体坐标系转换到世界坐标系
	FQuat Orientation = CurrentState.Rotation.Quaternion();
	FVector WorldForce = Orientation.RotateVector(TotalForce);

	// 计算线性加速度 (cm/s²)
	FVector LinearAcceleration = WorldForce / Mass;

	// 更新速度和位置 (使用欧拉积分)
	NewState.Velocity = CurrentState.Velocity + LinearAcceleration * DeltaTime;
	NewState.Position = CurrentState.Position + NewState.Velocity * DeltaTime;

	// 计算角加速度 (机体坐标系)
	FVector AngularAcceleration;
	AngularAcceleration.X = TotalTorque.X / MomentOfInertia.X;
	AngularAcceleration.Y = TotalTorque.Y / MomentOfInertia.Y;
	AngularAcceleration.Z = TotalTorque.Z / MomentOfInertia.Z;

	// 更新角速度 (简化模型，忽略陀螺效应)
	NewState.AngularVelocity = CurrentState.AngularVelocity + AngularAcceleration * DeltaTime;

	// 更新姿态 (使用角速度积分)
	FVector AngularVelocityDeg = NewState.AngularVelocity * (180.0f / PI); // 转换为度/秒
	FRotator DeltaRotation(
		AngularVelocityDeg.Y * DeltaTime, // Pitch
		AngularVelocityDeg.Z * DeltaTime, // Yaw
		AngularVelocityDeg.X * DeltaTime  // Roll
	);
	NewState.Rotation = (CurrentState.Rotation.Quaternion() * DeltaRotation.Quaternion()).Rotator();

	return NewState;
}

void UUAVDynamics::SetMotorThrusts(const TArray<float>& InThrusts)
{
	if (InThrusts.Num() == 4)
	{
		MotorThrusts = InThrusts;

		// 限制推力范围 [0, 1]
		for (float& Thrust : MotorThrusts)
		{
			Thrust = FMath::Clamp(Thrust, 0.0f, 1.0f);
		}
	}
}

void UUAVDynamics::ComputeForcesAndTorques(FVector& OutForce, FVector& OutTorque) const
{
	// 四旋翼配置 (X型):
	//     0(CW)
	//       |
	// 3(CCW)-+-1(CCW)
	//       |
	//     2(CW)

	// 计算每个电机的实际推力 (N)
	float T0 = MotorThrusts[0] * MaxThrust;
	float T1 = MotorThrusts[1] * MaxThrust;
	float T2 = MotorThrusts[2] * MaxThrust;
	float T3 = MotorThrusts[3] * MaxThrust;

	// 总推力 (机体坐标系，Z轴向上)
	// UE5使用cm，需要转换: 1N = 100 g·cm/s² / 1000g = 0.1 kg·cm/s²
	float TotalThrustN = T0 + T1 + T2 + T3;
	OutForce = FVector(0.0f, 0.0f, TotalThrustN * 100.0f); // 转换为UE5单位

	// 计算力矩 (机体坐标系)
	// Roll力矩 (绕X轴): 由左右电机推力差产生
	float RollTorque = (T1 + T2 - T0 - T3) * ArmLength * 0.707f; // 0.707 = sqrt(2)/2

	// Pitch力矩 (绕Y轴): 由前后电机推力差产生
	float PitchTorque = (T0 + T1 - T2 - T3) * ArmLength * 0.707f;

	// Yaw力矩 (绕Z轴): 由电机反扭矩产生 (简化模型)
	float YawTorque = (T0 + T2 - T1 - T3) * 0.01f; // 简化的反扭矩系数

	// 转换为UE5单位 (kg·cm²/s²)
	OutTorque = FVector(
		RollTorque * 100.0f,
		PitchTorque * 100.0f,
		YawTorque * 100.0f
	);
}
