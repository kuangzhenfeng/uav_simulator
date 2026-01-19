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
	// 使用RK4积分器提高数值精度和稳定性
	return IntegrateRK4(CurrentState, DeltaTime);
}

FUAVState UUAVDynamics::IntegrateRK4(const FUAVState& State, float DeltaTime)
{
	// RK4四阶龙格-库塔积分
	FUAVState k1 = ComputeDerivative(State, 0.0f);
	FUAVState k2 = ComputeDerivative(AddStates(State, ScaleState(k1, DeltaTime * 0.5f)), DeltaTime * 0.5f);
	FUAVState k3 = ComputeDerivative(AddStates(State, ScaleState(k2, DeltaTime * 0.5f)), DeltaTime * 0.5f);
	FUAVState k4 = ComputeDerivative(AddStates(State, ScaleState(k3, DeltaTime)), DeltaTime);

	// 组合结果: NewState = State + (k1 + 2*k2 + 2*k3 + k4) * dt/6
	FUAVState Result = State;
	Result.Position = State.Position + (k1.Position + k2.Position * 2.0f + k3.Position * 2.0f + k4.Position) * (DeltaTime / 6.0f);
	Result.Velocity = State.Velocity + (k1.Velocity + k2.Velocity * 2.0f + k3.Velocity * 2.0f + k4.Velocity) * (DeltaTime / 6.0f);
	Result.AngularVelocity = State.AngularVelocity + (k1.AngularVelocity + k2.AngularVelocity * 2.0f + k3.AngularVelocity * 2.0f + k4.AngularVelocity) * (DeltaTime / 6.0f);

	// 姿态更新使用四元数积分，使用积分后的角速度
	FVector AngularVelDeg = Result.AngularVelocity * (180.0f / PI);
	FRotator DeltaRotation(AngularVelDeg.Y * DeltaTime, AngularVelDeg.Z * DeltaTime, AngularVelDeg.X * DeltaTime);
	Result.Rotation = (State.Rotation.Quaternion() * DeltaRotation.Quaternion()).Rotator();

	return Result;
}

FUAVState UUAVDynamics::ComputeDerivative(const FUAVState& State, float Time)
{
	FUAVState Derivative;

	// 计算推力和力矩（机体坐标系）
	FVector ThrustForce;
	FVector TotalTorque;
	ComputeForcesAndTorques(ThrustForce, TotalTorque);

	// 将推力从机体坐标系转换到世界坐标系
	FQuat Orientation = State.Rotation.Quaternion();
	FVector WorldThrustForce = Orientation.RotateVector(ThrustForce);

	// 在世界坐标系中添加重力（始终垂直向下）
	FVector GravityForce = FVector(0.0f, 0.0f, -Mass * GravityAcceleration);

	// 在世界坐标系中添加空气阻力
	float VelocityMag = State.Velocity.Size();
	FVector DragForce = -DragCoefficient * State.Velocity - QuadraticDragCoefficient * VelocityMag * State.Velocity;

	// 总力（世界坐标系）
	FVector WorldForce = WorldThrustForce + GravityForce + DragForce;

	// 位置导数 = 速度
	Derivative.Position = State.Velocity;

	// 速度导数 = 加速度
	Derivative.Velocity = WorldForce / Mass;

	// 角速度导数 = 角加速度 (包含陀螺效应)
	FVector AngularAcceleration = ComputeAngularAcceleration(TotalTorque, State.AngularVelocity);
	Derivative.AngularVelocity = AngularAcceleration;

	// 姿态导数由角速度决定，在积分时处理
	Derivative.Rotation = FRotator::ZeroRotator;

	return Derivative;
}

FVector UUAVDynamics::ComputeAngularAcceleration(const FVector& Torque, const FVector& AngularVelocity) const
{
	// 完整的欧拉动力学方程，包含陀螺效应
	// dω/dt = I^(-1) * (τ - ω × (I * ω))

	FVector Iw; // I * ω
	Iw.X = MomentOfInertia.X * AngularVelocity.X;
	Iw.Y = MomentOfInertia.Y * AngularVelocity.Y;
	Iw.Z = MomentOfInertia.Z * AngularVelocity.Z;

	// ω × (I * ω) 陀螺力矩
	FVector GyroTorque = FVector::CrossProduct(AngularVelocity, Iw);

	// 有效力矩 = 外部力矩 - 陀螺力矩
	FVector EffectiveTorque = Torque - GyroTorque;

	// 角加速度 = I^(-1) * 有效力矩
	FVector AngularAcceleration;
	AngularAcceleration.X = EffectiveTorque.X / MomentOfInertia.X;
	AngularAcceleration.Y = EffectiveTorque.Y / MomentOfInertia.Y;
	AngularAcceleration.Z = EffectiveTorque.Z / MomentOfInertia.Z;

	return AngularAcceleration;
}

FUAVState UUAVDynamics::AddStates(const FUAVState& A, const FUAVState& B) const
{
	FUAVState Result;
	Result.Position = A.Position + B.Position;
	Result.Velocity = A.Velocity + B.Velocity;
	Result.Rotation = A.Rotation; // 姿态在RK4中特殊处理
	Result.AngularVelocity = A.AngularVelocity + B.AngularVelocity;
	return Result;
}

FUAVState UUAVDynamics::ScaleState(const FUAVState& State, float Scale) const
{
	FUAVState Result;
	Result.Position = State.Position * Scale;
	Result.Velocity = State.Velocity * Scale;
	Result.Rotation = State.Rotation; // 姿态不缩放
	Result.AngularVelocity = State.AngularVelocity * Scale;
	return Result;
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
	float PitchTorque = (T2 + T3 - T0 - T1) * ArmLength * 0.707f;

	// Yaw力矩 (绕Z轴): 由电机反扭矩产生
	// 基于物理参数计算: τ_yaw = k_m * (ω0² - ω1² + ω2² - ω3²)
	// 简化为: τ_yaw = k_m * (T0 - T1 + T2 - T3)
	// 其中 k_m 是电机扭矩系数，与推力系数的比值
	float YawTorque = (T0 - T1 + T2 - T3) * MotorTorqueCoefficient;

	// 保持为N·m单位，不转换（与转动惯量单位kg·m²匹配）
	OutTorque = FVector(
		RollTorque,
		PitchTorque,
		YawTorque
	);
}
