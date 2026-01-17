// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVActor.h"
#include "../Physics/UAVDynamics.h"
#include "../Sensors/SensorBase.h"
#include "../Control/AttitudeController.h"
#include "../Debug/DebugVisualizer.h"

AUAVActor::AUAVActor()
{
	PrimaryActorTick.bCanEverTick = true;

	// 创建物理模型组件
	DynamicsComponent = CreateDefaultSubobject<UUAVDynamics>(TEXT("DynamicsComponent"));

	// 创建姿态控制器组件
	AttitudeControllerComponent = CreateDefaultSubobject<UAttitudeController>(TEXT("AttitudeController"));

	// 创建调试可视化组件
	DebugVisualizerComponent = CreateDefaultSubobject<UDebugVisualizer>(TEXT("DebugVisualizer"));

	// 初始化状态
	CurrentState = FUAVState();
	TargetAttitude = FRotator::ZeroRotator;
	TargetPosition = FVector::ZeroVector;
}

void AUAVActor::BeginPlay()
{
	Super::BeginPlay();

	// 初始化位置和姿态
	CurrentState.Position = GetActorLocation();
	CurrentState.Rotation = GetActorRotation();
}

void AUAVActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// 更新传感器
	UpdateSensors(DeltaTime);

	// 更新控制器
	UpdateController(DeltaTime);

	// 更新物理模型
	UpdatePhysics(DeltaTime);

	// 更新Actor的位置和姿态
	SetActorLocation(CurrentState.Position);
	SetActorRotation(CurrentState.Rotation);

	// 更新调试可视化
	if (DebugVisualizerComponent)
	{
		DebugVisualizerComponent->DrawUAVState(CurrentState, GetActorLocation());
		DebugVisualizerComponent->DrawTrajectoryHistory(GetActorLocation());
	}
}

void AUAVActor::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void AUAVActor::SetTargetAttitude(FRotator InTargetAttitude)
{
	TargetAttitude = InTargetAttitude;
}

void AUAVActor::SetTargetPosition(FVector InTargetPosition)
{
	TargetPosition = InTargetPosition;
}

void AUAVActor::UpdateSensors(float DeltaTime)
{
	// 更新所有传感器
	for (USensorBase* Sensor : Sensors)
	{
		if (Sensor)
		{
			Sensor->UpdateSensor(CurrentState, DeltaTime);
		}
	}
}

void AUAVActor::UpdateController(float DeltaTime)
{
	if (AttitudeControllerComponent)
	{
		// 姿态控制器计算电机输出
		FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControl(
			CurrentState, TargetAttitude, DeltaTime);

		// 将电机输出传递给物理模型
		if (DynamicsComponent)
		{
			DynamicsComponent->SetMotorThrusts(MotorOutput.Thrusts);
		}
	}
}

void AUAVActor::UpdatePhysics(float DeltaTime)
{
	if (DynamicsComponent)
	{
		// 更新动力学模型
		CurrentState = DynamicsComponent->UpdateDynamics(CurrentState, DeltaTime);
	}
}
