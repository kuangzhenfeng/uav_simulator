// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVActor.h"
#include "../Physics/UAVDynamics.h"
#include "../Sensors/SensorBase.h"
#include "../Control/AttitudeController.h"
#include "../Control/PositionController.h"
#include "../Debug/DebugVisualizer.h"
#include "../Debug/UAVHUD.h"
#include "../Debug/UAVLogConfig.h"
#include "GameFramework/PlayerController.h"

AUAVActor::AUAVActor()
{
	PrimaryActorTick.bCanEverTick = true;

	// 创建物理模型组件
	DynamicsComponent = CreateDefaultSubobject<UUAVDynamics>(TEXT("DynamicsComponent"));

	// 创建姿态控制器组件
	AttitudeControllerComponent = CreateDefaultSubobject<UAttitudeController>(TEXT("AttitudeController"));

	// 创建位置控制器组件
	PositionControllerComponent = CreateDefaultSubobject<UPositionController>(TEXT("PositionController"));

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

	// 更新HUD显示
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{
		if (AUAVHUD* UAVHUDInstance = Cast<AUAVHUD>(PC->GetHUD()))
		{
			UAVHUDInstance->SetUAVState(CurrentState);
			if (DynamicsComponent)
			{
				UAVHUDInstance->SetMotorThrusts(DynamicsComponent->GetMotorThrusts());
			}
			// 设置控制器参数用于HUD显示
			UAVHUDInstance->SetControllerParams(AttitudeControllerComponent, PositionControllerComponent);
		}
	}
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
	if (bUsePositionControl && PositionControllerComponent)
	{
		// 位置控制模式
		FRotator DesiredAttitude;
		float DesiredThrust;

		// 位置控制器计算期望姿态和推力
		PositionControllerComponent->ComputeControl(
			CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, DesiredThrust, DeltaTime);

		// 使用姿态控制器跟踪期望姿态
		if (AttitudeControllerComponent)
		{
			FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControl(
				CurrentState, DesiredAttitude, DeltaTime);

			// 用位置控制器的推力替换姿态控制器的基础推力
			// 计算姿态控制器的控制增量（相对于HoverThrust的偏差）
			float HoverThrust = AttitudeControllerComponent->HoverThrust;
			
			// 调试日志：输出姿态控制器原始输出
			// UE_LOG(LogUAVActor, Log, TEXT("AttCtrl Raw: [%.3f, %.3f, %.3f, %.3f] | Hover: %.3f | DesThrust: %.3f"),
			// 	MotorOutput.Thrusts[0], MotorOutput.Thrusts[1], MotorOutput.Thrusts[2], MotorOutput.Thrusts[3],
			// 	HoverThrust, DesiredThrust);
			
			// 计算控制增量并应用到期望推力
			for (int32 i = 0; i < MotorOutput.Thrusts.Num(); i++)
			{
				float ControlDelta = MotorOutput.Thrusts[i] - HoverThrust;
				MotorOutput.Thrusts[i] = DesiredThrust + ControlDelta;
				// 确保推力在有效范围内
				MotorOutput.Thrusts[i] = FMath::Clamp(MotorOutput.Thrusts[i], 0.0f, 1.0f);
			}

			// 调试日志：输出最终电机推力
			// UE_LOG(LogUAVActor, Log, TEXT("Final Motors: [%.3f, %.3f, %.3f, %.3f]"),
			// 	MotorOutput.Thrusts[0], MotorOutput.Thrusts[1], MotorOutput.Thrusts[2], MotorOutput.Thrusts[3]);

			// 将电机输出传递给物理模型
			if (DynamicsComponent)
			{
				DynamicsComponent->SetMotorThrusts(MotorOutput.Thrusts);
			}
		}
	}
	else if (AttitudeControllerComponent)
	{
		// 姿态控制模式
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
