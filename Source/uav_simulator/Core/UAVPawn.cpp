// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVPawn.h"
#include "../Physics/UAVDynamics.h"
#include "../Sensors/SensorBase.h"
#include "../Control/AttitudeController.h"
#include "../Control/PositionController.h"
#include "../Debug/DebugVisualizer.h"
#include "../Debug/UAVHUD.h"
#include "../Debug/UAVLogConfig.h"
#include "../AI/UAVAIController.h"
#include "../Planning/TrajectoryTracker.h"
#include "../Planning/ObstacleManager.h"
#include "../Planning/PlanningVisualizer.h"
#include "../Utility/Debug.h"
#include "GameFramework/PlayerController.h"

AUAVPawn::AUAVPawn()
{
	PrimaryActorTick.bCanEverTick = true;

	// 创建根组件
	RootSceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootSceneComponent"));
	RootComponent = RootSceneComponent;

	// 创建物理模型组件
	DynamicsComponent = CreateDefaultSubobject<UUAVDynamics>(TEXT("DynamicsComponent"));

	// 创建姿态控制器组件
	AttitudeControllerComponent = CreateDefaultSubobject<UAttitudeController>(TEXT("AttitudeController"));

	// 创建位置控制器组件
	PositionControllerComponent = CreateDefaultSubobject<UPositionController>(TEXT("PositionController"));

	// 创建调试可视化组件
	DebugVisualizerComponent = CreateDefaultSubobject<UDebugVisualizer>(TEXT("DebugVisualizer"));

	// 创建轨迹跟踪组件
	TrajectoryTrackerComponent = CreateDefaultSubobject<UTrajectoryTracker>(TEXT("TrajectoryTracker"));

	// 创建障碍物管理组件
	ObstacleManagerComponent = CreateDefaultSubobject<UObstacleManager>(TEXT("ObstacleManager"));

	// 创建规划可视化组件
	PlanningVisualizerComponent = CreateDefaultSubobject<UPlanningVisualizer>(TEXT("PlanningVisualizer"));

	// 初始化状态
	CurrentState = FUAVState();
	TargetAttitude = FRotator::ZeroRotator;
	TargetPosition = FVector::ZeroVector;
	ControlMode = EUAVControlMode::Position;

	// 设置AI控制器
	AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;
}

void AUAVPawn::BeginPlay()
{
	Super::BeginPlay();

	// 初始化位置和姿态
	CurrentState.Position = GetActorLocation();
	CurrentState.Rotation = GetActorRotation();
	
	// 设置初始目标位置为当前位置
	TargetPosition = CurrentState.Position;
}

void AUAVPawn::Tick(float DeltaTime)
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
		// 绘制UAV状态（坐标轴、速度向量）
		DebugVisualizerComponent->DrawUAVState(CurrentState, GetActorLocation());
		// 绘制飞行轨迹历史
		DebugVisualizerComponent->DrawTrajectoryHistory(GetActorLocation());

		// 绘制轨迹跟踪状态
		if (TrajectoryTrackerComponent && TrajectoryTrackerComponent->IsTracking())
		{
			FTrajectoryPoint DesiredState = TrajectoryTrackerComponent->GetDesiredState();
			DebugVisualizerComponent->DrawTrackingState(DesiredState, GetActorLocation());
		}
	}

	// 更新规划可视化
	if (PlanningVisualizerComponent)
	{
		// 如果有活动轨迹，设置持久化轨迹用于绘制
		if (TrajectoryTrackerComponent && TrajectoryTrackerComponent->IsTracking())
		{
			const FTrajectory& CurrentTrajectory = TrajectoryTrackerComponent->GetTrajectory();
			if (CurrentTrajectory.bIsValid)
			{
				PlanningVisualizerComponent->SetPersistentTrajectory(CurrentTrajectory);
			}
		}

		// 绘制障碍物
		if (ObstacleManagerComponent)
		{
			TArray<FObstacleInfo> Obstacles = ObstacleManagerComponent->GetAllObstacles();
			if (Obstacles.Num() > 0)
			{
				PlanningVisualizerComponent->DrawObstacles(Obstacles);
			}
		}
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

void AUAVPawn::SetTargetAttitude(FRotator InTargetAttitude)
{
	TargetAttitude = InTargetAttitude;
}

void AUAVPawn::SetTargetPosition(FVector InTargetPosition)
{
	TargetPosition = InTargetPosition;
}

bool AUAVPawn::HasReachedTarget(float Tolerance) const
{
	return FVector::Dist(CurrentState.Position, TargetPosition) <= Tolerance;
}

void AUAVPawn::UpdateSensors(float DeltaTime)
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

void AUAVPawn::UpdateController(float DeltaTime)
{
	// 根据控制模式选择不同的控制策略
	switch (ControlMode)
	{
	case EUAVControlMode::Trajectory:
		{
			// 轨迹跟踪模式
			if (TrajectoryTrackerComponent && TrajectoryTrackerComponent->IsTracking() && PositionControllerComponent)
			{
				FTrajectoryPoint DesiredState = TrajectoryTrackerComponent->GetDesiredState();

				// 调试日志：轨迹跟踪状态
				float Progress = TrajectoryTrackerComponent->GetProgress();
				FVector PosError = DesiredState.Position - CurrentState.Position;
				UE_LOG(LogUAVActor, Log, TEXT("[Trajectory] Progress: %.2f%% | DesPos: (%.1f, %.1f, %.1f) | CurPos: (%.1f, %.1f, %.1f) | PosErr: (%.1f, %.1f, %.1f) | ErrMag: %.1f"),
					Progress * 100.0f,
					DesiredState.Position.X, DesiredState.Position.Y, DesiredState.Position.Z,
					CurrentState.Position.X, CurrentState.Position.Y, CurrentState.Position.Z,
					PosError.X, PosError.Y, PosError.Z,
					PosError.Size());

				UE_LOG(LogUAVActor, Log, TEXT("[Trajectory] DesVel: (%.1f, %.1f, %.1f) | CurVel: (%.1f, %.1f, %.1f) | DesAcc: (%.1f, %.1f, %.1f)"),
					DesiredState.Velocity.X, DesiredState.Velocity.Y, DesiredState.Velocity.Z,
					CurrentState.Velocity.X, CurrentState.Velocity.Y, CurrentState.Velocity.Z,
					DesiredState.Acceleration.X, DesiredState.Acceleration.Y, DesiredState.Acceleration.Z);

				FRotator DesiredAttitude;
				float DesiredThrust;

				// 使用轨迹点的速度和加速度作为前馈
				PositionControllerComponent->ComputeControl(
					CurrentState, DesiredState.Position, DesiredState.Velocity, DesiredAttitude, DesiredThrust, DeltaTime);

				UE_LOG(LogUAVActor, Log, TEXT("[Trajectory] DesAttitude: (P:%.2f, Y:%.2f, R:%.2f) | DesThrust: %.3f"),
					DesiredAttitude.Pitch, DesiredAttitude.Yaw, DesiredAttitude.Roll, DesiredThrust);

				if (AttitudeControllerComponent)
				{
					FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControl(
						CurrentState, DesiredAttitude, DeltaTime);

					float HoverThrust = AttitudeControllerComponent->HoverThrust;

					for (int32 i = 0; i < MotorOutput.Thrusts.Num(); i++)
					{
						float ControlDelta = MotorOutput.Thrusts[i] - HoverThrust;
						MotorOutput.Thrusts[i] = DesiredThrust + ControlDelta;
						MotorOutput.Thrusts[i] = FMath::Clamp(MotorOutput.Thrusts[i], 0.0f, 1.0f);
					}

					UE_LOG(LogUAVActor, Log, TEXT("[Trajectory] Motors: [%.3f, %.3f, %.3f, %.3f] | HoverThrust: %.3f"),
						MotorOutput.Thrusts[0], MotorOutput.Thrusts[1], MotorOutput.Thrusts[2], MotorOutput.Thrusts[3], HoverThrust);

					if (DynamicsComponent)
					{
						DynamicsComponent->SetMotorThrusts(MotorOutput.Thrusts);
					}
				}
			}
			else
			{
				// 轨迹跟踪完成或未激活，切换到位置保持
				UE_LOG(LogUAVActor, Log, TEXT("[Trajectory] Holding position - Tracker: %s, IsTracking: %s"),
					TrajectoryTrackerComponent ? TEXT("Valid") : TEXT("Null"),
					(TrajectoryTrackerComponent && TrajectoryTrackerComponent->IsTracking()) ? TEXT("Yes") : TEXT("No"));

				if (PositionControllerComponent && AttitudeControllerComponent)
				{
					FRotator DesiredAttitude;
					float DesiredThrust;

					PositionControllerComponent->ComputeControl(
						CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, DesiredThrust, DeltaTime);

					UE_LOG(LogUAVActor, Log, TEXT("[Trajectory] HoldPos: (%.1f, %.1f, %.1f) | CurPos: (%.1f, %.1f, %.1f) | Err: %.1f"),
						TargetPosition.X, TargetPosition.Y, TargetPosition.Z,
						CurrentState.Position.X, CurrentState.Position.Y, CurrentState.Position.Z,
						FVector::Dist(TargetPosition, CurrentState.Position));

					FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControl(
						CurrentState, DesiredAttitude, DeltaTime);

					float HoverThrust = AttitudeControllerComponent->HoverThrust;

					for (int32 i = 0; i < MotorOutput.Thrusts.Num(); i++)
					{
						float ControlDelta = MotorOutput.Thrusts[i] - HoverThrust;
						MotorOutput.Thrusts[i] = DesiredThrust + ControlDelta;
						MotorOutput.Thrusts[i] = FMath::Clamp(MotorOutput.Thrusts[i], 0.0f, 1.0f);
					}

					if (DynamicsComponent)
					{
						DynamicsComponent->SetMotorThrusts(MotorOutput.Thrusts);
					}
				}
			}
		}
		break;

	case EUAVControlMode::Position:
		{
			// 位置控制模式
			if (PositionControllerComponent)
			{
				FRotator DesiredAttitude;
				float DesiredThrust;

				PositionControllerComponent->ComputeControl(
					CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, DesiredThrust, DeltaTime);

				if (AttitudeControllerComponent)
				{
					FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControl(
						CurrentState, DesiredAttitude, DeltaTime);

					float HoverThrust = AttitudeControllerComponent->HoverThrust;

					// UE_LOG(LogUAVActor, Log, TEXT("AttCtrl Raw: [%.3f, %.3f, %.3f, %.3f] | Hover: %.3f | DesThrust: %.3f"),
					// 	MotorOutput.Thrusts[0], MotorOutput.Thrusts[1], MotorOutput.Thrusts[2], MotorOutput.Thrusts[3],
					// 	HoverThrust, DesiredThrust);

					for (int32 i = 0; i < MotorOutput.Thrusts.Num(); i++)
					{
						float ControlDelta = MotorOutput.Thrusts[i] - HoverThrust;
						MotorOutput.Thrusts[i] = DesiredThrust + ControlDelta;
						MotorOutput.Thrusts[i] = FMath::Clamp(MotorOutput.Thrusts[i], 0.0f, 1.0f);
					}

					// UE_LOG(LogUAVActor, Log, TEXT("Final Motors: [%.3f, %.3f, %.3f, %.3f]"),
					// 	MotorOutput.Thrusts[0], MotorOutput.Thrusts[1], MotorOutput.Thrusts[2], MotorOutput.Thrusts[3]);

					if (DynamicsComponent)
					{
						DynamicsComponent->SetMotorThrusts(MotorOutput.Thrusts);
					}
				}
			}
		}
		break;

	case EUAVControlMode::Attitude:
	default:
		{
			// 姿态控制模式
			if (AttitudeControllerComponent)
			{
				FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControl(
					CurrentState, TargetAttitude, DeltaTime);

				if (DynamicsComponent)
				{
					DynamicsComponent->SetMotorThrusts(MotorOutput.Thrusts);
				}
			}
		}
		break;
	}
}

void AUAVPawn::UpdatePhysics(float DeltaTime)
{
	if (DynamicsComponent)
	{
		CurrentState = DynamicsComponent->UpdateDynamics(CurrentState, DeltaTime);
	}
}

void AUAVPawn::SetTrajectory(const FTrajectory& InTrajectory)
{
	if (TrajectoryTrackerComponent)
	{
		TrajectoryTrackerComponent->SetTrajectory(InTrajectory);

		// 如果轨迹有效，设置最终位置为目标位置
		if (InTrajectory.bIsValid && InTrajectory.Points.Num() > 0)
		{
			TargetPosition = InTrajectory.Points.Last().Position;
		}
	}
}

void AUAVPawn::StartTrajectoryTracking()
{
	if (TrajectoryTrackerComponent)
	{
		UE_LOG(LogUAVActor, Log, TEXT("Starting trajectory tracking."));
		ControlMode = EUAVControlMode::Trajectory;
		TrajectoryTrackerComponent->StartTracking();
	}
}

void AUAVPawn::StopTrajectoryTracking()
{
	if (TrajectoryTrackerComponent)
	{
		UE_LOG(LogUAVActor, Log, TEXT("Stopping trajectory tracking."));
		// UDebug::PrintCallStack();
		TrajectoryTrackerComponent->StopTracking();
		ControlMode = EUAVControlMode::Position;
	}
}

float AUAVPawn::GetTrajectoryProgress() const
{
	if (TrajectoryTrackerComponent)
	{
		return TrajectoryTrackerComponent->GetProgress();
	}
	return 0.0f;
}

bool AUAVPawn::IsTrajectoryComplete() const
{
	if (TrajectoryTrackerComponent)
	{
		return TrajectoryTrackerComponent->IsComplete();
	}
	return true;
}

void AUAVPawn::SetControlMode(EUAVControlMode NewMode)
{
	ControlMode = NewMode;

	// 同步 bUsePositionControl 以保持向后兼容
	bUsePositionControl = (NewMode == EUAVControlMode::Position || NewMode == EUAVControlMode::Trajectory);

	// 如果从轨迹模式切换出去，停止轨迹跟踪
	if (NewMode != EUAVControlMode::Trajectory && TrajectoryTrackerComponent)
	{
		TrajectoryTrackerComponent->StopTracking();
	}
}
