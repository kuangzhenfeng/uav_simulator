// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVPawn.h"
#include "UAVProductManager.h"
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
#include "../Mission/MissionComponent.h"
#include "../Sensors/ObstacleDetector.h"
#include "../Debug/StabilityScorer.h"
#include "../Debug/ControlParameterTuner.h"
#include "../Planning/NMPCAvoidance.h"
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

	// 创建任务管理组件
	MissionComponent = CreateDefaultSubobject<UMissionComponent>(TEXT("MissionComponent"));

	// 创建障碍物感知传感器组件
	ObstacleDetectorComponent = CreateDefaultSubobject<UObstacleDetector>(TEXT("ObstacleDetector"));

	// 创建稳定性评分组件
	StabilityScorerComponent = CreateDefaultSubobject<UStabilityScorer>(TEXT("StabilityScorer"));

	// 创建PID调参组件
	ParameterTunerComponent = CreateDefaultSubobject<UControlParameterTuner>(TEXT("ParameterTuner"));

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

	// 按型号初始化物理/控制参数
	{
		const FUAVModelSpec Spec = FUAVProductManager::GetModelSpec(ModelID);
		DynamicsComponent->SetPhysicsParams(Spec.Mass, Spec.ArmLength, Spec.MomentOfInertia, Spec.MaxThrust);
		AttitudeControllerComponent->HoverThrust = Spec.HoverThrust;
		AttitudeControllerComponent->RollPID     = Spec.RollPID;
		AttitudeControllerComponent->PitchPID    = Spec.PitchPID;
		PositionControllerComponent->MaxVelocity          = Spec.MaxVelocity;
		PositionControllerComponent->UAVMass              = Spec.Mass;
		PositionControllerComponent->SingleMotorMaxThrust = Spec.MaxThrust;
		UE_LOG(LogUAVActor, Log, TEXT("[Model] %s | Mass=%.1fkg | MaxThrust=%.1fN | MaxVel=%.0fcm/s"),
			*Spec.ModelName, Spec.Mass, Spec.MaxThrust, Spec.MaxVelocity);
	}

	// 接线PID调参组件
	if (ParameterTunerComponent)
	{
		ParameterTunerComponent->SetAttitudeController(AttitudeControllerComponent);
		ParameterTunerComponent->SetPositionController(PositionControllerComponent);
	}

	NMPCComponent = NewObject<UNMPCAvoidance>(this);
}

void AUAVPawn::SetPayloadMass(float NewPayloadMass)
{
	const FUAVModelSpec Spec = FUAVProductManager::GetModelSpec(ModelID);
	CurrentPayloadMass = FMath::Clamp(NewPayloadMass, 0.0f, Spec.MaxPayloadKg);
	const float TotalMass = Spec.Mass + CurrentPayloadMass;
	DynamicsComponent->SetPhysicsParams(TotalMass, Spec.ArmLength, Spec.MomentOfInertia, Spec.MaxThrust);
	PositionControllerComponent->UAVMass = TotalMass;
	AttitudeControllerComponent->HoverThrust = Spec.HoverThrust * TotalMass / Spec.Mass;
}

void AUAVPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// 子步积分：将大帧拆成固定 0.02s 步长，保证物理精度的同时支持时间加速
	const float FixedStep = 0.02f;
	float Remaining = FMath::Min(DeltaTime, 1.0f); // 最多1秒防止卡顿帧
	while (Remaining > KINDA_SMALL_NUMBER)
	{
		const float Step = FMath::Min(Remaining, FixedStep);
		UpdateSensors(Step);
		UpdateController(Step);
		UpdatePhysics(Step);
		SetActorLocation(CurrentState.Position);
		SetActorRotation(CurrentState.Rotation);
		Remaining -= Step;
	}

	// 更新稳定性评分
	if (StabilityScorerComponent)
	{
		StabilityScorerComponent->UpdateFlightScore(CurrentState, ObstacleManagerComponent);
		StabilityScorerComponent->DrawScoreHUD(GetActorLocation());
	}

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
			if (!TrajectoryTrackerComponent || !TrajectoryTrackerComponent->IsTracking()
				|| !PositionControllerComponent || !AttitudeControllerComponent || !NMPCComponent)
			{
				// 位置保持
				if (PositionControllerComponent && AttitudeControllerComponent)
				{
					FRotator DesiredAttitude;
					float DesiredThrust;
					PositionControllerComponent->ComputeControl(
						CurrentState, TargetPosition, FVector::ZeroVector, DesiredAttitude, DesiredThrust, DeltaTime);
					FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControl(
						CurrentState, DesiredAttitude, DeltaTime);
					float HoverThrust = AttitudeControllerComponent->HoverThrust;
					for (int32 i = 0; i < MotorOutput.Thrusts.Num(); i++)
						MotorOutput.Thrusts[i] = FMath::Clamp(
							DesiredThrust + (MotorOutput.Thrusts[i] - HoverThrust), 0.0f, 1.0f);
					if (DynamicsComponent) DynamicsComponent->SetMotorThrusts(MotorOutput.Thrusts);
				}
				break;
			}

			// NMPC 调用节流：每 0.1s 求解一次，子步间复用缓存加速度
			// stuck 逃逸冷却期间跳过求解，让逃逸加速度持续生效
			const float NMPCSolveInterval = 0.1f;
			NMPCSolveAccumulator += DeltaTime;
			if (StuckEscapeCooldown > 0.0f)
			{
				StuckEscapeCooldown -= DeltaTime;
				NMPCSolveAccumulator = 0.0f; // 冷却期间不累积
			}
			else if (NMPCSolveAccumulator >= NMPCSolveInterval || !bHasCachedNMPC)
			{
				NMPCSolveAccumulator = 0.0f;

				// 采样参考轨迹点
				TArray<FVector> ReferencePoints;
				const int32 N = NMPCComponent->Config.PredictionSteps;
				const float Dt = NMPCComponent->Config.GetDt();
				const float CurrentTime = TrajectoryTrackerComponent->GetCurrentTime();
				for (int32 i = 0; i <= N; ++i)
					ReferencePoints.Add(TrajectoryTrackerComponent->GetDesiredState(CurrentTime + i * Dt).Position);

				// 获取附近障碍物（过滤超大地形障碍物，extents > 5000cm 视为地面/天花板平面）
				TArray<FObstacleInfo> NearbyObstacles;
				if (ObstacleManagerComponent)
				{
					for (const FObstacleInfo& Obs : ObstacleManagerComponent->GetObstaclesInRange(
						CurrentState.Position, NMPCComponent->Config.ObstacleInfluenceDistance * 2.0f))
					{
						if (Obs.Extents.GetMax() <= 5000.0f)
							NearbyObstacles.Add(Obs);
					}
				}

				// 参考点退化检测
				if (ReferencePoints.Num() > 1)
				{
					FVector EndRef = ReferencePoints.Last();
					bool bAllSame = true;
					for (int32 i = 0; i < ReferencePoints.Num() - 1; ++i)
					{
						if (!ReferencePoints[i].Equals(EndRef, 10.0f))
						{
							bAllSame = false;
							break;
						}
					}
					if (bAllSame && FVector::Dist(CurrentState.Position, EndRef) > 50.0f)
					{
						for (int32 i = 0; i <= N; ++i)
						{
							float Alpha = (float)i / FMath::Max(N, 1);
							ReferencePoints[i] = FMath::Lerp(CurrentState.Position, EndRef, Alpha);
						}
					}
				}

				// 修正穿过障碍物的参考点
				for (FVector& RefPt : ReferencePoints)
					for (const FObstacleInfo& Obs : NearbyObstacles)
					{
						float Dist = NMPCComponent->CalculateDistanceToObstacle(RefPt, Obs);
						if (Dist < NMPCComponent->Config.ObstacleSafeDistance)
						{
							FVector PushDir = (RefPt - Obs.Center).GetSafeNormal();
							if (PushDir.IsNearlyZero()) PushDir = FVector::UpVector;
							RefPt += PushDir * (NMPCComponent->Config.ObstacleSafeDistance - Dist);
						}
					}

				// NMPC 求解
				FNMPCAvoidanceResult Result = NMPCComponent->ComputeAvoidance(
					CurrentState.Position, CurrentState.Velocity, ReferencePoints, NearbyObstacles);
				bNMPCStuck = Result.bStuck;
				CachedNMPCAcceleration = Result.OptimalAcceleration;
				bHasCachedNMPC = true;
				if (Result.bStuck)
				{
					StuckEscapeCooldown = 0.3f;
					// 沿参考轨迹方向施加逃逸加速度，打破局部最优
					FVector DesiredPos = TrajectoryTrackerComponent->GetDesiredState().Position;
					FVector EscapeDir = (DesiredPos - CurrentState.Position).GetSafeNormal();
					if (!EscapeDir.IsNearlyZero())
						CachedNMPCAcceleration = EscapeDir * 400.0f;
				}
			}

			// 偏差保护：偏离参考轨迹过远时混合飞回加速度（不完全覆盖 NMPC）
			{
				FVector DesiredPos = TrajectoryTrackerComponent->GetDesiredState().Position;
				float Deviation = FVector::Dist(CurrentState.Position, DesiredPos);
				if (Deviation > 2000.0f)
				{
					FVector FlyBackDir = (DesiredPos - CurrentState.Position).GetSafeNormal();
					CachedNMPCAcceleration = FlyBackDir * 200.0f;
				}
			}

			// 加速度 → 姿态+推力 → 电机
			FRotator DesiredAttitude;
			float DesiredThrust;
			PositionControllerComponent->AccelerationToControl(
				CachedNMPCAcceleration, CurrentState.Rotation.Yaw, DesiredAttitude, DesiredThrust);
			FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControl(
				CurrentState, DesiredAttitude, DeltaTime);
			float HoverThrust = AttitudeControllerComponent->HoverThrust;
			for (int32 i = 0; i < MotorOutput.Thrusts.Num(); i++)
				MotorOutput.Thrusts[i] = FMath::Clamp(
					DesiredThrust + (MotorOutput.Thrusts[i] - HoverThrust), 0.0f, 1.0f);
			if (DynamicsComponent) DynamicsComponent->SetMotorThrusts(MotorOutput.Thrusts);
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

	// 如果从轨迹模式切换出去，停止轨迹跟踪
	if (NewMode != EUAVControlMode::Trajectory && TrajectoryTrackerComponent)
	{
		TrajectoryTrackerComponent->StopTracking();
	}
}

void AUAVPawn::SetWaypoints(const TArray<FVector>& InWaypoints)
{
	// 转发到 MissionComponent
	if (MissionComponent)
	{
		MissionComponent->SetWaypoints(InWaypoints);
	}

	// 保持向后兼容
	Waypoints = InWaypoints;
	UE_LOG(LogUAVActor, Log, TEXT("SetWaypoints: %d waypoints set (deprecated, use MissionComponent)"), Waypoints.Num());
}

TArray<FVector> AUAVPawn::GetWaypoints() const
{
	// 优先从 MissionComponent 获取
	if (MissionComponent)
	{
		return MissionComponent->GetWaypointPositions();
	}
	return Waypoints;
}

bool AUAVPawn::HasWaypoints() const
{
	// 优先从 MissionComponent 获取
	if (MissionComponent)
	{
		return MissionComponent->HasWaypoints();
	}
	return Waypoints.Num() > 0;
}

void AUAVPawn::ClearWaypoints()
{
	// 转发到 MissionComponent
	if (MissionComponent)
	{
		MissionComponent->ClearWaypoints();
	}
	Waypoints.Empty();
}


