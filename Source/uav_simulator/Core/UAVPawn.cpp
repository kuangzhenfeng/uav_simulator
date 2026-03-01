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
#include "../Utility/Filter.h"
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

	// 同步模型速度限制到 NMPC，并增大参考跟踪权重以限制避障偏差
	{
		const FUAVModelSpec Spec = FUAVProductManager::GetModelSpec(ModelID);
		NMPCComponent->Config.MaxVelocity = Spec.MaxVelocity;
		NMPCComponent->Config.WeightReference = 2.5f;  // 默认 0.5，增大以限制避障偏差
		NMPCComponent->Config.WeightTerminal = 3.0f;    // 默认 2.0，增大终端代价收敛
		NMPCComponent->Config.WeightObstacle = 3.0f;    // 增大避障权重，与参考跟踪匹配
		NMPCComponent->Config.ObstacleSafeDistance = 250.0f;  // 默认 150，增大安全距离缓冲
		NMPCComponent->Config.ObstacleInfluenceDistance = 1500.0f; // 默认 1000，增大感知范围
		NMPCComponent->Config.ObstacleAlpha = 0.9f;  // 默认 0.5，增大势垒陡度，安全距离边界处梯度更强
	}

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

// ========== NMPC 求解相关辅助方法 ==========

bool AUAVPawn::ShouldSolveNMPC(float DeltaTime)
{
	const float NMPCSolveInterval = 0.1f;
	NMPCSolveAccumulator += DeltaTime;

	if (StuckEscapeCooldown > 0.0f)
	{
		StuckEscapeCooldown -= DeltaTime;
		NMPCSolveAccumulator = 0.0f;
		return false;
	}

	if (NMPCSolveAccumulator >= NMPCSolveInterval || !bHasCachedNMPC)
	{
		NMPCSolveAccumulator = 0.0f;
		return true;
	}

	return false;
}

void AUAVPawn::PrepareReferencePoints(TArray<FVector>& OutReferencePoints)
{
	const int32 N = NMPCComponent->Config.PredictionSteps;
	const float Dt = NMPCComponent->Config.GetDt();
	const float CurrentTime = TrajectoryTrackerComponent->GetCurrentTime();

	for (int32 i = 0; i <= N; ++i)
	{
		OutReferencePoints.Add(TrajectoryTrackerComponent->GetDesiredState(CurrentTime + i * Dt).Position);
	}

	// 参考点退化检测
	if (OutReferencePoints.Num() > 1)
	{
		FVector EndRef = OutReferencePoints.Last();
		bool bAllSame = true;
		for (int32 i = 0; i < OutReferencePoints.Num() - 1; ++i)
		{
			if (!OutReferencePoints[i].Equals(EndRef, 10.0f))
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
				OutReferencePoints[i] = FMath::Lerp(CurrentState.Position, EndRef, Alpha);
			}
		}
	}
}

void AUAVPawn::FilterNearbyObstacles(TArray<FObstacleInfo>& OutObstacles)
{
	if (!ObstacleManagerComponent)
	{
		return;
	}

	for (const FObstacleInfo& Obs : ObstacleManagerComponent->GetObstaclesInRange(
		CurrentState.Position, NMPCComponent->Config.ObstacleInfluenceDistance * 2.0f))
	{
		if (Obs.Extents.GetMax() <= 5000.0f)
		{
			OutObstacles.Add(Obs);
		}
	}
}

void AUAVPawn::FixReferencePointsPenetratingObstacles(
	TArray<FVector>& ReferencePoints,
	const TArray<FObstacleInfo>& Obstacles)
{
	for (FVector& RefPt : ReferencePoints)
	{
		for (const FObstacleInfo& Obs : Obstacles)
		{
			float Dist = NMPCComponent->CalculateDistanceToObstacle(RefPt, Obs);
			if (Dist < NMPCComponent->Config.ObstacleSafeDistance)
			{
				FVector PushDir = (RefPt - Obs.Center).GetSafeNormal();
				if (PushDir.IsNearlyZero())
				{
					PushDir = FVector::UpVector;
				}
				RefPt += PushDir * (NMPCComponent->Config.ObstacleSafeDistance - Dist);
			}
		}
	}
}

void AUAVPawn::SolveNMPCAvoidance(float DeltaTime)
{
	TArray<FVector> ReferencePoints;
	PrepareReferencePoints(ReferencePoints);

	TArray<FObstacleInfo> NearbyObstacles;
	FilterNearbyObstacles(NearbyObstacles);

	FixReferencePointsPenetratingObstacles(ReferencePoints, NearbyObstacles);

	FNMPCAvoidanceResult Result = NMPCComponent->ComputeAvoidance(
		CurrentState.Position, CurrentState.Velocity, ReferencePoints, NearbyObstacles);

	bNMPCStuck = Result.bStuck;
	CachedNMPCAcceleration = Result.OptimalAcceleration;
	CachedNMPCCorrectedTarget = Result.CorrectedTarget;
	bHasCachedNMPC = true;
	bHasCachedNMPCTarget = true;

	// 缓存最近障碍物距离
	CachedNearestObsDist = MAX_FLT;
	for (const FObstacleInfo& Obs : NearbyObstacles)
	{
		float D = NMPCComponent->CalculateDistanceToObstacle(CurrentState.Position, Obs);
		CachedNearestObsDist = FMath::Min(CachedNearestObsDist, D);
	}

	if (Result.bStuck)
	{
		HandleStuckEscape(Result, NearbyObstacles);
	}
}

// ========== 逃逸逻辑辅助方法 ==========

FVector AUAVPawn::CalculateEscapeDirection(const TArray<FObstacleInfo>& NearbyObstacles)
{
	float NearestD = MAX_FLT;
	FVector NearestObsCenter = FVector::ZeroVector;

	for (const FObstacleInfo& Obs : NearbyObstacles)
	{
		float D = NMPCComponent->CalculateDistanceToObstacle(CurrentState.Position, Obs);
		if (D < NearestD)
		{
			NearestD = D;
			NearestObsCenter = Obs.Center;
		}
	}

	FVector AwayFromObs = (CurrentState.Position - NearestObsCenter).GetSafeNormal();
	FVector DesiredPos = TrajectoryTrackerComponent->GetDesiredState().Position;
	FVector ToRef = (DesiredPos - CurrentState.Position).GetSafeNormal();

	float ObsWeight = FMath::Clamp(1.0f - NearestD / NMPCComponent->Config.ObstacleSafeDistance, 0.0f, 1.0f);
	return (AwayFromObs * ObsWeight + ToRef * (1.0f - ObsWeight)).GetSafeNormal();
}

float AUAVPawn::CalculateEscapeAcceleration(float NearestObsDist)
{
	float CurrentSpeed = CurrentState.Velocity.Size();
	float BaseEscapeAccel = 300.0f;
	float SpeedFactor = FMath::Clamp(1.0f - CurrentSpeed / 500.0f, 0.3f, 1.0f);
	float DistFactor = FMath::Clamp(1.0f - NearestObsDist / NMPCComponent->Config.ObstacleSafeDistance, 0.5f, 1.5f);
	return BaseEscapeAccel * SpeedFactor * DistFactor;
}

void AUAVPawn::HandleStuckEscape(
	const FNMPCAvoidanceResult& Result,
	const TArray<FObstacleInfo>& NearbyObstacles)
{
	StuckEscapeCooldown = 0.3f;

	FVector EscapeDir = CalculateEscapeDirection(NearbyObstacles);
	if (!EscapeDir.IsNearlyZero())
	{
		float EscapeAccel = CalculateEscapeAcceleration(CachedNearestObsDist);
		CachedNMPCAcceleration = EscapeDir * EscapeAccel;
	}
}

// ========== 偏差保护辅助方法 ==========

FVector AUAVPawn::LimitLateralAcceleration(const FVector& Acceleration)
{
	FVector Result = Acceleration;
	float MaxLateralAccel = 300.0f;

	if (CachedNearestObsDist < NMPCComponent->Config.ObstacleSafeDistance * 2.0f)
	{
		MaxLateralAccel = 400.0f;
	}

	float LateralMag = FMath::Sqrt(Result.Y * Result.Y + Result.Z * Result.Z);
	if (LateralMag > MaxLateralAccel)
	{
		float Scale = MaxLateralAccel / LateralMag;
		Result.Y *= Scale;
		Result.Z *= Scale;
	}

	return Result;
}

FVector AUAVPawn::ApplyPDCorrection(const FVector& Acceleration)
{
	FVector Result = Acceleration;
	FVector DesiredPos = TrajectoryTrackerComponent->GetDesiredState().Position;

	float YError = DesiredPos.Y - CurrentState.Position.Y;
	float ZError = DesiredPos.Z - CurrentState.Position.Z;

	constexpr float Kp = 1.2f;
	constexpr float Kd = 1.5f;
	float YVelError = -CurrentState.Velocity.Y;
	float ZVelError = -CurrentState.Velocity.Z;

	Result.Y += Kp * YError + Kd * YVelError;
	Result.Z += Kp * ZError + Kd * ZVelError;

	return Result;
}

FVector AUAVPawn::ApplyHardLimitCorrection(const FVector& Acceleration, float CrossTrackDev)
{
	FVector Result = Acceleration;
	FVector DesiredPos = TrajectoryTrackerComponent->GetDesiredState().Position;

	float YError = DesiredPos.Y - CurrentState.Position.Y;
	float ZError = DesiredPos.Z - CurrentState.Position.Z;

	float HardLimit = 120.0f;
	if (CachedNearestObsDist < NMPCComponent->Config.ObstacleSafeDistance * 3.0f)
	{
		float ProximityRatio = FMath::Clamp(
			1.0f - CachedNearestObsDist / (NMPCComponent->Config.ObstacleSafeDistance * 3.0f), 0.0f, 1.0f);
		HardLimit = FMath::Lerp(120.0f, 130.0f, ProximityRatio);
	}

	if (CrossTrackDev > HardLimit)
	{
		FVector CorrDir(0.0f, YError, ZError);
		CorrDir = CorrDir.GetSafeNormal();
		constexpr float Kd = 1.5f;
		float YVelError = -CurrentState.Velocity.Y;
		float ZVelError = -CurrentState.Velocity.Z;
		Result.Y = CorrDir.Y * 600.0f + Kd * YVelError;
		Result.Z = CorrDir.Z * 600.0f + Kd * ZVelError;
	}

	return Result;
}

void AUAVPawn::UpdateSpeedScaleForObstacles()
{
	const float ObsSafe = NMPCComponent->Config.ObstacleSafeDistance;
	const float ObsInfluence = NMPCComponent->Config.ObstacleInfluenceDistance;
	float SpeedScale = 1.0f;

	if (CachedNearestObsDist < ObsInfluence)
	{
		SpeedScale = FMath::Lerp(0.35f, 1.0f,
			FMath::Clamp((CachedNearestObsDist - ObsSafe) / (ObsInfluence - ObsSafe), 0.0f, 1.0f));
	}

	TrajectoryTrackerComponent->SetSpeedScale(SpeedScale);
}

FVector AUAVPawn::ApplyDeviationProtection(const FVector& NMPCAcceleration)
{
	FVector DesiredPos = TrajectoryTrackerComponent->GetDesiredState().Position;

	float YError = DesiredPos.Y - CurrentState.Position.Y;
	float ZError = DesiredPos.Z - CurrentState.Position.Z;
	float CrossTrackDev = FMath::Sqrt(YError * YError + ZError * ZError);

	if (CrossTrackDev > 300.0f)
	{
		UE_LOG_THROTTLE(0.5, LogUAVActor, Warning,
			TEXT("[DevTrack] Cross=%.0f Pos=(%d,%d,%d)"),
			CrossTrackDev,
			(int)CurrentState.Position.X, (int)CurrentState.Position.Y, (int)CurrentState.Position.Z);
	}

	FVector EffectiveAccel = LimitLateralAcceleration(NMPCAcceleration);
	EffectiveAccel = ApplyPDCorrection(EffectiveAccel);
	EffectiveAccel = ApplyHardLimitCorrection(EffectiveAccel, CrossTrackDev);

	UpdateSpeedScaleForObstacles();

	return EffectiveAccel;
}

// ========== 速度钳位辅助方法 ==========

FVector AUAVPawn::ApplyVelocityClamp(const FVector& Acceleration)
{
	FVector Result = Acceleration;
	float MaxVel = PositionControllerComponent->MaxVelocity;
	float CurSpeed = CurrentState.Velocity.Size();

	if (CurSpeed > MaxVel * 0.9f && CurSpeed > KINDA_SMALL_NUMBER)
	{
		FVector VelDir = CurrentState.Velocity / CurSpeed;
		float AlongVel = FVector::DotProduct(Result, VelDir);
		if (AlongVel > 0.0f)
		{
			float Ratio = FMath::Clamp((CurSpeed - MaxVel * 0.9f) / (MaxVel * 0.1f), 0.0f, 1.0f);
			Result -= VelDir * AlongVel * Ratio;
		}
	}

	return Result;
}

// ========== 位置保持辅助方法 ==========

FVector AUAVPawn::GetTrajectoryEndpointOrTarget()
{
	FVector HoldTarget = TargetPosition;
	if (TrajectoryTrackerComponent)
	{
		const FTrajectory& Traj = TrajectoryTrackerComponent->GetTrajectory();
		if (Traj.bIsValid && Traj.Points.Num() > 0)
		{
			HoldTarget = Traj.Points.Last().Position;
		}
	}
	return HoldTarget;
}

void AUAVPawn::ExecutePositionHold()
{
	FVector HoldTarget = GetTrajectoryEndpointOrTarget();

	if (PositionControllerComponent && AttitudeControllerComponent)
	{
		FRotator DesiredAttitude;
		float DesiredThrust;
		PositionControllerComponent->ComputeControl(
			CurrentState, HoldTarget, FVector::ZeroVector, DesiredAttitude, DesiredThrust, 0.02f);

		FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControl(
			CurrentState, DesiredAttitude, 0.02f);

		float HoverThrust = AttitudeControllerComponent->HoverThrust;
		for (int32 i = 0; i < MotorOutput.Thrusts.Num(); i++)
		{
			MotorOutput.Thrusts[i] = FMath::Clamp(
				DesiredThrust + (MotorOutput.Thrusts[i] - HoverThrust), 0.0f, 1.0f);
		}

		if (DynamicsComponent)
		{
			DynamicsComponent->SetMotorThrusts(MotorOutput.Thrusts);
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
			// Early return: 组件未就绪或未跟踪
			if (!TrajectoryTrackerComponent || !TrajectoryTrackerComponent->IsTracking()
				|| !PositionControllerComponent || !AttitudeControllerComponent || !NMPCComponent)
			{
				ExecutePositionHold();
				break;
			}

			// NMPC 求解
			if (ShouldSolveNMPC(DeltaTime))
			{
				SolveNMPCAvoidance(DeltaTime);
			}

			// 应用偏差保护和速度钳位
			FVector EffectiveAccel = ApplyDeviationProtection(CachedNMPCAcceleration);
			EffectiveAccel = ApplyVelocityClamp(EffectiveAccel);

			// 加速度 → 姿态+推力 → 电机
			FRotator DesiredAttitude;
			float DesiredThrust;
			PositionControllerComponent->AccelerationToControl(
				EffectiveAccel, CurrentState.Rotation.Yaw, DesiredAttitude, DesiredThrust);

			FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControl(
				CurrentState, DesiredAttitude, DeltaTime);

			float HoverThrust = AttitudeControllerComponent->HoverThrust;
			for (int32 i = 0; i < MotorOutput.Thrusts.Num(); i++)
			{
				MotorOutput.Thrusts[i] = FMath::Clamp(
					DesiredThrust + (MotorOutput.Thrusts[i] - HoverThrust), 0.0f, 1.0f);
			}

			if (DynamicsComponent)
			{
				DynamicsComponent->SetMotorThrusts(MotorOutput.Thrusts);
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


