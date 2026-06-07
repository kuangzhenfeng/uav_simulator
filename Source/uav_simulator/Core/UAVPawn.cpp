// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVPawn.h"
#include "../uav_simulator.h"
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
#include "../Planning/LinearMPCAvoidance.h"
#include "../MultiAgent/AgentCommunicationComponent.h"
#include "../MultiAgent/FormationComponent.h"
#include "../MultiAgent/CBFQPFilter.h"
#include "../MultiAgent/AgentManager.h"
#include "../Utility/Filter.h"
#include "../Utility/Debug.h"
#include "../Environment/WindField.h"
#include "../Sensors/BarometerSensor.h"
#include "../Sensors/MagnetometerSensor.h"
#include "../Sensors/AnemometerSensor.h"
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

	// 创建多机通信组件
	CommunicationComponent = CreateDefaultSubobject<UAgentCommunicationComponent>(TEXT("CommunicationComponent"));

	// 创建编队控制组件
	FormationComponent = CreateDefaultSubobject<UFormationComponent>(TEXT("FormationComponent"));

	// 创建风场组件（Phase 14: 环境模拟）
	WindFieldComponent = CreateDefaultSubobject<UWindField>(TEXT("WindField"));
	// 默认风场配置：恒定微风 + 标准大气阻力参数
	{
		FWindConfig DefaultWindConfig;
		DefaultWindConfig.bEnabled = true;
		DefaultWindConfig.WindType = EWindFieldType::Constant;
		DefaultWindConfig.SteadyWindVelocity = FVector(300.0f, 0.0f, 0.0f); // X 方向 300 cm/s
		DefaultWindConfig.AirDensity = 1.225f;
		DefaultWindConfig.DragArea = 0.04f;
		DefaultWindConfig.DragCoefficient = 1.0f;
		WindFieldComponent->SetWindConfig(DefaultWindConfig);
	}

	// 创建气压计传感器
	BarometerSensor = CreateDefaultSubobject<UBarometerSensor>(TEXT("BarometerSensor"));
	Sensors.Add(BarometerSensor);

	// 创建磁力计传感器
	MagnetometerSensor = CreateDefaultSubobject<UMagnetometerSensor>(TEXT("MagnetometerSensor"));
	Sensors.Add(MagnetometerSensor);

	// 创建风速计传感器
	AnemometerSensor = CreateDefaultSubobject<UAnemometerSensor>(TEXT("AnemometerSensor"));
	Sensors.Add(AnemometerSensor);

	// 创建跟随相机系统
	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(RootComponent);
	CameraBoom->TargetArmLength = 900.0f;
	CameraBoom->bUsePawnControlRotation = false;
	// 位置延迟：低速跟随减少位移抖动
	CameraBoom->bEnableCameraLag = true;
	CameraBoom->CameraLagSpeed = 4.0f;
	// 相机方向固定在世界空间，不随无人机旋转
	CameraBoom->bInheritPitch = false;
	CameraBoom->bInheritYaw = false;
	CameraBoom->bInheritRoll = false;
	CameraBoom->SetRelativeRotation(FRotator(-30.0f, 0.0f, 0.0f));

	FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
	FollowCamera->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);
	FollowCamera->bUsePawnControlRotation = false;
	FollowCamera->FieldOfView = 90.0f;

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

		// 缓存 MaxVelocity 用于速度比例指标
		MetricsMaxVelocity = Spec.MaxVelocity;
	}

	// 接线PID调参组件
	if (ParameterTunerComponent)
	{
		ParameterTunerComponent->SetAttitudeController(AttitudeControllerComponent);
		ParameterTunerComponent->SetPositionController(PositionControllerComponent);
	}

	NMPCComponent = NewObject<UNMPCAvoidance>(this);
	LinearMPCComponent = nullptr;  // 懒加载，仅在需要时创建

	// 同步模型速度限制到 NMPC，优化权重配置实现快速安全飞行
	{
		const FUAVModelSpec Spec = FUAVProductManager::GetModelSpec(ModelID);
		auto& Cfg = NMPCComponent->Config;
		Cfg.Actuator.MaxVelocity = Spec.MaxVelocity;
		Cfg.Solver.PredictionHorizon = 4.0f;         // 预测时域 4s
		Cfg.Solver.PredictionSteps = 8;               // 8 步
		Cfg.Cost.WeightReference = 0.01f;             // 参考跟踪权重
		Cfg.Cost.WeightVelocity = 0.005f;             // 速度跟踪权重
		Cfg.Cost.WeightTerminal = 0.01f;              // 终端代价
		Cfg.Cost.WeightObstacle = 5.0f;               // 障碍物权重
		Cfg.Obstacle.ObstacleSafeDistance = 200.0f;   // 安全距离 2m
		Cfg.Obstacle.ObstacleInfluenceDistance = 1200.0f; // 感知范围 12m
		Cfg.Obstacle.ObstacleAlpha = 0.04f;          // 衰减系数
		Cfg.Obstacle.MaxObstacleCostPerStep = 600.0f;
		Cfg.Actuator.MaxAcceleration = 500.0f;        // 最大加速度
		Cfg.Solver.MaxIterations = 40;                // 迭代次数
		Cfg.Solver.InitialStepSize = 500.0f;          // 初始步长
		Cfg.Cost.WeightControl = 0.0001f;             // 控制代价
		Cfg.Cost.WeightTemporalConsistency = 0.001f;  // 时序一致性
	}

	// 多机协同注册
	AMultiAgentGameMode* MultiAgentGM = GetWorld()->GetAuthGameMode<AMultiAgentGameMode>();
	if (MultiAgentGM)
	{
		AgentID = MultiAgentGM->RegisterAgent(this);
		bIsMultiAgentMode = true;
		CBFQPFilter = NewObject<UCBFQPFilter>(this);
		CBFQPConfig = MultiAgentGM->DefaultCBFQPConfig;
		if (CommunicationComponent)
		{
			CommunicationComponent->SetOwnerAgentID(AgentID);
		}
		UE_LOG(LogUAVActor, Log, TEXT("[MultiAgent] Registered as Agent %d"), AgentID);
	}

	// Phase 14: 环境组件接线
	// 风速计引用风场组件获取真实风速
	if (AnemometerSensor && WindFieldComponent)
	{
		AnemometerSensor->SetWindField(WindFieldComponent);
	}

	UE_LOG(LogUAVActor, Log, TEXT("[Environment] WindField + sensors initialized"));

		// 出生点安全诊断：检测出生位置附近的所有障碍物
		if (ObstacleManagerComponent)
		{
			const TArray<FObstacleInfo>& AllObs = ObstacleManagerComponent->GetAllObstacles();
			UE_LOG(LogUAVActor, Log, TEXT("[SpawnSafety] UAV born at Pos=%s, total obstacles=%d"),
				*CurrentState.Position.ToString(), AllObs.Num());
			for (const FObstacleInfo& Obs : AllObs)
			{
				float D = NMPCComponent
					? NMPCComponent->CalculateDistanceToObstacle(CurrentState.Position, Obs)
					: FVector::Dist(CurrentState.Position, Obs.Center) - Obs.Extents.GetMax() - Obs.SafetyMargin;
				if (D < 500.0f)
				{
					UE_LOG(LogUAVActor, Warning,
						TEXT("[SpawnSafety] Nearby obstacle: ID=%d, Type=%d, Center=%s, Extents=%s, "
							 "SafetyMargin=%.0f, Dist=%.1fcm, LinkedActor=%s"),
						Obs.ObstacleID, (int32)Obs.Type,
						*Obs.Center.ToString(), *Obs.Extents.ToString(),
						Obs.SafetyMargin, D,
						Obs.LinkedActor.IsValid() ? *Obs.LinkedActor->GetName() : TEXT("none"));
				}
			}
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

	SCOPE_CYCLE_COUNTER(STAT_UAVPawnTick);

	// 子步积分：将大帧拆成固定 0.02s 步长，保证物理精度的同时支持时间加速
	const float FixedStep = 0.02f;
	float Remaining = FMath::Min(DeltaTime, 1.0f); // 最多1秒防止卡顿帧
	while (Remaining > KINDA_SMALL_NUMBER)
	{
		const float Step = FMath::Min(Remaining, FixedStep);

		// 炸机状态：仅更新物理（自由落体），跳过控制
		if (FlightState == EFlightState::Crashed)
		{
			{
				SCOPE_CYCLE_COUNTER(STAT_PawnPhysicsUpdate);
				UpdatePhysics(Step);
			}
			SetActorLocation(CurrentState.Position);
			SetActorRotation(CurrentState.Rotation);
			Remaining -= Step;
			continue;
		}

		{
			SCOPE_CYCLE_COUNTER(STAT_PhysicsSubStep);

			// Phase 14: 计算风阻力加速度并传递给动力学组件
			if (WindFieldComponent && DynamicsComponent)
			{
				FVector WindAccel = WindFieldComponent->ComputeWindDragAcceleration(
					CurrentState.Velocity, CurrentState.Position, DynamicsComponent->GetMass());
				DynamicsComponent->SetExternalWindAcceleration(WindAccel);
			}

			{
				SCOPE_CYCLE_COUNTER(STAT_PawnSensorUpdate);
				UpdateSensors(Step);
			}
			{
				SCOPE_CYCLE_COUNTER(STAT_PawnControllerUpdate);
				UpdateController(Step);
			}
			{
				SCOPE_CYCLE_COUNTER(STAT_PawnPhysicsUpdate);
				UpdatePhysics(Step);
			}

			// 碰撞检测
			CheckCollision();

			SetActorLocation(CurrentState.Position);
			SetActorRotation(CurrentState.Rotation);
		}
		Remaining -= Step;
	}

	// 更新仿真指标
	if (FlightState != EFlightState::Crashed)
	{
		UpdateMetricsLog(DeltaTime);
	}

	// 更新稳定性评分
	if (StabilityScorerComponent)
	{
		SCOPE_CYCLE_COUNTER(STAT_StabilityScorer);
		StabilityScorerComponent->UpdateFlightScore(CurrentState, ObstacleManagerComponent);
		StabilityScorerComponent->DrawScoreHUD(GetActorLocation());
	}

	// 更新调试可视化
	if (DebugVisualizerComponent)
	{
		SCOPE_CYCLE_COUNTER(STAT_PawnDebugDraw);
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
		SCOPE_CYCLE_COUNTER(STAT_PawnPlanningDraw);
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
			SCOPE_CYCLE_COUNTER(STAT_PawnHUDUpdate);
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
	const float NMPCSolveInterval = 0.05f;  // 20 Hz (提升避障响应速度)
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
	const int32 N = NMPCComponent->Config.Solver.PredictionSteps;
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

	// 仅获取 NMPC 感知范围内的障碍物（不再用 2x 范围）
	TArray<FObstacleInfo> RawObstacles;
	for (const FObstacleInfo& Obs : ObstacleManagerComponent->GetObstaclesInRange(
		CurrentState.Position, NMPCComponent->Config.Obstacle.ObstacleInfluenceDistance))
	{
		if (Obs.Extents.GetMax() > 5000.0f)
		{
			continue;
		}

		// 排除其他 UAV：机间避障由 CBF-QP 安全滤波器处理，不应进入 NMPC 障碍物列表
		if (Obs.LinkedActor.IsValid())
		{
			if (Cast<AUAVPawn>(Obs.LinkedActor.Get()))
			{
				continue;
			}
		}

		RawObstacles.Add(Obs);
	}

	// 空间去重：将距离 < 500cm 的障碍物合并为一个（取最近的）
	constexpr float MergeDistance = 500.0f;
	constexpr float MergeDistSq = MergeDistance * MergeDistance;

	for (const FObstacleInfo& Obs : RawObstacles)
	{
		bool bMerged = false;
		for (FObstacleInfo& Existing : OutObstacles)
		{
			if (FVector::DistSquared(Obs.Center, Existing.Center) < MergeDistSq)
			{
				float DistNew = FVector::Dist(Obs.Center, CurrentState.Position);
				float DistExisting = FVector::Dist(Existing.Center, CurrentState.Position);
				if (DistNew < DistExisting)
				{
					Existing = Obs;
				}
				bMerged = true;
				break;
			}
		}
		if (!bMerged)
		{
			OutObstacles.Add(Obs);
		}
	}

	// 硬上限：只保留最近的 15 个障碍物，防止 NMPC 计算量爆炸
	constexpr int32 MaxObstaclesForNMPC = 15;
	if (OutObstacles.Num() > MaxObstaclesForNMPC)
	{
		OutObstacles.Sort([this](const FObstacleInfo& A, const FObstacleInfo& B)
		{
			float DistA = FVector::Dist(A.Center, CurrentState.Position);
			float DistB = FVector::Dist(B.Center, CurrentState.Position);
			return DistA < DistB;
		});
		OutObstacles.SetNum(MaxObstaclesForNMPC);
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
			if (Dist < NMPCComponent->Config.Obstacle.ObstacleSafeDistance)
			{
				FVector PushDir = (RefPt - Obs.Center).GetSafeNormal();
				if (PushDir.IsNearlyZero())
				{
					PushDir = FVector::UpVector;
				}
				RefPt += PushDir * (NMPCComponent->Config.Obstacle.ObstacleSafeDistance - Dist);
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

	// 诊断：障碍物数量统计
	{
		int32 TotalObs = ObstacleManagerComponent ? ObstacleManagerComponent->GetAllObstacles().Num() : 0;
		UE_LOG_THROTTLE(2.0, LogUAVActor, Log,
			TEXT("[ObstacleDiag] Agent %d: Total=%d, Nearby=%d"),
			AgentID, TotalObs, NearbyObstacles.Num());
	}

	FixReferencePointsPenetratingObstacles(ReferencePoints, NearbyObstacles);

	// 动态切换MPC类型
	FNMPCAvoidanceResult Result;
	if (NMPCComponent->Config.MPCType == EMPCType::Linear)
	{
		// 懒加载线性MPC
		if (!LinearMPCComponent)
		{
			LinearMPCComponent = NewObject<ULinearMPCAvoidance>(this);
			LinearMPCComponent->Config = NMPCComponent->Config;
		}
		Result = LinearMPCComponent->ComputeAvoidance(
			CurrentState.Position, CurrentState.Velocity, ReferencePoints, NearbyObstacles);
	}
	else
	{
		Result = NMPCComponent->ComputeAvoidance(
			CurrentState.Position, CurrentState.Velocity, ReferencePoints, NearbyObstacles);
	}

	bNMPCStuck = Result.bStuck;
	CachedNMPCAcceleration = Result.OptimalAcceleration;
	CachedNMPCCorrectedTarget = Result.CorrectedTarget;
	bHasCachedNMPC = true;
	bHasCachedNMPCTarget = true;

	// 收集 NMPC 求解时间用于分位数统计
	MetricsSolveTimes.Add(Result.Diagnostics.SolveTimeMs);
	if (MetricsSolveTimes.Num() > 1000) { MetricsSolveTimes.RemoveAt(0); }

	// Stuck 事件计数
	if (bNMPCStuck && !bMetricsPrevStuck) { MetricsNMPCStuckCount++; }
	bMetricsPrevStuck = bNMPCStuck;

	// 缓存最近障碍物距离
	CachedNearestObsDist = MAX_FLT;
	bool bInsideObstacle = false;
	for (const FObstacleInfo& Obs : NearbyObstacles)
	{
		float D = NMPCComponent->CalculateDistanceToObstacle(CurrentState.Position, Obs);
		CachedNearestObsDist = FMath::Min(CachedNearestObsDist, D);
		if (D < 0.0f)
		{
			bInsideObstacle = true;
		}
	}

	// 穿透障碍物检测：UAV 位于障碍物内部时，强制触发逃逸
	if (bInsideObstacle && !Result.bStuck)
	{
		bNMPCStuck = true;
		UE_LOG_THROTTLE(0.5, LogUAVActor, Warning,
			TEXT("[Penetration] UAV inside obstacle! Dist=%.0fcm, forcing escape"),
			CachedNearestObsDist);
	}

	if (Result.bStuck)
	{
		HandleStuckEscape(Result, NearbyObstacles);
	}

	// NMPC 净空不足制动：轨迹距障碍过近时无条件施加保守制动
	// 不限制最低速度：低速时也需要覆盖原有危险加速度
	if (Result.Diagnostics.bClearanceInsufficient && !Result.bStuck)
	{
		float Speed = CurrentState.Velocity.Size();
		FVector VelDir = Speed > 10.0f ? CurrentState.Velocity.GetSafeNormal() : FVector::ForwardVector;
		CachedNMPCAcceleration = -VelDir * 400.0f;
		UE_LOG_THROTTLE(0.5, LogUAVActor, Warning,
			TEXT("[NMPC] Clearance insufficient (%.0fcm), applying brake"),
			Result.Diagnostics.MinPredictedClearance);
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

	float ObsWeight = FMath::Clamp(1.0f - NearestD / NMPCComponent->Config.Obstacle.ObstacleSafeDistance, 0.0f, 1.0f);
	return (AwayFromObs * ObsWeight + ToRef * (1.0f - ObsWeight)).GetSafeNormal();
}

float AUAVPawn::CalculateEscapeAcceleration(float NearestObsDist)
{
	float CurrentSpeed = CurrentState.Velocity.Size();
	float BaseEscapeAccel = 600.0f;
	float SpeedFactor = FMath::Clamp(1.0f - CurrentSpeed / 600.0f, 0.3f, 1.0f);
	float DistFactor = FMath::Clamp(1.0f - NearestObsDist / NMPCComponent->Config.Obstacle.ObstacleSafeDistance, 0.5f, 1.5f);
	return BaseEscapeAccel * SpeedFactor * DistFactor;
}

void AUAVPawn::HandleStuckEscape(
	const FNMPCAvoidanceResult& Result,
	const TArray<FObstacleInfo>& NearbyObstacles)
{
	// 逃逸冷却期间不进行 NMPC 求解，直接使用逃逸加速度
	StuckEscapeCooldown = 0.5f;

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
	// 基础最大横向加速度限制（cm/s²）
	float MaxLateralAccel = 300.0f;

	// 近障碍时放宽横向限制（400），允许更积极的侧向避障
	if (CachedNearestObsDist < NMPCComponent->Config.Obstacle.ObstacleSafeDistance * 2.0f)
	{
		MaxLateralAccel = 400.0f;
	}

	// 限制 Y（侧向）和 Z（高度）方向的加速度幅度，保持前进方向不受限
	float LateralMag = FMath::Sqrt(Result.Y * Result.Y + Result.Z * Result.Z);
	if (LateralMag > MaxLateralAccel)
	{
		float Scale = MaxLateralAccel / LateralMag;
		Result.Y *= Scale;
		Result.Z *= Scale;
	}

	return Result;
}

// PD 横向修正：当轨迹跟踪有横向偏差时，叠加 P+D 校正
FVector AUAVPawn::ApplyPDCorrection(const FVector& Acceleration)
{
	FVector Result = Acceleration;
	FVector DesiredPos = TrajectoryTrackerComponent->GetDesiredState().Position;

	// 计算横向位置误差（Y: 左右，Z: 高度）
	float YError = DesiredPos.Y - CurrentState.Position.Y;
	float ZError = DesiredPos.Z - CurrentState.Position.Z;

	constexpr float Kp = 1.2f;
	constexpr float Kd = 1.5f;
	// 速度反馈：抑制横向振荡
	float YVelError = -CurrentState.Velocity.Y;
	float ZVelError = -CurrentState.Velocity.Z;

	Result.Y += Kp * YError + Kd * YVelError;
	Result.Z += Kp * ZError + Kd * ZVelError;

	return Result;
}

// 硬限制修正：偏差超过阈值时，用固定强度加速度强制拉回
FVector AUAVPawn::ApplyHardLimitCorrection(const FVector& Acceleration, float CrossTrackDev)
{
	FVector Result = Acceleration;
	FVector DesiredPos = TrajectoryTrackerComponent->GetDesiredState().Position;

	float YError = DesiredPos.Y - CurrentState.Position.Y;
	float ZError = DesiredPos.Z - CurrentState.Position.Z;

	// 基础偏差硬限制 120cm (1.2m)，近障碍时适度放宽到 130cm
	float HardLimit = 120.0f;
	if (CachedNearestObsDist < NMPCComponent->Config.Obstacle.ObstacleSafeDistance * 3.0f)
	{
		float ProximityRatio = FMath::Clamp(
			1.0f - CachedNearestObsDist / (NMPCComponent->Config.Obstacle.ObstacleSafeDistance * 3.0f), 0.0f, 1.0f);
		HardLimit = FMath::Lerp(120.0f, 130.0f, ProximityRatio);
	}

	// 偏差超过硬限制时，用固定强度（600 cm/s²）+ 速度阻尼强制拉回
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
	const float ObsSafe = 150.0f;      // 安全距离 1.5m（仅在极近距离减速）
	const float ObsInfluence = 300.0f; // 影响范围 3m（大幅缩小，避免远距离减速）
	float SpeedScale = 1.0f;

	// 仅对前方的障碍物减速：排除 UAV 身后的障碍物
	// 避免被出生点附近的障碍物锁死速度
	FVector VelDir = CurrentState.Velocity.GetSafeNormal();
	float EffectiveObsDist = CachedNearestObsDist;

	if (bHasCachedNMPC && VelDir.SizeSquared() > KINDA_SMALL_NUMBER)
	{
		// 重新计算前方最近障碍物距离
		if (ObstacleManagerComponent)
		{
			float ForwardMinDist = MAX_FLT;
			for (const FObstacleInfo& Obs : ObstacleManagerComponent->GetObstaclesInRange(
				CurrentState.Position, ObsInfluence * 2.0f))
			{
				if (Obs.Extents.GetMax() > 5000.0f) continue;

				FVector ToObs = Obs.Center - CurrentState.Position;
				float ForwardDot = FVector::DotProduct(ToObs.GetSafeNormal(), VelDir);

				// 只对前方半球（dot > 0.3，约 ±72°）的障碍物减速
				if (ForwardDot > 0.3f)
				{
					float Dist = NMPCComponent->CalculateDistanceToObstacle(CurrentState.Position, Obs);
					ForwardMinDist = FMath::Min(ForwardMinDist, Dist);
				}
			}
			EffectiveObsDist = ForwardMinDist;
		}
	}

	if (EffectiveObsDist < ObsInfluence)
	{
		// 分段减速策略：近距离适度减速，远距离几乎不减速
		// 让 NMPC 主导避障，速度缩放仅作为辅助安全机制
		if (EffectiveObsDist < ObsSafe) // <1.5m: 减速至 70%（放宽，让 NMPC 主导）
		{
			SpeedScale = FMath::Lerp(0.70f, 0.85f,
				FMath::Clamp(EffectiveObsDist / ObsSafe, 0.0f, 1.0f));
		}
		else // 1.5-3m: 渐进减速至 85%
		{
			SpeedScale = FMath::Lerp(0.85f, 1.0f,
				FMath::Clamp((EffectiveObsDist - ObsSafe) / (ObsInfluence - ObsSafe), 0.0f, 1.0f));
		}
	}

	TrajectoryTrackerComponent->SetSpeedScale(SpeedScale);
}

FVector AUAVPawn::ApplyEmergencyBraking(const FVector& Acceleration)
{
	// 紧急制动仅在近距离真正危险时触发
	// 阈值设计：基于实际制动距离，附加适度安全裕度
	float CurrentSpeed = CurrentState.Velocity.Size();

	// 低速时无需紧急制动（<200cm/s = 2m/s）：制动距离极短
	if (CurrentSpeed < 150.0f)
		return Acceleration;

	// 距离 > 6m 时无需干预：NMPC 完全可以处理
	if (CachedNearestObsDist >= 250.0f)
		return Acceleration;

	// 计算理论制动距离: d = v²/(2a)
	const float MaxDecel = 400.0f; // cm/s²
	float BrakingDist = (CurrentSpeed * CurrentSpeed) / (2.0f * MaxDecel);

	// 安全裕度：2.0 倍制动距离 + 1m 基础距离
	// 此公式确保只有在真正可能碰撞时才触发，不会干扰 NMPC 正常避障
	float RequiredDist = BrakingDist * 1.1f + 25.0f;

	if (CachedNearestObsDist < RequiredDist)
	{
		// 触发紧急制动: 沿速度反方向施加最大减速度
		FVector VelDir = CurrentState.Velocity.GetSafeNormal();
		float BrakeIntensity = FMath::Clamp(
			1.0f - CachedNearestObsDist / RequiredDist, 0.0f, 1.0f);

		FVector BrakeAccel = -VelDir * MaxDecel * BrakeIntensity;

		// 与NMPC加速度混合: 距离越近，制动权重越高
		float BrakeWeight = FMath::Clamp(
			(RequiredDist - CachedNearestObsDist) / RequiredDist, 0.0f, 0.8f);

		FVector Result = Acceleration * (1.0f - BrakeWeight) + BrakeAccel * BrakeWeight;

		UE_LOG_THROTTLE(0.5, LogUAVActor, Warning,
			TEXT("[EmergencyBrake] Dist=%.0fcm Speed=%.0fcm/s BrakeDist=%.0fcm Weight=%.2f"),
			CachedNearestObsDist, CurrentSpeed, BrakingDist, BrakeWeight);

		return Result;
	}

	return Acceleration;
}

FVector AUAVPawn::ApplyDeviationProtection(const FVector& NMPCAcceleration)
{
	FVector DesiredPos = TrajectoryTrackerComponent->GetDesiredState().Position;

	float YError = DesiredPos.Y - CurrentState.Position.Y;
	float ZError = DesiredPos.Z - CurrentState.Position.Z;
	float CrossTrackDev = FMath::Sqrt(YError * YError + ZError * ZError);

	// 持久化横向偏差用于指标统计
	MetricsMaxCrossTrackDev = FMath::Max(MetricsMaxCrossTrackDev, CrossTrackDev);

	// 仅在极端偏差时记录警告（阈值提高到 5m）
	if (CrossTrackDev > 500.0f)
	{
		UE_LOG_THROTTLE(0.5, LogUAVActor, Warning,
			TEXT("[DevTrack] Large deviation: %.0f cm at Pos=(%d,%d,%d)"),
			CrossTrackDev,
			(int)CurrentState.Position.X, (int)CurrentState.Position.Y, (int)CurrentState.Position.Z);
	}

	// 更新速度缩放（基于障碍物距离）
	UpdateSpeedScaleForObstacles();

	// 偏差保护：当偏差超过 2m 时，叠加 PD 纠偏，防止 NMPC 陷入局部最优
	// 降低阈值以更早介入，特别是在高速飞行时
	FVector Result = NMPCAcceleration;
	if (CrossTrackDev > 200.0f)
	{
		Result = ApplyPDCorrection(Result);

		// 硬限制：偏差超过 3m 时强制纠偏
		if (CrossTrackDev > 300.0f)
		{
			Result = ApplyHardLimitCorrection(Result, CrossTrackDev);
		}
	}

	return Result;
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
			// Early return: 组件未就绪
			if (!TrajectoryTrackerComponent || !PositionControllerComponent
				|| !AttitudeControllerComponent || !NMPCComponent)
			{
				ExecutePositionHold();
				break;
			}

			// 轨迹完成检查：IsComplete() 为 true 且速度足够低时才切换到位置保持
			const bool bTrajectoryDone = TrajectoryTrackerComponent->IsComplete();
			const float CurrentSpeed = CurrentState.Velocity.Size();
			if (bTrajectoryDone && CurrentSpeed < 150.0f)
			{
				ExecutePositionHold();
				break;
			}

			// NMPC 求解
			if (ShouldSolveNMPC(DeltaTime))
			{
				SolveNMPCAvoidance(DeltaTime);
			}

			// 应用偏差保护和速度钳位：卡死时跳过EMA，直接使用原始控制量
			if (bNMPCStuck)
			{
				SmoothedNMPCAcceleration = CachedNMPCAcceleration;
			}
			else
			{
				SmoothedNMPCAcceleration = SmoothedNMPCAcceleration * 0.4f + CachedNMPCAcceleration * 0.6f;
			}
			// 最小加速度保证：当 UAV 速度极低且 NMPC 控制量过小时，注入最小前进加速度
			// 防止 NMPC 因自适应时间缩放冻结参考点 + 温启动零控制形成死循环
			{
				const float CurSpeed = CurrentState.Velocity.Size();
				const float NMPCMag = SmoothedNMPCAcceleration.Size();
				const float MinSpeed = 400.0f;      // 4 m/s
				const float MinControlMag = 50.0f;  // NMPC 控制量低于此值视为过小

				if (CurSpeed < MinSpeed && NMPCMag < MinControlMag)
				{
					// 朝向轨迹期望位置注入最小加速度
					FVector DesiredPos = TrajectoryTrackerComponent->GetDesiredState().Position;
					FVector ToRef = (DesiredPos - CurrentState.Position);
					float DistToRef = ToRef.Size();

					if (DistToRef > 50.0f) // 距参考点 > 50cm 时才注入
					{
						FVector MinAccelVec = ToRef.GetSafeNormal() * 200.0f; // 2 m/s^2
						SmoothedNMPCAcceleration = SmoothedNMPCAcceleration + MinAccelVec;
					}
				}
			}

			FVector EffectiveAccel = ApplyDeviationProtection(SmoothedNMPCAcceleration);
			EffectiveAccel = ApplyVelocityClamp(EffectiveAccel);

			// ---- CBF-QP 统一安全滤波 ----
			bMetricsCBFActiveThisFrame = false;
			bool bMetricsCBFDegraded = false;
			FVector CBFAccelSnapshot = FVector::ZeroVector;
				// 支持 ShadowLog (只记录)/Active (修改控制)/Disabled 模式
				if (CBFQPFilter && CBFQPConfig.Mode != ECBFMode::Disabled)
				{
					// 收集障碍物（静态和动态障碍均通过 CBF 处理，动态障碍使用相对速度）
					TArray<FObstacleInfo> ObsForCBF;
					if (ObstacleManagerComponent)
					{
						const TArray<FObstacleInfo>& AllObs = ObstacleManagerComponent->GetAllObstacles();
						for (const FObstacleInfo& Obs : AllObs)
						{
							float Dist = NMPCComponent
								? NMPCComponent->CalculateDistanceToObstacle(CurrentState.Position, Obs)
								: FVector::Dist(CurrentState.Position, Obs.Center) - Obs.Extents.GetMax();
							if (Dist < CBFQPConfig.StaticInfluenceDistance)
								ObsForCBF.Add(Obs);
						}
					}

					// 收集邻居状态
					TArray<FAgentStateSnapshot> NeighborStates;
					if (bIsMultiAgentMode && CommunicationComponent)
					{
						NeighborStates = CommunicationComponent->ReceiveNeighborStates(CBFQPConfig.DSafe * 6.0f);
					}

					FCBFQPResult CBFResult = CBFQPFilter->FilterV2(
						EffectiveAccel, CurrentState, NeighborStates, ObsForCBF, CBFQPConfig);

					if (CBFResult.bWasFiltered || CBFResult.StaticConstraintCount > 0 || CBFResult.AgentConstraintCount > 0)
					{
						UE_LOG(LogUAVMetrics, Log,
							TEXT("[CBF_SOLVE] Agent=%d MinH=%.0f Residual=%.4f Violation=%.4f StaticSlack=%.4f AgentSlack=%.4f Active=%d Status=%d Ms=%.2f"),
							AgentID, CBFResult.MinHValue, CBFResult.KKTResidual,
							CBFResult.MaxConstraintViolation, CBFResult.StaticSlack, CBFResult.AgentSlack,
							CBFResult.ActiveConstraintCount, (int32)CBFResult.SolveStatus,
							CBFResult.SolveTimeMs);
						bMetricsCBFActiveThisFrame = true;
					}

					if (CBFQPConfig.Mode == ECBFMode::Active)
					{
						// Active 模式: 使用 QP 求解的安全加速度
						EffectiveAccel = CBFResult.SafeAcceleration;
						CBFAccelSnapshot = EffectiveAccel;

						bMetricsCBFDegraded = true;
						// QP 求解失败时进入降级模式
						if (CBFResult.SolveStatus == ECBFQPStatus::NumericalFailure ||
							CBFResult.SolveStatus == ECBFQPStatus::MaxIterations)
						{
							UE_LOG_THROTTLE(1.0, LogUAVActor, Warning,
								TEXT("[CBF-QP] Agent %d: QP degraded mode, status=%d, Residual=%.2f, Violation=%.2f, StaticSlack=%.2f, StaticConstraints=%d, AgentConstraints=%d"),
								AgentID, (int32)CBFResult.SolveStatus, CBFResult.KKTResidual,
								CBFResult.MaxConstraintViolation, CBFResult.StaticSlack,
								CBFResult.StaticConstraintCount, CBFResult.AgentConstraintCount);

							// 降级: 保守制动
							FVector VelDir = CurrentState.Velocity.GetSafeNormal();
							float Speed = CurrentState.Velocity.Size();
							if (Speed > 100.0f)
							{
								// 制动幅度不超过模型真实最大加速度能力
								constexpr float DegradedBrakeAccel = 400.0f;
								FVector BrakeAccel = -VelDir * DegradedBrakeAccel;

								// 验证制动方向不会朝向最近障碍物
								// 若制动方向指向障碍物，改为垂直于速度的逃逸方向
								FVector WorstObsDir = FVector::ZeroVector;
								float MinDist = MAX_FLT;
								for (const FObstacleInfo& Obs : ObsForCBF)
								{
									// ä½¿ç¨ç»ä¸æç¬¦å·è·ç¦»å½æ°ï¼æ¯æçãæè½¬çååæ±
								float D = NMPCComponent->CalculateDistanceToObstacle(CurrentState.Position, Obs);
									if (D < MinDist)
									{
										MinDist = D;
										WorstObsDir = (CurrentState.Position - Obs.Center).GetSafeNormal();
									}
								}
								if (WorstObsDir.IsNormalized() && FVector::DotProduct(BrakeAccel, -WorstObsDir) > 0.0f)
								{
									// 制动会朝向障碍物：改为垂直逃逸
									FVector EscapeDir = WorstObsDir - VelDir * FVector::DotProduct(WorstObsDir, VelDir);
									if (EscapeDir.SizeSquared() < 1.0f)
									{
										EscapeDir = FVector::CrossProduct(VelDir, FVector::UpVector);
									}
									EscapeDir.Normalize();
									BrakeAccel = EscapeDir * DegradedBrakeAccel;
								}
								EffectiveAccel = BrakeAccel;
							}
						}

						// 注意：不再对 QP 输出做幅值限幅
						// QP 已有逐轴约束 |u_i| <= MaxAccelerationQP，事后限幅会破坏 CBF 障碍约束
					}
					else // ShadowLog 模式: 记录诊断，但继续执行旧安全链
					{
						if (bIsMultiAgentMode && CBFQPFilter && CBFQPConfig.bEnabled)
						{
							TArray<FAgentStateSnapshot> LegacyNeighbors =
								CommunicationComponent->ReceiveNeighborStates(CBFQPConfig.DSafe * 6.0f);
							if (LegacyNeighbors.Num() > 0)
							{
								FCBFQPResult LegacyResult = CBFQPFilter->Filter(
									EffectiveAccel, CurrentState, LegacyNeighbors, CBFQPConfig);
								if (LegacyResult.bWasFiltered)
								{
									EffectiveAccel = LegacyResult.SafeAcceleration;
									bMetricsCBFActiveThisFrame = true;
								}
							}
						}
						EffectiveAccel = ApplyEmergencyBraking(EffectiveAccel);
						{
							float AccelMag = EffectiveAccel.Size();
							if (AccelMag > 800.0f)
								EffectiveAccel *= 800.0f / AccelMag;
						}
						if (bIsMultiAgentMode && CommunicationComponent)
						{
							TArray<FAgentStateSnapshot> RepelNeighbors =
								CommunicationComponent->ReceiveNeighborStates(CBFQPConfig.DSafe * 2.0f);
							for (const FAgentStateSnapshot& Neighbor : RepelNeighbors)
							{
								float Dist = FVector::Dist(CurrentState.Position, Neighbor.State.Position);
								float RepelThreshold = CBFQPConfig.DSafe;
								if (Dist < RepelThreshold && Dist > 1.0f)
								{
									FVector RepelDir = (CurrentState.Position - Neighbor.State.Position).GetSafeNormal();
									float Ratio = 1.0f - (Dist / RepelThreshold);
									float RepelStrength = FMath::Lerp(400.0f, 2000.0f, Ratio);
									EffectiveAccel += RepelDir * RepelStrength;
									FVector ToNeighbor = (Neighbor.State.Position - CurrentState.Position).GetSafeNormal();
									float VelToward = FVector::DotProduct(CurrentState.Velocity, ToNeighbor);
									if (VelToward > 0.0f)
									{
										float DampStrength = FMath::Lerp(200.0f, 800.0f, Ratio);
										EffectiveAccel -= ToNeighbor * FMath::Min(VelToward * 2.0f, DampStrength);
									}
								}
							}
						}
					}
				}
				else
				{
					// 旧安全链 (Mode == Disabled 时使用)
					if (bIsMultiAgentMode && CBFQPFilter && CBFQPConfig.bEnabled)
					{
						TArray<FAgentStateSnapshot> NeighborStates =
							CommunicationComponent->ReceiveNeighborStates(CBFQPConfig.DSafe * 6.0f);
						if (NeighborStates.Num() > 0)
						{
							FCBFQPResult CBFResult = CBFQPFilter->Filter(
								EffectiveAccel, CurrentState, NeighborStates, CBFQPConfig);
							if (CBFResult.bWasFiltered)
							{
								UE_LOG(LogUAVMultiAgent, Log,
									TEXT("[CBF-QP] Agent %d: filtered accel, minH=%.0f"),
									AgentID, CBFResult.MinHValue);
							}
							EffectiveAccel = CBFResult.SafeAcceleration;
						}
					}

					// 紧急制动层（旧安全链）
					EffectiveAccel = ApplyEmergencyBraking(EffectiveAccel);

					// 加速度幅值限制（旧安全链）
					{
						float AccelMag = EffectiveAccel.Size();
						const float MaxAccelCmd = 800.0f;
						if (AccelMag > MaxAccelCmd)
							EffectiveAccel *= MaxAccelCmd / AccelMag;
					}

					// 紧急排斥（旧安全链）
					if (bIsMultiAgentMode && CommunicationComponent)
					{
						TArray<FAgentStateSnapshot> Neighbors =
							CommunicationComponent->ReceiveNeighborStates(CBFQPConfig.DSafe * 2.0f);
						for (const FAgentStateSnapshot& Neighbor : Neighbors)
						{
							float Dist = FVector::Dist(CurrentState.Position, Neighbor.State.Position);
							float RepelThreshold = CBFQPConfig.DSafe;
							if (Dist < RepelThreshold && Dist > 1.0f)
							{
								FVector RepelDir = (CurrentState.Position - Neighbor.State.Position).GetSafeNormal();
								float Ratio = 1.0f - (Dist / RepelThreshold);
								float RepelStrength = FMath::Lerp(400.0f, 2000.0f, Ratio);
								EffectiveAccel += RepelDir * RepelStrength;

									FVector ToNeighbor = (Neighbor.State.Position - CurrentState.Position).GetSafeNormal();
									float VelToward = FVector::DotProduct(CurrentState.Velocity, ToNeighbor);
									if (VelToward > 0.0f)
									{
										float DampStrength = FMath::Lerp(200.0f, 800.0f, Ratio);
										EffectiveAccel -= ToNeighbor * FMath::Min(VelToward * 2.0f, DampStrength);
									}
							UE_LOG_THROTTLE(0.5, LogUAVMultiAgent, Warning,
								TEXT("[CBF-QP] Agent %d: EMERGENCY repel from %d, Dist=%.0fcm, Strength=%.0f"),
								AgentID, Neighbor.AgentID, Dist, RepelStrength);
							}
						}
					}
				}

				// 计算期望角加速度（用于前馈控制）
			FRotator DesiredAngularAccel = ComputeAngularAccelerationFromLinearAccel(
				EffectiveAccel, CurrentState.Rotation.Yaw);

			// CBF 完整性检查：Active 模式下，确保 CBF 输出未被意外修改
			if (CBFQPConfig.Mode == ECBFMode::Active && bMetricsCBFActiveThisFrame)
			{
				float IntegrityDelta = (EffectiveAccel - CBFAccelSnapshot).Size();
				if (IntegrityDelta > 1.0f)
				{
					UE_LOG_THROTTLE(5.0, LogUAVMetrics, Warning,
						TEXT("[CBF_INTEGRITY] Agent=%d Delta=%.1f AfterCBF"),
						AgentID, IntegrityDelta);
				}
			}

			// 加速度 → 姿态+推力 → 电机
			FRotator DesiredAttitude;
			float DesiredThrust;
			PositionControllerComponent->AccelerationToControl(
				EffectiveAccel, CurrentState.Rotation.Yaw, DesiredAttitude, DesiredThrust);

			FMotorOutput MotorOutput = AttitudeControllerComponent->ComputeControlWithFeedforward(
				CurrentState, DesiredAttitude, DesiredAngularAccel, DeltaTime);

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

void AUAVPawn::CheckCollision()
{
	// 已炸机状态不再检测
	if (FlightState == EFlightState::Crashed)
	{
		return;
	}

	// 障碍物穿透检测：UAV 中心点进入障碍物内部时触发炸机
	if (!ObstacleManagerComponent)
	{
		return;
	}

	const TArray<FObstacleInfo>& Obstacles = ObstacleManagerComponent->GetAllObstacles();
	for (const FObstacleInfo& Obs : Obstacles)
	{
		// 复用 NMPC 的障碍物距离计算（ObstacleManager 的同名方法为 private）
		float D = MAX_FLT;
		if (NMPCComponent)
		{
			D = NMPCComponent->CalculateDistanceToObstacle(CurrentState.Position, Obs);
		}
		else
		{
			// 简化回退：球体近似
			D = FVector::Dist(CurrentState.Position, Obs.Center) - Obs.Extents.X - Obs.SafetyMargin;
		}

		if (D < 0.0f)
		{
			UE_LOG(LogUAVActor, Warning,
				TEXT("[Crash] UAV collided with obstacle ID=%d, Type=%d, Center=%s, Extents=%s, "
					 "SafetyMargin=%.0f, Dist=%.1fcm, UAVPos=%s, LinkedActor=%s"),
				Obs.ObstacleID, (int32)Obs.Type,
				*Obs.Center.ToString(), *Obs.Extents.ToString(),
				Obs.SafetyMargin, D,
				*CurrentState.Position.ToString(),
				Obs.LinkedActor.IsValid() ? *Obs.LinkedActor->GetName() : TEXT("none"));
			TriggerCrash();
			return;
		}
	}
}

void AUAVPawn::TriggerCrash()
{
	FlightState = EFlightState::Crashed;

	// 停桨
	if (DynamicsComponent)
	{
		DynamicsComponent->EmergencyStopMotors();
	}

	// 停止轨迹跟踪
	if (TrajectoryTrackerComponent)
	{
		TrajectoryTrackerComponent->StopTracking();
	}

	// 停止任务
	if (MissionComponent)
	{
		MissionComponent->StopMission();
	}

	UE_LOG(LogUAVActor, Warning,
		TEXT("[Crash] UAV crashed at Pos=(%d,%d,%d), motors stopped, falling under gravity"),
		(int)CurrentState.Position.X, (int)CurrentState.Position.Y, (int)CurrentState.Position.Z);
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

bool AUAVPawn::IsTrajectoryTimedOut() const
{
	if (TrajectoryTrackerComponent)
	{
		return TrajectoryTrackerComponent->IsTimedOut();
	}
	return false;
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

FRotator AUAVPawn::ComputeAngularAccelerationFromLinearAccel(
	const FVector& LinearAccel,
	float CurrentYaw)
{
	// Mellinger & Kumar 微分平坦: 用 jerk 解析推导角速度，再数值微分得角加速度
	// 相比旧方案（双重微分 z_B），只对加速度做一次微分，噪声显著降低

	constexpr float G = 980.0f;
	constexpr float DegToRad = PI / 180.0f;
	constexpr float RadToDeg = 180.0f / PI;

	// Step 1: 推力向量 → z_B
	FVector ThrustVec(LinearAccel.X, LinearAccel.Y, G + LinearAccel.Z);
	float ThrustMag = ThrustVec.Size();

	if (ThrustMag < 1.0f)
	{
		PrevFeedforwardAccel = FVector::ZeroVector;
		PrevDesiredAngularVel = FVector::ZeroVector;
		PrevFilteredAngularAccel = FVector::ZeroVector;
		FeedforwardWarmupCount = 0;
		return FRotator::ZeroRotator;
	}

	FVector z_B = ThrustVec / ThrustMag;

	// Step 2: SO(3) 旋转矩阵 [x_B, y_B, z_B]
	float YawRad = CurrentYaw * DegToRad;
	FVector x_C(FMath::Cos(YawRad), FMath::Sin(YawRad), 0.0f);

	FVector y_B = FVector::CrossProduct(z_B, x_C);
	float y_B_Mag = y_B.Size();

	if (y_B_Mag < KINDA_SMALL_NUMBER)
	{
		FVector x_C_alt(-FMath::Sin(YawRad), FMath::Cos(YawRad), 0.0f);
		y_B = FVector::CrossProduct(z_B, x_C_alt).GetSafeNormal();
	}
	else
	{
		y_B /= y_B_Mag;
	}

	FVector x_B = FVector::CrossProduct(y_B, z_B);

	float dt = GetWorld()->GetDeltaSeconds();
	if (dt < KINDA_SMALL_NUMBER)
	{
		dt = 0.02f;
	}

	// Step 3: 预热期 — 前2帧只积累状态，返回零
	if (FeedforwardWarmupCount < 2)
	{
		PrevFeedforwardAccel = LinearAccel;
		PrevDesiredAngularVel = FVector::ZeroVector;
		PrevFilteredAngularAccel = FVector::ZeroVector;
		FeedforwardWarmupCount++;
		return FRotator::ZeroRotator;
	}

	// Step 4: jerk = 加速度的数值微分（单次微分，输入是平滑的 NMPC 输出）
	FVector Jerk = (LinearAccel - PrevFeedforwardAccel) / dt;

	// Step 5: Mellinger 公式 — 从 jerk 解析得到角速度
	// h_ω = (1/|T|) × (jerk - (z_B·jerk)·z_B)
	float z_dot_jerk = FVector::DotProduct(z_B, Jerk);
	FVector h_omega = (Jerk - z_dot_jerk * z_B) / ThrustMag;

	// 投影到体轴: p = -h_ω·y_B, q = h_ω·x_B
	float p = -FVector::DotProduct(h_omega, y_B);
	float q =  FVector::DotProduct(h_omega, x_B);
	FVector DesiredAngVel(p, q, 0.0f);

	// Step 6: 低通滤波角速度 (α=0.2，降低滤波系数以减少相位滞后)
	constexpr float AlphaAngVel = 0.2f;
	DesiredAngVel = AlphaAngVel * DesiredAngVel
		+ (1.0f - AlphaAngVel) * PrevDesiredAngularVel;

	// Step 7: 角加速度 = 角速度的数值微分
	FVector RawAngAccel = (DesiredAngVel - PrevDesiredAngularVel) / dt;

	// Step 8: 低通滤波角加速度 (α=0.15，进一步降低以抑制高频噪声)
	constexpr float AlphaAngAccel = 0.15f;
	FVector FilteredAngAccel = AlphaAngAccel * RawAngAccel
		+ (1.0f - AlphaAngAccel) * PrevFilteredAngularAccel;

	// Step 9: 限幅 ±30 deg/s²（降低限幅，防止过度前馈导致震荡）
	constexpr float MaxAngAccelRad = 30.0f * DegToRad; // ≈0.524 rad/s²
	FilteredAngAccel.X = FMath::Clamp(FilteredAngAccel.X, -MaxAngAccelRad, MaxAngAccelRad);
	FilteredAngAccel.Y = FMath::Clamp(FilteredAngAccel.Y, -MaxAngAccelRad, MaxAngAccelRad);
	FilteredAngAccel.Z = FMath::Clamp(FilteredAngAccel.Z, -MaxAngAccelRad, MaxAngAccelRad);

	// Step 10: 更新历史状态
	PrevFeedforwardAccel = LinearAccel;
	PrevDesiredAngularVel = DesiredAngVel;
	PrevFilteredAngularAccel = FilteredAngAccel;

	// 返回 deg/s²: FRotator(Pitch←q, Yaw←r, Roll←p)
	return FRotator(
		FilteredAngAccel.Y * RadToDeg,
		FilteredAngAccel.Z * RadToDeg,
		FilteredAngAccel.X * RadToDeg
	);
}

void AUAVPawn::UpdateMetricsLog(float DeltaTime)
{
	// ---- 速度指标 ----
	float Speed = CurrentState.Velocity.Size();
	float SpeedRatio = MetricsMaxVelocity > 0.0f ? Speed / MetricsMaxVelocity : 0.0f;

	// 低速检测（< 30% MaxVelocity）
	if (SpeedRatio < 0.3f)
	{
		if (!bMetricsInLowSpeed)
		{
			bMetricsInLowSpeed = true;
			MetricsLowSpeedTimer = 0.0f;
		}
		MetricsLowSpeedTimer += DeltaTime;
		MetricsMaxLowSpeedDuration = FMath::Max(MetricsMaxLowSpeedDuration, MetricsLowSpeedTimer);
	}
	else if (bMetricsInLowSpeed)
	{
		UE_LOG(LogUAVMetrics, Log, TEXT("[LOW_SPEED_END] Agent=%d Duration=%.1f"),
			AgentID, MetricsLowSpeedTimer);
		bMetricsInLowSpeed = false;
		MetricsLowSpeedTimer = 0.0f;
	}

	UE_LOG_THROTTLE(2.0, LogUAVMetrics, Log,
		TEXT("[SPEED_METRICS] Agent=%d Speed=%.0f MaxVel=%.0f Ratio=%.2f LowSpeedDur=%.1f"),
		AgentID, Speed, MetricsMaxVelocity, SpeedRatio, MetricsLowSpeedTimer);

	// ---- 姿态指标 ----
	float AbsRoll = FMath::Abs(CurrentState.Rotation.Roll);
	float AbsPitch = FMath::Abs(CurrentState.Rotation.Pitch);
	MetricsMaxRoll = FMath::Max(MetricsMaxRoll, AbsRoll);
	MetricsMaxPitch = FMath::Max(MetricsMaxPitch, AbsPitch);

	if (AbsRoll > 60.0f || AbsPitch > 60.0f)
	{
		MetricsAttitudeInstabilityTime += DeltaTime;
		UE_LOG_THROTTLE(0.5, LogUAVMetrics, Warning,
			TEXT("[ATTITUDE] Agent=%d Roll=%.1f Pitch=%.1f"),
			AgentID, CurrentState.Rotation.Roll, CurrentState.Rotation.Pitch);
	}

	// ---- 横向偏差严重度 ----
	if (MetricsMaxCrossTrackDev > 1000.0f) // > 10m
	{
		MetricsHighDeviationTimer += DeltaTime;
		bMetricsInSevereDeviation = true;
		if (MetricsHighDeviationTimer > 20.0f && !bMetricsTriggeredSevere)
		{
			bMetricsTriggeredSevere = true;
			UE_LOG(LogUAVMetrics, Warning,
				TEXT("[DEVIATION_SEVERE] Agent=%d Dev=%.1f Duration=%.1f Reason=Over10m20s"),
				AgentID, MetricsMaxCrossTrackDev, MetricsHighDeviationTimer);
		}
	}
	else
	{
		if (bMetricsInSevereDeviation)
		{
			bMetricsInSevereDeviation = false;
			MetricsHighDeviationTimer = 0.0f;
		}
	}

	if (MetricsMaxCrossTrackDev > 1500.0f && !bMetricsTriggeredSevere)
	{
		bMetricsTriggeredSevere = true;
		UE_LOG(LogUAVMetrics, Warning,
			TEXT("[DEVIATION_SEVERE] Agent=%d Dev=%.1f Reason=Over15m"),
			AgentID, MetricsMaxCrossTrackDev);
	}

	// ---- 避障恢复追踪 ----
	if (!bMetricsCBFActiveThisFrame && bMetricsWasAvoiding)
	{
		MetricsLastAvoidanceEnd = GetWorld()->GetTimeSeconds();
		UE_LOG(LogUAVMetrics, Log, TEXT("[AVOIDANCE_END] Agent=%d Time=%.1f"),
			AgentID, MetricsLastAvoidanceEnd);
	}
	if (bMetricsCBFActiveThisFrame)
	{
		bMetricsWasAvoiding = true;
	}
	else if (bMetricsWasAvoiding && MetricsLastAvoidanceEnd > 0.0f)
	{
		float TimeSinceEnd = GetWorld()->GetTimeSeconds() - MetricsLastAvoidanceEnd;
		if (SpeedRatio >= 0.4f)
		{
			UE_LOG(LogUAVMetrics, Log,
				TEXT("[AVOIDANCE_RECOVERY] Agent=%d Recovered=true RecoveryTime=%.1f"),
				AgentID, TimeSinceEnd);
			bMetricsWasAvoiding = false;
		}
		else if (TimeSinceEnd > 8.0f)
		{
			UE_LOG(LogUAVMetrics, Warning,
				TEXT("[AVOIDANCE_RECOVERY] Agent=%d Recovered=false RecoveryTime=%.1f"),
				AgentID, TimeSinceEnd);
			bMetricsWasAvoiding = false;
		}
	}

	// ---- 强制完成检测 ----
	if (TrajectoryTrackerComponent && TrajectoryTrackerComponent->IsTimedOut())
	{
		if (MetricsForceCompleteCount == 0)
		{
			MetricsForceCompleteCount = 1;
			UE_LOG(LogUAVMetrics, Warning,
				TEXT("[SIM_RESULT] Agent=%d Event=ForceComplete Reason=OvertimeTimeout"),
				AgentID);
		}
	}

	// ---- 汇总日志（每 5s） ----
	MetricsSummaryTimer += DeltaTime;
	if (MetricsSummaryTimer >= 5.0f)
	{
		MetricsSummaryTimer = 0.0f;

		// NMPC 求解时间分位数
		if (MetricsSolveTimes.Num() > 0)
		{
			TArray<float> Sorted = MetricsSolveTimes;
			Sorted.Sort();
			int32 Count = Sorted.Num();
			float P50 = Sorted[FMath::Min(FMath::FloorToInt(Count * 0.50f), Count - 1)];
			float P95 = Sorted[FMath::Min(FMath::FloorToInt(Count * 0.95f), Count - 1)];
			float P99 = Sorted[FMath::Min(FMath::FloorToInt(Count * 0.99f), Count - 1)];
			UE_LOG(LogUAVMetrics, Log,
				TEXT("[NMPC_PERCENTILE] Agent=%d P50=%.2f P95=%.2f P99=%.2f Count=%d"),
				AgentID, P50, P95, P99, Count);
		}

		UE_LOG(LogUAVMetrics, Log,
			TEXT("[SIM_SUMMARY] Agent=%d SpeedRatio=%.2f LowSpeedDur=%.1f MaxDev=%.0f "
				 "MaxRoll=%.1f MaxPitch=%.1f InstabTime=%.1f Stuck=%d ForceComplete=%d"),
			AgentID, SpeedRatio, MetricsMaxLowSpeedDuration, MetricsMaxCrossTrackDev,
			MetricsMaxRoll, MetricsMaxPitch, MetricsAttitudeInstabilityTime,
			MetricsNMPCStuckCount, MetricsForceCompleteCount);
	}
}
