// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTService_UAVPathPlanning.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "uav_simulator/Core/UAVPawn.h"
#include "uav_simulator/Mission/MissionComponent.h"
#include "uav_simulator/Planning/AStarPathPlanner.h"
#include "uav_simulator/Planning/RRTPathPlanner.h"
#include "uav_simulator/Planning/TrajectoryOptimizer.h"
#include "uav_simulator/Planning/ObstacleManager.h"
#include "uav_simulator/Planning/LocalAvoidance.h"
#include "uav_simulator/Planning/PlanningVisualizer.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UBTService_UAVPathPlanning::UBTService_UAVPathPlanning()
{
	NodeName = TEXT("UAV Path Planning Service");
	Interval = 0.5f;
	RandomDeviation = 0.1f;

	LastTargetLocation = FVector::ZeroVector;
	LastPlanningTime = 0.0f;
	bWaypointsProcessed = false;
	LocalPlannerStuckCount = 0;

	TargetLocationKey.AddVectorFilter(this, GET_MEMBER_NAME_CHECKED(UBTService_UAVPathPlanning, TargetLocationKey));
}

void UBTService_UAVPathPlanning::TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	Super::TickNode(OwnerComp, NodeMemory, DeltaSeconds);

	AAIController* AIController = OwnerComp.GetAIOwner();
	if (!AIController)
	{
		return;
	}

	AUAVPawn* UAVPawn = Cast<AUAVPawn>(AIController->GetPawn());
	if (!UAVPawn)
	{
		return;
	}

	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	if (!BlackboardComp)
	{
		return;
	}

	// ========== Global Planner ==========
	if (bUsePresetWaypoints)
	{
		// 预定义航点模式：逐段 A* 避障
		UMissionComponent* MissionComp = UAVPawn->GetMissionComponent();
		bool bHasWaypoints = MissionComp ? MissionComp->HasWaypoints() : UAVPawn->HasWaypoints();

		if (!bWaypointsProcessed && bHasWaypoints)
		{
			UE_LOG(LogUAVAI, Log, TEXT("Processing preset waypoints for UAVPawn %s"), *UAVPawn->GetName());
			ProcessPresetWaypoints(UAVPawn);
			bWaypointsProcessed = true;
		}
	}
	else
	{
		// 动态目标模式：单点 A*/RRT 规划
		FVector TargetLocation = BlackboardComp->GetValueAsVector(TargetLocationKey.SelectedKeyName);

		if (ShouldReplan(TargetLocation))
		{
			UE_LOG(LogUAVAI, Warning, TEXT("Target moved significantly (%.1fcm), triggering replan"), FVector::Dist(TargetLocation, LastTargetLocation));
			PerformPathPlanning(UAVPawn, TargetLocation);
			LastTargetLocation = TargetLocation;
		}
	}

	// ========== Local Planner (APF) ==========
	if (bEnableDynamicAvoidance)
	{
		UE_LOG(LogUAVAI, Log, TEXT("Performing local collision check and avoidance"));
		CheckCollisionAndAvoid(UAVPawn, DeltaSeconds);
	}
}

bool UBTService_UAVPathPlanning::ShouldReplan(const FVector& CurrentTarget) const
{
	float Distance = FVector::Dist(CurrentTarget, LastTargetLocation);
	bool bShouldReplan = Distance > ReplanningThreshold;
	UE_LOG(LogUAVAI, Log, TEXT("ShouldReplan: Distance=%.1f, Threshold=%.1f, Result=%s"),
		Distance, ReplanningThreshold, bShouldReplan ? TEXT("YES") : TEXT("NO"));
	return bShouldReplan;
}

// ========== 获取或创建 LocalAvoidance 实例 ==========
ULocalAvoidance* UBTService_UAVPathPlanning::GetLocalAvoidance()
{
	if (!LocalAvoidanceInstance)
	{
		LocalAvoidanceInstance = NewObject<ULocalAvoidance>(this);
		LocalAvoidanceInstance->InfluenceDistance = CollisionCheckDistance;
	}
	return LocalAvoidanceInstance;
}

// ========== 创建路径规划器（消除重复代码） ==========
UPathPlanner* UBTService_UAVPathPlanning::CreatePathPlanner(AUAVPawn* UAVPawn, const TArray<FObstacleInfo>& Obstacles)
{
	UPathPlanner* Planner = nullptr;

	if (PathPlanningAlgorithm == EPathPlanningAlgorithm::AStar)
	{
		Planner = NewObject<UAStarPathPlanner>(UAVPawn);
	}
	else
	{
		Planner = NewObject<URRTPathPlanner>(UAVPawn);
	}

	if (Planner)
	{
		Planner->SetObstacles(Obstacles);
		FPlanningConfig Config;
		Config.SafetyMargin = SafetyMargin;
		Planner->SetPlanningConfig(Config);
	}

	return Planner;
}

// ========== 全局路径精简：line-of-sight 去除冗余中间点 ==========
void UBTService_UAVPathPlanning::SimplifyGlobalPath(TArray<FVector>& Path, const UPathPlanner* Planner, float CollisionRadius) const
{
	if (!Planner || Path.Num() < 3)
	{
		return;
	}

	TArray<FVector> Simplified;
	Simplified.Add(Path[0]);

	int32 Current = 0;
	while (Current < Path.Num() - 1)
	{
		// 从当前点尝试直连尽可能远的点
		int32 Farthest = Current + 1;
		for (int32 j = Path.Num() - 1; j > Current + 1; --j)
		{
			if (!Planner->CheckLineCollision(Path[Current], Path[j], CollisionRadius))
			{
				Farthest = j;
				break;
			}
		}
		Simplified.Add(Path[Farthest]);
		Current = Farthest;
	}

	UE_LOG(LogUAVPlanning, Log, TEXT("[SimplifyGlobalPath] %d -> %d points"), Path.Num(), Simplified.Num());
	Path = MoveTemp(Simplified);
}

// ========== 多航段路径规划：逐段 A* + 拼接 + 全局精简 ==========
bool UBTService_UAVPathPlanning::PlanMultiSegmentPath(AUAVPawn* UAVPawn, const TArray<FVector>& Waypoints, TArray<FVector>& OutPath)
{
	if (!UAVPawn || Waypoints.Num() < 2)
	{
		return false;
	}

	// 获取障碍物
	TArray<FObstacleInfo> Obstacles;
	UObstacleManager* ObstacleManager = UAVPawn->GetObstacleManager();
	if (ObstacleManager)
	{
		Obstacles = ObstacleManager->GetAllObstacles();
	}

	UE_LOG(LogUAVPlanning, Warning, TEXT("========== Multi-Segment Path Planning Started =========="));
	UE_LOG(LogUAVPlanning, Warning, TEXT("Waypoints: %d, Obstacles: %d, Algorithm: %s"),
		Waypoints.Num(), Obstacles.Num(),
		PathPlanningAlgorithm == EPathPlanningAlgorithm::AStar ? TEXT("A*") : TEXT("RRT"));

	// 如果没有障碍物，直接使用原始航点（无需规划）
	if (Obstacles.Num() == 0)
	{
		OutPath = Waypoints;
		UE_LOG(LogUAVPlanning, Log, TEXT("No obstacles, using original waypoints directly"));
		return true;
	}

	// 创建规划器
	UPathPlanner* Planner = CreatePathPlanner(UAVPawn, Obstacles);
	if (!Planner)
	{
		UE_LOG(LogUAVPlanning, Error, TEXT("Failed to create path planner!"));
		return false;
	}

	// 逐段规划并拼接
	OutPath.Empty();
	OutPath.Add(Waypoints[0]);

	bool bAllSegmentsOK = true;
	for (int32 i = 0; i < Waypoints.Num() - 1; ++i)
	{
		const FVector& SegStart = Waypoints[i];
		const FVector& SegEnd = Waypoints[i + 1];

		UE_LOG(LogUAVPlanning, Log, TEXT("  Segment [%d->%d]: %s -> %s"), i, i + 1, *SegStart.ToString(), *SegEnd.ToString());

		// 先检查直连是否可行
		if (!Planner->CheckLineCollision(SegStart, SegEnd, SafetyMargin))
		{
			// 直连无碰撞，只添加终点
			OutPath.Add(SegEnd);
			UE_LOG(LogUAVPlanning, Log, TEXT("    Direct connection clear, skipping planning"));
			continue;
		}

		// 直连有碰撞，执行 A*/RRT 规划
		TArray<FVector> SegmentPath;
		bool bSegFound = Planner->PlanPath(SegStart, SegEnd, SegmentPath);

		if (bSegFound && SegmentPath.Num() >= 2)
		{
			// 跳过第一个点（与上一段终点重复）
			for (int32 j = 1; j < SegmentPath.Num(); ++j)
			{
				OutPath.Add(SegmentPath[j]);
			}
			UE_LOG(LogUAVPlanning, Log, TEXT("    Planning succeeded, added %d path points"), SegmentPath.Num() - 1);
		}
		else
		{
			// 规划失败，回退到直连（保证路径连续性）
			OutPath.Add(SegEnd);
			bAllSegmentsOK = false;
			UE_LOG(LogUAVPlanning, Warning, TEXT("    Segment [%d->%d] planning failed, falling back to direct connection"), i, i + 1);
		}
	}

	// 全局 line-of-sight 精简
	int32 BeforeSimplify = OutPath.Num();
	SimplifyGlobalPath(OutPath, Planner, SafetyMargin);

	UE_LOG(LogUAVPlanning, Warning, TEXT("========== Multi-Segment Path Planning Completed =========="));
	UE_LOG(LogUAVPlanning, Warning, TEXT("Total path points: %d (before simplify: %d), All segments OK: %s"),
		OutPath.Num(), BeforeSimplify, bAllSegmentsOK ? TEXT("YES") : TEXT("NO"));

	return OutPath.Num() >= 2;
}

// ========== 处理预定义航点：逐段 A* 避障 + 全局精简 + 轨迹优化 ==========
void UBTService_UAVPathPlanning::ProcessPresetWaypoints(AUAVPawn* UAVPawn)
{
	if (!UAVPawn)
	{
		return;
	}

	// 优先从 MissionComponent 获取航点
	TArray<FVector> Waypoints;
	UMissionComponent* MissionComp = UAVPawn->GetMissionComponent();
	if (MissionComp && MissionComp->HasWaypoints())
	{
		Waypoints = MissionComp->GetWaypointPositions();
		UE_LOG(LogUAVPlanning, Log, TEXT("Using waypoints from MissionComponent"));
	}
	else
	{
		// 向后兼容：从 UAVPawn 获取
		Waypoints = UAVPawn->GetWaypoints();
		UE_LOG(LogUAVPlanning, Log, TEXT("Using waypoints from UAVPawn (deprecated)"));
	}

	if (Waypoints.Num() < 2)
	{
		UE_LOG(LogUAVPlanning, Warning, TEXT("Preset waypoints insufficient: %d (need at least 2)"), Waypoints.Num());
		return;
	}

	// 保存原始航点（用于 Global Replan）
	OriginalWaypoints = Waypoints;

	UE_LOG(LogUAVPlanning, Warning, TEXT("========== Processing Preset Waypoints =========="));
	UE_LOG(LogUAVPlanning, Warning, TEXT("Waypoint count: %d"), Waypoints.Num());
	for (int32 i = 0; i < Waypoints.Num(); ++i)
	{
		UE_LOG(LogUAVPlanning, Log, TEXT("  [%d] %s"), i, *Waypoints[i].ToString());
	}

	// 逐段 A* 避障规划
	TArray<FVector> PlannedPath;
	bool bSuccess = PlanMultiSegmentPath(UAVPawn, Waypoints, PlannedPath);

	if (!bSuccess || PlannedPath.Num() < 2)
	{
		UE_LOG(LogUAVPlanning, Warning, TEXT("Multi-segment planning failed, falling back to direct waypoints"));
		PlannedPath = Waypoints;
	}

	// 保存全局规划路径（用于可视化）
	GlobalPlannedPath = PlannedPath;

	// 可视化全局规划路径
	UPlanningVisualizer* Visualizer = UAVPawn->GetPlanningVisualizer();
	if (Visualizer)
	{
		Visualizer->SetPersistentPath(GlobalPlannedPath);
	}

	// 轨迹优化
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>(UAVPawn);
	if (Optimizer)
	{
		FTrajectory Trajectory = Optimizer->OptimizeTrajectory(PlannedPath, MaxVelocity, MaxAcceleration);
		UE_LOG(LogUAVPlanning, Warning, TEXT("Trajectory optimization: %s, Points: %d, Duration: %.2fs"),
			Trajectory.bIsValid ? TEXT("VALID") : TEXT("INVALID"),
			Trajectory.Points.Num(),
			Trajectory.TotalDuration);

		if (Trajectory.bIsValid)
		{
			UAVPawn->SetTrajectory(Trajectory);
			UAVPawn->StartTrajectoryTracking();

			// 可视化轨迹
			if (Visualizer)
			{
				Visualizer->SetPersistentTrajectory(Trajectory);
			}

			// 启动任务
			if (MissionComp)
			{
				MissionComp->StartMission();
			}

			UE_LOG(LogUAVPlanning, Warning, TEXT("========== Preset Waypoints Processed, Tracking Started =========="));
		}
		else
		{
			UE_LOG(LogUAVPlanning, Error, TEXT("Trajectory optimization failed!"));
		}
	}
}

// ========== 单目标点路径规划 ==========
void UBTService_UAVPathPlanning::PerformPathPlanning(AUAVPawn* UAVPawn, const FVector& TargetLocation)
{
	if (!UAVPawn)
	{
		return;
	}

	FVector CurrentLocation = UAVPawn->GetActorLocation();

	UE_LOG(LogUAVPlanning, Warning, TEXT("========== Path Planning Started =========="));
	UE_LOG(LogUAVPlanning, Warning, TEXT("Start: %s, Goal: %s"), *CurrentLocation.ToString(), *TargetLocation.ToString());

	// 获取障碍物
	TArray<FObstacleInfo> Obstacles;
	UObstacleManager* ObstacleManager = UAVPawn->GetObstacleManager();
	if (ObstacleManager)
	{
		Obstacles = ObstacleManager->GetAllObstacles();
		UE_LOG(LogUAVPlanning, Log, TEXT("Obstacle count: %d"), Obstacles.Num());
	}

	// 创建规划器并规划
	UPathPlanner* Planner = CreatePathPlanner(UAVPawn, Obstacles);
	if (!Planner)
	{
		return;
	}

	TArray<FVector> PlannedPath;
	bool bPathFound = Planner->PlanPath(CurrentLocation, TargetLocation, PlannedPath);

	UE_LOG(LogUAVPlanning, Warning, TEXT("Planning result: %s, Path points: %d"),
		bPathFound ? TEXT("SUCCESS") : TEXT("FAILED"), PlannedPath.Num());

	if (bPathFound && PlannedPath.Num() >= 2)
	{
		// 保存全局路径
		GlobalPlannedPath = PlannedPath;

		// 可视化
		UPlanningVisualizer* Visualizer = UAVPawn->GetPlanningVisualizer();
		if (Visualizer)
		{
			Visualizer->SetPersistentPath(GlobalPlannedPath);
		}

		// 轨迹优化
		UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>(UAVPawn);
		if (Optimizer)
		{
			FTrajectory Trajectory = Optimizer->OptimizeTrajectory(PlannedPath, MaxVelocity, MaxAcceleration);
			if (Trajectory.bIsValid)
			{
				UAVPawn->SetTrajectory(Trajectory);
				UAVPawn->StartTrajectoryTracking();

				if (Visualizer)
				{
					Visualizer->SetPersistentTrajectory(Trajectory);
				}

				UE_LOG(LogUAVPlanning, Warning, TEXT("========== Path Planning Completed, Trajectory Tracking Started =========="));
			}
			else
			{
				UE_LOG(LogUAVPlanning, Error, TEXT("Trajectory optimization failed!"));
			}
		}
	}
	else
	{
		UE_LOG(LogUAVPlanning, Error, TEXT("Path planning failed! bPathFound=%s, PathNum=%d"),
			bPathFound ? TEXT("true") : TEXT("false"), PlannedPath.Num());
	}
}

// ========== Local Planner: APF 局部避障 ==========
void UBTService_UAVPathPlanning::CheckCollisionAndAvoid(AUAVPawn* UAVPawn, float DeltaSeconds)
{
	if (!UAVPawn)
	{
		return;
	}

	UObstacleManager* ObstacleManager = UAVPawn->GetObstacleManager();
	if (!ObstacleManager)
	{
		return;
	}

	FVector CurrentPosition = UAVPawn->GetActorLocation();
	FVector CurrentVelocity = UAVPawn->GetUAVState().Velocity;

	// 速度接近零时跳过
	if (CurrentVelocity.IsNearlyZero())
	{
		return;
	}

	// 获取检测范围内的障碍物
	TArray<FObstacleInfo> NearbyObstacles = ObstacleManager->GetObstaclesInRange(CurrentPosition, CollisionCheckDistance);

	if (NearbyObstacles.Num() == 0)
	{
		// 附近无障碍物，重置 stuck 计数
		LocalPlannerStuckCount = 0;
		return;
	}

	// 检查前方是否有碰撞风险
	FObstacleInfo NearestObstacle;
	float NearestDist = ObstacleManager->GetDistanceToNearestObstacle(CurrentPosition, NearestObstacle);

	if (NearestDist > CollisionWarningDistance)
	{
		// 距离安全，重置 stuck 计数
		LocalPlannerStuckCount = 0;
		return;
	}

	UE_LOG(LogUAVPlanning, Warning, TEXT("[LocalPlanner] Collision warning! Nearest obstacle ID=%d, Distance=%.1fcm"),
		NearestObstacle.ObstacleID, NearestDist);

	// 使用 APF 计算避障修正
	ULocalAvoidance* APF = GetLocalAvoidance();
	if (!APF)
	{
		return;
	}

	FVector TargetPos = UAVPawn->GetTargetPosition();
	FLocalAvoidanceResult Result = APF->ComputeAvoidance(CurrentPosition, TargetPos, NearbyObstacles, CurrentVelocity);

	if (Result.bStuck)
	{
		LocalPlannerStuckCount++;
		UE_LOG(LogUAVPlanning, Warning, TEXT("[LocalPlanner] Stuck in local minimum! Consecutive count: %d/%d"),
			LocalPlannerStuckCount, LocalPlannerFailThreshold);

		if (LocalPlannerStuckCount >= LocalPlannerFailThreshold)
		{
			// Local Planner 连续失败，触发 Global Replan
			UE_LOG(LogUAVPlanning, Warning, TEXT("[LocalPlanner] Consecutive failures reached threshold, triggering global replan"));
			TriggerGlobalReplan(UAVPawn);
			LocalPlannerStuckCount = 0;
			return;
		}
	}
	else
	{
		LocalPlannerStuckCount = 0;
	}

	if (Result.bNeedsCorrection)
	{
		// 用 APF 合力方向修正目标位置
		float LookaheadDist = FMath::Min(CurrentVelocity.Size() * 0.5f, CollisionWarningDistance);
		FVector CorrectedTarget = CurrentPosition + Result.CorrectedDirection * LookaheadDist;
		UAVPawn->SetTargetPosition(CorrectedTarget);

		UE_LOG(LogUAVPlanning, Log, TEXT("[LocalPlanner] APF correction: Dir=(%.2f,%.2f,%.2f), CorrectedTarget=%s"),
			Result.CorrectedDirection.X, Result.CorrectedDirection.Y, Result.CorrectedDirection.Z,
			*CorrectedTarget.ToString());
	}
}

// ========== 触发全局重规划 ==========
void UBTService_UAVPathPlanning::TriggerGlobalReplan(AUAVPawn* UAVPawn)
{
	if (!UAVPawn)
	{
		return;
	}

	// 停止当前轨迹跟踪
	UAVPawn->StopTrajectoryTracking();

	if (bUsePresetWaypoints && OriginalWaypoints.Num() >= 2)
	{
		// 航点模式：从当前位置出发，经过剩余航点重新规划
		FVector CurrentPos = UAVPawn->GetActorLocation();

		// 找到最近的未到达航点
		int32 NearestIdx = 0;
		float MinDist = FLT_MAX;
		for (int32 i = 0; i < OriginalWaypoints.Num(); ++i)
		{
			float Dist = FVector::Dist(CurrentPos, OriginalWaypoints[i]);
			if (Dist < MinDist)
			{
				MinDist = Dist;
				NearestIdx = i;
			}
		}

		// 构建剩余航点列表（从当前位置开始）
		TArray<FVector> RemainingWaypoints;
		RemainingWaypoints.Add(CurrentPos);
		for (int32 i = NearestIdx; i < OriginalWaypoints.Num(); ++i)
		{
			RemainingWaypoints.Add(OriginalWaypoints[i]);
		}

		UE_LOG(LogUAVPlanning, Warning, TEXT("[GlobalReplan] Replanning from waypoint %d, remaining %d waypoints"),
			NearestIdx, RemainingWaypoints.Num());

		// 重新执行多航段规划
		TArray<FVector> NewPath;
		bool bSuccess = PlanMultiSegmentPath(UAVPawn, RemainingWaypoints, NewPath);

		if (bSuccess && NewPath.Num() >= 2)
		{
			GlobalPlannedPath = NewPath;

			UPlanningVisualizer* Visualizer = UAVPawn->GetPlanningVisualizer();
			if (Visualizer)
			{
				Visualizer->SetPersistentPath(GlobalPlannedPath);
			}

			UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>(UAVPawn);
			if (Optimizer)
			{
				FTrajectory Trajectory = Optimizer->OptimizeTrajectory(NewPath, MaxVelocity, MaxAcceleration);
				if (Trajectory.bIsValid)
				{
					UAVPawn->SetTrajectory(Trajectory);
					UAVPawn->StartTrajectoryTracking();

					if (Visualizer)
					{
						Visualizer->SetPersistentTrajectory(Trajectory);
					}

					UE_LOG(LogUAVPlanning, Warning, TEXT("[GlobalReplan] Replan succeeded, trajectory tracking restarted"));
				}
			}
		}
		else
		{
			UE_LOG(LogUAVPlanning, Error, TEXT("[GlobalReplan] Replan failed!"));
			// 回退：悬停在当前位置
			UAVPawn->SetTargetPosition(CurrentPos);
		}
	}
	else
	{
		// 单目标模式：重新规划到目标
		FVector TargetLocation = UAVPawn->GetTargetPosition();
		PerformPathPlanning(UAVPawn, TargetLocation);
	}
}

// ========== 行为树节点描述 ==========
FString UBTService_UAVPathPlanning::GetStaticDescription() const
{
	FString AlgorithmName;
	switch (PathPlanningAlgorithm)
	{
	case EPathPlanningAlgorithm::AStar:
		AlgorithmName = TEXT("A*");
		break;
	case EPathPlanningAlgorithm::RRT:
		AlgorithmName = TEXT("RRT");
		break;
	case EPathPlanningAlgorithm::RRTStar:
		AlgorithmName = TEXT("RRT*");
		break;
	}

	if (bUsePresetWaypoints)
	{
		return FString::Printf(TEXT("Path Planning Service\nMode: Preset Waypoints (Multi-Segment %s)\nLocal Planner (APF): %s"),
			*AlgorithmName,
			bEnableDynamicAvoidance ? TEXT("Enabled") : TEXT("Disabled"));
	}

	return FString::Printf(TEXT("Path Planning Service\nMode: Dynamic Planning\nAlgorithm: %s\nLocal Planner (APF): %s"),
		*AlgorithmName,
		bEnableDynamicAvoidance ? TEXT("Enabled") : TEXT("Disabled"));
}
