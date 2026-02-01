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
#include "uav_simulator/Debug/UAVLogConfig.h"

UBTService_UAVPathPlanning::UBTService_UAVPathPlanning()
{
	NodeName = TEXT("UAV Path Planning Service");
	Interval = 0.5f;  // 每0.5秒检查一次
	RandomDeviation = 0.1f;

	LastTargetLocation = FVector::ZeroVector;
	LastPlanningTime = 0.0f;
	bWaypointsProcessed = false;

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

	// 预定义航点模式
	if (bUsePresetWaypoints)
	{
		// 优先从 MissionComponent 获取航点
		UMissionComponent* MissionComp = UAVPawn->GetMissionComponent();
		bool bHasWaypoints = MissionComp ? MissionComp->HasWaypoints() : UAVPawn->HasWaypoints();

		if (!bWaypointsProcessed && bHasWaypoints)
		{
			ProcessPresetWaypoints(UAVPawn);
			bWaypointsProcessed = true;
		}
	}
	else
	{
		// 动态路径规划模式
		FVector TargetLocation = BlackboardComp->GetValueAsVector(TargetLocationKey.SelectedKeyName);

		// 检查是否需要重规划
		if (ShouldReplan(TargetLocation))
		{
			PerformPathPlanning(UAVPawn, TargetLocation);
			LastTargetLocation = TargetLocation;
		}
	}

	// 动态避障检测
	if (bEnableDynamicAvoidance)
	{
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

void UBTService_UAVPathPlanning::PerformPathPlanning(AUAVPawn* UAVPawn, const FVector& TargetLocation)
{
	if (!UAVPawn)
	{
		return;
	}

	FVector CurrentLocation = UAVPawn->GetActorLocation();
	TArray<FVector> PlannedPath;
	bool bPathFound = false;

	UE_LOG(LogUAVPlanning, Warning, TEXT("========== Path Planning Started =========="));
	UE_LOG(LogUAVPlanning, Warning, TEXT("Start: %s"), *CurrentLocation.ToString());
	UE_LOG(LogUAVPlanning, Warning, TEXT("Goal: %s"), *TargetLocation.ToString());
	UE_LOG(LogUAVPlanning, Warning, TEXT("Algorithm: %s"),
		PathPlanningAlgorithm == EPathPlanningAlgorithm::AStar ? TEXT("A*") : TEXT("RRT"));

	// 获取障碍物
	TArray<FObstacleInfo> Obstacles;
	UObstacleManager* ObstacleManager = UAVPawn->GetObstacleManager();
	if (ObstacleManager)
	{
		Obstacles = ObstacleManager->GetAllObstacles();
		UE_LOG(LogUAVPlanning, Warning, TEXT("Obstacle count: %d"), Obstacles.Num());
		for (int32 i = 0; i < Obstacles.Num(); ++i)
		{
			UE_LOG(LogUAVPlanning, Log, TEXT("  Obstacle[%d]: Center=%s, Type=%d, Extents=%s"),
				i, *Obstacles[i].Center.ToString(), (int32)Obstacles[i].Type, *Obstacles[i].Extents.ToString());
		}
	}
	else
	{
		UE_LOG(LogUAVPlanning, Error, TEXT("ObstacleManager is NULL! Cannot get obstacle info"));
	}

	// 根据算法选择规划器
	if (PathPlanningAlgorithm == EPathPlanningAlgorithm::AStar)
	{
		UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>(UAVPawn);
		if (Planner)
		{
			Planner->SetObstacles(Obstacles);
			FPlanningConfig Config;
			Config.SafetyMargin = SafetyMargin;
			Planner->SetPlanningConfig(Config);
			bPathFound = Planner->PlanPath(CurrentLocation, TargetLocation, PlannedPath);
			UE_LOG(LogUAVPlanning, Warning, TEXT("A* result: %s, Waypoint count: %d"),
				bPathFound ? TEXT("SUCCESS") : TEXT("FAILED"), PlannedPath.Num());
		}
	}
	else
	{
		URRTPathPlanner* Planner = NewObject<URRTPathPlanner>(UAVPawn);
		if (Planner)
		{
			Planner->SetObstacles(Obstacles);
			FPlanningConfig Config;
			Config.SafetyMargin = SafetyMargin;
			Planner->SetPlanningConfig(Config);
			bPathFound = Planner->PlanPath(CurrentLocation, TargetLocation, PlannedPath);
			UE_LOG(LogUAVPlanning, Warning, TEXT("RRT result: %s, Waypoint count: %d"),
				bPathFound ? TEXT("SUCCESS") : TEXT("FAILED"), PlannedPath.Num());
		}
	}

	// 打印规划的路径点
	if (bPathFound)
	{
		UE_LOG(LogUAVPlanning, Warning, TEXT("Planned waypoints:"));
		for (int32 i = 0; i < PlannedPath.Num(); ++i)
		{
			UE_LOG(LogUAVPlanning, Log, TEXT("  [%d] %s"), i, *PlannedPath[i].ToString());
		}
	}

	if (bPathFound && PlannedPath.Num() >= 2)
	{
		// 生成优化轨迹
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
				UE_LOG(LogUAVPlanning, Warning, TEXT("========== Path Planning Completed, Tracking Started =========="));
			}
			else
			{
				UE_LOG(LogUAVPlanning, Error, TEXT("Trajectory optimization failed!"));
			}
		}
	}
	else
	{
		UE_LOG(LogUAVPlanning, Error, TEXT("Path planning failed or insufficient waypoints! bPathFound=%s, PathNum=%d"),
			bPathFound ? TEXT("true") : TEXT("false"), PlannedPath.Num());
	}
}

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

	UE_LOG(LogUAVPlanning, Warning, TEXT("========== Processing Preset Waypoints =========="));
	UE_LOG(LogUAVPlanning, Warning, TEXT("Waypoint count: %d"), Waypoints.Num());
	for (int32 i = 0; i < Waypoints.Num(); ++i)
	{
		UE_LOG(LogUAVPlanning, Log, TEXT("  [%d] %s"), i, *Waypoints[i].ToString());
	}

	// 直接使用预定义航点生成轨迹
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>(UAVPawn);
	if (Optimizer)
	{
		FTrajectory Trajectory = Optimizer->OptimizeTrajectory(Waypoints, MaxVelocity, MaxAcceleration);
		UE_LOG(LogUAVPlanning, Warning, TEXT("Trajectory optimization: %s, Points: %d, Duration: %.2fs"),
			Trajectory.bIsValid ? TEXT("VALID") : TEXT("INVALID"),
			Trajectory.Points.Num(),
			Trajectory.TotalDuration);

		if (Trajectory.bIsValid)
		{
			UAVPawn->SetTrajectory(Trajectory);
			UAVPawn->StartTrajectoryTracking();

			// 如果有 MissionComponent，启动任务
			if (MissionComp)
			{
				MissionComp->StartMission();
			}

			UE_LOG(LogUAVPlanning, Warning, TEXT("========== Preset Waypoints Processed, Tracking Started =========="));
		}
		else
		{
			UE_LOG(LogUAVPlanning, Error, TEXT("Trajectory optimization failed for preset waypoints!"));
		}
	}
}

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

	// 检查前方是否有障碍物
	if (!CurrentVelocity.IsNearlyZero())
	{
		FVector CheckPoint = CurrentPosition + CurrentVelocity.GetSafeNormal() * CollisionCheckDistance;

		FObstacleInfo NearestObstacle;
		float Distance = ObstacleManager->GetDistanceToNearestObstacle(CheckPoint, NearestObstacle);

		UE_LOG(LogUAVPlanning, Verbose, TEXT("Collision check: Point=%s, NearestDist=%.1fcm, WarningDist=%.1fcm"),
			*CheckPoint.ToString(), Distance, CollisionWarningDistance);

		if (Distance < CollisionWarningDistance)
		{
			// 触发紧急避障 - 停止当前轨迹跟踪并悬停
			UAVPawn->StopTrajectoryTracking();
			UAVPawn->SetTargetPosition(CurrentPosition);

			UE_LOG(LogUAVPlanning, Warning, TEXT("!!! COLLISION WARNING !!! Distance: %.1fcm, ObstacleCenter: %s"),
				Distance, *NearestObstacle.Center.ToString());

			// 触发重规划
			FVector TargetLocation = UAVPawn->GetTargetPosition();
			UE_LOG(LogUAVPlanning, Warning, TEXT("Triggering replan, Target: %s"), *TargetLocation.ToString());
			PerformPathPlanning(UAVPawn, TargetLocation);
		}
	}
}

FString UBTService_UAVPathPlanning::GetStaticDescription() const
{
	if (bUsePresetWaypoints)
	{
		return FString::Printf(TEXT("Path Planning Service\nMode: Preset Waypoints\nDynamic Avoidance: %s"),
			bEnableDynamicAvoidance ? TEXT("Enabled") : TEXT("Disabled"));
	}

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

	return FString::Printf(TEXT("Path Planning Service\nMode: Dynamic Planning\nAlgorithm: %s\nDynamic Avoidance: %s"),
		*AlgorithmName,
		bEnableDynamicAvoidance ? TEXT("Enabled") : TEXT("Disabled"));
}

