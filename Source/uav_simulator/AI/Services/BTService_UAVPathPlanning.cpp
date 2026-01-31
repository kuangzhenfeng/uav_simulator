// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTService_UAVPathPlanning.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "uav_simulator/Core/UAVPawn.h"
#include "uav_simulator/Planning/AStarPathPlanner.h"
#include "uav_simulator/Planning/RRTPathPlanner.h"
#include "uav_simulator/Planning/TrajectoryOptimizer.h"
#include "uav_simulator/Planning/ObstacleManager.h"

UBTService_UAVPathPlanning::UBTService_UAVPathPlanning()
{
	NodeName = TEXT("UAV Path Planning Service");
	Interval = 0.5f;  // 每0.5秒检查一次
	RandomDeviation = 0.1f;

	LastTargetLocation = FVector::ZeroVector;
	LastPlanningTime = 0.0f;

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

	FVector TargetLocation = BlackboardComp->GetValueAsVector(TargetLocationKey.SelectedKeyName);

	// 检查是否需要重规划
	if (ShouldReplan(TargetLocation))
	{
		PerformPathPlanning(UAVPawn, TargetLocation);
		LastTargetLocation = TargetLocation;
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
	return Distance > ReplanningThreshold;
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

	// 获取障碍物
	TArray<FObstacleInfo> Obstacles;
	UObstacleManager* ObstacleManager = UAVPawn->GetObstacleManager();
	if (ObstacleManager)
	{
		Obstacles = ObstacleManager->GetAllObstacles();
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
		}
	}

	if (bPathFound && PlannedPath.Num() >= 2)
	{
		// 生成优化轨迹
		UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>(UAVPawn);
		if (Optimizer)
		{
			FTrajectory Trajectory = Optimizer->OptimizeTrajectory(PlannedPath, MaxVelocity, MaxAcceleration);
			if (Trajectory.bIsValid)
			{
				UAVPawn->SetTrajectory(Trajectory);
				UAVPawn->StartTrajectoryTracking();
			}
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

		if (Distance < CollisionWarningDistance)
		{
			// 触发紧急避障 - 停止当前轨迹跟踪并悬停
			UAVPawn->StopTrajectoryTracking();
			UAVPawn->SetTargetPosition(CurrentPosition);

			UE_LOG(LogTemp, Warning, TEXT("UAV Path Planning: Collision warning! Distance: %.1f cm"), Distance);

			// 触发重规划
			FVector TargetLocation = UAVPawn->GetTargetPosition();
			PerformPathPlanning(UAVPawn, TargetLocation);
		}
	}
}

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

	return FString::Printf(TEXT("Path Planning Service\nAlgorithm: %s\nDynamic Avoidance: %s"),
		*AlgorithmName,
		bEnableDynamicAvoidance ? TEXT("Enabled") : TEXT("Disabled"));
}
