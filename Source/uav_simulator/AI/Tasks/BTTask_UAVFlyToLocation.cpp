// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTTask_UAVFlyToLocation.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "uav_simulator/Core/UAVPawn.h"
#include "uav_simulator/Planning/AStarPathPlanner.h"
#include "uav_simulator/Planning/RRTPathPlanner.h"
#include "uav_simulator/Planning/TrajectoryOptimizer.h"
#include "uav_simulator/Planning/ObstacleManager.h"

UBTTask_UAVFlyToLocation::UBTTask_UAVFlyToLocation()
{
	NodeName = TEXT("UAV Fly To Location");
	bNotifyTick = true;

	// 设置默认的黑板键过滤器
	TargetLocationKey.AddVectorFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_UAVFlyToLocation, TargetLocationKey));
}

EBTNodeResult::Type UBTTask_UAVFlyToLocation::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (!AIController)
	{
		return EBTNodeResult::Failed;
	}

	APawn* ControlledPawn = AIController->GetPawn();
	if (!ControlledPawn)
	{
		return EBTNodeResult::Failed;
	}

	// 获取目标位置
	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	if (!BlackboardComp)
	{
		return EBTNodeResult::Failed;
	}

	FVector TargetLocation = BlackboardComp->GetValueAsVector(TargetLocationKey.SelectedKeyName);

	// 检查是否已经在目标位置
	float Distance = FVector::Dist(ControlledPawn->GetActorLocation(), TargetLocation);
	if (Distance <= AcceptableRadius)
	{
		return EBTNodeResult::Succeeded;
	}

	AUAVPawn* UAVPawn = Cast<AUAVPawn>(ControlledPawn);
	if (!UAVPawn)
	{
		return EBTNodeResult::Failed;
	}

	// 重置状态
	bPathInitialized = false;
	CachedTrajectory.Clear();

	if (bUsePathPlanning)
	{
		// 使用路径规划
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
		else // RRT or RRT*
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

		if (!bPathFound || PlannedPath.Num() < 2)
		{
			UE_LOG(LogTemp, Warning, TEXT("BTTask_UAVFlyToLocation: Path planning failed, using direct path"));
			PlannedPath.Empty();
			PlannedPath.Add(CurrentLocation);
			PlannedPath.Add(TargetLocation);
		}

		// 轨迹优化
		if (bOptimizeTrajectory)
		{
			UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>(UAVPawn);
			if (Optimizer)
			{
				CachedTrajectory = Optimizer->OptimizeTrajectory(PlannedPath, FlySpeed, MaxAcceleration);
			}
		}

		// 如果有有效轨迹，使用轨迹跟踪
		if (CachedTrajectory.bIsValid)
		{
			UAVPawn->SetTrajectory(CachedTrajectory);
			UAVPawn->StartTrajectoryTracking();
			bPathInitialized = true;
		}
		else
		{
			// 回退到简单位置控制
			UAVPawn->SetTargetPosition(TargetLocation);
		}
	}
	else
	{
		// 直接位置控制
		UAVPawn->SetTargetPosition(TargetLocation);
	}

	return EBTNodeResult::InProgress;
}

void UBTTask_UAVFlyToLocation::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (!AIController)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	APawn* ControlledPawn = AIController->GetPawn();
	if (!ControlledPawn)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	if (!BlackboardComp)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	FVector TargetLocation = BlackboardComp->GetValueAsVector(TargetLocationKey.SelectedKeyName);
	FVector CurrentLocation = ControlledPawn->GetActorLocation();

	// 检查是否到达目标
	float Distance = FVector::Dist(CurrentLocation, TargetLocation);
	if (Distance <= AcceptableRadius)
	{
		AUAVPawn* UAVPawn = Cast<AUAVPawn>(ControlledPawn);
		if (UAVPawn && bPathInitialized)
		{
			UAVPawn->StopTrajectoryTracking();
		}
		FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
		return;
	}

	// 如果使用轨迹跟踪，检查轨迹是否完成
	if (bUsePathPlanning && bPathInitialized)
	{
		AUAVPawn* UAVPawn = Cast<AUAVPawn>(ControlledPawn);
		if (UAVPawn && UAVPawn->IsTrajectoryComplete())
		{
			// 轨迹完成但未到达目标，切换到位置保持
			UAVPawn->SetControlMode(EUAVControlMode::Position);
			UAVPawn->SetTargetPosition(TargetLocation);
		}
	}
	else
	{
		// 持续更新目标位置（以防黑板值改变）
		AUAVPawn* UAVPawn = Cast<AUAVPawn>(ControlledPawn);
		if (UAVPawn)
		{
			UAVPawn->SetTargetPosition(TargetLocation);
		}
	}
}

FString UBTTask_UAVFlyToLocation::GetStaticDescription() const
{
	FString Description = FString::Printf(TEXT("Fly to: %s\nAcceptable Radius: %.1f"),
		*TargetLocationKey.SelectedKeyName.ToString(),
		AcceptableRadius);

	if (bUsePathPlanning)
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
		Description += FString::Printf(TEXT("\nPath Planning: %s"), *AlgorithmName);
		if (bOptimizeTrajectory)
		{
			Description += TEXT("\nTrajectory Optimization: Enabled");
		}
	}

	return Description;
}
