// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTTask_UAVFollowTrajectory.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "uav_simulator/Core/UAVPawn.h"
#include "uav_simulator/Planning/TrajectoryTracker.h"
#include "uav_simulator/Planning/TrajectoryOptimizer.h"
#include "uav_simulator/Planning/TrajectoryData.h"

UBTTask_UAVFollowTrajectory::UBTTask_UAVFollowTrajectory()
{
	NodeName = TEXT("UAV Follow Trajectory");
	bNotifyTick = true;
	bNotifyTaskFinished = true;
}

EBTNodeResult::Type UBTTask_UAVFollowTrajectory::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (!AIController)
	{
		return EBTNodeResult::Failed;
	}

	AUAVPawn* UAVPawn = Cast<AUAVPawn>(AIController->GetPawn());
	if (!UAVPawn)
	{
		return EBTNodeResult::Failed;
	}

	UTrajectoryTracker* Tracker = UAVPawn->GetTrajectoryTracker();
	if (!Tracker)
	{
		return EBTNodeResult::Failed;
	}

	// 获取或生成轨迹
	if (bUseTrajectoryFromBlackboard)
	{
		// 从黑板获取轨迹
		UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
		if (!BlackboardComp)
		{
			UE_LOG(LogTemp, Warning, TEXT("BTTask_UAVFollowTrajectory: No blackboard component"));
			return EBTNodeResult::Failed;
		}

		UObject* TrajectoryObject = BlackboardComp->GetValueAsObject(TrajectoryKey.SelectedKeyName);
		UTrajectoryData* TrajectoryData = Cast<UTrajectoryData>(TrajectoryObject);

		if (!TrajectoryData || !TrajectoryData->IsValid())
		{
			UE_LOG(LogTemp, Warning, TEXT("BTTask_UAVFollowTrajectory: No valid trajectory in blackboard"));
			return EBTNodeResult::Failed;
		}

		CachedTrajectory = TrajectoryData->GetTrajectory();
	}
	else
	{
		// 使用航点生成轨迹
		UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
		if (!BlackboardComp)
		{
			return EBTNodeResult::Failed;
		}

		// 获取航点（这里简化为从当前位置到目标位置）
		FVector TargetLocation = BlackboardComp->GetValueAsVector(WaypointsKey.SelectedKeyName);
		FVector CurrentLocation = UAVPawn->GetActorLocation();

		TArray<FVector> Waypoints;
		Waypoints.Add(CurrentLocation);
		Waypoints.Add(TargetLocation);

		// 生成优化轨迹
		UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>(UAVPawn);
		if (Optimizer)
		{
			CachedTrajectory = Optimizer->OptimizeTrajectory(Waypoints, MaxVelocity, MaxAcceleration);
		}

		if (!CachedTrajectory.bIsValid)
		{
			UE_LOG(LogTemp, Warning, TEXT("BTTask_UAVFollowTrajectory: Failed to generate trajectory"));
			return EBTNodeResult::Failed;
		}
	}

	// 设置轨迹并开始跟踪
	UAVPawn->SetTrajectory(CachedTrajectory);
	UAVPawn->StartTrajectoryTracking();

	return EBTNodeResult::InProgress;
}

void UBTTask_UAVFollowTrajectory::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (!AIController)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	AUAVPawn* UAVPawn = Cast<AUAVPawn>(AIController->GetPawn());
	if (!UAVPawn)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	// 检查轨迹跟踪是否完成
	if (UAVPawn->IsTrajectoryComplete() || UAVPawn->GetTrajectoryProgress() >= CompletionThreshold)
	{
		if (bHoldPositionOnComplete)
		{
			// 切换到位置保持模式
			UAVPawn->SetControlMode(EUAVControlMode::Position);
		}

		FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
		return;
	}
}

void UBTTask_UAVFollowTrajectory::OnTaskFinished(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, EBTNodeResult::Type TaskResult)
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (AIController)
	{
		AUAVPawn* UAVPawn = Cast<AUAVPawn>(AIController->GetPawn());
		if (UAVPawn && TaskResult != EBTNodeResult::Succeeded)
		{
			// 任务失败时停止轨迹跟踪
			UAVPawn->StopTrajectoryTracking();
		}
	}

	// 清除缓存的轨迹
	CachedTrajectory.Clear();
}

FString UBTTask_UAVFollowTrajectory::GetStaticDescription() const
{
	return FString::Printf(TEXT("Follow Trajectory\nMax Velocity: %.1f cm/s\nMax Acceleration: %.1f cm/s²"),
		MaxVelocity, MaxAcceleration);
}
