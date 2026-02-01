// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTTask_UAVFollowTrajectory.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "BehaviorTree/BehaviorTree.h"
#include "uav_simulator/Core/UAVPawn.h"
#include "uav_simulator/Planning/TrajectoryTracker.h"
#include "uav_simulator/Planning/TrajectoryOptimizer.h"
#include "uav_simulator/Planning/TrajectoryData.h"
#include "uav_simulator/Planning/WaypointsData.h"

UBTTask_UAVFollowTrajectory::UBTTask_UAVFollowTrajectory()
{
	NodeName = TEXT("UAV Follow Trajectory");
	bNotifyTick = true;
	bNotifyTaskFinished = true;

	// 设置黑板键过滤器，只允许选择Object类型的键
	TrajectoryKey.AddObjectFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_UAVFollowTrajectory, TrajectoryKey), UTrajectoryData::StaticClass());
	WaypointsKey.AddObjectFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_UAVFollowTrajectory, WaypointsKey), UWaypointsData::StaticClass());
}

void UBTTask_UAVFollowTrajectory::InitializeFromAsset(UBehaviorTree& Asset)
{
	Super::InitializeFromAsset(Asset);

	if (UBlackboardData* BBAsset = GetBlackboardAsset())
	{
		TrajectoryKey.ResolveSelectedKey(*BBAsset);
		WaypointsKey.ResolveSelectedKey(*BBAsset);
	}
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
			UE_LOG(LogTemp, Warning, TEXT("BTTask_UAVFollowTrajectory: No blackboard component"));
			return EBTNodeResult::Failed;
		}

		// 从黑板获取航点数据对象
		UObject* WaypointsObject = BlackboardComp->GetValueAsObject(WaypointsKey.SelectedKeyName);
		UWaypointsData* WaypointsData = Cast<UWaypointsData>(WaypointsObject);

		if (!WaypointsData || !WaypointsData->IsValid())
		{
			UE_LOG(LogTemp, Warning, TEXT("BTTask_UAVFollowTrajectory: No valid waypoints in blackboard (need at least 2 waypoints)"));
			return EBTNodeResult::Failed;
		}

		// 获取航点数组
		TArray<FVector> Waypoints = WaypointsData->GetWaypoints();
		FVector CurrentLocation = UAVPawn->GetActorLocation();

		// 如果第一个航点距离当前位置较远，则将当前位置插入到航点数组开头
		const float InsertThreshold = 100.0f; // 100cm阈值
		if (Waypoints.Num() > 0 && FVector::Dist(CurrentLocation, Waypoints[0]) > InsertThreshold)
		{
			Waypoints.Insert(CurrentLocation, 0);
			UE_LOG(LogTemp, Log, TEXT("BTTask_UAVFollowTrajectory: Inserted current location as first waypoint"));
		}

		UE_LOG(LogTemp, Log, TEXT("BTTask_UAVFollowTrajectory: Generating trajectory with %d waypoints"), Waypoints.Num());
		for (int32 i = 0; i < Waypoints.Num(); ++i)
		{
			UE_LOG(LogTemp, Log, TEXT("  Waypoint[%d]: (%.1f, %.1f, %.1f)"),
				i, Waypoints[i].X, Waypoints[i].Y, Waypoints[i].Z);
		}

		// 生成优化轨迹
		UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>(UAVPawn);
		if (Optimizer)
		{
			CachedTrajectory = Optimizer->OptimizeTrajectory(Waypoints, MaxVelocity, MaxAcceleration);
		}

		if (!CachedTrajectory.bIsValid)
		{
			UE_LOG(LogTemp, Warning, TEXT("BTTask_UAVFollowTrajectory: Failed to generate trajectory from %d waypoints"), Waypoints.Num());
			return EBTNodeResult::Failed;
		}

		UE_LOG(LogTemp, Log, TEXT("BTTask_UAVFollowTrajectory: Generated trajectory with %d points, duration: %.2f seconds"),
			CachedTrajectory.Points.Num(), CachedTrajectory.TotalDuration);
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
		UE_LOG(LogTemp, Warning, TEXT("BTTask_UAVFollowTrajectory: No AIController"));
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	AUAVPawn* UAVPawn = Cast<AUAVPawn>(AIController->GetPawn());
	if (!UAVPawn)
	{
		UE_LOG(LogTemp, Warning, TEXT("BTTask_UAVFollowTrajectory: No UAVPawn"));
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

		UE_LOG(LogTemp, Log, TEXT("BTTask_UAVFollowTrajectory: Trajectory completed"));
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
