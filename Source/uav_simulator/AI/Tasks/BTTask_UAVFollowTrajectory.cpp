// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTTask_UAVFollowTrajectory.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "BehaviorTree/BehaviorTree.h"
#include "uav_simulator/Core/UAVPawn.h"
#include "uav_simulator/Mission/MissionComponent.h"
#include "uav_simulator/Planning/TrajectoryTracker.h"
#include "uav_simulator/Planning/TrajectoryOptimizer.h"
#include "uav_simulator/Planning/TrajectoryData.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UBTTask_UAVFollowTrajectory::UBTTask_UAVFollowTrajectory()
{
	NodeName = TEXT("UAV Follow Trajectory");
	bNotifyTick = true;
	bNotifyTaskFinished = true;

	// 设置黑板键过滤器，只允许选择Object类型的键
	TrajectoryKey.AddObjectFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_UAVFollowTrajectory, TrajectoryKey), UTrajectoryData::StaticClass());
}

void UBTTask_UAVFollowTrajectory::InitializeFromAsset(UBehaviorTree& Asset)
{
	Super::InitializeFromAsset(Asset);

	if (UBlackboardData* BBAsset = GetBlackboardAsset())
	{
		TrajectoryKey.ResolveSelectedKey(*BBAsset);
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

	// 若轨迹已在跟踪中（BTService 已启动），直接复用，避免双重启动
	if (Tracker->IsTracking())
	{
		UE_LOG(LogUAVAI, Log, TEXT("BTTask_UAVFollowTrajectory: Trajectory already tracking, skipping restart"));
		return EBTNodeResult::InProgress;
	}

	// 若 BTService 已启动轨迹跟踪，直接接管，不重复初始化
	if (Tracker->IsTracking())
	{
		UE_LOG(LogUAVAI, Log, TEXT("BTTask_UAVFollowTrajectory: Trajectory already running, skipping re-init"));
		return EBTNodeResult::InProgress;
	}

	// 获取或生成轨迹
	if (bUseTrajectoryFromBlackboard)
	{
		// 从黑板获取轨迹
		UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
		if (!BlackboardComp)
		{
			UE_LOG(LogUAVAI, Warning, TEXT("BTTask_UAVFollowTrajectory: No blackboard component"));
			return EBTNodeResult::Failed;
		}

		UObject* TrajectoryObject = BlackboardComp->GetValueAsObject(TrajectoryKey.SelectedKeyName);
		UTrajectoryData* TrajectoryData = Cast<UTrajectoryData>(TrajectoryObject);

		if (!TrajectoryData || !TrajectoryData->IsValid())
		{
			UE_LOG(LogUAVAI, Warning, TEXT("BTTask_UAVFollowTrajectory: No valid trajectory in blackboard"));
			return EBTNodeResult::Failed;
		}

		CachedTrajectory = TrajectoryData->GetTrajectory();
	}
	else
	{
		// 从 MissionComponent 或 UAVPawn 获取航点生成轨迹
		TArray<FVector> Waypoints;

		if (bUseMissionComponent)
		{
			// 优先从 MissionComponent 获取航点
			UMissionComponent* MissionComp = UAVPawn->GetMissionComponent();
			if (MissionComp && MissionComp->HasWaypoints())
			{
				Waypoints = MissionComp->GetWaypointPositions();
				UE_LOG(LogUAVAI, Log, TEXT("BTTask_UAVFollowTrajectory: Using waypoints from MissionComponent"));
			}
			else
			{
				UE_LOG(LogUAVAI, Warning, TEXT("BTTask_UAVFollowTrajectory: MissionComponent has no waypoints"));
				return EBTNodeResult::Failed;
			}
		}
		else
		{
			// 向后兼容：从 UAVPawn 获取航点
			if (UAVPawn->HasWaypoints())
			{
				Waypoints = UAVPawn->GetWaypoints();
				UE_LOG(LogUAVAI, Log, TEXT("BTTask_UAVFollowTrajectory: Using waypoints from UAVPawn (deprecated)"));
			}
			else
			{
				UE_LOG(LogUAVAI, Warning, TEXT("BTTask_UAVFollowTrajectory: UAVPawn has no waypoints"));
				return EBTNodeResult::Failed;
			}
		}

		if (Waypoints.Num() < 2)
		{
			UE_LOG(LogUAVAI, Warning, TEXT("BTTask_UAVFollowTrajectory: Need at least 2 waypoints, got %d"), Waypoints.Num());
			return EBTNodeResult::Failed;
		}

		FVector CurrentLocation = UAVPawn->GetActorLocation();

		// 如果第一个航点距离当前位置较远，则将当前位置插入到航点数组开头
		const float InsertThreshold = 100.0f; // 100cm阈值
		if (Waypoints.Num() > 0 && FVector::Dist(CurrentLocation, Waypoints[0]) > InsertThreshold)
		{
			Waypoints.Insert(CurrentLocation, 0);
			UE_LOG(LogUAVAI, Log, TEXT("BTTask_UAVFollowTrajectory: Inserted current location as first waypoint"));
		}

		UE_LOG(LogUAVAI, Log, TEXT("BTTask_UAVFollowTrajectory: Generating trajectory with %d waypoints"), Waypoints.Num());
		for (int32 i = 0; i < Waypoints.Num(); ++i)
		{
			UE_LOG(LogUAVAI, Verbose, TEXT("  Waypoint[%d]: (%.1f, %.1f, %.1f)"),
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
			UE_LOG(LogUAVAI, Warning, TEXT("BTTask_UAVFollowTrajectory: Failed to generate trajectory from %d waypoints"), Waypoints.Num());
			return EBTNodeResult::Failed;
		}

		UE_LOG(LogUAVAI, Log, TEXT("BTTask_UAVFollowTrajectory: Generated trajectory with %d points, duration: %.2f seconds"),
			CachedTrajectory.Points.Num(), CachedTrajectory.TotalDuration);

		// 如果使用 MissionComponent，启动任务
		if (bUseMissionComponent)
		{
			UMissionComponent* MissionComp = UAVPawn->GetMissionComponent();
			if (MissionComp)
			{
				MissionComp->StartMission();
			}
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
		UE_LOG(LogUAVAI, Warning, TEXT("BTTask_UAVFollowTrajectory: No AIController"));
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	AUAVPawn* UAVPawn = Cast<AUAVPawn>(AIController->GetPawn());
	if (!UAVPawn)
	{
		UE_LOG(LogUAVAI, Warning, TEXT("BTTask_UAVFollowTrajectory: No UAVPawn"));
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	// 检查轨迹跟踪是否完成
	if (UAVPawn->IsTrajectoryComplete())
	{
		if (bHoldPositionOnComplete)
		{
			// 切换到位置保持模式
			UAVPawn->SetControlMode(EUAVControlMode::Position);
		}

		// 如果使用 MissionComponent，通知任务完成
		if (bUseMissionComponent && !bUseTrajectoryFromBlackboard)
		{
			UMissionComponent* MissionComp = UAVPawn->GetMissionComponent();
			if (MissionComp && MissionComp->IsMissionRunning())
			{
				// 任务完成由 MissionComponent 内部处理
				// 这里只是通知轨迹跟踪完成
			}
		}

		UE_LOG(LogUAVAI, Log, TEXT("BTTask_UAVFollowTrajectory: Trajectory completed"));
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
	FString SourceDesc;
	if (bUseTrajectoryFromBlackboard)
	{
		SourceDesc = TEXT("Source: Blackboard Trajectory");
	}
	else if (bUseMissionComponent)
	{
		SourceDesc = TEXT("Source: MissionComponent");
	}
	else
	{
		SourceDesc = TEXT("Source: UAVPawn (deprecated)");
	}

	return FString::Printf(TEXT("Follow Trajectory\n%s\nMax Velocity: %.1f cm/s\nMax Acceleration: %.1f cm/s²"),
		*SourceDesc, MaxVelocity, MaxAcceleration);
}
