// Copyright Epic Games, Inc. All Rights Reserved.

#include "TaskMonitor.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UTaskMonitor::UTaskMonitor()
	: ElapsedTime(0.0f)
	, ProgressCheckAccumulator(0.0f)
	, bInitialized(false)
{
}

void UTaskMonitor::Initialize(const FTaskAllocationResult& Allocation,
	const FTaskMonitorConfig& InConfig)
{
	Config = InConfig;
	TaskProgresses.Empty();
	AgentPositionHistory.Empty();
	StalledFrameCount.Empty();
	DeviationFrameCount.Empty();
	ElapsedTime = 0.0f;
	ProgressCheckAccumulator = 0.0f;
	bInitialized = true;

	// 初始化任务进度
	for (const FTaskAssignment& Assignment : Allocation.Assignments)
	{
		FTaskProgress Progress;
		Progress.TaskID = Assignment.TaskID;
		Progress.Status = ETaskStatus::Assigned;
		Progress.StartTime = Assignment.EstimatedStartTime;
		Progress.EstimatedEndTime = Assignment.EstimatedCompletionTime;
		Progress.Progress = 0.0f;
		Progress.AssignedAgentID = Assignment.AgentID;
		Progress.TargetLocation = FVector::ZeroVector; // TODO: 从任务描述填充
		TaskProgresses.Add(Progress);
	}

	UE_LOG(LogUAVMultiAgent, Log, TEXT("[TaskMonitor] Initialized with %d tasks"), TaskProgresses.Num());
}

void UTaskMonitor::Update(float DeltaTime, const TArray<FAgentStateSnapshot>& AgentStates)
{
	if (!bInitialized || TaskProgresses.Num() == 0)
	{
		return;
	}

	ElapsedTime += DeltaTime;
	ProgressCheckAccumulator += DeltaTime;

	if (ProgressCheckAccumulator < Config.ProgressCheckInterval)
	{
		return;
	}
	ProgressCheckAccumulator = 0.0f;

	// 构建 AgentID -> State 映射
	TMap<int32, const FAgentStateSnapshot*> AgentStateMap;
	for (const FAgentStateSnapshot& State : AgentStates)
	{
		AgentStateMap.Add(State.AgentID, &State);
	}

	// 更新每个任务进度
	for (FTaskProgress& Progress : TaskProgresses)
	{
		if (Progress.Status == ETaskStatus::Completed ||
			Progress.Status == ETaskStatus::Failed ||
			Progress.Status == ETaskStatus::Cancelled)
		{
			continue;
		}

		// 检查是否到开始时间
		if (ElapsedTime < Progress.StartTime)
		{
			Progress.Status = ETaskStatus::Assigned;
			continue;
		}

		// 检查是否超时
		if (DetectTimeout(Progress.TaskID))
		{
			Progress.Status = ETaskStatus::Failed;
			UE_LOG(LogUAVMultiAgent, Warning, TEXT("[TaskMonitor] Task %d timed out"), Progress.TaskID);
			OnTaskFailed.Broadcast(Progress.TaskID, Progress.AssignedAgentID, TEXT("Timeout"));
			OnReplanRequested.Broadcast(FString::Printf(TEXT("Task %d timed out"), Progress.TaskID));
			continue;
		}

		// 更新状态为执行中
		Progress.Status = ETaskStatus::InProgress;

		// 查询负责 Agent 的状态
		const FAgentStateSnapshot** AgentStatePtr = AgentStateMap.Find(Progress.AssignedAgentID);
		if (!AgentStatePtr || !(*AgentStatePtr))
		{
			continue;
		}

		const FAgentStateSnapshot& AgentState = **AgentStatePtr;

		// 记录位置历史（用于停滞检测）
		AgentPositionHistory.FindOrAdd(Progress.AssignedAgentID).Add(AgentState.State.Position);
		// 限制历史长度
		TArray<FVector>& History = AgentPositionHistory[Progress.AssignedAgentID];
		if (History.Num() > 60)
		{
			History.RemoveAt(0, History.Num() - 60);
		}

		// 检测停滞
		if (DetectStalledAgent(Progress.AssignedAgentID, AgentState))
		{
			UE_LOG(LogUAVMultiAgent, Warning, TEXT("[TaskMonitor] Agent %d stalled on task %d"),
				Progress.AssignedAgentID, Progress.TaskID);
		}

		// 检测偏离
		if (DetectDeviation(Progress.AssignedAgentID, AgentState))
		{
			UE_LOG(LogUAVMultiAgent, Warning, TEXT("[TaskMonitor] Agent %d deviated from task %d"),
				Progress.AssignedAgentID, Progress.TaskID);
		}

		// 计算进度（基于时间）
		float Duration = Progress.EstimatedEndTime - Progress.StartTime;
		if (Duration > 0.0f)
		{
			Progress.Progress = FMath::Clamp((ElapsedTime - Progress.StartTime) / Duration, 0.0f, 1.0f);
		}

		// 检查完成
		if (Progress.Progress >= 1.0f)
		{
			Progress.Status = ETaskStatus::Completed;
			Progress.Progress = 1.0f;
			UE_LOG(LogUAVMultiAgent, Log, TEXT("[TaskMonitor] Task %d completed by Agent %d"),
				Progress.TaskID, Progress.AssignedAgentID);
			OnTaskCompleted.Broadcast(Progress.TaskID, Progress.AssignedAgentID);
		}
	}
}

ETaskStatus UTaskMonitor::GetTaskStatus(int32 TaskID) const
{
	for (const FTaskProgress& Progress : TaskProgresses)
	{
		if (Progress.TaskID == TaskID)
		{
			return Progress.Status;
		}
	}
	return ETaskStatus::Pending;
}

float UTaskMonitor::GetTaskProgress(int32 TaskID) const
{
	for (const FTaskProgress& Progress : TaskProgresses)
	{
		if (Progress.TaskID == TaskID)
		{
			return Progress.Progress;
		}
	}
	return 0.0f;
}

float UTaskMonitor::GetOverallProgress() const
{
	if (TaskProgresses.Num() == 0)
	{
		return 0.0f;
	}

	float TotalProgress = 0.0f;
	for (const FTaskProgress& Progress : TaskProgresses)
	{
		TotalProgress += Progress.Progress;
	}
	return TotalProgress / TaskProgresses.Num();
}

int32 UTaskMonitor::GetCompletedTaskCount() const
{
	int32 Count = 0;
	for (const FTaskProgress& Progress : TaskProgresses)
	{
		if (Progress.Status == ETaskStatus::Completed)
		{
			Count++;
		}
	}
	return Count;
}

int32 UTaskMonitor::GetTotalTaskCount() const
{
	return TaskProgresses.Num();
}

bool UTaskMonitor::DetectStalledAgent(int32 AgentID, const FAgentStateSnapshot& State)
{
	TArray<FVector>* HistoryPtr = AgentPositionHistory.Find(AgentID);
	if (!HistoryPtr || HistoryPtr->Num() < 30)
	{
		return false;
	}

	// 检查最近 30 帧的位置变化
	const TArray<FVector>& History = *HistoryPtr;
	FVector OldestPos = History[History.Num() - 30];
	FVector NewestPos = History.Last();
	float Displacement = FVector::Dist(OldestPos, NewestPos);

	// 如果位移很小，认为停滞
	if (Displacement < 50.0f) // 50cm
	{
		int32& Count = StalledFrameCount.FindOrAdd(AgentID, 0);
		Count++;
		if (Count > 10) // 连续 10 次检查都停滞
		{
			return true;
		}
	}
	else
	{
		StalledFrameCount.Remove(AgentID);
	}

	return false;
}

bool UTaskMonitor::DetectDeviation(int32 AgentID, const FAgentStateSnapshot& State)
{
	// 查找分配给此 Agent 的当前任务
	for (const FTaskProgress& Progress : TaskProgresses)
	{
		if (Progress.AssignedAgentID == AgentID &&
			Progress.Status == ETaskStatus::InProgress)
		{
			float Distance = FVector::Dist(State.State.Position, Progress.TargetLocation);
			if (Distance > Config.MaxDeviationDistance)
			{
				int32& Count = DeviationFrameCount.FindOrAdd(AgentID, 0);
				Count++;
				if (Count > 5)
				{
					return true;
				}
			}
			else
			{
				DeviationFrameCount.Remove(AgentID);
			}
			break;
		}
	}
	return false;
}

bool UTaskMonitor::DetectTimeout(int32 TaskID) const
{
	for (const FTaskProgress& Progress : TaskProgresses)
	{
		if (Progress.TaskID == TaskID)
		{
			// 超过预计完成时间一定比例视为超时
			return ElapsedTime > Progress.EstimatedEndTime * 1.5f;
		}
	}
	return false;
}
