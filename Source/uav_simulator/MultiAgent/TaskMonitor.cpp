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
	const FTaskMonitorConfig& InConfig, const TArray<FTaskDescriptor>& Tasks)
{
	Config = InConfig;
	TaskProgresses.Empty();
	AgentPositionHistory.Empty();
	StalledFrameCount.Empty();
	DeviationFrameCount.Empty();
	ElapsedTime = 0.0f;
	ProgressCheckAccumulator = 0.0f;
	bInitialized = true;

	for (const FTaskAssignment& Assignment : Allocation.Assignments)
	{
		FTaskProgress Progress;
		Progress.TaskID = Assignment.TaskID;
		Progress.Status = Tasks.IsEmpty() ? ETaskStatus::InProgress : ETaskStatus::Assigned;
		Progress.StartTime = Assignment.EstimatedStartTime;
		Progress.EstimatedEndTime = Assignment.EstimatedCompletionTime;
		Progress.Progress = 0.0f;
		Progress.AssignedAgentID = Assignment.AgentID;
		Progress.TargetLocation = FVector::ZeroVector;
        for (const FTaskDescriptor& Task : Tasks)
        {
            if (Task.TaskID == Assignment.TaskID)
            {
                Progress.TargetLocation = Task.TargetLocation;
                Progress.ServiceDuration = Task.EstimatedDuration;
                Progress.Deadline = FMath::Min(Task.Deadline, Task.LatestFinish);
                Progress.EarliestStart = Task.EarliestStart;
                Progress.bHasTarget = true;
                break;
            }
        }
		TaskProgresses.Add(Progress);
	}

	UE_LOG(LogUAVMultiAgent, Log, TEXT("[TaskMonitor] Initialized with %d tasks"), TaskProgresses.Num());
}

void UTaskMonitor::Reset()
{
	TaskProgresses.Empty();
	AgentPositionHistory.Empty();
	StalledFrameCount.Empty();
	DeviationFrameCount.Empty();
	ElapsedTime = 0.0f;
	ProgressCheckAccumulator = 0.0f;
	bInitialized = false;
	CompletedTaskIDs.Empty();
	FailedTaskIDs.Empty();
}

void UTaskMonitor::ReportExecution(int32 TaskID,int32 AgentID,ETaskStatus Status,float ActualProgress)
{
	if(CompletedTaskIDs.Contains(TaskID)) return;
	FTaskProgress* P=TaskProgresses.FindByPredicate([TaskID](const auto& V){return V.TaskID==TaskID;});
	if(!P)
	{
		FTaskProgress Entry{};Entry.TaskID=TaskID;Entry.AssignedAgentID=AgentID;
		TaskProgresses.Add(Entry);P=&TaskProgresses.Last();
	}
	P->AssignedAgentID=AgentID;P->Status=Status;P->Progress=FMath::Clamp(ActualProgress,0.0f,1.0f);
	if(Status==ETaskStatus::Completed) {P->Progress=1;CompletedTaskIDs.Add(TaskID);}
	if(Status==ETaskStatus::Failed) FailedTaskIDs.Add(TaskID);
}

void UTaskMonitor::ActivateTask(int32 TaskID)
{
    for (FTaskProgress& P : TaskProgresses)
        if (P.TaskID == TaskID && P.Status == ETaskStatus::Assigned)
        {
            P.Status = ETaskStatus::InProgress;
            P.StartTime = ElapsedTime;
            AgentPositionHistory.Remove(P.AssignedAgentID);
            StalledFrameCount.Remove(P.AssignedAgentID);
            DeviationFrameCount.Remove(P.AssignedAgentID);
        }
}

void UTaskMonitor::PreserveCompleted(const TSet<int32>& IDs)
{
    CompletedTaskIDs = IDs;
}

void UTaskMonitor::PreserveFailed(const TSet<int32>& IDs) { FailedTaskIDs = IDs; }
void UTaskMonitor::MarkFailed(int32 TaskID) { FailedTaskIDs.Add(TaskID); for (auto& P : TaskProgresses) if (P.TaskID==TaskID) P.Status=ETaskStatus::Failed; }

TSet<int32> UTaskMonitor::GetCompletedTaskIDs() const { return CompletedTaskIDs; }

int32 UTaskMonitor::GetActiveTaskID(int32 AgentID) const
{
    for (const FTaskProgress& P : TaskProgresses)
        if (P.AssignedAgentID == AgentID && P.Status == ETaskStatus::InProgress) return P.TaskID;
    return INDEX_NONE;
}

void UTaskMonitor::Update(float DeltaTime, const TArray<FAgentStateSnapshot>& AgentStates)
{
    if (!bInitialized) return;
    ElapsedTime += DeltaTime;
    ProgressCheckAccumulator += DeltaTime;
    if (ProgressCheckAccumulator < Config.ProgressCheckInterval) return;
    const float Step = ProgressCheckAccumulator;
    ProgressCheckAccumulator = 0.0f;
    // 先收集事件，避免委托中的重规划使正在遍历的数组失效。
    TArray<TPair<int32,int32>> Completed, Failed;
    FString ReplanReason;
    for (FTaskProgress& P : TaskProgresses)
    {
        if (P.Status != ETaskStatus::InProgress) continue;
        const FAgentStateSnapshot* State = AgentStates.FindByPredicate(
            [&P](const FAgentStateSnapshot& S){return S.AgentID == P.AssignedAgentID;});
        if (!State) { ReplanReason = TEXT("Agent unavailable"); continue; }
        if (DetectTimeout(P.TaskID))
        {
            P.Status = ETaskStatus::Failed;
            FailedTaskIDs.Add(P.TaskID);
            Failed.Emplace(P.TaskID, P.AssignedAgentID);
            ReplanReason = TEXT("Task deadline exceeded");
            continue;
        }
        if (!P.bHasTarget)
        {
            auto& History = AgentPositionHistory.FindOrAdd(P.AssignedAgentID);
            History.Add(State->State.Position);
            if (History.Num()>60) History.RemoveAt(0,History.Num()-60);
            DetectStalledAgent(P.AssignedAgentID,*State);
            continue;
        }
        const float Distance = FVector::Dist(State->State.Position, P.TargetLocation);
        if (P.InitialDistance <= 0.0f) P.InitialDistance = FMath::Max(Distance, 1.0f);
        const bool bArrived = Distance <= 200.0f && State->State.Velocity.Size() <= 150.0f;
        P.ServiceElapsed = bArrived && ElapsedTime >= P.EarliestStart ? P.ServiceElapsed + FMath::Min(Step,ElapsedTime-P.EarliestStart) : 0.0f;
        P.Progress = bArrived ? 0.9f + 0.1f * FMath::Clamp(P.ServiceElapsed / FMath::Max(P.ServiceDuration,0.01f),0.0f,1.0f)
            : 0.9f * FMath::Clamp(1.0f - Distance/P.InitialDistance,0.0f,1.0f);
        if (bArrived && P.ServiceElapsed >= P.ServiceDuration)
        {
            P.Status = ETaskStatus::Completed;
            P.Progress = 1.0f;
            CompletedTaskIDs.Add(P.TaskID);
            Completed.Emplace(P.TaskID,P.AssignedAgentID);
            continue;
        }
        if (!bArrived)
        {
            TArray<FVector>& History = AgentPositionHistory.FindOrAdd(P.AssignedAgentID);
            History.Add(State->State.Position);
            if (History.Num()>60) History.RemoveAt(0, History.Num()-60);
            if (DetectStalledAgent(P.AssignedAgentID,*State)) ReplanReason=TEXT("Agent stalled");
            if (DetectDeviation(P.AssignedAgentID,*State)) ReplanReason=TEXT("Agent deviated");
        }
    }
    for (const auto& E : Completed) OnTaskCompleted.Broadcast(E.Key,E.Value);
    for (const auto& E : Failed) OnTaskFailed.Broadcast(E.Key,E.Value,TEXT("Deadline"));
    if (!ReplanReason.IsEmpty()) OnReplanRequested.Broadcast(ReplanReason);
}

ETaskStatus UTaskMonitor::GetTaskStatus(int32 TaskID) const
{
	if (CompletedTaskIDs.Contains(TaskID)) return ETaskStatus::Completed;
	if (FailedTaskIDs.Contains(TaskID)) return ETaskStatus::Failed;
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
    if(CompletedTaskIDs.Contains(TaskID)) return 1.0f;
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
	if (GetTotalTaskCount() == 0) return 0.0f;

	float TotalProgress = CompletedTaskIDs.Num();
	for (const FTaskProgress& Progress : TaskProgresses)
	{
		if (!CompletedTaskIDs.Contains(Progress.TaskID)) TotalProgress += Progress.Progress;
	}
	return TotalProgress / FMath::Max(GetTotalTaskCount(),1);
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
	return CompletedTaskIDs.Num();
}

int32 UTaskMonitor::GetTotalTaskCount() const
{
	TSet<int32> IDs=CompletedTaskIDs;
	IDs.Append(FailedTaskIDs);
	for (const FTaskProgress& P : TaskProgresses) IDs.Add(P.TaskID);
	int32 Count=IDs.Num();
	return Count;
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
		if (Count * Config.ProgressCheckInterval > Config.StalledTimeout) // 连续 10 次检查都停滞
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
			float Distance = FMath::Max(0.0f, float(FVector::Dist(State.State.Position, Progress.TargetLocation)) - Progress.InitialDistance);
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
			return ElapsedTime > (Progress.bHasTarget ? Progress.Deadline : Progress.EstimatedEndTime * 1.5f);
		}
	}
	return false;
}
