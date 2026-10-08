// Copyright Epic Games, Inc. All Rights Reserved.

#include "TaskAllocator.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UTaskAllocator::UTaskAllocator()
{
}

TArray<int32> UTaskAllocator::MatchMinimumCost(const TArray<TArray<double>>& Costs)
{
    const int32 Rows=Costs.Num(),Columns=Rows>0 ? Costs[0].Num() : 0;
    TArray<int32> Result;Result.Init(INDEX_NONE,Rows);
    if(Rows==0 || Columns==0) return Result;
    double Scale=1;
    for(const auto& Row:Costs)
    {
        if(Row.Num()!=Columns) return Result;
        for(double Cost:Row) if(FMath::IsFinite(Cost) && FMath::Abs(Cost)<1e100) Scale=FMath::Max(Scale,FMath::Abs(Cost));
    }
    // 虚拟列允许部分匹配，惩罚大于所有有效边的代价差之和，优先保留可执行任务。
    const double IdleCost=(2*Rows+1)*Scale,ForbiddenCost=(Rows+1)*IdleCost;
    const int32 Width=Columns+Rows;
    TArray<double> RowPotential,ColumnPotential;RowPotential.Init(0,Rows+1);ColumnPotential.Init(0,Width+1);
    TArray<int32> Owner,Previous;Owner.Init(0,Width+1);Previous.Init(0,Width+1);
    const auto CostAt=[&](int32 Row,int32 Column)
    {
        if(Column>Columns) return IdleCost;
        const double Cost=Costs[Row-1][Column-1];
        return FMath::IsFinite(Cost) && FMath::Abs(Cost)<1e100 ? Cost : ForbiddenCost;
    };
    for(int32 Row=1;Row<=Rows;++Row)
    {
        Owner[0]=Row;int32 Column=0;
        TArray<double> Slack;Slack.Init(DBL_MAX,Width+1);
        TArray<bool> Visited;Visited.Init(false,Width+1);
        do
        {
            Visited[Column]=true;const int32 ActiveRow=Owner[Column];
            double Delta=DBL_MAX;int32 Next=0;
            for(int32 J=1;J<=Width;++J) if(!Visited[J])
            {
                const double Reduced=CostAt(ActiveRow,J)-RowPotential[ActiveRow]-ColumnPotential[J];
                if(Reduced<Slack[J]) {Slack[J]=Reduced;Previous[J]=Column;}
                if(Slack[J]<Delta) {Delta=Slack[J];Next=J;}
            }
            for(int32 J=0;J<=Width;++J)
            {
                if(Visited[J]) {RowPotential[Owner[J]]+=Delta;ColumnPotential[J]-=Delta;}
                else Slack[J]-=Delta;
            }
            Column=Next;
        } while(Owner[Column]!=0);
        do {const int32 Parent=Previous[Column];Owner[Column]=Owner[Parent];Column=Parent;} while(Column!=0);
    }
    for(int32 J=1;J<=Columns;++J) if(Owner[J]!=0 && CostAt(Owner[J],J)<IdleCost)
        Result[Owner[J]-1]=J-1;
    return Result;
}

void UTaskAllocator::Reset()
{
	CurrentAllocation = FTaskAllocationResult();
	CurrentTasks.Empty();
	bHasValidAllocation = false;
}

FTaskAllocationResult UTaskAllocator::Allocate(
	const TArray<FTaskDescriptor>& Tasks,
	const TArray<FUAVCapability>& UAVCapabilities,
	const FTaskAllocationConfig& Config)
{
	FTaskAllocationResult Result;

	if (Tasks.Num() == 0 || UAVCapabilities.Num() == 0)
	{
		UE_LOG(LogUAVMultiAgent, Warning, TEXT("[TaskAllocator] Empty tasks or agents"));
		return Result;
	}

	// 延迟创建求解器（避免在构造函数中调用 NewObject）
	if (!MILPSolverInstance)
	{
		MILPSolverInstance = NewObject<UMILPSolver>(this);
	}

	// 构建 MILP 模型
	TArray<float> Objective;
	TArray<TArray<float>> AIneq, AEq;
	TArray<float> BIneq, BEq;
	TArray<float> VarLB, VarUB;
	TArray<int32> IntegerIndices;

	BuildMILPModel(Tasks, UAVCapabilities, Objective, AIneq, BIneq, AEq, BEq, VarLB, VarUB, IntegerIndices);

	// 求解 MILP
	FMILPResult MILPResult = MILPSolverInstance->Solve(
		Objective, AIneq, BIneq, AEq, BEq, VarLB, VarUB, IntegerIndices, Config.MILPConfig);

	if (!MILPResult.bIsFeasible)
	{
		UE_LOG(LogUAVMultiAgent, Warning, TEXT("[TaskAllocator] No feasible allocation found"));
		return Result;
	}

	// 解析结果
	Result = ParseMILPSolution(MILPResult, Tasks, UAVCapabilities);
	Result.SolveTimeSeconds = MILPResult.SolveTimeSeconds;

	CurrentTasks = Tasks;
	// 保存当前分配
	CurrentAllocation = Result;
	bHasValidAllocation = true;

	UE_LOG(LogUAVMultiAgent, Log,
		TEXT("[TaskAllocator] Allocation complete: %d assignments, cost=%.2f, makespan=%.1fs, solve=%.3fs"),
		Result.Assignments.Num(), Result.TotalCost, Result.Makespan, Result.SolveTimeSeconds);

	return Result;
}

FTaskAllocationResult UTaskAllocator::Reallocate(
	const FTaskAllocationResult& PreviousAllocation,
	const TArray<FTaskDescriptor>& NewTasks,
	const TArray<int32>& FailedAgentIDs,
	const TArray<FAgentStateSnapshot>& CurrentStates,
	const TArray<FUAVCapability>& UAVCapabilities,
	const FTaskAllocationConfig& Config)
{
    TArray<FTaskDescriptor> RemainingTasks = CurrentTasks;
    for (const FTaskDescriptor& Task : NewTasks)
    {
        RemainingTasks.RemoveAll([&Task](const FTaskDescriptor& T){return T.TaskID == Task.TaskID;});
        RemainingTasks.Add(Task);
    }
    TArray<FUAVCapability> AvailableAgents;
    for (const FUAVCapability& Cap : UAVCapabilities)
        if (!FailedAgentIDs.Contains(Cap.AgentID)) AvailableAgents.Add(Cap);
    return Allocate(RemainingTasks, AvailableAgents, Config);
}

void UTaskAllocator::BuildMILPModel(
	const TArray<FTaskDescriptor>& Tasks,
	const TArray<FUAVCapability>& Capabilities,
	TArray<float>& OutObjective,
	TArray<TArray<float>>& OutAIneq,
	TArray<float>& OutBIneq,
	TArray<TArray<float>>& OutAEq,
	TArray<float>& OutBEq,
	TArray<float>& OutLB,
	TArray<float>& OutUB,
	TArray<int32>& OutIntegerIndices)
{
	int32 NumAgents = Capabilities.Num();
	int32 NumTasks = Tasks.Num();
	int32 NumVars = NumAgents * NumTasks; // x[i*NumTasks+j] = 1 iff Agent i assigned Task j

	OutObjective.SetNum(NumVars);
	OutLB.SetNum(NumVars);
	OutUB.SetNum(NumVars);
	OutIntegerIndices.SetNum(NumVars);

	// 目标函数：最小化加权代价
	for (int32 i = 0; i < NumAgents; ++i)
	{
		for (int32 j = 0; j < NumTasks; ++j)
		{
			int32 VarIdx = i * NumTasks + j;
			int32 VarIdxFinal = VarIdx; // 避免重复定义

			// 代价 = 飞行距离代价 - 奖励
			float TravelCost = EstimateTravelCost(
				Capabilities[i].CurrentPosition,
				Tasks[j].TargetLocation,
				Capabilities[i].MaxSpeed);

			// 优先级权重
			float PriorityWeight = 1.0f;
			switch (Tasks[j].Priority)
			{
			case ETaskPriority::Critical: PriorityWeight = 4.0f; break;
			case ETaskPriority::High: PriorityWeight = 2.0f; break;
			case ETaskPriority::Normal: PriorityWeight = 1.0f; break;
			case ETaskPriority::Low: PriorityWeight = 0.5f; break;
			}

			// 目标：最小化 (飞行代价 - 奖励 * 优先级)
			OutObjective[VarIdxFinal] = TravelCost - Tasks[j].Reward * PriorityWeight;

			// 变量界：[0, 1]
			OutLB[VarIdxFinal] = 0.0f;
			OutUB[VarIdxFinal] = 1.0f;

			// 整数变量索引
			OutIntegerIndices[VarIdxFinal] = VarIdxFinal;

			// 预处理：能力不匹配的变量直接固定为 0
			if ((Tasks[j].RequiredCapabilities & Capabilities[i].CapabilityMask) != Tasks[j].RequiredCapabilities)
			{
				OutUB[VarIdxFinal] = 0.0f;
			}
			// 载荷不满足的变量固定为 0
			if (Tasks[j].RequiredPayload > (Capabilities[i].MaxPayloadKg - Capabilities[i].CurrentPayloadMass))
			{
				OutUB[VarIdxFinal] = 0.0f;
			}
		}
	}

	// 约束 1: 每个任务最多分配给一个 Agent
	// sum_i x[i*M+j] <= 1 for all j
	OutAEq.SetNum(NumTasks);
	OutBEq.SetNum(NumTasks);
	for (int32 j = 0; j < NumTasks; ++j)
	{
		OutAEq[j].SetNum(NumVars);
		for (int32 i = 0; i < NumAgents; ++i)
		{
			OutAEq[j][i * NumTasks + j] = 1.0f;
		}
		OutBEq[j] = 1.0f; // 每个任务恰好分配一次
	}

	// 约束 2: 每个 Agent 的续航约束
	// sum_j x[i*M+j] * estimated_duration + travel_time <= remaining_flight_time
	OutAIneq.SetNum(NumAgents);
	OutBIneq.SetNum(NumAgents);
	for (int32 i = 0; i < NumAgents; ++i)
	{
		OutAIneq[i].SetNum(NumVars);
		float TravelTimeBudget = 0.0f;

		for (int32 j = 0; j < NumTasks; ++j)
		{
			int32 VarIdx = i * NumTasks + j;
			float TravelTime = EstimateTravelCost(
				Capabilities[i].CurrentPosition,
				Tasks[j].TargetLocation,
				Capabilities[i].MaxSpeed);
			OutAIneq[i][VarIdx] = Tasks[j].EstimatedDuration + TravelTime;
		}

		OutBIneq[i] = Capabilities[i].RemainingFlightTime;
	}
}

FTaskAllocationResult UTaskAllocator::ParseMILPSolution(
	const FMILPResult& Solution,
	const TArray<FTaskDescriptor>& Tasks,
	const TArray<FUAVCapability>& Capabilities)
{
	FTaskAllocationResult Result;
	Result.bIsFeasible = true;

	int32 NumAgents = Capabilities.Num();
	int32 NumTasks = Tasks.Num();
	float GlobalMakespan = 0.0f;

	for (int32 i = 0; i < NumAgents; ++i)
	{
		float AgentEndTime = 0.0f;
		FVector AgentPos = Capabilities[i].CurrentPosition;

		for (int32 j = 0; j < NumTasks; ++j)
		{
			int32 VarIdx = i * NumTasks + j;
			if (VarIdx < Solution.Solution.Num() && Solution.Solution[VarIdx] > 0.5f)
			{
				FTaskAssignment Assignment;
				Assignment.TaskID = Tasks[j].TaskID;
				Assignment.AgentID = Capabilities[i].AgentID;
				Assignment.EstimatedTravelDistance = FVector::Dist(AgentPos, Tasks[j].TargetLocation);
				Assignment.EstimatedStartTime = AgentEndTime;

				float TravelTime = EstimateTravelCost(AgentPos, Tasks[j].TargetLocation, Capabilities[i].MaxSpeed);
				Assignment.EstimatedCompletionTime = AgentEndTime + TravelTime + Tasks[j].EstimatedDuration;

				Result.Assignments.Add(Assignment);
				Result.TotalCost += Assignment.EstimatedTravelDistance;

				AgentEndTime = Assignment.EstimatedCompletionTime;
				AgentPos = Tasks[j].TargetLocation;
			}
		}

		GlobalMakespan = FMath::Max(GlobalMakespan, AgentEndTime);
	}

	Result.Makespan = GlobalMakespan;
	Result.TotalCost = Solution.ObjectiveValue;
	return Result;
}

float UTaskAllocator::EstimateTravelCost(const FVector& From, const FVector& To, float Speed) const
{
	float Distance = FVector::Dist(From, To);
	if (Speed < KINDA_SMALL_NUMBER)
	{
		return MAX_FLT;
	}
	return Distance / Speed; // 返回飞行时间 (s)
}

TArray<FUAVCapability> UTaskAllocator::DeriveCapabilities(
	const TArray<FAgentStateSnapshot>& AgentStates)
{
	TArray<FUAVCapability> Capabilities;
	for (const FAgentStateSnapshot& State : AgentStates)
	{
		FUAVCapability Cap;
		Cap.AgentID = State.AgentID;
		Cap.CurrentPosition = State.State.Position;
		Cap.MaxSpeed = 2000.0f;      // 默认值
		Cap.MaxPayloadKg = 10.0f;     // 默认值
		Cap.RemainingFlightTime = 600.0f; // 默认值
		Capabilities.Add(Cap);
	}
	return Capabilities;
}
