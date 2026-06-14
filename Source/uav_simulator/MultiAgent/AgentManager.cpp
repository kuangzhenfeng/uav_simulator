// Copyright Epic Games, Inc. All Rights Reserved.

#include "AgentManager.h"
#include "../uav_simulator.h"
#include "uav_simulator/Core/UAVPawn.h"
#include "uav_simulator/Core/UAVPlayerController.h"
#include "uav_simulator/Mission/MissionComponent.h"
#include "uav_simulator/Mission/MissionTypes.h"
#include "JointNMPCSolver.h"
#include "TaskAllocator.h"
#include "TaskMonitor.h"
#include "uav_simulator/Debug/UAVHUD.h"
#include "uav_simulator/Debug/UAVLogConfig.h"
#include "uav_simulator/Planning/ObstacleManager.h"
#include "uav_simulator/Planning/TrajectoryTracker.h"
#include "uav_simulator/Environment/WindField.h"
#include "uav_simulator/Environment/EnvironmentTypes.h"
#include "../Scenario/ScenarioLoader.h"
#include "../Scenario/ScenarioTypes.h"
#include "../Scenario/ScenarioEvaluator.h"
#include "uav_simulator/Utility/Filter.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "UObject/SavePackage.h"
#include "UObject/ConstructorHelpers.h"

DEFINE_LOG_CATEGORY_STATIC(LogAgentManager, Log, All);

AMultiAgentGameMode::AMultiAgentGameMode(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	// 设置 HUD 和 PlayerController
	HUDClass = AUAVHUD::StaticClass();
	PlayerControllerClass = AUAVPlayerController::StaticClass();

	// 默认 UAV 蓝图类：编译期用 FClassFinder 设默认值为 BP_UAVPawn_Default。
	// 该蓝图自带 AIController + 行为树，使 ScenarioLoader 回退 Spawn 的 lead UAV
	// 能走完整飞行驱动链（行为树读 MissionComponent 航点 → 生成轨迹 → 轨迹跟踪）。
	// 蓝图缺失时回退到空（LoadSynchronous 时再退化到裸 AUAVPawn，保证不崩溃）。
	static ConstructorHelpers::FClassFinder<AUAVPawn> DefaultUAVBPFinder(
		TEXT("/Game/UAV/Blueprints/UAVs/BP_UAVPawn_Default"));
	if (DefaultUAVBPFinder.Succeeded())
	{
		DefaultUAVClass = DefaultUAVBPFinder.Class;
	}

	// 创建场景级 WindField 单例（ADR-0002）：风场属环境，全关卡共享。
	// 默认配置保持与原 UAVPawn 自建 WindField 一致，保证行为不退化。
	WindField = CreateDefaultSubobject<UWindField>(TEXT("SceneWindField"));
	{
		FWindConfig DefaultWindConfig;
		DefaultWindConfig.bEnabled = true;
		DefaultWindConfig.WindType = EWindFieldType::Constant;
		DefaultWindConfig.SteadyWindVelocity = FVector(300.0f, 0.0f, 0.0f); // X 方向 300 cm/s
		DefaultWindConfig.AirDensity = 1.225f;
		DefaultWindConfig.DragArea = 0.04f;
		DefaultWindConfig.DragCoefficient = 1.0f;
		WindField->SetWindConfig(DefaultWindConfig);
	}
}

void AMultiAgentGameMode::BeginPlay()
{
	Super::BeginPlay();

	// 创建联合 NMPC 求解器实例
	JointNMPCSolverInstance = NewObject<UJointNMPCSolver>(this);

	// 创建任务分配器和监控器
	TaskAllocatorInstance = NewObject<UTaskAllocator>(this);
	TaskMonitorInstance = NewObject<UTaskMonitor>(this);

	// 场景系统装配（ADR-0001）：解析 -Scenario= 命令行（优先），否则用 DefaultScenario。
	LoadAndAssembleScenario();

	UE_LOG(LogUAVMultiAgent, Log, TEXT("[AgentManager] Initialized, all subsystems created"));
}

void AMultiAgentGameMode::LoadAndAssembleScenario()
{
	// 命令行 -Scenario=<资产路径> 优先
	FString ScenarioAssetPath;
	const bool bFromCmdLine = FParse::Value(FCommandLine::Get(), TEXT("-Scenario="), ScenarioAssetPath);

	UScenario* ScenarioToLoad = nullptr;
	if (bFromCmdLine && !ScenarioAssetPath.IsEmpty())
	{
		const FString PackagePath = ScenarioAssetPath;
		ScenarioToLoad = LoadObject<UScenario>(nullptr, *PackagePath);
		UE_LOG(LogUAVMultiAgent, Log, TEXT("[Scenario] Loaded from command line: %s (%s)"),
			ScenarioToLoad ? *ScenarioToLoad->Name : TEXT("FAILED"), *PackagePath);
	}
	else
	{
		ScenarioToLoad = DefaultScenario.LoadSynchronous();
		UE_LOG(LogUAVMultiAgent, Log, TEXT("[Scenario] Loaded from DefaultScenario: %s"),
			ScenarioToLoad ? *ScenarioToLoad->Name : TEXT("none"));
	}

	if (!ScenarioToLoad)
	{
		return; // 无场景，保持原行为（非场景化关卡零影响）
	}

	ActiveScenario = ScenarioToLoad;
	UScenarioLoader* Loader = NewObject<UScenarioLoader>(this);
	ScenarioLoaderInstance = Loader;

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	// 装配顺序：风场 → 障碍 → 机队/任务（机队物理积分前风场已就绪）
	Loader->AssembleWind(ScenarioToLoad, WindField);

	// 障碍装配到首个 UAV 的 ObstacleManager（机队 Spawn 后才有）。
	// 这里先记录待装配，机队 Spawn 完成后立即装配。
	TArray<AUAVPawn*> Fleet;
	// 默认 UAV 类：优先用数据驱动的 DefaultUAVClass（BP_UAVPawn_Default，带 AIController+行为树），
	// 蓝图未配置时回退裸 AUAVPawn（最终兜底，保证不崩，但无飞行驱动）。
	TSubclassOf<AUAVPawn> FallbackUAVClass;
	if (!DefaultUAVClass.IsNull())
	{
		FallbackUAVClass = DefaultUAVClass.LoadSynchronous();
	}
	if (!FallbackUAVClass)
	{
		FallbackUAVClass = AUAVPawn::StaticClass();
	}
	Loader->AssembleFleetAndMission(ScenarioToLoad, World, Fleet, FallbackUAVClass);
	ScenarioFleet = Fleet;

	if (Fleet.Num() > 0 && Fleet[0])
	{
		if (UObstacleManager* ObstacleManager = Fleet[0]->FindComponentByClass<UObstacleManager>())
		{
			Loader->AssembleObstacles(ScenarioToLoad, ObstacleManager, World);
		}

		// 挂载验收器组件：周期快照 + 最终判决
		ScenarioEvaluatorComponent = NewObject<UScenarioEvaluatorComponent>(this);
		ScenarioEvaluatorComponent->RegisterComponent();
		ScenarioEvaluatorComponent->Initialize(ScenarioToLoad, Fleet[0]);
	}

	UE_LOG(LogUAVMultiAgent, Log, TEXT("[Scenario] Assembly complete: %d UAV(s)"), Fleet.Num());
}

void AMultiAgentGameMode::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	SCOPE_CYCLE_COUNTER(STAT_AgentManagerTick);

	UE_LOG_THROTTLE(5.0, LogUAVMultiAgent, Log, TEXT("[AgentManager] Tick, dt=%.4f, agents=%d"), DeltaTime, AgentRegistry.Num());

	// 刷新状态缓存
	StateCacheAccumulator += DeltaTime;
	if (StateCacheAccumulator >= StateCacheUpdateInterval)
	{
		StateCacheAccumulator = 0.0f;
		RefreshStateCache();
	}

	// 联合 NMPC 求解（仅 Leader 触发）
	if (AgentRegistry.Num() > 1 && JointNMPCSolverInstance)
	{
		float SolveInterval = 1.0f / FMath::Max(JointNMPCConfig.SolveFrequency, 1.0f);
		JointNMPCSolveAccumulator += DeltaTime;
		if (JointNMPCSolveAccumulator >= SolveInterval)
		{
			JointNMPCSolveAccumulator = 0.0f;
			SolveJointNMPC();
		}
	}

	// 任务监控更新
	if (TaskMonitorInstance && AgentRegistry.Num() > 0)
	{
		TaskMonitorInstance->Update(DeltaTime, GetAllAgentStates());
	}

	// 每 5s 输出任务进度汇总
	UE_LOG_THROTTLE(5.0, LogUAVMetrics, Log,
		TEXT("[TASK_SUMMARY] Completed=%d Total=%d"),
		TaskMonitorInstance ? TaskMonitorInstance->GetCompletedTaskCount() : 0,
		TaskMonitorInstance ? TaskMonitorInstance->GetTotalTaskCount() : 0);
}

int32 AMultiAgentGameMode::RegisterAgent(AUAVPawn* Agent)
{
	if (!Agent)
	{
		UE_LOG(LogUAVMultiAgent, Warning, TEXT("[AgentManager] RegisterAgent called with null agent"));
		return -1;
	}

	int32 AssignedID = NextAgentID++;
	AgentRegistry.Add(AssignedID, Agent);

	// 初始化状态缓存
	FAgentStateSnapshot InitialState;
	InitialState.AgentID = AssignedID;
	InitialState.State = Agent->GetUAVState();
	InitialState.Timestamp = GetWorld()->GetTimeSeconds();
	StateCache.Add(AssignedID, InitialState);

	UE_LOG(LogUAVMultiAgent, Log, TEXT("[AgentManager] Agent %d registered, total agents: %d"),
		AssignedID, AgentRegistry.Num());

	return AssignedID;
}

void AMultiAgentGameMode::UnregisterAgent(int32 AgentID)
{
	if (AgentRegistry.Remove(AgentID) > 0)
	{
		StateCache.Remove(AgentID);
		JointNMPCResultCache.Remove(AgentID);
		UE_LOG(LogUAVMultiAgent, Log, TEXT("[AgentManager] Agent %d unregistered, remaining: %d"),
			AgentID, AgentRegistry.Num());
	}
}

bool AMultiAgentGameMode::GetAgentState(int32 AgentID, FAgentStateSnapshot& OutState) const
{
	if (const FAgentStateSnapshot* Found = StateCache.Find(AgentID))
	{
		OutState = *Found;
		return true;
	}
	return false;
}

TArray<FAgentStateSnapshot> AMultiAgentGameMode::GetAllAgentStates() const
{
	TArray<FAgentStateSnapshot> Result;
	StateCache.GenerateValueArray(Result);
	return Result;
}

TArray<FAgentStateSnapshot> AMultiAgentGameMode::GetNeighborStates(int32 RequesterID, float Radius) const
{
	TArray<FAgentStateSnapshot> Result;

	// 获取请求方位置
	const FAgentStateSnapshot* RequesterState = StateCache.Find(RequesterID);
	if (!RequesterState)
	{
		return Result;
	}

	FVector RequesterPos = RequesterState->State.Position;
	float RadiusSq = Radius * Radius;

	for (const auto& Pair : StateCache)
	{
		if (Pair.Key == RequesterID)
		{
			continue;
		}

		float DistSq = FVector::DistSquared(RequesterPos, Pair.Value.State.Position);
		if (DistSq <= RadiusSq)
		{
			Result.Add(Pair.Value);
		}
	}

	return Result;
}

int32 AMultiAgentGameMode::GetAgentCount() const
{
	return AgentRegistry.Num();
}

void AMultiAgentGameMode::SetFormation(const FFormationConfig& InConfig)
{
	FFormationConfig PrevConfig = FormationConfig;
	FormationConfig = InConfig;

	// 使编队偏移缓存失效
	CachedFormationType = EFormationType::None;

	// 计算并分发编队偏移量给各 Agent
	int32 NumAgents = AgentRegistry.Num();
	if (NumAgents == 0 || FormationConfig.Type == EFormationType::None)
	{
		return;
	}

	TArray<FVector> Offsets = ComputeFormationOffsets(NumAgents);
	int32 OffsetIdx = 0;
	for (const auto& Pair : AgentRegistry)
	{
		if (Pair.Value.IsValid() && OffsetIdx < Offsets.Num())
		{
			AUAVPawn* Pawn = Pair.Value.Get();
			// 编队偏移通过 FormationComponent 设置
			// UAVPawn 集成后实现
			OffsetIdx++;
		}
	}

	UE_LOG(LogUAVMultiAgent, Log, TEXT("[AgentManager] Formation changed to %s, spacing=%.0f, agents=%d"),
		*UEnum::GetValueAsString(FormationConfig.Type),
		FormationConfig.Spacing,
		NumAgents);
}

bool AMultiAgentGameMode::GetFormationTarget(int32 AgentID, FVector& OutTarget) const
{
	if (FormationConfig.Type == EFormationType::None)
	{
		return false;
	}

	// 获取偏移量数组
	TArray<FVector> Offsets = ComputeFormationOffsets(AgentRegistry.Num());
	int32 AgentIndex = 0;
	for (const auto& Pair : AgentRegistry)
	{
		if (Pair.Key == AgentID)
		{
			break;
		}
		AgentIndex++;
	}

	if (AgentIndex >= Offsets.Num())
	{
		return false;
	}

	// 获取 Leader 位置
	FVector LeaderPosition = FVector::ZeroVector;
	if (FormationConfig.LeaderID >= 0)
	{
		const FAgentStateSnapshot* LeaderState = StateCache.Find(FormationConfig.LeaderID);
		if (LeaderState)
		{
			LeaderPosition = LeaderState->State.Position;
		}
	}
	else
	{
		// 质心模式：计算所有 Agent 的质心
		FVector Centroid = FVector::ZeroVector;
		int32 Count = 0;
		for (const auto& Pair : StateCache)
		{
			Centroid += Pair.Value.State.Position;
			Count++;
		}
		if (Count > 0)
		{
			LeaderPosition = Centroid / Count;
		}
	}

	OutTarget = LeaderPosition + Offsets[AgentIndex];
	return true;
}

void AMultiAgentGameMode::SetJointNMPCCache(int32 AgentID, const FVector& Acceleration)
{
	JointNMPCResultCache.Add(AgentID, Acceleration);
}

bool AMultiAgentGameMode::GetJointNMPCCache(int32 AgentID, FVector& OutAcceleration) const
{
	const FVector* Found = JointNMPCResultCache.Find(AgentID);
	if (Found)
	{
		OutAcceleration = *Found;
		return true;
	}
	return false;
}

void AMultiAgentGameMode::RefreshStateCache()
{
	UE_LOG_THROTTLE(5.0, LogUAVMultiAgent, Log, TEXT("[AgentManager] RefreshStateCache: AgentRegistry.Num=%d, StateCache.Num=%d"),
		AgentRegistry.Num(), StateCache.Num());

	// 拷贝 AgentRegistry 的 key 集合，避免迭代时修改
	TArray<int32> AgentIDs;
	AgentRegistry.GenerateKeyArray(AgentIDs);

	for (int32 AgentID : AgentIDs)
	{
		TWeakObjectPtr<AUAVPawn>* PawnPtr = AgentRegistry.Find(AgentID);
		if (PawnPtr && PawnPtr->IsValid())
		{
			AUAVPawn* Pawn = PawnPtr->Get();
			FAgentStateSnapshot& Snapshot = StateCache.FindOrAdd(AgentID);
			Snapshot.AgentID = AgentID;
			Snapshot.State = Pawn->GetUAVState();
			Snapshot.TargetPosition = Pawn->GetTargetPosition();
			Snapshot.Timestamp = GetWorld()->GetTimeSeconds();
			Snapshot.NMPCAcceleration = Pawn->GetNMPCAcceleration();
		}
	}
}

void AMultiAgentGameMode::SolveJointNMPC()
{
	if (!JointNMPCSolverInstance || AgentRegistry.Num() < 2)
	{
		return;
	}

	// 收集所有 Agent 的状态
	TArray<FAgentStateSnapshot> AllStates = GetAllAgentStates();

	// 收集所有 Agent 的参考轨迹点
	TArray<TArray<FVector>> RefPointsPerAgent;
	// 拷贝 AgentRegistry 的 key 集合，避免迭代时潜在的容器修改
	TArray<int32> RefAgentIDs;
	AgentRegistry.GenerateKeyArray(RefAgentIDs);
	for (int32 AgentID : RefAgentIDs)
	{
		TArray<FVector> RefPoints;
		TWeakObjectPtr<AUAVPawn>* PawnPtr = AgentRegistry.Find(AgentID);
		if (PawnPtr && PawnPtr->IsValid())
		{
			AUAVPawn* Pawn = PawnPtr->Get();
			// 从 TrajectoryTracker 采样未来 N+1 个参考点
			UTrajectoryTracker* Tracker = Pawn->GetTrajectoryTracker();
			if (Tracker && Tracker->IsTracking())
			{
				int32 N = JointNMPCConfig.BaseConfig.Solver.PredictionSteps;
				float Dt = JointNMPCConfig.BaseConfig.GetDt();
				float CurrentTime = Tracker->GetCurrentTime();
				for (int32 i = 0; i <= N; ++i)
				{
					RefPoints.Add(Tracker->GetDesiredState(CurrentTime + i * Dt).Position);
				}
			}
			else
			{
				// 无轨迹跟踪时回退到当前位置
				RefPoints.Add(Pawn->GetUAVState().Position);
			}
		}
		RefPointsPerAgent.Add(RefPoints);
	}

	// 收集静态障碍物（从任意 Agent 获取，假设共享）
	TArray<FObstacleInfo> StaticObstacles;
	for (int32 AgentID : RefAgentIDs)
	{
		TWeakObjectPtr<AUAVPawn>* PawnPtr = AgentRegistry.Find(AgentID);
		if (PawnPtr && PawnPtr->IsValid())
		{
			AUAVPawn* Pawn = PawnPtr->Get();
			UObstacleManager* ObsMgr = Pawn->GetObstacleManager();
			if (ObsMgr)
			{
				StaticObstacles = ObsMgr->GetPreregisteredObstacles();
				break;
			}
		}
	}

	// 调用联合 NMPC 求解
	FJointNMPCSolveResult Result = JointNMPCSolverInstance->Solve(
		AllStates, RefPointsPerAgent, StaticObstacles, JointNMPCConfig);

	// 缓存每个 Agent 的加速度结果
	if (Result.bConverged)
	{
		for (int32 i = 0; i < AllStates.Num() && i < Result.OptimalAccelerations.Num(); ++i)
		{
			SetJointNMPCCache(AllStates[i].AgentID, Result.OptimalAccelerations[i]);
		}
	}
		UE_LOG(LogUAVMultiAgent, Log, TEXT("[JointNMPC] Solved: agents=%d cost=%.1f converged=%s"),
			AllStates.Num(), Result.TotalCost, Result.bConverged ? TEXT("Y") : TEXT("N"));
}

// ---- 任务分配 ----

FTaskAllocationResult AMultiAgentGameMode::SubmitTasks(const TArray<FTaskDescriptor>& Tasks)
{
	if (!TaskAllocatorInstance)
	{
		UE_LOG(LogUAVMultiAgent, Error, TEXT("[AgentManager] TaskAllocator not initialized"));
		FTaskAllocationResult EmptyResult;
		return EmptyResult;
	}

	// 从注册表派生 UAV 能力
	TArray<FUAVCapability> Capabilities = UTaskAllocator::DeriveCapabilities(GetAllAgentStates());

	// 求解分配
	FTaskAllocationResult Result = TaskAllocatorInstance->Allocate(Tasks, Capabilities, TaskAllocationConfig);

	// 初始化监控
	if (Result.bIsFeasible && TaskMonitorInstance)
	{
		TaskMonitorInstance->Initialize(Result, TaskMonitorConfig);
	}

	// 将分配结果推送到各 Agent 的 MissionComponent
	if (Result.bIsFeasible)
	{
		for (const FTaskAssignment& Assignment : Result.Assignments)
		{
			TWeakObjectPtr<AUAVPawn>* PawnPtr = AgentRegistry.Find(Assignment.AgentID);
			if (PawnPtr && PawnPtr->IsValid())
			{
				AUAVPawn* Pawn = PawnPtr->Get();
				UMissionComponent* Mission = Pawn->GetMissionComponent();
				if (Mission)
				{
					// 查找任务描述
					for (const FTaskDescriptor& Task : Tasks)
					{
						if (Task.TaskID == Assignment.TaskID)
						{
							Mission->ClearWaypoints();
							FMissionWaypoint WP;
							WP.Position = Task.TargetLocation;
							WP.DesiredSpeed = 1000.0f; // 默认速度
							Mission->AddMissionWaypoint(WP);
							Mission->StartMission();
							break;
						}
					}
				}
			}
		}
	}

	UE_LOG(LogUAVMultiAgent, Log, TEXT("[AgentManager] Submitted %d tasks, %d assignments"),
		Tasks.Num(), Result.Assignments.Num());

	return Result;
}

FTaskAllocationResult AMultiAgentGameMode::TriggerReplan(const FString& Reason)
{
	UE_LOG(LogUAVMultiAgent, Log, TEXT("[AgentManager] Replan triggered: %s"), *Reason);

	if (!TaskAllocatorInstance || !TaskAllocatorInstance->HasValidAllocation())
	{
		FTaskAllocationResult EmptyResult;
		return EmptyResult;
	}

	// 获取当前 Agent 状态
	TArray<FAgentStateSnapshot> CurrentStates = GetAllAgentStates();
	TArray<FUAVCapability> Capabilities = UTaskAllocator::DeriveCapabilities(CurrentStates);

	// 使用当前分配进行重规划
	return TaskAllocatorInstance->Reallocate(
		TaskAllocatorInstance->GetCurrentAllocation(),
		TArray<FTaskDescriptor>(), // 无新任务
		TArray<int32>(),           // 无失败 Agent
		CurrentStates,
		Capabilities,
		TaskAllocationConfig);
}

FTaskAllocationResult AMultiAgentGameMode::GetCurrentTaskAllocation() const
{
	if (TaskAllocatorInstance)
	{
		return TaskAllocatorInstance->GetCurrentAllocation();
	}
	FTaskAllocationResult EmptyResult;
	return EmptyResult;
}

// ---- 编队计算 ----

TArray<FVector> AMultiAgentGameMode::ComputeFormationOffsets(int32 NumAgents) const
{
	TArray<FVector> Offsets;

	if (NumAgents <= 0 || FormationConfig.Type == EFormationType::None)
	{
		return Offsets;
	}

	// 检查缓存是否有效
	if (CachedFormationType == FormationConfig.Type &&
		CachedFormationNumAgents == NumAgents)
	{
		return CachedFormationOffsets;
	}

	float S = FormationConfig.Spacing;

	switch (FormationConfig.Type)
	{
	case EFormationType::Line:
		// 线形编队：沿 X 轴排列
		for (int32 i = 0; i < NumAgents; ++i)
		{
			Offsets.Add(FVector((i - (NumAgents - 1) / 2.0f) * S, 0.0f, 0.0f));
		}
		break;

	case EFormationType::VShape:
		// V 形编队
		for (int32 i = 0; i < NumAgents; ++i)
		{
			// Leader 在最前方（索引0），其余在两侧交替
			if (i == 0)
			{
				Offsets.Add(FVector::ZeroVector);
			}
			else
			{
				int32 Side = (i % 2 == 1) ? 1 : -1; // 左右交替
				int32 Row = (i + 1) / 2;
				Offsets.Add(FVector(-Row * S * 0.5f, Side * Row * S, 0.0f));
			}
		}
		break;

	case EFormationType::Circle:
		// 环形编队：等角分布
		for (int32 i = 0; i < NumAgents; ++i)
		{
			float Angle = 2.0f * PI * i / NumAgents;
			float Radius = S / (2.0f * FMath::Sin(PI / FMath::Max(NumAgents, 1)));
			Offsets.Add(FVector(Radius * FMath::Cos(Angle), Radius * FMath::Sin(Angle), 0.0f));
		}
		break;

	case EFormationType::Diamond:
		// 菱形编队
		if (NumAgents == 1)
		{
			Offsets.Add(FVector::ZeroVector);
		}
		else if (NumAgents == 2)
		{
			Offsets.Add(FVector(S * 0.5f, 0.0f, 0.0f));
			Offsets.Add(FVector(-S * 0.5f, 0.0f, 0.0f));
		}
		else if (NumAgents == 3)
		{
			Offsets.Add(FVector(0.0f, S * 0.5f, 0.0f));
			Offsets.Add(FVector(-S * 0.5f, 0.0f, 0.0f));
			Offsets.Add(FVector(0.0f, -S * 0.5f, 0.0f));
		}
		else
		{
			// 4+ 架：前-左-后-右 + 额外位置
			Offsets.Add(FVector(S, 0.0f, 0.0f));       // 前
			Offsets.Add(FVector(0.0f, S, 0.0f));        // 右
			Offsets.Add(FVector(-S, 0.0f, 0.0f));       // 后
			Offsets.Add(FVector(0.0f, -S, 0.0f));       // 左
			// 额外 Agent 填充到间隙
			for (int32 i = 4; i < NumAgents; ++i)
			{
				float Angle = 2.0f * PI * i / NumAgents;
				Offsets.Add(FVector(S * FMath::Cos(Angle), S * FMath::Sin(Angle), 0.0f));
			}
		}
		break;

	default:
		for (int32 i = 0; i < NumAgents; ++i)
		{
			Offsets.Add(FVector::ZeroVector);
		}
		break;
	}

	// 更新缓存（const_cast 用于缓存优化）
	AMultiAgentGameMode* MutableThis = const_cast<AMultiAgentGameMode*>(this);
	MutableThis->CachedFormationOffsets = Offsets;
	MutableThis->CachedFormationNumAgents = NumAgents;
	MutableThis->CachedFormationType = FormationConfig.Type;

	return Offsets;
}
