// Copyright Epic Games, Inc. All Rights Reserved.

#include "AgentManager.h"
#include "../uav_simulator.h"
#include "uav_simulator/Core/UAVPawn.h"
#include "uav_simulator/Core/UAVPlayerController.h"
#include "uav_simulator/Mission/MissionComponent.h"
#include "uav_simulator/Mission/MissionTypes.h"
#include "uav_simulator/AI/Tasks/BTTask_ExitSimulation.h"
#include "uav_simulator/Planning/DynamicObstacleActor.h"
#include "JointNMPCSolver.h"
#include "TaskAllocator.h"
#include "TaskMonitor.h"
#include "../AI/UAVAIController.h"
#include "../Planning/AStarPathPlanner.h"
#include "../Planning/TrajectoryOptimizer.h"
#include "FormationComponent.h"
#include "uav_simulator/Debug/UAVHUD.h"
#include "uav_simulator/Debug/UAVLogConfig.h"
#include "uav_simulator/Planning/ObstacleManager.h"
#include "uav_simulator/Planning/TrajectoryTracker.h"
#include "uav_simulator/Environment/WindField.h"
#include "uav_simulator/Environment/EnvironmentTypes.h"
#include "../Scenario/ScenarioLoader.h"
#include "../Scenario/ScenarioTypes.h"
#include "../Scenario/ScenarioEvaluator.h"
#include "../Telemetry/TelemetryRecorder.h"
#include "../Network/HttpControlComponent.h"
#include "uav_simulator/Utility/Filter.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "UObject/SavePackage.h"
#include "UObject/ConstructorHelpers.h"
#include "EngineUtils.h"
#include "GameFramework/WorldSettings.h"

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
	TaskMonitorInstance->OnTaskFailed.AddDynamic(this, &AMultiAgentGameMode::HandleTaskFailed);
	TaskMonitorInstance->OnTaskCompleted.AddDynamic(this, &AMultiAgentGameMode::HandleTaskCompleted);
	TaskMonitorInstance->OnReplanRequested.AddDynamic(this, &AMultiAgentGameMode::HandleTaskReplan);

	// 创建遥测记录器并注入共享风场：作为可视化 Web 的专用 ndjson 数据源。
	// 始终创建（非场景化关卡也记录，便于单机 PIE 可视化）。
	TelemetryRecorder = NewObject<UTelemetryRecorder>(this);
	TelemetryRecorder->RegisterComponent();
	TelemetryRecorder->SetWindField(WindField);

	// 场景系统装配（ADR-0001）：解析 -Scenario= 命令行（优先），否则用 DefaultScenario。
	// 场景系统装配（ADR-0001）：解析 -Scenario= 命令行（优先），否则用 DefaultScenario。
	LoadAndAssembleScenario();

	// Web 控制端：headless 也启动，供外部（Python 反代）触发重载/调参。
	// 独立于 SoftUEBridge（后者 -unattended 跳过），命令面板正是为自动化仿真设计。
	HttpControl = NewObject<UHttpControlComponent>(this);
	HttpControl->RegisterComponent();

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

	AssembleScenario(ScenarioToLoad, /*bIsReload=*/false);
}

void AMultiAgentGameMode::AssembleScenario(UScenario* ScenarioToLoad, bool bIsReload)
{
	if (!ScenarioToLoad)
	{
		return;
	}

	ActiveScenario = ScenarioToLoad;
	FMath::RandInit(ScenarioToLoad->RandomSeed);
	FormationConfig = ScenarioToLoad->Formation;
	JointNMPCSolverInstance = NewObject<UJointNMPCSolver>(this);
	if (TelemetryRecorder)
	{
		TelemetryRecorder->SetScenarioName(ScenarioToLoad->Name);
	}
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
	if (ScenarioToLoad->bCooperationDemo)
		for (AUAVPawn* Pawn : Fleet)
		{
			if (auto* AI = Cast<AUAVAIController>(Pawn->GetController())) AI->StopBehaviorTree();
			Pawn->StopTrajectoryTracking();
			Pawn->SetTargetPosition(Pawn->GetUAVState().Position);
		}
	RefreshStateCache();
	if (FormationConfig.Type!=EFormationType::None)
    {
        auto* LeaderPtr=AgentRegistry.Find(FormationConfig.LeaderID);
        const FVector Anchor=LeaderPtr && LeaderPtr->IsValid() ? LeaderPtr->Get()->GetUAVState().Position : FVector::ZeroVector;
        for(const auto& Pair:AgentRegistry) if(Pair.Value.IsValid() && Pair.Value->GetFormationComponent())
            Pair.Value->GetFormationComponent()->InitializeOffset(Pair.Value->GetUAVState().Position-Anchor);
    }
    SetFormation(ScenarioToLoad->Formation);

	if (Fleet.Num() > 0 && Fleet[0])
	{
		if (UObstacleManager* ObstacleManager = Fleet[0]->FindComponentByClass<UObstacleManager>())
		{
			Loader->AssembleObstacles(ScenarioToLoad, ObstacleManager, World);
		}

		if (!ScenarioToLoad->Agriculture.bEnabled && !ScenarioToLoad->Tasks.IsEmpty()) SubmitTasks(ScenarioToLoad->Tasks);
		else if (ScenarioToLoad->bCooperationDemo && !ScenarioToLoad->Agriculture.bEnabled)
			for (AUAVPawn* Pawn : Fleet) StartScenarioWaypoint(Pawn);

		// 挂载验收器组件：周期快照 + 最终判决
		// 热重载时复用旧组件（先 Reset 再 Initialize），冷启动则新建。
		if (!ScenarioEvaluatorComponent)
		{
			ScenarioEvaluatorComponent = NewObject<UScenarioEvaluatorComponent>(this);
			ScenarioEvaluatorComponent->RegisterComponent();
		}
		else
		{
			ScenarioEvaluatorComponent->Reset();
		}
		ScenarioEvaluatorComponent->Initialize(ScenarioToLoad, Fleet[0]);
		// 把遥测记录器注入验收器，使周期/最终判决同步落 ndjson
		ScenarioEvaluatorComponent->SetTelemetryRecorder(TelemetryRecorder);
	}

	// 热重载完成：truncate 重开 telemetry 并写一行 reload 事件通知前端刷新。
	if (bIsReload && TelemetryRecorder)
	{
		TelemetryRecorder->ResetForNewScenario();
		TelemetryRecorder->WriteReloadMarker();
	}

	UE_LOG(LogUAVMultiAgent, Log, TEXT("[Scenario] Assembly complete: %d UAV(s)%s"),
		Fleet.Num(), bIsReload ? TEXT(" (reload)") : TEXT(""));
}

void AMultiAgentGameMode::TearDownForReload()
{
	UWorld* World = GetWorld();

	TaskPool.Empty();
    TaskAssignedAgents.Empty(); TaskQueues.Empty(); FailedAgentIDs.Empty(); CompletedTaskIDs.Empty();
    FailedTaskIDs.Empty();
	AppliedAllocation = FTaskAllocationResult(); LastReplanReason.Empty();
	// 1. 取消 BTTask_ExitSimulation 挂起的延迟退出，避免旧场景任务完成误杀进程
	UBTTask_ExitSimulation::CancelPendingExit();

	// 2. 销毁旧机队：触发各 AUAVPawn::EndPlay（反注册 Agent、清残骸、断风场引用）
	for (TObjectPtr<AUAVPawn>& Pawn : ScenarioFleet)
	{
		if (Pawn)
		{
			Pawn->ResetMetrics();
			Pawn->Destroy();
		}
	}
	ScenarioFleet.Empty();

	// 3. 兜底清理注册表（EndPlay 反注册后应为空，此处防御性清空）
	AgentRegistry.Empty();
	StateCache.Empty();
	JointNMPCResultCache.Empty();
	NextAgentID = 0;
	JointNMPCSolveAccumulator = 0.0f;
	StateCacheAccumulator = 0.0f;
	CachedFormationType = EFormationType::None;

	// 4. 销毁关卡中残留的动态障碍驱动 Actor（热重载可能遗留未随 Pawn 销毁的 Actor）
	if (World)
	{
		for (TActorIterator<ADynamicObstacleActor> It(World); It; ++It)
		{
			if (*It)
			{
				It->Destroy();
			}
		}
	}

	// 5. 复位持久子组件状态
	if (ScenarioEvaluatorComponent)
	{
		ScenarioEvaluatorComponent->Reset();
	}
	if (TelemetryRecorder)
	{
		TelemetryRecorder->ResetForNewScenario();
	}
	if (WindField)
	{
		WindField->ResetDynamicState();
	}
	if (TaskAllocatorInstance)
	{
		TaskAllocatorInstance->Reset();
	}
	if (TaskMonitorInstance)
	{
		TaskMonitorInstance->Reset();
	}
}

bool AMultiAgentGameMode::ReloadScenario(UScenario* NewScenario)
{
	if (!NewScenario)
	{
		UE_LOG(LogUAVMultiAgent, Warning, TEXT("[Scenario] ReloadScenario: null scenario"));
		return false;
	}
	if (bReloading)
	{
		UE_LOG(LogUAVMultiAgent, Warning, TEXT("[Scenario] ReloadScenario: already reloading, rejected"));
		return false;
	}

	bReloading = true;
	UE_LOG(LogUAVMultiAgent, Log, TEXT("[Scenario] Reload start: '%s'"), *NewScenario->Name);

	TearDownForReload();
	AssembleScenario(NewScenario, /*bIsReload=*/true);

	bReloading = false;
	UE_LOG(LogUAVMultiAgent, Log, TEXT("[Scenario] Reload done: %d UAV(s)"), ScenarioFleet.Num());
	return true;
}

void AMultiAgentGameMode::SetSlomo(float Scale)
{
	if (Scale <= 0.0f)
	{
		UE_LOG(LogUAVMultiAgent, Warning, TEXT("[AgentManager] SetSlomo: invalid scale %.3f"), Scale);
		return;
	}
	if (UWorld* World = GetWorld())
	{
		if (AWorldSettings* WS = World->GetWorldSettings())
		{
			WS->SetTimeDilation(Scale);
			UE_LOG(LogUAVMultiAgent, Log, TEXT("[AgentManager] Slomo set to %.3f"), Scale);
		}
	}
}

TArray<AUAVPawn*> AMultiAgentGameMode::GetScenarioFleet() const
{
	TArray<AUAVPawn*> Result;
	Result.Reserve(ScenarioFleet.Num());
	for (const TObjectPtr<AUAVPawn>& Pawn : ScenarioFleet)
	{
		Result.Add(Pawn.Get());
	}
	return Result;
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

	TArray<int32> CrashedIDs;
	for (const auto& Pair : AgentRegistry)
		if (Pair.Value.IsValid() && Pair.Value->IsCrashed() && !FailedAgentIDs.Contains(Pair.Key)) CrashedIDs.Add(Pair.Key);
	for (int32 ID : CrashedIDs) FailAgent(ID);
	// 联合 NMPC 求解（仅 Leader 触发）
	if (AgentRegistry.Num() > 1 && JointNMPCSolverInstance)
	{
		float SolveInterval = 1.0f / FMath::Max(JointNMPCConfig.SolveFrequency, 1.0f);
		JointNMPCSolveAccumulator += DeltaTime;
		if (JointNMPCSolveAccumulator >= SolveInterval)
		{
			JointNMPCSolveAccumulator = 0.0f;
			UpdateTrafficScales();
			SolveJointNMPC();
		}
	}

	// 演示航线由统一入口逐航点推进，防止行为树重复覆盖参考轨迹。
    if (ActiveScenario && ActiveScenario->bCooperationDemo && !ActiveScenario->Agriculture.bEnabled && TaskPool.IsEmpty())
        for (AUAVPawn* Pawn : GetScenarioFleet())
        {
            if (!Pawn || Pawn->IsCrashed()) continue;
            auto* Mission = Pawn->GetMissionComponent();
            if (Mission->IsMissionRunning() && Mission->HasReachedCurrentWaypoint())
            {
                Mission->AdvanceToNextWaypoint();
                if (!Mission->IsMissionCompleted()) StartScenarioWaypoint(Pawn);
            }
        }

    // 任务监控更新
	if (TaskMonitorInstance && AgentRegistry.Num() > 0 && !(ActiveScenario && ActiveScenario->Agriculture.bEnabled))
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
	Result.Sort([](const FAgentStateSnapshot& A,const FAgentStateSnapshot& B){return A.AgentID<B.AgentID;});
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
    TArray<int32> IDs;AgentRegistry.GenerateKeyArray(IDs);IDs.Sort();
    const int32 LeaderIndex=IDs.IndexOfByKey(FormationConfig.LeaderID);
    const FVector Anchor=Offsets.IsValidIndex(LeaderIndex) ? Offsets[LeaderIndex] : FVector::ZeroVector;
    for (int32 I=0;I<IDs.Num();++I)
        if (auto* P=AgentRegistry[IDs[I]].Get())
            if (auto* F=P->GetFormationComponent()) F->SetTargetOffset(Offsets[I]-Anchor);

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
	TArray<int32> SortedIDs; AgentRegistry.GenerateKeyArray(SortedIDs); SortedIDs.Sort();
	for (int32 ID : SortedIDs)
	{
		if (ID == AgentID)
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

	FVector Offset=Offsets[AgentIndex];
    const int32 LeaderIndex=SortedIDs.IndexOfByKey(FormationConfig.LeaderID);
    if(Offsets.IsValidIndex(LeaderIndex)) Offset-=Offsets[LeaderIndex];
    if(const auto* Ptr=AgentRegistry.Find(AgentID))
        if(Ptr->IsValid() && Ptr->Get()->GetFormationComponent()) Offset=Ptr->Get()->GetFormationComponent()->GetCurrentOffset();
    OutTarget = LeaderPosition + Offset;
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

void AMultiAgentGameMode::UpdateTrafficScales()
{
    TArray<int32> IDs;
    AgentRegistry.GenerateKeyArray(IDs);
    IDs.Sort();
    // 在完整航迹上预判交会，按稳定次序让行，避免两机同时横向逃逸。
    for (int32 ID : IDs)
    {
        AUAVPawn* Pawn = AgentRegistry[ID].Get();
        if (!Pawn) continue;
        auto* Tracker = Pawn->GetTrajectoryTracker();
        Tracker->SetTrafficScale(1.0f);
        if (FormationConfig.Type != EFormationType::None || !Tracker->IsTracking() || Pawn->IsParked() || Pawn->IsCrashed()) continue;
        for (int32 OtherID : IDs)
        {
            if (OtherID >= ID) break;
            AUAVPawn* Other = AgentRegistry[OtherID].Get();
            if (!Other || Other->IsParked() || Other->IsCrashed()) continue;
            auto* OtherTracker = Other->GetTrajectoryTracker();
            if (!OtherTracker->IsTracking()) continue;
            const float Clearance = JointNMPCConfig.InterAgentSafeDistance
                + Pawn->GetCollisionRadius() + Other->GetCollisionRadius() + 300.0f;
            FVector PrevA = Pawn->GetUAVState().Position;
            FVector PrevB = Other->GetUAVState().Position;
            bool bConflict = false;
            for (int32 K = 1; K <= 24; ++K)
            {
                const float T = K * 0.5f;
                const FVector A = Tracker->GetDesiredState(Tracker->GetCurrentTime() + T).Position;
                const FVector B = OtherTracker->GetDesiredState(OtherTracker->GetCurrentTime() + T).Position;
                const FVector R = PrevA - PrevB;
                const FVector D = (A-B)-R;
                const float Alpha = FMath::Clamp(-FVector::DotProduct(R,D)/FMath::Max(D.SizeSquared(),1.e-6),0.0,1.0);
                if ((R+D*Alpha).SizeSquared() < FMath::Square(Clearance)) { bConflict=true; break; }
                PrevA=A; PrevB=B;
            }
            if (bConflict)
            {
                Tracker->SetTrafficScale(0.2f);
                UE_LOG_THROTTLE(2.0,LogUAVMultiAgent,Log,TEXT("[Traffic] Agent=%d yields to=%d clearance=%.0f"),ID,OtherID,Clearance);
                break;
            }
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
	AllStates.RemoveAll([this](const FAgentStateSnapshot& S){
		const auto* P=AgentRegistry.Find(S.AgentID);
		return FailedAgentIDs.Contains(S.AgentID) || (P && P->IsValid() && P->Get()->IsParked());
	});
	if(AllStates.Num()<2) {JointNMPCResultCache.Empty();return;}

	// 收集所有 Agent 的参考轨迹点
	TArray<TArray<FVector>> RefPointsPerAgent;
	// 拷贝 AgentRegistry 的 key 集合，避免迭代时潜在的容器修改
	TArray<int32> RefAgentIDs;
	AgentRegistry.GenerateKeyArray(RefAgentIDs);
	RefAgentIDs.Sort();
	RefAgentIDs.RemoveAll([&AllStates](int32 ID){return !AllStates.ContainsByPredicate([ID](const auto& S){return S.AgentID==ID;});});
	for (int32 AgentID : RefAgentIDs)
	{
		TArray<FVector> RefPoints;
		TWeakObjectPtr<AUAVPawn>* PawnPtr = AgentRegistry.Find(AgentID);
		if (PawnPtr && PawnPtr->IsValid())
		{
			AUAVPawn* Pawn = PawnPtr->Get();
			// 从 TrajectoryTracker 采样未来 N+1 个参考点
			UTrajectoryTracker* Tracker = Pawn->GetTrajectoryTracker();
			FVector FormationTarget;
			if (!IsLeader(AgentID) && GetFormationTarget(AgentID, FormationTarget))
			{
				const auto* LeaderPtr=AgentRegistry.Find(FormationConfig.LeaderID);
                const AUAVPawn* Leader=LeaderPtr && LeaderPtr->IsValid() ? LeaderPtr->Get() : nullptr;
                auto* LeaderTracker=Leader ? Leader->GetTrajectoryTracker() : nullptr;
                const FVector Offset=FormationTarget-(Leader ? Leader->GetUAVState().Position : FVector::ZeroVector);
                for(int32 K=0;K<=JointNMPCConfig.BaseConfig.Solver.PredictionSteps;++K)
                    RefPoints.Add(LeaderTracker && LeaderTracker->IsTracking()
                        ? LeaderTracker->GetPredictionState(K*JointNMPCConfig.BaseConfig.GetDt()).Position+Offset
                        : FormationTarget);
			}
			else if (Tracker && (Tracker->IsTracking() || Tracker->IsComplete()))
			{
				int32 N = JointNMPCConfig.BaseConfig.Solver.PredictionSteps;
				float Dt = JointNMPCConfig.BaseConfig.GetDt();
				for (int32 i = 0; i <= N; ++i)
				{
					RefPoints.Add(Tracker->GetPredictionState(i * Dt).Position);
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
	FJointNMPCConfig SolveConfig=JointNMPCConfig;
	if (FormationConfig.Type==EFormationType::None) SolveConfig.WeightFormation=0;
	FJointNMPCSolveResult Result = JointNMPCSolverInstance->Solve(
		AllStates, RefPointsPerAgent, StaticObstacles, SolveConfig);

	// 缓存每个 Agent 的加速度结果
	JointNMPCResultCache.Empty();
	if (Result.bUsableControls)
	{
		for (int32 i = 0; i < AllStates.Num() && i < Result.OptimalAccelerations.Num(); ++i)
		{
			SetJointNMPCCache(AllStates[i].AgentID, Result.OptimalAccelerations[i]);
		}
	}
		UE_LOG_THROTTLE(5.0, LogUAVMultiAgent, Log, TEXT("[JointNMPC] Solved: agents=%d cost=%.1f converged=%s"),
			AllStates.Num(), Result.TotalCost, Result.bConverged ? TEXT("Y") : TEXT("N"));
}

// ---- 任务分配 ----

bool AMultiAgentGameMode::KeepsDemoOpen() const
{
    return ActiveScenario && ActiveScenario->bCooperationDemo;
}

FTaskAllocationResult AMultiAgentGameMode::SubmitTasks(const TArray<FTaskDescriptor>& Tasks)
{
    TaskPool = Tasks;
    TaskAssignedAgents.Empty();
    CompletedTaskIDs.Empty();
    FailedTaskIDs.Empty();
    TaskPoolEpoch = GetWorld()->GetTimeSeconds();
    LastReplanReason = TEXT("Initial allocation");
    return AllocatePendingTasks();
}

FTaskAllocationResult AMultiAgentGameMode::InsertTask(FTaskDescriptor Task)
{
    int32 NextID = 0;
    for (const auto& T : TaskPool) NextID = FMath::Max(NextID,T.TaskID+1);
    Task.TaskID = NextID;
    TaskPool.Add(Task);
    return TriggerReplan(TEXT("Urgent task inserted"));
}

FTaskAllocationResult AMultiAgentGameMode::AllocatePendingTasks()
{
    RefreshStateCache();
    TArray<FTaskDescriptor> Pending;
    const float Age = GetWorld()->GetTimeSeconds()-TaskPoolEpoch;
    for (FTaskDescriptor T : TaskPool)
        if (!CompletedTaskIDs.Contains(T.TaskID) && !FailedTaskIDs.Contains(T.TaskID))
        {
            T.Deadline -= Age; T.LatestFinish -= Age;
            T.EarliestStart = FMath::Max(0.0f,T.EarliestStart-Age);
            if(FMath::Min(T.Deadline,T.LatestFinish)<=0) FailedTaskIDs.Add(T.TaskID);
            else Pending.Add(T);
        }
    Pending.StableSort([](const FTaskDescriptor& A,const FTaskDescriptor& B)
    { return A.Priority != B.Priority ? A.Priority > B.Priority : A.TaskID < B.TaskID; });
    TArray<FAgentStateSnapshot> States = GetAllAgentStates();
    States.RemoveAll([this](const FAgentStateSnapshot& S){return FailedAgentIDs.Contains(S.AgentID);});
    FTaskAllocationResult Result;
    if (Pending.IsEmpty()) Result.bIsFeasible = true;
    else
    {
        auto Capabilities=UTaskAllocator::DeriveCapabilities(States);
        for(auto& C:Capabilities) C.MaxSpeed=TaskExecutionSpeedCm;
        Result = TaskAllocatorInstance->Allocate(Pending,Capabilities,TaskAllocationConfig);
    }
    AppliedAllocation = Result;
    if (ScenarioEvaluatorComponent) ScenarioEvaluatorComponent->InvalidateFinalResult();
    if (!Result.bIsFeasible)
    {
        for(const auto& T:Pending) FailedTaskIDs.Add(T.TaskID);
        TaskMonitorInstance->Reset();
        TaskMonitorInstance->PreserveCompleted(CompletedTaskIDs);
        TaskMonitorInstance->PreserveFailed(FailedTaskIDs);
        TaskQueues.Empty();
        for (const auto& Pair : AgentRegistry)
            if (Pair.Value.IsValid())
            {
                auto* P=Pair.Value.Get();
                if (auto* AI=Cast<AUAVAIController>(P->GetController())) AI->StopBehaviorTree();
                P->StopTrajectoryTracking(); P->SetTargetPosition(P->GetUAVState().Position);
            }
        LastReplanReason += TEXT(" / allocation infeasible");
        UE_LOG(LogUAVMultiAgent, Error, TEXT("[TaskExecution] Allocation infeasible: %s"), *LastReplanReason);
        return Result;
    }
    TaskMonitorInstance->Initialize(Result, TaskMonitorConfig, Pending);
    TaskMonitorInstance->PreserveCompleted(CompletedTaskIDs);
    TaskMonitorInstance->PreserveFailed(FailedTaskIDs);
    ApplyTaskAllocation(Result);
    return Result;
}

void AMultiAgentGameMode::ApplyTaskAllocation(const FTaskAllocationResult& Result)
{
    TaskQueues.Empty();
    JointNMPCResultCache.Empty();
    for (const auto& A : Result.Assignments) { TaskQueues.FindOrAdd(A.AgentID).Add(A.TaskID); TaskAssignedAgents.Add(A.TaskID,A.AgentID); }
    for (const auto& Pair : AgentRegistry)
    {
        if (!Pair.Value.IsValid() || FailedAgentIDs.Contains(Pair.Key)) continue;
        AUAVPawn* P = Pair.Value.Get();
        if (auto* AI = Cast<AUAVAIController>(P->GetController())) AI->StopBehaviorTree();
        P->StopTrajectoryTracking();
        P->SetTargetPosition(P->GetUAVState().Position);
        P->GetMissionComponent()->ClearWaypoints();
        DispatchNextTask(Pair.Key);
    }
}

bool AMultiAgentGameMode::StartRoute(AUAVPawn* Pawn, const FVector& Target, float Speed, float Acceleration, const FVector& EndVelocity)
{
    if(!Pawn || Target.ContainsNaN()) return false;
    const FVector Start=Pawn->GetUAVState().Position;
    if(Start.Equals(Target,1)) {Pawn->StopTrajectoryTracking();Pawn->SetTargetPosition(Target);return true;}
    FTrajectory Trajectory;
    if(!PlanRoute(Pawn,{Target},Speed,Acceleration,EndVelocity,Trajectory)) return false;
    Pawn->SetTrajectory(Trajectory);
    Pawn->StartTrajectoryTracking();
    return true;
}

bool AMultiAgentGameMode::PlanRoute(AUAVPawn* Pawn,const TArray<FVector>& Targets,float Speed,float Acceleration,const FVector& EndVelocity,FTrajectory& Out)
{
    Out=FTrajectory();
    if(!Pawn || Targets.IsEmpty()) return false;
    UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>(Pawn);
    TArray<FObstacleInfo> RouteObstacles=Pawn->GetObstacleManager()->GetAllObstacles();
    RouteObstacles.RemoveAll([](const FObstacleInfo& O){
        const AUAVPawn* Other=Cast<AUAVPawn>(O.LinkedActor.Get());
        return Other && !Other->IsParked() && !Other->IsCrashed();
    });
    Planner->SetObstacles(RouteObstacles);
    TArray<FVector> Path={Pawn->GetUAVState().Position};
    for(const FVector& Target:Targets)
    {
        if(Target.ContainsNaN()) return false;
        const FVector Start=Path.Last();
        if(Start.Equals(Target,1)) continue;
        TArray<FVector> Leg;
        if(!Planner->CheckLineCollision(Start,Target,Pawn->GetCollisionRadius())) Leg={Start,Target};
        else if(!Planner->PlanPath(Start,Target,Leg) || Leg.Num()<2) return false;
        for(int32 I=1;I<Leg.Num();++I) Path.Add(Leg[I]);
    }
    if(Path.Num()<2) return false;
    UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>(Pawn);
    Optimizer->SetStartVelocity(Pawn->GetUAVState().Velocity);
    Optimizer->SetEndVelocity(EndVelocity);
    if(Pawn->GetTrajectoryTracker()->IsTracking())
        Optimizer->SetStartAcceleration(Pawn->GetTrajectoryTracker()->GetDesiredState().Acceleration);
    Out=Optimizer->OptimizeTrajectory(Path,Speed,Acceleration);
    return Out.bIsValid && Out.Points.Num()>=2 && IsRouteClear(Pawn,Out);
}

bool AMultiAgentGameMode::IsRouteClear(AUAVPawn* Pawn,const FTrajectory& Trajectory) const
{
    if(!Pawn || !Trajectory.bIsValid || Trajectory.Points.Num()<2) return false;
    UAStarPathPlanner* Planner=NewObject<UAStarPathPlanner>(Pawn);
    auto Obstacles=Pawn->GetObstacleManager()->GetAllObstacles();
    Obstacles.RemoveAll([](const FObstacleInfo& O){
        const auto* Other=Cast<AUAVPawn>(O.LinkedActor.Get());
        return Other && !Other->IsParked() && !Other->IsCrashed();
    });
    Planner->SetObstacles(Obstacles);
    // 多项式转弯可能偏离规划折线，必须检查实际执行曲线的净空。
    for(int32 I=1;I<Trajectory.Points.Num();++I)
        if(Planner->CheckLineCollision(Trajectory.Points[I-1].Position,Trajectory.Points[I].Position,Pawn->GetCollisionRadius()))
        {
            UE_LOG(LogUAVPlanning,Warning,TEXT("[Route] Clearance rejected Agent=%d Time=%.3f From=%s To=%s Radius=%.1f"),
                Pawn->GetAgentID(),Trajectory.Points[I].TimeStamp,*Trajectory.Points[I-1].Position.ToString(),
                *Trajectory.Points[I].Position.ToString(),Pawn->GetCollisionRadius());
            for(const auto& Obstacle:Obstacles)
            {
                Planner->SetObstacles({Obstacle});
                if(Planner->CheckLineCollision(Trajectory.Points[I-1].Position,Trajectory.Points[I].Position,Pawn->GetCollisionRadius()))
                {
                    UE_LOG(LogUAVPlanning,Warning,TEXT("[Route] Blocking obstacle Center=%s Extents=%s Margin=%.1f Actor=%s"),
                        *Obstacle.Center.ToString(),*Obstacle.Extents.ToString(),Obstacle.SafetyMargin,*GetNameSafe(Obstacle.LinkedActor.Get()));
                    break;
                }
            }
            return false;
        }
    return true;
}

void AMultiAgentGameMode::StartScenarioWaypoint(AUAVPawn* Pawn)
{
    if (!Pawn || (FormationConfig.Type!=EFormationType::None && !IsLeader(Pawn->GetAgentID()))) return;
    UMissionComponent* Mission = Pawn->GetMissionComponent();
    FMissionWaypoint Waypoint;
    if (!Mission->GetCurrentWaypoint(Waypoint)) return;
    if (!Mission->IsMissionRunning()) Mission->StartMission();
    if (!StartRoute(Pawn,Waypoint.Position,Mission->GetRemainingTrajectorySpeedLimit(),TaskExecutionAccelerationCm))
        Mission->FailMission(TEXT("Scenario route infeasible"));
}

void AMultiAgentGameMode::DispatchNextTask(int32 AgentID)
{
    auto* Ptr = AgentRegistry.Find(AgentID);
    auto* Queue = TaskQueues.Find(AgentID);
    if (!Ptr || !Ptr->IsValid() || !Queue || Queue->IsEmpty()) return;
    const int32 ID = (*Queue)[0];
    const auto* Task = TaskPool.FindByPredicate([ID](const FTaskDescriptor& T){return T.TaskID==ID;});
    if (!Task) return;
    AUAVPawn* Pawn = Ptr->Get();
    UMissionComponent* Mission = Pawn->GetMissionComponent();
    Mission->SetMissionMode(EMissionMode::Once);
    Mission->SetMissionWaypoints({FMissionWaypoint(Task->TargetLocation,Task->EstimatedDuration,TaskExecutionSpeedCm)});
    Mission->StartMission();
    if (!StartRoute(Pawn,Task->TargetLocation,TaskExecutionSpeedCm,TaskExecutionAccelerationCm))
    {
        FailedTaskIDs.Add(ID);
        TaskMonitorInstance->MarkFailed(ID);
        Queue->Remove(ID);
        LastReplanReason = TEXT("Task route infeasible");
        Mission->FailMission(LastReplanReason);
        UE_LOG(LogUAVMultiAgent, Error, TEXT("[TaskExecution] Route infeasible: task=%d agent=%d"),ID,AgentID);
        DispatchNextTask(AgentID);
        return;
    }
    TaskMonitorInstance->ActivateTask(ID);
    UE_LOG(LogUAVMultiAgent, Log, TEXT("[TaskExecution] Started task=%d agent=%d"),ID,AgentID);
}

void AMultiAgentGameMode::HandleTaskCompleted(int32 TaskID, int32 AgentID)
{
    CompletedTaskIDs.Add(TaskID);
    if (auto* Queue = TaskQueues.Find(AgentID)) Queue->Remove(TaskID);
    UE_LOG(LogUAVMultiAgent, Log, TEXT("[TaskExecution] Completed task=%d agent=%d"),TaskID,AgentID);
    DispatchNextTask(AgentID);
}

void AMultiAgentGameMode::HandleTaskFailed(int32 TaskID, int32 AgentID, const FString& Reason)
{
    FailedTaskIDs.Add(TaskID);
    if (auto* Queue=TaskQueues.Find(AgentID)) Queue->Remove(TaskID);
    LastReplanReason=Reason;
}

void AMultiAgentGameMode::HandleTaskReplan(const FString& Reason) { TriggerReplan(Reason); }

void AMultiAgentGameMode::FailAgent(int32 AgentID)
{
    auto* Ptr = AgentRegistry.Find(AgentID);
    if (!Ptr || !Ptr->IsValid() || FailedAgentIDs.Contains(AgentID)) return;
    FailedAgentIDs.Add(AgentID);
    AUAVPawn* P = Ptr->Get();
    if (auto* AI = Cast<AUAVAIController>(P->GetController())) AI->StopBehaviorTree();
    P->StopTrajectoryTracking();
    P->SetTargetPosition(P->GetUAVState().Position);
    P->GetMissionComponent()->FailMission(TEXT("Agent disabled"));
    TriggerReplan(FString::Printf(TEXT("Agent %d disabled"),AgentID));
}

FTaskAllocationResult AMultiAgentGameMode::TriggerReplan(const FString& Reason)
{
    LastReplanReason = Reason;
    return AllocatePendingTasks();
}

FTaskAllocationResult AMultiAgentGameMode::GetCurrentTaskAllocation() const { return AppliedAllocation; }

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
