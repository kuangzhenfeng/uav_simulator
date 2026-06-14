// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "MultiAgentTypes.h"
#include "TaskAllocationTypes.h"
#include "AgentManager.generated.h"

class AUAVPawn;
class UJointNMPCSolver;
class UTaskAllocator;
class UTaskMonitor;
class UWindField;
class UScenario;
class UScenarioLoader;
class UScenarioEvaluatorComponent;

/**
 * 多机协同 GameMode
 *
 * 关卡级单例，管理所有 Agent 的注册、状态查询、编队配置和联合规划。
 * 在关卡中设置此 GameMode 即可启用多机协同功能。
 * 未使用此 GameMode 的关卡保持单机模式，零开销。
 *
 * 持有场景级 WindField 单例（ADR-0002）：风场是环境而非机载设备，
 * 全关卡共享一个实例，所有 UAV 经历同一片风。
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintType))
class UAV_SIMULATOR_API AMultiAgentGameMode : public AGameModeBase
{
	GENERATED_UCLASS_BODY()

public:
	AMultiAgentGameMode();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	/** 获取场景级 WindField 单例（ADR-0002） */
	UFUNCTION(BlueprintCallable, Category = "Environment")
	UWindField* GetWindField() const { return WindField; }

	/** 默认场景资产引用（命令行未指定 -Scenario= 时使用） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	TSoftObjectPtr<UScenario> DefaultScenario;

	// ---- Agent 注册与管理 ----

	/**
	 * 注册 Agent，返回分配的 AgentID
	 * 由 AUAVPawn::BeginPlay 自动调用
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent")
	int32 RegisterAgent(AUAVPawn* Agent);

	/**
	 * 注销 Agent
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent")
	void UnregisterAgent(int32 AgentID);

	/**
	 * 获取指定 Agent 的状态快照
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent")
	bool GetAgentState(int32 AgentID, FAgentStateSnapshot& OutState) const;

	/**
	 * 获取所有 Agent 的状态快照
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent")
	TArray<FAgentStateSnapshot> GetAllAgentStates() const;

	/**
	 * 获取指定范围内的邻居 Agent 状态
	 * @param RequesterID 请求方 AgentID
	 * @param Radius 查询半径 (cm)
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent")
	TArray<FAgentStateSnapshot> GetNeighborStates(int32 RequesterID, float Radius) const;

	/**
	 * 获取已注册的 Agent 数量
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent")
	int32 GetAgentCount() const;

	/**
	 * 获取 Leader AgentID
	 */
	int32 GetLeaderID() const { return FormationConfig.LeaderID; }

	/**
	 * 检查指定 Agent 是否为 Leader
	 */
	bool IsLeader(int32 AgentID) const { return AgentID == FormationConfig.LeaderID; }

	// ---- 编队控制 ----

	/**
	 * 设置编队配置
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|Formation")
	void SetFormation(const FFormationConfig& InConfig);

	/**
	 * 获取当前编队配置
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|Formation")
	FFormationConfig GetFormationConfig() const { return FormationConfig; }

	/**
	 * 获取指定 Agent 的编队目标位置（世界坐标）
	 * @param AgentID Agent 标识
	 * @param OutTarget 输出目标位置
	 * @return 是否成功计算
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|Formation")
	bool GetFormationTarget(int32 AgentID, FVector& OutTarget) const;

	// ---- 联合 NMPC ----

	/**
	 * 获取联合 NMPC 求解器实例
	 */
	UJointNMPCSolver* GetJointNMPCSolver() const { return JointNMPCSolverInstance; }

	/**
	 * 存储联合 NMPC 求解结果（由 Leader 调用）
	 */
	void SetJointNMPCCache(int32 AgentID, const FVector& Acceleration);

	/**
	 * 获取联合 NMPC 缓存结果（由各 Agent 读取）
	 */
	bool GetJointNMPCCache(int32 AgentID, FVector& OutAcceleration) const;

	// ---- 任务分配 ----

	/**
	 * 获取任务分配器
	 */
	UTaskAllocator* GetTaskAllocator() const { return TaskAllocatorInstance; }

	/**
	 * 获取任务监控器
	 */
	UTaskMonitor* GetTaskMonitor() const { return TaskMonitorInstance; }

	/**
	 * 提交任务池（触发分配求解）
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|TaskAllocation")
	FTaskAllocationResult SubmitTasks(const TArray<FTaskDescriptor>& Tasks);

	/**
	 * 触发重规划
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|TaskAllocation")
	FTaskAllocationResult TriggerReplan(const FString& Reason);

	/**
	 * 获取当前分配
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|TaskAllocation")
	FTaskAllocationResult GetCurrentTaskAllocation() const;

	// ---- 配置 ----

	// 编队配置
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MultiAgent")
	FFormationConfig FormationConfig;

	// 联合 NMPC 配置
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MultiAgent")
	FJointNMPCConfig JointNMPCConfig;

	// CBF-QP 默认配置（分配给每个 Agent）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MultiAgent")
	FCBFQPConfig DefaultCBFQPConfig;

	// 任务分配配置
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MultiAgent|TaskAllocation")
	FTaskAllocationConfig TaskAllocationConfig;

	// 任务监控配置
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MultiAgent|TaskAllocation")
	FTaskMonitorConfig TaskMonitorConfig;

	// Agent 状态缓存刷新间隔 (秒)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MultiAgent")
	float StateCacheUpdateInterval = 0.05f;

protected:
	// 场景级 WindField 单例（ADR-0002）。风场属环境，全关卡共享。
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Environment")
	TObjectPtr<UWindField> WindField;

	/** 装配场景：解析 -Scenario= 命令行或回退 DefaultScenario，按子流程装配世界 */
	void LoadAndAssembleScenario();

	// 当前生效的场景资产
	UPROPERTY(Transient, BlueprintReadOnly, Category = "Scenario")
	TObjectPtr<UScenario> ActiveScenario;

	// 场景装配器实例
	UPROPERTY(Transient, BlueprintReadOnly, Category = "Scenario")
	TObjectPtr<UScenarioLoader> ScenarioLoaderInstance;

	// 场景验收器组件（周期快照 + 最终判决）
	UPROPERTY(Transient, BlueprintReadOnly, Category = "Scenario")
	TObjectPtr<UScenarioEvaluatorComponent> ScenarioEvaluatorComponent;

	// 场景装配出的机队（供 Evaluator / 外部查询）
	UPROPERTY(Transient, BlueprintReadOnly, Category = "Scenario")
	TArray<TObjectPtr<AUAVPawn>> ScenarioFleet;

private:
	// Agent 注册表
	UPROPERTY()
	TMap<int32, TWeakObjectPtr<AUAVPawn>> AgentRegistry;

	// Agent 状态快照缓存
	UPROPERTY()
	TMap<int32, FAgentStateSnapshot> StateCache;

	// 联合 NMPC 结果缓存（每 Agent 最优加速度）
	TMap<int32, FVector> JointNMPCResultCache;

	// 下一个可用 AgentID
	int32 NextAgentID = 0;

	// 联合 NMPC 求解器实例
	UPROPERTY()
	TObjectPtr<UJointNMPCSolver> JointNMPCSolverInstance;

	// 任务分配器实例
	UPROPERTY()
	TObjectPtr<UTaskAllocator> TaskAllocatorInstance;

	// 任务监控器实例
	UPROPERTY()
	TObjectPtr<UTaskMonitor> TaskMonitorInstance;

	// 联合 NMPC 求解频率累加器
	float JointNMPCSolveAccumulator = 0.0f;

	// 状态缓存更新累加器
	float StateCacheAccumulator = 0.0f;

	// 编队偏移量缓存（当编队配置不变时复用）
	TArray<FVector> CachedFormationOffsets;
	int32 CachedFormationNumAgents = -1;
	EFormationType CachedFormationType = EFormationType::None;

	/**
	 * 计算编队偏移量
	 * @param NumAgents Agent 数量
	 * @return 每个 Agent 相对于 Leader 的偏移量
	 */
	TArray<FVector> ComputeFormationOffsets(int32 NumAgents) const;

	/**
	 * 刷新所有 Agent 的状态快照缓存
	 */
	void RefreshStateCache();

	/**
	 * 联合 NMPC 求解（仅 Leader 调用）
	 */
	void SolveJointNMPC();
};
