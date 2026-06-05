// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MultiAgentTypes.h"
#include "TaskAllocationTypes.h"
#include "MILPSolver.h"
#include "TaskAllocator.generated.h"

/**
 * 任务分配器
 *
 * 将任务分配问题建模为 MILP，调用自包含求解器求解。
 * 支持层次化耦合：上层 MILP 分配 → 下层 Joint NMPC 轨迹优化 → 反馈迭代。
 * 支持增量重规划：处理 Agent 故障、新任务插入等动态事件。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UTaskAllocator : public UObject
{
	GENERATED_BODY()

public:
	UTaskAllocator();

	/**
	 * 一次性任务分配
	 * @param Tasks 待分配任务列表
	 * @param UAVCapabilities UAV 能力列表
	 * @param Config 分配配置
	 * @return 分配结果
	 */
	FTaskAllocationResult Allocate(
		const TArray<FTaskDescriptor>& Tasks,
		const TArray<FUAVCapability>& UAVCapabilities,
		const FTaskAllocationConfig& Config);

	/**
	 * 增量重规划
	 * @param PreviousAllocation 之前的分配结果
	 * @param NewTasks 新增任务
	 * @param FailedAgentIDs 失败的 Agent ID 列表
	 * @param CurrentStates 当前 Agent 状态
	 * @param UAVCapabilities UAV 能力列表
	 * @param Config 分配配置
	 * @return 新的分配结果
	 */
	FTaskAllocationResult Reallocate(
		const FTaskAllocationResult& PreviousAllocation,
		const TArray<FTaskDescriptor>& NewTasks,
		const TArray<int32>& FailedAgentIDs,
		const TArray<FAgentStateSnapshot>& CurrentStates,
		const TArray<FUAVCapability>& UAVCapabilities,
		const FTaskAllocationConfig& Config);

	/**
	 * 获取当前分配结果
	 */
	const FTaskAllocationResult& GetCurrentAllocation() const { return CurrentAllocation; }

	/**
	 * 是否有有效分配
	 */
	bool HasValidAllocation() const { return bHasValidAllocation; }

	// ---- 内部方法 (public for testing) ----

	/**
	 * 构建 MILP 模型
	 * 决策变量：x[i*M+j] in {0,1} 表示 Agent i 执行 Task j
	 * 目标：最小化加权总代价（飞行距离 + 时间惩罚 - 奖励）
	 * 约束：任务唯一分配、UAV 续航、载荷、能力匹配
	 */
	void BuildMILPModel(
		const TArray<FTaskDescriptor>& Tasks,
		const TArray<FUAVCapability>& Capabilities,
		TArray<float>& OutObjective,
		TArray<TArray<float>>& OutAIneq,
		TArray<float>& OutBIneq,
		TArray<TArray<float>>& OutAEq,
		TArray<float>& OutBEq,
		TArray<float>& OutLB,
		TArray<float>& OutUB,
		TArray<int32>& OutIntegerIndices);

	/**
	 * 从 MILP 解析分配结果
	 */
	FTaskAllocationResult ParseMILPSolution(
		const FMILPResult& Solution,
		const TArray<FTaskDescriptor>& Tasks,
		const TArray<FUAVCapability>& Capabilities);

	/**
	 * 估算飞行代价（欧几里得距离 / 速度）
	 */
	float EstimateTravelCost(const FVector& From, const FVector& To, float Speed) const;

	/**
	 * 从 Agent 状态派生 UAV 能力
	 */
	static TArray<FUAVCapability> DeriveCapabilities(
		const TArray<FAgentStateSnapshot>& AgentStates);

private:
	// MILP 求解器实例
	UPROPERTY()
	TObjectPtr<UMILPSolver> MILPSolverInstance;

	// 当前分配结果
	FTaskAllocationResult CurrentAllocation;

	// 是否有有效分配
	bool bHasValidAllocation = false;
};
