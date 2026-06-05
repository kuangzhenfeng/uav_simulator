// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MultiAgentTypes.h"
#include "TaskAllocationTypes.h"
#include "TaskMonitor.generated.h"

/**
 * 任务监控器
 *
 * 实时监控任务执行进度，检测异常（停滞、偏离、超时），
 * 通过委托触发重规划请求。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UTaskMonitor : public UObject
{
	GENERATED_BODY()

public:
	UTaskMonitor();

	/**
	 * 初始化监控（设置分配结果和配置）
	 */
	void Initialize(const FTaskAllocationResult& Allocation,
		const FTaskMonitorConfig& Config);

	/**
	 * 每帧更新（在 GameMode::Tick 中调用）
	 */
	void Update(float DeltaTime, const TArray<FAgentStateSnapshot>& AgentStates);

	/**
	 * 查询任务状态
	 */
	ETaskStatus GetTaskStatus(int32 TaskID) const;

	/**
	 * 查询任务进度 (0.0 - 1.0)
	 */
	float GetTaskProgress(int32 TaskID) const;

	/**
	 * 查询总体进度 (0.0 - 1.0)
	 */
	float GetOverallProgress() const;

	/**
	 * 获取已完成任务数
	 */
	int32 GetCompletedTaskCount() const;

	/**
	 * 获取总任务数
	 */
	int32 GetTotalTaskCount() const;

	// 事件委托
	UPROPERTY()
	FOnTaskCompleted OnTaskCompleted;

	UPROPERTY()
	FOnTaskFailed OnTaskFailed;

	UPROPERTY()
	FOnReplanRequested OnReplanRequested;

	// ---- 内部方法 (public for testing) ----

	/**
	 * 检测停滞（位置长时间不变）
	 */
	bool DetectStalledAgent(int32 AgentID, const FAgentStateSnapshot& State);

	/**
	 * 检测偏离（距离目标过远）
	 */
	bool DetectDeviation(int32 AgentID, const FAgentStateSnapshot& State);

	/**
	 * 检测超时
	 */
	bool DetectTimeout(int32 TaskID) const;

private:
	// 任务执行状态跟踪
	struct FTaskProgress
	{
		int32 TaskID;
		ETaskStatus Status;
		float StartTime;
		float EstimatedEndTime;
		float Progress;
		int32 AssignedAgentID;
		FVector TargetLocation;
	};

	// 任务进度列表
	TArray<FTaskProgress> TaskProgresses;

	// Agent 历史位置（用于停滞检测）
	TMap<int32, TArray<FVector>> AgentPositionHistory;

	// 停滞帧计数
	TMap<int32, int32> StalledFrameCount;

	// 偏离帧计数
	TMap<int32, int32> DeviationFrameCount;

	// 配置
	FTaskMonitorConfig Config;

	// 已过时间
	float ElapsedTime;

	// 进度检查累加器
	float ProgressCheckAccumulator;

	// 是否已初始化
	bool bInitialized;
};
