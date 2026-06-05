// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "../Core/UAVTypes.h"
#include "TaskAllocationTypes.generated.h"

// ---- 前向声明 ----

class UTaskAllocator;
class UTaskMonitor;
class UMILPSolver;

// ---- 任务相关枚举 ----

/**
 * 任务优先级
 */
UENUM(BlueprintType)
enum class ETaskPriority : uint8
{
	Low UMETA(DisplayName = "Low"),
	Normal UMETA(DisplayName = "Normal"),
	High UMETA(DisplayName = "High"),
	Critical UMETA(DisplayName = "Critical")
};

/**
 * 任务状态
 */
UENUM(BlueprintType)
enum class ETaskStatus : uint8
{
	Pending UMETA(DisplayName = "Pending"),
	Assigned UMETA(DisplayName = "Assigned"),
	InProgress UMETA(DisplayName = "InProgress"),
	Completed UMETA(DisplayName = "Completed"),
	Failed UMETA(DisplayName = "Failed"),
	Cancelled UMETA(DisplayName = "Cancelled")
};

/**
 * 任务类型
 */
UENUM(BlueprintType)
enum class ETaskType : uint8
{
	Waypoint UMETA(DisplayName = "Waypoint"),
	AreaCoverage UMETA(DisplayName = "AreaCoverage"),
	Surveillance UMETA(DisplayName = "Surveillance"),
	PayloadDelivery UMETA(DisplayName = "PayloadDelivery"),
	SearchAndRescue UMETA(DisplayName = "SearchAndRescue")
};

// ---- 任务数据结构 ----

/**
 * 单个任务描述
 */
USTRUCT(BlueprintType)
struct FTaskDescriptor
{
	GENERATED_BODY()

	// 任务唯一标识
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	int32 TaskID = -1;

	// 任务类型
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	ETaskType Type = ETaskType::Waypoint;

	// 任务优先级
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	ETaskPriority Priority = ETaskPriority::Normal;

	// 目标位置 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	FVector TargetLocation;

	// 预估耗时 (s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float EstimatedDuration = 10.0f;

	// 截止时间（相对于任务池创建时刻, s）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float Deadline = 300.0f;

	// 所需载荷 (kg)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float RequiredPayload = 0.0f;

	// 所需能力位掩码
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	int32 RequiredCapabilities = 0;

	// 完成奖励值（用于目标函数）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float Reward = 1.0f;

	// 时间窗 [EarliestStart, LatestFinish]
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float EarliestStart = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float LatestFinish = 600.0f;

	FTaskDescriptor()
		: TargetLocation(FVector::ZeroVector)
	{}
};

/**
 * UAV 能力描述（从 FUAVModelSpec + 运行时状态派生）
 */
USTRUCT(BlueprintType)
struct FUAVCapability
{
	GENERATED_BODY()

	// Agent 唯一标识
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	int32 AgentID = -1;

	// 最大载荷 (kg)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float MaxPayloadKg = 10.0f;

	// 剩余续航 (s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float RemainingFlightTime = 600.0f;

	// 最大速度 (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float MaxSpeed = 2000.0f;

	// 能力位掩码
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	int32 CapabilityMask = 0;

	// 当前位置 (cm)
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	FVector CurrentPosition;

	// 当前已载载荷 (kg)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float CurrentPayloadMass = 0.0f;

	FUAVCapability()
		: CurrentPosition(FVector::ZeroVector)
	{}
};

/**
 * 分配结果中的单条目
 */
USTRUCT(BlueprintType)
struct FTaskAssignment
{
	GENERATED_BODY()

	// 任务 ID
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	int32 TaskID = -1;

	// 分配的 Agent ID
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	int32 AgentID = -1;

	// 预计开始时间 (s)
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	float EstimatedStartTime = 0.0f;

	// 预计完成时间 (s)
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	float EstimatedCompletionTime = 0.0f;

	// 预估飞行距离 (cm)
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	float EstimatedTravelDistance = 0.0f;
};

/**
 * 完整分配结果
 */
USTRUCT(BlueprintType)
struct FTaskAllocationResult
{
	GENERATED_BODY()

	// 分配条目列表
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	TArray<FTaskAssignment> Assignments;

	// 目标函数值
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	float TotalCost = MAX_FLT;

	// 最大完工时间 (Makespan, s)
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	float Makespan = MAX_FLT;

	// 是否存在可行解
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	bool bIsFeasible = false;

	// 求解耗时 (s)
	UPROPERTY(BlueprintReadOnly, Category = "TaskAllocation")
	float SolveTimeSeconds = 0.0f;
};

// ---- 求解器配置 ----

/**
 * MILP 求解器配置
 */
USTRUCT(BlueprintType)
struct FMILPSolverConfig
{
	GENERATED_BODY()

	// 分支定界最大节点数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MILP")
	int32 MaxBranchAndBoundNodes = 1000;

	// LP 收敛容差
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MILP")
	float LPConvergenceTolerance = 1e-4f;

	// LP 最大迭代次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MILP")
	int32 LPMaxIterations = 200;

	// 最优性间隙 (5%)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MILP")
	float MIPGap = 0.05f;

	// 求解时间限制 (s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MILP")
	float TimeLimitSeconds = 2.0f;
};

/**
 * 任务分配配置
 */
USTRUCT(BlueprintType)
struct FTaskAllocationConfig
{
	GENERATED_BODY()

	// MILP 求解器配置
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	FMILPSolverConfig MILPConfig;

	// 启用层次化耦合（MILP + Joint NMPC 反馈）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	bool bEnableHierarchicalCoupling = true;

	// 最大反馈迭代次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	int32 MaxFeedbackIterations = 3;

	// 定期重规划间隔 (s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	float ReplanIntervalSeconds = 10.0f;

	// 启用优先级调度
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskAllocation")
	bool bEnablePriorityScheduling = true;
};

/**
 * 任务监控配置
 */
USTRUCT(BlueprintType)
struct FTaskMonitorConfig
{
	GENERATED_BODY()

	// 进度检查间隔 (s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskMonitor")
	float ProgressCheckInterval = 0.5f;

	// 无进展超时 (s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskMonitor")
	float StalledTimeout = 30.0f;

	// 最大允许偏离距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TaskMonitor")
	float MaxDeviationDistance = 5000.0f;
};

// ---- 委托声明 ----

// 任务完成委托
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnTaskCompleted, int32, TaskID, int32, AgentID);

// 任务失败委托
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnTaskFailed, int32, TaskID, int32, AgentID, const FString&, Reason);

// 重规划请求委托
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnReplanRequested, const FString&, Reason);
