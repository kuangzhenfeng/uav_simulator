// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "MultiAgentTypes.h"
#include "../Planning/NMPCAvoidance.h"
#include "JointNMPCSolver.generated.h"

/**
 * 联合 NMPC 求解结果
 */
USTRUCT(BlueprintType)
struct FJointNMPCSolveResult
{
	GENERATED_BODY()

	// 每 Agent 的最优加速度 (cm/s²)
	UPROPERTY(BlueprintReadOnly, Category = "JointNMPC")
	TArray<FVector> OptimalAccelerations;

	// 总代价
	UPROPERTY(BlueprintReadOnly, Category = "JointNMPC")
	float TotalCost = MAX_FLT;

	// 是否收敛
	UPROPERTY(BlueprintReadOnly, Category = "JointNMPC")
	bool bConverged = false;
};

/**
 * 联合 NMPC 求解器
 *
 * 将单机 NMPC 扩展为多机联合优化:
 * - 联合状态空间: [p1, v1, p2, v2, ..., pN, vN] (6N 维)
 * - 联合控制: [a1, a2, ..., aN] (3N 维)
 * - 代价函数: 单机参考跟踪 + 机间碰撞代价 + 编队偏差代价
 * - 求解器: 投影梯度下降 + 回溯线搜索 + 温启动
 *
 * 各 Agent 动力学独立解耦（仅通过代价函数耦合），
 * 因此梯度计算时扰动单个 Agent 的控制不影响其他 Agent 的前向仿真。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UJointNMPCSolver : public UObject
{
	GENERATED_BODY()

public:
	UJointNMPCSolver();

	/**
	 * 联合 NMPC 求解
	 * @param AgentStates 所有 Agent 的当前状态
	 * @param ReferencePointsPerAgent 每 Agent 的参考轨迹点 [AgentIdx][StepIdx]
	 * @param StaticObstacles 静态障碍物列表（所有 Agent 共享）
	 * @param Config 联合 NMPC 配置
	 * @return 求解结果
	 */
	FJointNMPCSolveResult Solve(
		const TArray<FAgentStateSnapshot>& AgentStates,
		const TArray<TArray<FVector>>& ReferencePointsPerAgent,
		const TArray<FObstacleInfo>& StaticObstacles,
		const FJointNMPCConfig& Config);

	// ---- 内部方法 (public for testing) ----

	/**
	 * 单 Agent 前向仿真（与 NMPCAvoidance 相同模式）
	 * @param InitPos 初始位置
	 * @param InitVel 初始速度
	 * @param Controls 控制序列 (N 个加速度)
	 * @param Dt 时间步长
	 * @param MaxVel 最大速度
	 * @param OutPositions 输出位置序列 (N+1)
	 * @param OutVelocities 输出速度序列 (N+1)
	 */
	void ForwardSimulateAgent(
		const FVector& InitPos,
		const FVector& InitVel,
		const TArray<FVector>& Controls,
		float Dt,
		float MaxVel,
		TArray<FVector>& OutPositions,
		TArray<FVector>& OutVelocities) const;

	/**
	 * 计算联合代价
	 * @param AllPositions [AgentIdx][StepIdx] 位置
	 * @param AllVelocities [AgentIdx][StepIdx] 速度
	 * @param AllControls [AgentIdx][StepIdx] 控制
	 * @param AllReferences [AgentIdx][StepIdx] 参考点
	 * @param StaticObstacles 静态障碍物
	 * @param FormationTargets 编队目标位置 (每 Agent)
	 * @param Config 联合 NMPC 配置
	 * @return 总代价
	 */
	float ComputeJointCost(
		const TArray<TArray<FVector>>& AllPositions,
		const TArray<TArray<FVector>>& AllVelocities,
		const TArray<TArray<FVector>>& AllControls,
		const TArray<TArray<FVector>>& AllReferences,
		const TArray<FObstacleInfo>& StaticObstacles,
		const TArray<FVector>& FormationTargets,
		const FJointNMPCConfig& Config) const;

	/**
	 * 计算机间碰撞代价（指数势垒，复用 NMPCAvoidance 模式）
	 */
	float ComputeInterAgentCollisionCost(
		const FVector& PosA, const FVector& PosB,
		float SafeDist, float InfluenceDist, float Alpha) const;

	/**
	 * 计算编队偏差代价
	 */
	float ComputeFormationCost(
		const FVector& ActualPos, const FVector& DesiredPos) const;

	/**
	 * 投影控制到可行域（每 Agent 独立约束）
	 */
	void ProjectAgentControls(
		TArray<FVector>& Controls, const FVector& InitVel,
		float MaxAccel, float MaxVel) const;

private:
	// 上次求解控制序列（温启动）
	TArray<TArray<FVector>> PreviousAllControls;
	bool bHasPreviousSolve = false;

	// 上次总代价
	float PreviousTotalCost = MAX_FLT;

	/**
	 * 有限差分计算联合梯度
	 * 利用动力学解耦特性：扰动 Agent i 的控制只影响 Agent i 的前向仿真
	 */
	void ComputeJointGradient(
		const TArray<FAgentStateSnapshot>& AgentStates,
		const TArray<TArray<FVector>>& AllControls,
		const TArray<TArray<FVector>>& AllPositions,
		const TArray<TArray<FVector>>& AllVelocities,
		const TArray<TArray<FVector>>& AllReferences,
		const TArray<FObstacleInfo>& StaticObstacles,
		const TArray<FVector>& FormationTargets,
		const FJointNMPCConfig& Config,
		TArray<TArray<FVector>>& OutGradient) const;
};
