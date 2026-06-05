// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "TaskAllocationTypes.h"
#include "MILPSolver.generated.h"

/**
 * LP 求解结果
 */
USTRUCT(BlueprintType)
struct FLPResult
{
	GENERATED_BODY()

	// 变量值
	UPROPERTY(BlueprintReadOnly, Category = "MILP")
	TArray<float> Solution;

	// 目标函数值
	UPROPERTY(BlueprintReadOnly, Category = "MILP")
	float ObjectiveValue = MAX_FLT;

	// 是否可行
	UPROPERTY(BlueprintReadOnly, Category = "MILP")
	bool bIsFeasible = false;

	// 求解迭代次数
	UPROPERTY(BlueprintReadOnly, Category = "MILP")
	int32 Iterations = 0;
};

/**
 * MILP 求解结果
 */
USTRUCT(BlueprintType)
struct FMILPResult
{
	GENERATED_BODY()

	// 变量值
	UPROPERTY(BlueprintReadOnly, Category = "MILP")
	TArray<float> Solution;

	// 目标函数值
	UPROPERTY(BlueprintReadOnly, Category = "MILP")
	float ObjectiveValue = MAX_FLT;

	// 是否找到可行解
	UPROPERTY(BlueprintReadOnly, Category = "MILP")
	bool bIsFeasible = false;

	// 最优性间隙
	UPROPERTY(BlueprintReadOnly, Category = "MILP")
	float OptimalityGap = 1.0f;

	// B&B 探索节点数
	UPROPERTY(BlueprintReadOnly, Category = "MILP")
	int32 NodesExplored = 0;

	// 求解耗时 (s)
	UPROPERTY(BlueprintReadOnly, Category = "MILP")
	float SolveTimeSeconds = 0.0f;
};

/**
 * 自包含 MILP 求解器
 *
 * 使用分支定界法 + LP 松弛求解混合整数线性规划:
 *   minimize   c^T x
 *   subject to A_ineq * x <= b_ineq
 *              A_eq * x = b_eq
 *              lb <= x <= ub
 *              x(integer_indices) in Z
 *
 * LP 松弛使用修正单纯形法，支持温启动。
 * 适用于小规模问题（4-8 UAV, 10-30 任务）。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UMILPSolver : public UObject
{
	GENERATED_BODY()

public:
	UMILPSolver();

	/**
	 * 求解 MILP
	 * @param c 目标函数系数 (n 个)
	 * @param AIneq 不等式约束矩阵 (m_ineq x n)
	 * @param bIneq 不等式约束右端 (m_ineq)
	 * @param AEq 等式约束矩阵 (m_eq x n)
	 * @param bEq 等式约束右端 (m_eq)
	 * @param LB 变量下界 (n)
	 * @param UB 变量上界 (n)
	 * @param IntegerIndices 整数变量索引列表
	 * @param Config 求解器配置
	 * @return 求解结果
	 */
	FMILPResult Solve(
		const TArray<float>& c,
		const TArray<TArray<float>>& AIneq,
		const TArray<float>& bIneq,
		const TArray<TArray<float>>& AEq,
		const TArray<float>& bEq,
		const TArray<float>& LB,
		const TArray<float>& UB,
		const TArray<int32>& IntegerIndices,
		const FMILPSolverConfig& Config);

	// ---- 内部方法 (public for testing) ----

	/**
	 * LP 松弛求解（修正单纯形法）
	 */
	FLPResult SolveLP(
		const TArray<float>& c,
		const TArray<TArray<float>>& AIneq,
		const TArray<float>& bIneq,
		const TArray<TArray<float>>& AEq,
		const TArray<float>& bEq,
		const TArray<float>& LB,
		const TArray<float>& UB,
		const FMILPSolverConfig& Config);

	/**
	 * 分支定界主循环
	 */
	FMILPResult BranchAndBound(
		const TArray<float>& c,
		const TArray<TArray<float>>& AIneq,
		const TArray<float>& bIneq,
		const TArray<TArray<float>>& AEq,
		const TArray<float>& bEq,
		const TArray<float>& LB,
		const TArray<float>& UB,
		const TArray<int32>& IntegerIndices,
		const FMILPSolverConfig& Config);

	/**
	 * 选择分支变量：最接近 0.5 的整数变量
	 */
	int32 SelectBranchingVariable(
		const TArray<float>& LPSolution,
		const TArray<int32>& IntegerIndices) const;

	/**
	 * 取整：将 LP 解的整数分量四舍五入
	 */
	TArray<float> RoundSolution(
		const TArray<float>& LPSolution,
		const TArray<int32>& IntegerIndices) const;

	/**
	 * 检查解是否满足整数约束
	 */
	bool IsIntegerFeasible(
		const TArray<float>& Solution,
		const TArray<int32>& IntegerIndices,
		float Tolerance = 0.01f) const;

	/**
	 * 检查解是否满足所有约束
	 */
	bool IsFeasible(
		const TArray<float>& Solution,
		const TArray<TArray<float>>& AIneq,
		const TArray<float>& bIneq,
		const TArray<TArray<float>>& AEq,
		const TArray<float>& bEq,
		const TArray<float>& LB,
		const TArray<float>& UB,
		float Tolerance = 1e-3f) const;

private:
	// 当前最优整数解
	TArray<float> IncumbentSolution;
	float IncumbentObjective;
	bool bHasIncumbent;

	// B&B 节点计数
	int32 NodesExplored;

	// 起始时间（用于时间限制）
	double SolveStartTime;
	float TimeLimit;
};
