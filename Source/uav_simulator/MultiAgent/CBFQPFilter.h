// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "MultiAgentTypes.h"
#include "CBFQPFilter.generated.h"

/**
 * CBF-QP 安全滤波结果
 */
USTRUCT(BlueprintType)
struct FCBFQPResult
{
	GENERATED_BODY()

	// 滤波后的安全加速度 (cm/s²)
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	FVector SafeAcceleration;

	// 是否触发了滤波（CBF 约束被激活）
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	bool bWasFiltered = false;

	// 最小 h 值（诊断用，h < 0 表示安全约束被违反）
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	float MinHValue = MAX_FLT;

	FCBFQPResult()
		: SafeAcceleration(FVector::ZeroVector)
	{}
};

/**
 * CBF-QP 安全滤波器
 *
 * 在 NMPC 输出上叠加安全约束，保证机间安全距离。
 *
 * 原理:
 * - CBF 函数: h(x) = ||pi - pj||² - DSafe²
 * - 相对阶数 2（双积分模型 x' = v, v' = u）
 * - 约束: h_ddot + α₁·h_dot + α₀·h ≥ 0
 * - QP: min ||u - u_nmpc||² s.t. A_k·u ≤ b_k
 *
 * 求解器使用投影梯度法，对 ≤3 个约束 2-5 次迭代即可收敛。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UCBFQPFilter : public UObject
{
	GENERATED_BODY()

public:
	UCBFQPFilter();

	/**
	 * 对 NMPC 输出进行安全滤波
	 * @param NominalAcceleration NMPC 输出的期望加速度 (cm/s²)
	 * @param MyState 本机状态
	 * @param NeighborStates 邻居状态列表
	 * @param Config CBF-QP 配置
	 * @return 滤波结果
	 */
	FCBFQPResult Filter(
		const FVector& NominalAcceleration,
		const FUAVState& MyState,
		const TArray<FAgentStateSnapshot>& NeighborStates,
		const FCBFQPConfig& Config);

	// ---- 内部方法 (public for testing) ----

	/**
	 * 计算 CBF 函数值 h = ||pi - pj||² - DSafe²
	 */
	float ComputeHValue(const FVector& Pi, const FVector& Pj, float DSafe) const;

	/**
	 * 计算 CBF 一阶导数 h_dot = 2*(pi - pj)·(vi - vj)
	 */
	float ComputeHDot(const FVector& Pi, const FVector& Pj,
		const FVector& Vi, const FVector& Vj) const;

	/**
	 * 构建 CBF 约束
	 * 对于双积分模型的二阶 CBF:
	 *   约束形式: A_k · u_i ≤ b_k
	 *   其中 A_k = -2*(pi - pj), b_k = α₁·h_dot + α₀·h - 2*||vi - vj||² + 2*(pi-pj)·uj
	 */
	void BuildCBFConstraints(
		const FUAVState& MyState,
		const TArray<FAgentStateSnapshot>& NeighborStates,
		const FCBFQPConfig& Config,
		TArray<FVector>& OutConstraintNormals,
		TArray<float>& OutConstraintBounds) const;

	/**
	 * 投影梯度 QP 求解器
	 * min ||u - u_nominal||²  s.t. A_k · u ≤ b_k
	 */
	FVector SolveProjectedGradientQP(
		const FVector& NominalAccel,
		const TArray<FVector>& ConstraintNormals,
		const TArray<float>& ConstraintBounds,
		const FCBFQPConfig& Config) const;
};
