// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "MultiAgentTypes.h"
#include "../Planning/NMPCAvoidance.h"
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

	// 真实最小 h 值: min(||pi-pj||² - DSafe²)，h < 0 表示安全距离已违反
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	float MinHValue = MAX_FLT;

	// 活跃约束数量
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	int32 ActiveConstraintCount = 0;

	// 求解时间 (ms)
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	float SolveTimeMs = 0.0f;

	// ---- 新增诊断字段 ----

	// QP 求解状态
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	ECBFQPStatus SolveStatus = ECBFQPStatus::Solved;

	// KKT 残差
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	float KKTResidual = 0.0f;

	// 最大约束违反量
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	float MaxConstraintViolation = 0.0f;

	// 静态 slack 值
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	float StaticSlack = 0.0f;

	// 机间 slack 值
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	float AgentSlack = 0.0f;

	// 静态障碍约束数量
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	int32 StaticConstraintCount = 0;

	// 机间约束数量
	UPROPERTY(BlueprintReadOnly, Category = "CBFQP")
	int32 AgentConstraintCount = 0;

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
	 * 投影梯度 QP 求解器（旧实现，保留向后兼容）
	 * min ||u - u_nominal||²  s.t. A_k · u ≤ b_k
	 */
	FVector SolveProjectedGradientQP(
		const FVector& NominalAccel,
		const TArray<FVector>& ConstraintNormals,
		const TArray<float>& ConstraintBounds,
		const FCBFQPConfig& Config) const;

	// ---- 统一 QP V2 接口 ----

	/**
	 * 统一安全滤波 V2: 同时处理静态障碍 + 机间 CBF
	 * @param NominalAcceleration NMPC 输出加速度
	 * @param MyState 本机状态
	 * @param NeighborStates 邻居状态列表
	 * @param StaticObstacles 静态障碍物列表
	 * @param Config CBF-QP 配置
	 * @return 滤波结果
	 */
	FCBFQPResult FilterV2(
		const FVector& NominalAcceleration,
		const FUAVState& MyState,
		const TArray<FAgentStateSnapshot>& NeighborStates,
		const TArray<FObstacleInfo>& StaticObstacles,
		const FCBFQPConfig& Config);

	/**
	 * 构建静态障碍 HOCBF 约束
	 * h = d(p) - d_safe, ḣ = ∇d·v, 约束: -∇d·u - ξ_s ≤ -(α₁·ḣ + α₀·h)
	 */
	void BuildStaticObstacleConstraints(
		const FUAVState& MyState,
		const TArray<FObstacleInfo>& Obstacles,
		const FCBFQPConfig& Config,
		TArray<float>& OutAFlat, // 每行 5 个元素的 A 矩阵（行优先）
		TArray<float>& OutBounds) const;

	/**
	 * Active-Set QP 求解器
	 * min 0.5·z'Hz + g'z  s.t. Az ≤ b
	 * 决策变量 z = [ux, uy, uz, ξ_static, ξ_agent]
	 */
	void SolveActiveSetQP(
		int32 N,                    // 决策变量维度 (5)
		int32 M,                    // 约束数量
		const TArray<float>& H,    // Hessian (NxN 行优先)
		const TArray<float>& G,    // 线性项 (N)
		const TArray<float>& A,    // 约束矩阵 (MxN 行优先)
		const TArray<float>& B,    // 约束上界 (M)
		const FCBFQPConfig& Config,
		TArray<float>& OutZ,       // 解向量 (N)
		ECBFQPStatus& OutStatus,
		float& OutKKTResidual,
		float& OutMaxViolation,
		int32& OutIterations) const;

private:
	// 上次 QP 解（用于热启动）
	TArray<float> PreviousQPSolution;
	bool bHasPreviousQPSolution = false;
};
