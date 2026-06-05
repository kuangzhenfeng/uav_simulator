// Copyright Epic Games, Inc. All Rights Reserved.

#include "CBFQPFilter.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UCBFQPFilter::UCBFQPFilter()
{
}

FCBFQPResult UCBFQPFilter::Filter(
	const FVector& NominalAcceleration,
	const FUAVState& MyState,
	const TArray<FAgentStateSnapshot>& NeighborStates,
	const FCBFQPConfig& Config)
{
	FCBFQPResult Result;
	Result.SafeAcceleration = NominalAcceleration;

	if (NeighborStates.Num() == 0)
	{
		// 无邻居，直接通过
		return Result;
	}

	// 构建 CBF 约束
	TArray<FVector> ConstraintNormals;
	TArray<float> ConstraintBounds;
	BuildCBFConstraints(MyState, NeighborStates, Config, ConstraintNormals, ConstraintBounds);

	if (ConstraintNormals.Num() == 0)
	{
		// 无活跃约束
		return Result;
	}

	// 计算最小 h 值（诊断）
	Result.MinHValue = MAX_FLT;
	for (int32 k = 0; k < ConstraintBounds.Num(); ++k)
	{
		float HVal = ConstraintBounds[k]; // b_k 包含了 h 相关项
		Result.MinHValue = FMath::Min(Result.MinHValue, HVal);
	}

	// 检查是否需要滤波：当前加速度是否违反任何约束
	bool bNeedsFiltering = false;
	for (int32 k = 0; k < ConstraintNormals.Num(); ++k)
	{
		float Violation = FVector::DotProduct(ConstraintNormals[k], NominalAcceleration) - ConstraintBounds[k];
		if (Violation > 0.0f)
		{
			bNeedsFiltering = true;
			break;
		}
	}

	if (!bNeedsFiltering)
	{
		// 所有约束已满足，无需滤波
		return Result;
	}

	// 求解 QP
	Result.SafeAcceleration = SolveProjectedGradientQP(
		NominalAcceleration, ConstraintNormals, ConstraintBounds, Config);
	Result.bWasFiltered = true;

	return Result;
}

float UCBFQPFilter::ComputeHValue(const FVector& Pi, const FVector& Pj, float DSafe) const
{
	float DistSq = FVector::DistSquared(Pi, Pj);
	return DistSq - DSafe * DSafe;
}

float UCBFQPFilter::ComputeHDot(const FVector& Pi, const FVector& Pj,
	const FVector& Vi, const FVector& Vj) const
{
	FVector DeltaP = Pi - Pj;
	FVector DeltaV = Vi - Vj;
	return 2.0f * FVector::DotProduct(DeltaP, DeltaV);
}

void UCBFQPFilter::BuildCBFConstraints(
	const FUAVState& MyState,
	const TArray<FAgentStateSnapshot>& NeighborStates,
	const FCBFQPConfig& Config,
	TArray<FVector>& OutConstraintNormals,
	TArray<float>& OutConstraintBounds) const
{
	OutConstraintNormals.Empty();
	OutConstraintBounds.Empty();

	FVector Pi = MyState.Position;
	FVector Vi = MyState.Velocity;

	for (const FAgentStateSnapshot& Neighbor : NeighborStates)
	{
		FVector Pj = Neighbor.State.Position;
		FVector Vj = Neighbor.State.Velocity;

		// 计算 CBF 值
		float H = ComputeHValue(Pi, Pj, Config.DSafe);
		float HDot = ComputeHDot(Pi, Pj, Vi, Vj);

		FVector DeltaP = Pi - Pj;
		FVector DeltaV = Vi - Vj;

		// 二阶 CBF 约束 (relative degree 2):
		// h_ddot + α₁·h_dot + α₀·h ≥ 0
		// h_ddot = 2*||Δv||² + 2*Δp·(uj - ui)
		// 约束关于 ui:
		//   -2*Δp·ui + (2*||Δv||² + 2*Δp·uj + α₁*h_dot + α₀*h) ≥ 0
		//   即: 2*Δp·ui ≤ 2*||Δv||² + 2*Δp·uj + α₁*h_dot + α₀*h
		//
		// 标准形式 A·u ≤ b:
		//   A = -2*Δp  (注意符号: 约束是 2*Δp·ui ≤ ... → -2*Δp·ui ≥ -...)
		//   但我们用 A·u ≤ b 形式: A = 2*Δp, b = ...
		//   即: (2*Δp)·ui ≤ (2*||Δv||² + 2*Δp·uj + α₁*h_dot + α₀*h)

		// 使用邻居的 NMPC 加速度作为 uj 的估计（协作假设）
		FVector Uj = Neighbor.NMPCAcceleration;

		float VelNormSq = DeltaV.SizeSquared();
		float DeltaPDotUj = FVector::DotProduct(DeltaP, Uj);

		float Bound = VelNormSq + DeltaPDotUj
			+ 0.5f * Config.Alpha1 * HDot
			+ 0.5f * Config.Alpha0 * H;

		// 约束: (2*DeltaP) · ui ≤ Bound → A_k = 2*DeltaP, b_k = Bound
		// 但更常用的标准形式是 A·u ≤ b:
		FVector ConstraintNormal = 2.0f * DeltaP;

		OutConstraintNormals.Add(ConstraintNormal);
		OutConstraintBounds.Add(Bound);
	}
}

FVector UCBFQPFilter::SolveProjectedGradientQP(
	const FVector& NominalAccel,
	const TArray<FVector>& ConstraintNormals,
	const TArray<float>& ConstraintBounds,
	const FCBFQPConfig& Config) const
{
	// 投影梯度法求解:
	// min ||u - u_nominal||²
	// s.t. A_k · u ≤ b_k  for all k
	//
	// 算法: 逐个投影违反约束的超平面，直到收敛
	// 对于 ≤3 个约束，2-5 次迭代即可收敛

	FVector U = NominalAccel;

	for (int32 Iter = 0; Iter < Config.MaxIterations; ++Iter)
	{
		FVector U_Prev = U;
		bool bAllSatisfied = true;

		for (int32 k = 0; k < ConstraintNormals.Num(); ++k)
		{
			float Violation = FVector::DotProduct(ConstraintNormals[k], U) - ConstraintBounds[k];

			if (Violation > 0.0f)
			{
				// 约束违反，投影到超平面
				float NormalNormSq = ConstraintNormals[k].SizeSquared();
				if (NormalNormSq > KINDA_SMALL_NUMBER)
				{
					U = U - (Violation / NormalNormSq) * ConstraintNormals[k];
				}
				bAllSatisfied = false;
			}
		}

		// 收敛检查
		if (bAllSatisfied || (U - U_Prev).SizeSquared() < Config.ConvergenceTolerance * Config.ConvergenceTolerance)
		{
			break;
		}
	}

	return U;
}
