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
	const double StartTime = FPlatformTime::Seconds();
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

	// 计算真实最小 h 值（诊断）
	Result.MinHValue = MAX_FLT;
	for (const FAgentStateSnapshot& Neighbor : NeighborStates)
	{
		float HVal = ComputeHValue(MyState.Position, Neighbor.State.Position, Config.DSafe);
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

	// 安全加速度限幅：当 h 严重为负时，限制朝向邻居的加速度分量
	// 防止投影梯度法在多约束时产生朝向邻居的残余加速度
	FVector SafeAccel = Result.SafeAcceleration;
	for (int32 k = 0; k < ConstraintNormals.Num(); ++k)
	{
		if (ConstraintBounds[k] < 0.0f) // h < 0: 安全距离已违反
		{
			FVector Normal = ConstraintNormals[k].GetSafeNormal();
			float DotAccel = FVector::DotProduct(SafeAccel, Normal);
			if (DotAccel > 0.0f)
			{
				// 加速度有朝向邻居的分量，强制反转为排斥方向
				SafeAccel -= DotAccel * Normal;
			}
		}
	}
	Result.SafeAcceleration = SafeAccel;

	// 统计活跃约束
	for (int32 k = 0; k < ConstraintNormals.Num(); ++k)
	{
		float Violation = FVector::DotProduct(ConstraintNormals[k], NominalAcceleration) - ConstraintBounds[k];
		if (Violation > 0.0f)
		{
			++Result.ActiveConstraintCount;
		}
	}

	Result.SolveTimeMs = static_cast<float>((FPlatformTime::Seconds() - StartTime) * 1000.0);

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

		// 二阶 CBF 约束 (设计文档 §7.1):
		//   -2·Δp·ui ≤ 2·‖Δv‖² - 2·Δp·uj + α₁·ḣ + α₀·h
		//   A = -2·Δp, b = 2·‖Δv‖² - 2·Δp·uj + α₁·ḣ + α₀·h
		float Bound = 2.0f * VelNormSq - 2.0f * DeltaPDotUj
			+ Config.Alpha1 * HDot
			+ Config.Alpha0 * H;

		FVector ConstraintNormal = -2.0f * DeltaP;

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


// ========== Active-Set QP 求解器 ==========

void UCBFQPFilter::SolveActiveSetQP(
    int32 N, int32 M,
    const TArray<float>& H, const TArray<float>& G,
    const TArray<float>& A, const TArray<float>& B,
    const FCBFQPConfig& Config,
    TArray<float>& OutZ,
    ECBFQPStatus& OutStatus,
    float& OutKKTResidual,
    float& OutMaxViolation,
    int32& OutIterations) const
{
    OutZ.SetNumZeroed(N);
    OutStatus = ECBFQPStatus::NumericalFailure;
    OutKKTResidual = MAX_FLT;
    OutMaxViolation = MAX_FLT;
    OutIterations = 0;

    if (N <= 0 || M < 0 || H.Num() != N * N || G.Num() != N ||
        A.Num() != M * N || B.Num() != M)
    {
        return;
    }

    // 无约束: 直接求解对角 Hessian
    if (M == 0)
    {
        for (int32 i = 0; i < N; ++i)
            OutZ[i] = (H[i * N + i] > KINDA_SMALL_NUMBER) ? -G[i] / H[i * N + i] : 0.0f;
        OutStatus = ECBFQPStatus::Solved;
        OutKKTResidual = 0.0f;
        OutMaxViolation = 0.0f;
        return;
    }

    const double FeasibilityTolerance =
        FMath::Max(static_cast<double>(Config.QPKKTTolerance), 1e-6);
    const double ActiveTolerance = FeasibilityTolerance * 10.0;
    const double DirectionTolerance = 1e-8;
    const double DualTolerance = 1e-7;

    // 从无约束最优解 z* = -H⁻¹g 出发
    TArray<double> Z;
    Z.SetNumZeroed(N);

    // 构造无约束最优解
    for (int32 i = 0; i < N; ++i)
    {
        const double Hii = H[i * N + i];
        if (Hii <= KINDA_SMALL_NUMBER)
        {
            // Hessian 对角线退化 → 无唯一最优解，放弃
            return;
        }
        Z[i] = -static_cast<double>(G[i]) / Hii;
    }
    // 将前 3 个分量（ux, uy, uz）钳位到执行器边界，保证物理可行
    for (int32 i = 0; i < FMath::Min(N, 3); ++i)
    {
        Z[i] = FMath::Clamp(
            Z[i],
            -static_cast<double>(Config.MaxAccelerationQP),
            static_cast<double>(Config.MaxAccelerationQP));
    }

    // RowScale: 计算约束行向量的 L2 范数（下限为 1，避免除零）
    // 用于将约束残差归一化，使得不同量纲的约束具有可比性
    auto RowScale = [&](int32 ConstraintIndex)
    {
        double NormSq = 0.0;
        for (int32 j = 0; j < N; ++j)
        {
            const double Value = A[ConstraintIndex * N + j];
            NormSq += Value * Value;
        }
        return FMath::Max(1.0, FMath::Sqrt(NormSq));
    };

    // ConstraintValue: 计算约束 k 在 Point 处的值 = A_k·Point - B_k
    // > 0 表示违反约束（A·z ≤ b），= 0 表示恰好激活
    auto ConstraintValue = [&](int32 ConstraintIndex, const TArray<double>& Point)
    {
        double Value = -static_cast<double>(B[ConstraintIndex]);
        for (int32 j = 0; j < N; ++j)
        {
            Value += static_cast<double>(A[ConstraintIndex * N + j]) * Point[j];
        }
        return Value;
    };

    // ExpandSlackForFeasibility: 对指定 slack 变量（z[3]=ξ_static 或 z[4]=ξ_agent），
    // 遍历所有使用该 slack 的约束，计算使所有约束满足所需的最小 slack 值。
    // 前提：slack 在约束中系数为负（如 -1），即 A_k = [..., -1, ...] 意味着
    //   A_k·z ≤ B_k  ⟹  ... - ξ_s ≤ B_k  ⟹  ξ_s ≥ (... - B_k) / 1
    // 当 SlackCoeff >= 0（不使用此 slack 或方向错误）时跳过。
    auto ExpandSlackForFeasibility = [&](int32 SlackIndex)
    {
        double RequiredSlack = FMath::Max(0.0, Z[SlackIndex]);
        for (int32 k = 0; k < M; ++k)
        {
            const double SlackCoeff = A[k * N + SlackIndex];
            if (SlackCoeff >= -KINDA_SMALL_NUMBER)
            {
                continue;
            }

            double ValueWithoutSlack = -static_cast<double>(B[k]);
            for (int32 j = 0; j < N; ++j)
            {
                if (j != SlackIndex)
                {
                    ValueWithoutSlack += static_cast<double>(A[k * N + j]) * Z[j];
                }
            }
            RequiredSlack = FMath::Max(
                RequiredSlack, ValueWithoutSlack / -SlackCoeff);
        }
        Z[SlackIndex] = RequiredSlack;
    };

    // 如果 N >= 5（即决策变量含 ξ_static=z[3] 和 ξ_agent=z[4]），
    // 通过扩展 slack 变量保证初始点可行
    if (N >= 5)
    {
        Z[3] = 0.0;
        Z[4] = 0.0;
        ExpandSlackForFeasibility(3);
        ExpandSlackForFeasibility(4);
    }

    // 初始可行性检查：归一化残差超过容限时直接返回失败
    // （正常调用路径下，slack 扩展应保证通过此检查）
    double InitialMaxNormalizedViolation = 0.0;
    for (int32 k = 0; k < M; ++k)
    {
        InitialMaxNormalizedViolation = FMath::Max(
            InitialMaxNormalizedViolation,
            ConstraintValue(k, Z) / RowScale(k));
    }
    if (InitialMaxNormalizedViolation > FeasibilityTolerance)
    {
        return;
    }

    TArray<int32> ActiveSet;

    auto ContainsConstraint = [&](int32 ConstraintIndex)
    {
        return ActiveSet.Contains(ConstraintIndex);
    };

    // IsLinearlyIndependent: 使用 Modified Gram-Schmidt (MGS) 检查候选约束行
    // 是否与当前活跃集中已有约束线性无关。RowScale 归一化后进行正交化，
    // 残差范数 <= 1e-12 时判定为线性相关，拒绝加入活跃集。
    auto IsLinearlyIndependent = [&](int32 CandidateIndex)
    {
        TArray<TArray<double>> Basis;
        Basis.Reserve(N);

        auto AddRowToBasis = [&](int32 ConstraintIndex)
        {
            TArray<double> Vector;
            Vector.SetNumZeroed(N);
            const double Scale = RowScale(ConstraintIndex);
            for (int32 j = 0; j < N; ++j)
            {
                Vector[j] = static_cast<double>(A[ConstraintIndex * N + j]) / Scale;
            }

            for (const TArray<double>& BasisVector : Basis)
            {
                double Projection = 0.0;
                for (int32 j = 0; j < N; ++j)
                {
                    Projection += Vector[j] * BasisVector[j];
                }
                for (int32 j = 0; j < N; ++j)
                {
                    Vector[j] -= Projection * BasisVector[j];
                }
            }

            double NormSq = 0.0;
            for (double Value : Vector)
            {
                NormSq += Value * Value;
            }
            if (NormSq <= 1e-12)
            {
                return false;
            }

            const double InvNorm = 1.0 / FMath::Sqrt(NormSq);
            for (double& Value : Vector)
            {
                Value *= InvNorm;
            }
            Basis.Add(MoveTemp(Vector));
            return true;
        };

        for (int32 ConstraintIndex : ActiveSet)
        {
            AddRowToBasis(ConstraintIndex);
        }
        return AddRowToBasis(CandidateIndex);
    };

    // 初始化活跃集：找到所有在初始点处"近似激活"（归一化残差 ≈ 0）的约束，
    // 跳过线性相关的约束，活跃集大小不超过 N（满秩）。
    for (int32 k = 0; k < M && ActiveSet.Num() < N; ++k)
    {
        const double NormalizedResidual = ConstraintValue(k, Z) / RowScale(k);
        if (FMath::Abs(NormalizedResidual) <= ActiveTolerance &&
            IsLinearlyIndependent(k))
        {
            ActiveSet.Add(k);
        }
    }

    // SolveKKT: 求解 KKT 系统（活跃集固定的等式约束 QP）
    // 增广矩阵布局 (Dimension × (Dimension+1))：
    //   [  H   A_c^T | -g ]   其中 A_c = 活跃约束矩阵（行归一化）
    //   [ A_c    0   |  0 ]   g = H·z + G 是当前梯度
    // 使用 Gauss-Jordan 全消元（带部分主元选取）求解方向 d 和乘子 λ。
    auto SolveKKT = [&](TArray<double>& OutDirection, TArray<double>& OutMultipliers)
    {
        const int32 ActiveCount = ActiveSet.Num();
        const int32 Dimension = N + ActiveCount;
        const int32 AugmentedStride = Dimension + 1;
        TArray<double> Augmented;
        Augmented.SetNumZeroed(Dimension * AugmentedStride);

        TArray<double> Gradient;
        Gradient.SetNumZeroed(N);
        for (int32 i = 0; i < N; ++i)
        {
            Gradient[i] = G[i];
            for (int32 j = 0; j < N; ++j)
            {
                Gradient[i] += static_cast<double>(H[i * N + j]) * Z[j];
                Augmented[i * AugmentedStride + j] = H[i * N + j];
            }
            Augmented[i * AugmentedStride + Dimension] = -Gradient[i];
        }

        for (int32 ActiveIndex = 0; ActiveIndex < ActiveCount; ++ActiveIndex)
        {
            const int32 ConstraintIndex = ActiveSet[ActiveIndex];
            const double Scale = RowScale(ConstraintIndex);
            for (int32 j = 0; j < N; ++j)
            {
                const double Value =
                    static_cast<double>(A[ConstraintIndex * N + j]) / Scale;
                Augmented[j * AugmentedStride + N + ActiveIndex] = Value;
                Augmented[(N + ActiveIndex) * AugmentedStride + j] = Value;
            }
        }

        for (int32 Column = 0; Column < Dimension; ++Column)
        {
            int32 PivotRow = Column;
            double PivotMagnitude =
                FMath::Abs(Augmented[Column * AugmentedStride + Column]);
            for (int32 Row = Column + 1; Row < Dimension; ++Row)
            {
                const double CandidateMagnitude =
                    FMath::Abs(Augmented[Row * AugmentedStride + Column]);
                if (CandidateMagnitude > PivotMagnitude)
                {
                    PivotMagnitude = CandidateMagnitude;
                    PivotRow = Row;
                }
            }
            if (PivotMagnitude <= 1e-10)
            {
                return false;
            }

            if (PivotRow != Column)
            {
                for (int32 Entry = Column; Entry <= Dimension; ++Entry)
                {
                    Swap(
                        Augmented[Column * AugmentedStride + Entry],
                        Augmented[PivotRow * AugmentedStride + Entry]);
                }
            }

            const double Pivot = Augmented[Column * AugmentedStride + Column];
            for (int32 Entry = Column; Entry <= Dimension; ++Entry)
            {
                Augmented[Column * AugmentedStride + Entry] /= Pivot;
            }

            for (int32 Row = 0; Row < Dimension; ++Row)
            {
                if (Row == Column)
                {
                    continue;
                }
                const double Factor = Augmented[Row * AugmentedStride + Column];
                if (FMath::Abs(Factor) <= 1e-15)
                {
                    continue;
                }
                for (int32 Entry = Column; Entry <= Dimension; ++Entry)
                {
                    Augmented[Row * AugmentedStride + Entry] -=
                        Factor * Augmented[Column * AugmentedStride + Entry];
                }
            }
        }

        OutDirection.SetNumZeroed(N);
        for (int32 i = 0; i < N; ++i)
        {
            OutDirection[i] = Augmented[i * AugmentedStride + Dimension];
        }
        OutMultipliers.SetNumZeroed(ActiveCount);
        for (int32 i = 0; i < ActiveCount; ++i)
        {
            OutMultipliers[i] =
                Augmented[(N + i) * AugmentedStride + Dimension];
        }
        return true;
    };

    // Active-Set 主迭代循环
    // 每轮：求解 KKT → 检查方向范数 → 非零时计算步长并前进 → 零时检查对偶可行性
    bool bConverged = false;
    TArray<double> Direction;
    TArray<double> Multipliers;
    const int32 MaxIterations = FMath::Max(Config.QPMaxIterations, 1);

    for (int32 Iter = 0; Iter < MaxIterations; ++Iter)
    {
        OutIterations = Iter + 1;

        // KKT 奇异时移除最后加入的约束，重试
        if (!SolveKKT(Direction, Multipliers))
        {
            if (ActiveSet.Num() > 0)
            {
                ActiveSet.Pop(EAllowShrinking::No);
                continue;
            }
            return;
        }

        double DirectionNormSq = 0.0;
        for (double Value : Direction)
        {
            DirectionNormSq += Value * Value;
        }

        // 方向 d ≈ 0：当前点是活跃集约束下的驻点
        if (DirectionNormSq <= DirectionTolerance * DirectionTolerance)
        {
            // 对偶可行性检查：所有活跃约束的乘子应 >= 0（不等式约束的 KKT 条件）
            // 找到最违反的乘子（最负），若不存在则收敛
            int32 RemoveIndex = INDEX_NONE;
            double MostNegativeMultiplier = -DualTolerance;
            for (int32 i = 0; i < Multipliers.Num(); ++i)
            {
                if (Multipliers[i] < MostNegativeMultiplier)
                {
                    MostNegativeMultiplier = Multipliers[i];
                    RemoveIndex = i;
                }
            }

            if (RemoveIndex == INDEX_NONE)
            {
                // 所有乘子 >= 0：KKT 条件完全满足，最优解已找到
                bConverged = true;
                break;
            }

            // 移除最违反的约束，释放自由度
            ActiveSet.RemoveAt(RemoveIndex, 1, EAllowShrinking::No);
            continue;
        }

        // 非零方向：计算最大可行步长（ratio test）
        // Step = min over inactive k: -ConstraintValue(k, Z) / (A_k · d)
        // 找到使新约束首次被违反的步长
        double Step = 1.0;
        int32 BlockingConstraint = INDEX_NONE;
        for (int32 k = 0; k < M; ++k)
        {
            if (ContainsConstraint(k))
            {
                continue;
            }

            // A_k · d > 0 时沿 d 方向会违反约束 k，需要计算最大可行步长
            double DirectionalDerivative = 0.0;
            for (int32 j = 0; j < N; ++j)
            {
                DirectionalDerivative +=
                    static_cast<double>(A[k * N + j]) * Direction[j];
            }
            if (DirectionalDerivative <= 1e-12 * RowScale(k))
            {
                continue;
            }

            // Margin = -ConstraintValue(k, Z) = B_k - A_k·Z（到约束边界的余量）
            const double Margin = -ConstraintValue(k, Z);
            const double CandidateStep = Margin / DirectionalDerivative;
            if (CandidateStep < Step)
            {
                Step = FMath::Max(0.0, CandidateStep);
                BlockingConstraint = k;
            }
        }

        // 沿方向前进 Step 步
        for (int32 j = 0; j < N; ++j)
        {
            Z[j] += Step * Direction[j];
        }

        // 将 blocking constraint 加入活跃集（如果线性无关且未满秩）
        if (BlockingConstraint != INDEX_NONE &&
            ActiveSet.Num() < N &&
            IsLinearlyIndependent(BlockingConstraint))
        {
            ActiveSet.Add(BlockingConstraint);
        }
    }

    // 将 double 解转换为 float 输出，同时检查 NaN/Inf
    OutZ.SetNumZeroed(N);
    for (int32 i = 0; i < N; ++i)
    {
        if (!FMath::IsFinite(Z[i]))
        {
            return;
        }
        OutZ[i] = static_cast<float>(Z[i]);
    }

    // 计算最终约束违反量（原始尺度用于诊断，归一化尺度用于收敛判定）
    double MaxNormalizedViolation = 0.0;
    OutMaxViolation = 0.0f;
    for (int32 k = 0; k < M; ++k)
    {
        double RawViolation = -static_cast<double>(B[k]);
        for (int32 j = 0; j < N; ++j)
        {
            RawViolation +=
                static_cast<double>(A[k * N + j]) * OutZ[j];
        }
        OutMaxViolation = FMath::Max(
            OutMaxViolation, static_cast<float>(RawViolation));
        MaxNormalizedViolation = FMath::Max(
            MaxNormalizedViolation, RawViolation / RowScale(k));
    }
    // 约束违反量截断到 0（解满足约束时无违反）
    OutMaxViolation = FMath::Max(0.0f, OutMaxViolation);

    // 状态判定：收敛且可行 → Solved/SolvedWithSlack，否则 → MaxIterations
    if (bConverged && MaxNormalizedViolation <= FeasibilityTolerance)
    {
        bool bHasSlack = false;
        if (N >= 5)
        {
            if (OutZ[3] > Config.QPKKTTolerance) bHasSlack = true;
            if (OutZ[4] > Config.QPKKTTolerance) bHasSlack = true;
        }
        OutStatus = bHasSlack ? ECBFQPStatus::SolvedWithSlack : ECBFQPStatus::Solved;
    }
    else
    {
        OutStatus = ECBFQPStatus::MaxIterations;
    }

    // 驻点残差: ‖∇_z L‖ = ‖Hz + g + Σ λ_k · A_k / RowScale_k‖
    // 使用活跃集乘子和原始约束行（带 RowScale 归一化）计算，
    // 比单纯用梯度范数更准确反映 KKT 条件的满足程度。
    double StationaritySq = 0.0;
    for (int32 i = 0; i < N; ++i)
    {
        double Gradient = G[i];
        for (int32 j = 0; j < N; ++j)
        {
            Gradient += static_cast<double>(H[i * N + j]) * OutZ[j];
        }
        for (int32 ActiveIndex = 0; ActiveIndex < ActiveSet.Num() &&
            ActiveIndex < Multipliers.Num(); ++ActiveIndex)
        {
            const int32 ConstraintIndex = ActiveSet[ActiveIndex];
            Gradient += Multipliers[ActiveIndex] *
                static_cast<double>(A[ConstraintIndex * N + i]) /
                RowScale(ConstraintIndex);
        }
        StationaritySq += Gradient * Gradient;
    }
    OutKKTResidual = static_cast<float>(FMath::Sqrt(StationaritySq));
}

// ========== 静态障碍 HOCBF 约束 ==========

void UCBFQPFilter::BuildStaticObstacleConstraints(
    const FUAVState& MyState,
    const TArray<FObstacleInfo>& Obstacles,
    const FCBFQPConfig& Config,
    TArray<float>& OutAFlat,
    TArray<float>& OutBounds) const
{
    OutAFlat.Empty();
    OutBounds.Empty();

    // 需要 NMPC 距离计算器来获取梯度和距离
    // 这里直接使用 FObstacleInfo 的几何信息计算
    const int32 NCols = 5; // [ux, uy, uz, xi_static, xi_agent]

    for (const FObstacleInfo& Obs : Obstacles)
    {
        // 计算有符号距离
        float d = 0.0f;
        FVector GradD = FVector::ZeroVector;

        switch (Obs.Type)
        {
        case EObstacleType::Sphere:
        {
            FVector Delta = MyState.Position - Obs.Center;
            float Dist = Delta.Size();
            d = Dist - Obs.Extents.X - Obs.SafetyMargin - Config.DSafeStatic;
            if (Dist > KINDA_SMALL_NUMBER)
                GradD = Delta / Dist;
            else
                GradD = FVector::UpVector;
            break;
        }
        case EObstacleType::Box:
        {
            FVector LocalPoint = Obs.Rotation.UnrotateVector(MyState.Position - Obs.Center);
            FVector Clamped;
            Clamped.X = FMath::Clamp(LocalPoint.X, -Obs.Extents.X, Obs.Extents.X);
            Clamped.Y = FMath::Clamp(LocalPoint.Y, -Obs.Extents.Y, Obs.Extents.Y);
            Clamped.Z = FMath::Clamp(LocalPoint.Z, -Obs.Extents.Z, Obs.Extents.Z);

            FVector LocalGrad;
            if (LocalPoint.Equals(Clamped, KINDA_SMALL_NUMBER))
            {
                // 内部: 最小穿透轴
                float PenX = Obs.Extents.X - FMath::Abs(LocalPoint.X);
                float PenY = Obs.Extents.Y - FMath::Abs(LocalPoint.Y);
                float PenZ = Obs.Extents.Z - FMath::Abs(LocalPoint.Z);
                float MinPen = FMath::Min3(PenX, PenY, PenZ);
                d = -MinPen - Obs.SafetyMargin - Config.DSafeStatic;

                if (PenX <= PenY && PenX <= PenZ)
                    LocalGrad = FVector(LocalPoint.X > 0 ? 1.0f : -1.0f, 0.0f, 0.0f);
                else if (PenY <= PenX && PenY <= PenZ)
                    LocalGrad = FVector(0.0f, LocalPoint.Y > 0 ? 1.0f : -1.0f, 0.0f);
                else
                    LocalGrad = FVector(0.0f, 0.0f, LocalPoint.Z > 0 ? 1.0f : -1.0f);
            }
            else
            {
                d = FVector::Dist(LocalPoint, Clamped) - Obs.SafetyMargin - Config.DSafeStatic;
                LocalGrad = (LocalPoint - Clamped).GetSafeNormal();
            }
            GradD = Obs.Rotation.RotateVector(LocalGrad);
            break;
        }
        case EObstacleType::Cylinder:
        {
            FVector LocalPoint = Obs.Rotation.UnrotateVector(MyState.Position - Obs.Center);
            float HDist = FVector2D(LocalPoint.X, LocalPoint.Y).Size();
            float VDist = FMath::Abs(LocalPoint.Z);
            float HPen = HDist - Obs.Extents.X;
            float VPen = VDist - Obs.Extents.Z;

            FVector LocalGrad;
            if (HPen < 0 && VPen < 0)
            {
                d = FMath::Max(HPen, VPen) - Obs.SafetyMargin - Config.DSafeStatic;
                if (FMath::Abs(HPen) <= FMath::Abs(VPen))
                    LocalGrad = HDist > KINDA_SMALL_NUMBER
                        ? FVector(LocalPoint.X, LocalPoint.Y, 0.0f).GetSafeNormal()
                        : FVector(1.0f, 0.0f, 0.0f);
                else
                    LocalGrad = FVector(0.0f, 0.0f, LocalPoint.Z > 0 ? 1.0f : -1.0f);
            }
            else if (HPen < 0)
            {
                d = VPen - Obs.SafetyMargin - Config.DSafeStatic;
                LocalGrad = FVector(0.0f, 0.0f, LocalPoint.Z > 0 ? 1.0f : -1.0f);
            }
            else if (VPen < 0)
            {
                d = HPen - Obs.SafetyMargin - Config.DSafeStatic;
                LocalGrad = HDist > KINDA_SMALL_NUMBER
                    ? FVector(LocalPoint.X, LocalPoint.Y, 0.0f).GetSafeNormal()
                    : FVector(1.0f, 0.0f, 0.0f);
            }
            else
            {
                d = FMath::Sqrt(HPen * HPen + VPen * VPen) - Obs.SafetyMargin - Config.DSafeStatic;
                FVector2D HDir = HDist > KINDA_SMALL_NUMBER
                    ? FVector2D(LocalPoint.X, LocalPoint.Y).GetSafeNormal()
                    : FVector2D(1.0f, 0.0f);
                float VSign = LocalPoint.Z > 0 ? 1.0f : -1.0f;
                LocalGrad = FVector(HDir.X * HPen, HDir.Y * HPen, VSign * VPen).GetSafeNormal();
            }
            GradD = Obs.Rotation.RotateVector(LocalGrad);
            break;
        }
        default:
        {
            FVector Delta = MyState.Position - Obs.Center;
            float Dist = Delta.Size();
            d = Dist - Obs.Extents.GetMax() - Obs.SafetyMargin - Config.DSafeStatic;
            GradD = Dist > KINDA_SMALL_NUMBER ? Delta / Dist : FVector::UpVector;
            break;
        }
        }

        // CBF 函数: h = d(p) - d_safe (已经包含了 DSafeStatic 在上面的 d 计算中)
        // 实际上 h = d_without_dsafe - d_safe, 我们上面直接算了 d = d_without_dsafe - d_safe
        // 所以 h = d (已经减去了 DSafeStatic)
        float HVal = d;
        // 使用相对速度：动态障碍物需要减去其自身速度（P3 动态障碍修复）
        FVector RelativeVelocity = MyState.Velocity - (Obs.bIsDynamic ? Obs.Velocity : FVector::ZeroVector);
        float HDot = FVector::DotProduct(GradD, RelativeVelocity);

        // 忽略远离且安全的障碍物
        if (HVal > Config.StaticInfluenceDistance)
            continue;

        // 二阶 CBF 约束 (§7.2):
        //   ḧ + α₁·ḣ + α₀·h ≥ -ξ_static
        //   ∇d·u + α₁·ḣ + α₀·h ≥ -ξ_static
        //   -(∇d·u) - ξ_static ≤ (α₁·ḣ + α₀·h)
        //
        // A 行: [-∇d.x, -∇d.y, -∇d.z, -1, 0]
        // Bound: (α₁·ḣ + α₀·h)
        //
        // Hessian 项 v'·H_d·v 对球体为 (1/r)(|v|² - (v·r̂)²)，保守取 0
        float HDotDContribution = 0.0f; // 保守忽略 Hessian 项

        float Bound = Config.StaticAlpha1 * HDot + Config.StaticAlpha0 * HVal + HDotDContribution;

        // A 行: [-GradD.x, -GradD.y, -GradD.z, -1, 0]
        OutAFlat.Add(-GradD.X);
        OutAFlat.Add(-GradD.Y);
        OutAFlat.Add(-GradD.Z);
        OutAFlat.Add(-1.0f); // -ξ_static 系数
        OutAFlat.Add(0.0f);  // ξ_agent 系数
        OutBounds.Add(Bound);
    }
}

// ========== 统一 Filter ==========

FCBFQPResult UCBFQPFilter::FilterV2(
    const FVector& NominalAcceleration,
    const FUAVState& MyState,
    const TArray<FAgentStateSnapshot>& NeighborStates,
    const TArray<FObstacleInfo>& StaticObstacles,
    const FCBFQPConfig& Config)
{
    const double StartTime = FPlatformTime::Seconds();
    FCBFQPResult Result;
    Result.SafeAcceleration = NominalAcceleration;

    const int32 N = 5; // [ux, uy, uz, xi_static, xi_agent]

    // ---- 收集所有约束 ----
    TArray<float> AllAFlat;  // 行优先 A 矩阵
    TArray<float> AllBounds; // b 向量

    // 1. 静态障碍 HOCBF 约束
    TArray<float> StaticAFlat, StaticBounds;
    BuildStaticObstacleConstraints(MyState, StaticObstacles, Config, StaticAFlat, StaticBounds);
    int32 StaticCount = StaticBounds.Num();
    AllAFlat.Append(StaticAFlat);
    AllBounds.Append(StaticBounds);
    Result.StaticConstraintCount = StaticCount;

    // 2. 机间二阶 CBF 约束
    TArray<FVector> AgentNormals;
    TArray<float> AgentBoundsOld;
    BuildCBFConstraints(MyState, NeighborStates, Config, AgentNormals, AgentBoundsOld);
    int32 AgentCount = AgentNormals.Num();
    for (int32 k = 0; k < AgentCount; ++k)
    {
        // 旧格式: A = ConstraintNormal (3D), b = Bound
        // 新格式: A 行 = [Normal.x, Normal.y, Normal.z, 0, -1]
        // 因为旧约束是 A·u ≤ b，即 2·Δp·ui ≤ bound
        // 但我们的新 A·z ≤ b 格式需要:
        // 机间约束中 ξ_agent 对应 z[4]
        // 旧约束 A_k·u ≤ b 等价于 [A_k.x, A_k.y, A_k.z, 0, 0]·z ≤ b
        // 但还需要 ξ_agent 项: 修改为 [A_k.x, A_k.y, A_k.z, 0, -1]·z ≤ b
        // 即 A_k·u - ξ_agent ≤ b，或 A_k·u ≤ b + ξ_agent
        AllAFlat.Add(AgentNormals[k].X);
        AllAFlat.Add(AgentNormals[k].Y);
        AllAFlat.Add(AgentNormals[k].Z);
        AllAFlat.Add(0.0f);   // ξ_static 系数
        AllAFlat.Add(-1.0f);  // -ξ_agent 系数
        AllBounds.Add(AgentBoundsOld[k]);
    }
    Result.AgentConstraintCount = AgentCount;

    // 3. Slack 非负性: -ξ_static ≤ 0, -ξ_agent ≤ 0
    // 即 z[3] >= 0 → [-1·z[3]] ≤ 0
    {
        float Row1[] = {0, 0, 0, -1.0f, 0};
        AllAFlat.Append(Row1, 5);
        AllBounds.Add(0.0f);
        float Row2[] = {0, 0, 0, 0, -1.0f};
        AllAFlat.Append(Row2, 5);
        AllBounds.Add(0.0f);
    }

    // 4. 加速度界限: |u_i| ≤ a_max → u_i ≤ a_max 和 -u_i ≤ a_max
    {
        float AMax = Config.MaxAccelerationQP;
        // +X: u_x ≤ a_max
        float RowPosX[] = {1, 0, 0, 0, 0};
        AllAFlat.Append(RowPosX, 5); AllBounds.Add(AMax);
        float RowNegX[] = {-1, 0, 0, 0, 0};
        AllAFlat.Append(RowNegX, 5); AllBounds.Add(AMax);
        // +Y
        float RowPosY[] = {0, 1, 0, 0, 0};
        AllAFlat.Append(RowPosY, 5); AllBounds.Add(AMax);
        float RowNegY[] = {0, -1, 0, 0, 0};
        AllAFlat.Append(RowNegY, 5); AllBounds.Add(AMax);
        // +Z
        float RowPosZ[] = {0, 0, 1, 0, 0};
        AllAFlat.Append(RowPosZ, 5); AllBounds.Add(AMax);
        float RowNegZ[] = {0, 0, -1, 0, 0};
        AllAFlat.Append(RowNegZ, 5); AllBounds.Add(AMax);
    }

    int32 M = AllBounds.Num();

    // ---- 构建 QP 目标函数 ----
    // min 0.5·(u-u_nom)'·W·(u-u_nom) + ρ_s·ξ_s² + ρ_a·ξ_a²
    // H = diag(Wx, Wy, Wz, 2·ρ_s, 2·ρ_a)
    // g = [-Wx·u_nom_x, -Wy·u_nom_y, -Wz·u_nom_z, 0, 0]
    TArray<float> HMat, GVec;
    HMat.SetNumZeroed(N * N);
    GVec.SetNumZeroed(N);

    float Wx = 1.0f, Wy = 1.0f, Wz = 1.0f;
    HMat[0 * N + 0] = Wx;
    HMat[1 * N + 1] = Wy;
    HMat[2 * N + 2] = Wz;
    HMat[3 * N + 3] = 2.0f * Config.RhoStatic;
    HMat[4 * N + 4] = 2.0f * Config.RhoAgent;

    GVec[0] = -Wx * NominalAcceleration.X;
    GVec[1] = -Wy * NominalAcceleration.Y;
    GVec[2] = -Wz * NominalAcceleration.Z;
    // GVec[3] = 0 (no linear cost on ξ_static)
    // GVec[4] = 0 (no linear cost on ξ_agent)

    // ---- 求解 QP ----
    TArray<float> Z;
    ECBFQPStatus Status;
    float KKTRes, MaxViol;
    int32 Iters;

    SolveActiveSetQP(N, M, HMat, GVec, AllAFlat, AllBounds, Config,
        Z, Status, KKTRes, MaxViol, Iters);

    // ---- 提取结果 ----
    if (Z.Num() >= 3)
    {
        Result.SafeAcceleration = FVector(Z[0], Z[1], Z[2]);
    }
    Result.StaticSlack = Z.Num() >= 4 ? FMath::Max(0.0f, Z[3]) : 0.0f;
    Result.AgentSlack = Z.Num() >= 5 ? FMath::Max(0.0f, Z[4]) : 0.0f;

    // 是否触发滤波
    Result.bWasFiltered = (StaticCount > 0 || AgentCount > 0) &&
        !Result.SafeAcceleration.Equals(NominalAcceleration, 1.0f);

    // 计算真实最小 h 值（机间）
    Result.MinHValue = MAX_FLT;
    for (const FAgentStateSnapshot& Neighbor : NeighborStates)
    {
        float HVal = ComputeHValue(MyState.Position, Neighbor.State.Position, Config.DSafe);
        Result.MinHValue = FMath::Min(Result.MinHValue, HVal);
    }

    // 统计活跃约束
    Result.ActiveConstraintCount = 0;
    for (int32 k = 0; k < M; ++k)
    {
        float AZ = 0.0f;
        for (int32 j = 0; j < N; ++j)
            AZ += AllAFlat[k * N + j] * (Z.Num() > j ? Z[j] : 0.0f);
        if (AZ > AllBounds[k] - Config.QPKKTTolerance)
            ++Result.ActiveConstraintCount;
    }

    Result.SolveStatus = Status;
    Result.KKTResidual = KKTRes;
    Result.MaxConstraintViolation = MaxViol;
    Result.SolveTimeMs = static_cast<float>((FPlatformTime::Seconds() - StartTime) * 1000.0);

    // 保存解用于诊断（不用于热启动覆盖决策变量）
    PreviousQPSolution = Z;
    bHasPreviousQPSolution = true;

    return Result;
}
