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

    // 初始解: 无约束最优 z0 = H^{-1} * (-g)，对角 Hessian
    // 注意: 不直接用上帧解覆盖，否则约束可行时立即返回旧解，
    // 忽略新标称加速度方向（P1 热启动冻结修复）
    for (int32 i = 0; i < N; ++i)
        OutZ[i] = (H[i * N + i] > KINDA_SMALL_NUMBER) ? -G[i] / H[i * N + i] : 0.0f;

    // 使用逐约束循环投影（Dykstra-like），直到所有约束满足
    // 对角 Hessian 加权投影: z_i = z_i - (a_k·z - b_k) / (a_k' · diag(H)^{-1} . a_k) * (1/H_ii) * a_k_i
    // 简化为: z = z - alpha * a_k, 其中 alpha = violation / (sum_j a_k_j^2 / H_jj)
    int32 MaxIter = Config.QPMaxIterations;
    bool bConverged = false;

    for (int32 Iter = 0; Iter < MaxIter; ++Iter)
    {
        OutIterations = Iter + 1;

        // 找最大违反约束
        float MaxViolation = -MAX_FLT;
        int32 WorstK = -1;

        for (int32 k = 0; k < M; ++k)
        {
            float AZ = 0.0f;
            for (int32 j = 0; j < N; ++j)
                AZ += A[k * N + j] * OutZ[j];
            float Viol = AZ - B[k];
            if (Viol > MaxViolation)
            {
                MaxViolation = Viol;
                WorstK = k;
            }
        }

        OutMaxViolation = FMath::Max(0.0f, MaxViolation);

        if (MaxViolation < Config.QPKKTTolerance)
        {
            bConverged = true;
            break;
        }

        if (WorstK < 0)
        {
            bConverged = true;
            break;
        }

        // Hessian 加权投影到约束 WorstK 的超平面
        // alpha = (a_k . z - b_k) / (sum_j a_k_j^2 / H_jj)
        float Denom = 0.0f;
        float Violation = 0.0f;
        for (int32 j = 0; j < N; ++j)
        {
            float Akj = A[WorstK * N + j];
            float Hjj = H[j * N + j];
            Denom += Akj * Akj / FMath::Max(Hjj, KINDA_SMALL_NUMBER);
            Violation += Akj * OutZ[j];
        }
        Violation -= B[WorstK];

        if (Denom < KINDA_SMALL_NUMBER)
            continue;

        float Alpha = Violation / Denom;

        for (int32 j = 0; j < N; ++j)
        {
            float Akj = A[WorstK * N + j];
            float Hjj = H[j * N + j];
            OutZ[j] -= Alpha * Akj / FMath::Max(Hjj, KINDA_SMALL_NUMBER);
        }

        // NaN 检查
        bool bHasNaN = false;
        for (int32 i = 0; i < N; ++i)
        {
            if (!FMath::IsFinite(OutZ[i])) { bHasNaN = true; break; }
        }
        if (bHasNaN)
        {
            OutStatus = ECBFQPStatus::NumericalFailure;
            for (int32 i = 0; i < N; ++i)
                OutZ[i] = (H[i * N + i] > KINDA_SMALL_NUMBER) ? -G[i] / H[i * N + i] : 0.0f;
            return;
        }
    }

    // 设置求解状态
    if (bConverged)
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

    // 计算 KKT 残差: ||H·z + g|| (梯度范数)
    OutKKTResidual = 0.0f;
    for (int32 i = 0; i < N; ++i)
    {
        float Grad_i = G[i];
        for (int32 j = 0; j < N; ++j)
            Grad_i += H[i * N + j] * OutZ[j];
        OutKKTResidual += Grad_i * Grad_i;
    }
    OutKKTResidual = FMath::Sqrt(OutKKTResidual);
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

// ========== 统一 FilterV2 ==========

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

