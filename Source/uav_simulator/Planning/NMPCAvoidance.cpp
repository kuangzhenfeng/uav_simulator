// Copyright Epic Games, Inc. All Rights Reserved.

#include "NMPCAvoidance.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UNMPCAvoidance::UNMPCAvoidance()
{
}

// ========== 前向仿真 ==========
void UNMPCAvoidance::ForwardSimulate(
	const FVector& InitPos,
	const FVector& InitVel,
	const TArray<FVector>& Controls,
	TArray<FVector>& OutPositions,
	TArray<FVector>& OutVelocities) const
{
	const int32 N = Controls.Num();
	const float dt = Config.GetDt();

	OutPositions.SetNum(N + 1);
	OutVelocities.SetNum(N + 1);

	OutPositions[0] = InitPos;
	OutVelocities[0] = InitVel;

	for (int32 k = 0; k < N; ++k)
	{
		OutPositions[k + 1] = OutPositions[k] + OutVelocities[k] * dt;
		OutVelocities[k + 1] = OutVelocities[k] + Controls[k] * dt;
	}
}

// ========== 点到障碍物距离 ==========
float UNMPCAvoidance::CalculateDistanceToObstacle(const FVector& Point, const FObstacleInfo& Obstacle) const
{
	switch (Obstacle.Type)
	{
	case EObstacleType::Sphere:
		return FVector::Dist(Point, Obstacle.Center) - Obstacle.Extents.X - Obstacle.SafetyMargin;

	case EObstacleType::Box:
		{
			FVector LocalPoint = Obstacle.Rotation.UnrotateVector(Point - Obstacle.Center);
			FVector ClosestPoint;
			ClosestPoint.X = FMath::Clamp(LocalPoint.X, -Obstacle.Extents.X, Obstacle.Extents.X);
			ClosestPoint.Y = FMath::Clamp(LocalPoint.Y, -Obstacle.Extents.Y, Obstacle.Extents.Y);
			ClosestPoint.Z = FMath::Clamp(LocalPoint.Z, -Obstacle.Extents.Z, Obstacle.Extents.Z);
			return FVector::Dist(LocalPoint, ClosestPoint) - Obstacle.SafetyMargin;
		}

	case EObstacleType::Cylinder:
		{
			FVector LocalPoint = Point - Obstacle.Center;
			float HorizontalDist = FVector2D(LocalPoint.X, LocalPoint.Y).Size();
			float VerticalDist = FMath::Abs(LocalPoint.Z);

			float HorizontalPen = HorizontalDist - Obstacle.Extents.X;
			float VerticalPen = VerticalDist - Obstacle.Extents.Z;

			if (HorizontalPen < 0 && VerticalPen < 0)
			{
				return FMath::Max(HorizontalPen, VerticalPen) - Obstacle.SafetyMargin;
			}
			else if (HorizontalPen < 0)
			{
				return VerticalPen - Obstacle.SafetyMargin;
			}
			else if (VerticalPen < 0)
			{
				return HorizontalPen - Obstacle.SafetyMargin;
			}
			else
			{
				return FMath::Sqrt(HorizontalPen * HorizontalPen + VerticalPen * VerticalPen) - Obstacle.SafetyMargin;
			}
		}

	default:
		return FVector::Dist(Point, Obstacle.Center) - Obstacle.Extents.GetMax() - Obstacle.SafetyMargin;
	}
}

// ========== 障碍物代价 (指数势垒) ==========
float UNMPCAvoidance::ComputeObstacleCost(const FVector& Position, const FObstacleInfo& Obstacle) const
{
	float Distance = CalculateDistanceToObstacle(Position, Obstacle);

	if (Distance > Config.ObstacleInfluenceDistance)
	{
		return 0.0f;
	}

	// 障碍物代价：exp(-alpha * (d - d_safe))，对数软截断保留梯度
	float Exponent = -Config.ObstacleAlpha * (Distance - Config.ObstacleSafeDistance);
	float RawCost = FMath::Exp(FMath::Clamp(Exponent, -20.0f, 20.0f));
	float SoftMax = Config.MaxObstacleCostPerStep;
	return SoftMax * FMath::Loge(1.0f + RawCost / SoftMax);
}

// ========== 动态障碍物预测 ==========
FObstacleInfo UNMPCAvoidance::PredictObstacle(const FObstacleInfo& Obstacle, float DeltaTime) const
{
	FObstacleInfo Predicted = Obstacle;
	if (Obstacle.bIsDynamic)
	{
		Predicted.Center = Obstacle.Center + Obstacle.Velocity * DeltaTime;
	}
	return Predicted;
}

// ========== 计算总代价 ==========
float UNMPCAvoidance::ComputeCost(
	const TArray<FVector>& Positions,
	const TArray<FVector>& Velocities,
	const TArray<FVector>& Controls,
	const TArray<FVector>& ReferencePoints,
	const TArray<FObstacleInfo>& Obstacles) const
{
	const int32 N = Controls.Num();
	const float dt = Config.GetDt();
	float Cost = 0.0f;

	for (int32 k = 0; k < N; ++k)
	{
		// 参考跟踪代价
		if (k < ReferencePoints.Num())
		{
			float RefDist = FVector::Dist(Positions[k], ReferencePoints[k]);
			Cost += Config.WeightReference * RefDist;
		}

		// 速度跟踪代价: 鼓励沿参考方向运动
		if (k + 1 < ReferencePoints.Num() && k < ReferencePoints.Num())
		{
			FVector DesiredVel = (ReferencePoints[k + 1] - ReferencePoints[k]) / FMath::Max(dt, KINDA_SMALL_NUMBER);
			float VelDist = FVector::Dist(Velocities[k], DesiredVel);
			Cost += Config.WeightVelocity * VelDist;
		}

		// 控制代价
		Cost += Config.WeightControl * Controls[k].SizeSquared();

		// 障碍物代价
		for (const FObstacleInfo& Obstacle : Obstacles)
		{
			FObstacleInfo PredObs = PredictObstacle(Obstacle, k * dt);
			Cost += Config.WeightObstacle * ComputeObstacleCost(Positions[k], PredObs);
		}
	}

	// 终端代价
	if (ReferencePoints.Num() > N)
	{
		float TermDist = FVector::Dist(Positions[N], ReferencePoints[N]);
		Cost += Config.WeightTerminal * TermDist;
	}

	return Cost;
}

// ========== 有限差分梯度 ==========
void UNMPCAvoidance::ComputeGradient(
	const FVector& InitPos,
	const FVector& InitVel,
	const TArray<FVector>& Controls,
	const TArray<FVector>& ReferencePoints,
	const TArray<FObstacleInfo>& Obstacles,
	TArray<FVector>& OutGradient) const
{
	const int32 N = Controls.Num();
	const float Eps = Config.FiniteDiffEpsilon;

	OutGradient.SetNum(N);

	// 基准代价
	TArray<FVector> BasePos, BaseVel;
	ForwardSimulate(InitPos, InitVel, Controls, BasePos, BaseVel);
	float BaseCost = ComputeCost(BasePos, BaseVel, Controls, ReferencePoints, Obstacles);

	// 对每个控制变量求偏导
	TArray<FVector> PerturbedControls = Controls;
	for (int32 k = 0; k < N; ++k)
	{
		FVector Grad = FVector::ZeroVector;

		// X 分量
		PerturbedControls[k].X = Controls[k].X + Eps;
		TArray<FVector> PPos, PVel;
		ForwardSimulate(InitPos, InitVel, PerturbedControls, PPos, PVel);
		Grad.X = (ComputeCost(PPos, PVel, PerturbedControls, ReferencePoints, Obstacles) - BaseCost) / Eps;
		PerturbedControls[k].X = Controls[k].X;

		// Y 分量
		PerturbedControls[k].Y = Controls[k].Y + Eps;
		ForwardSimulate(InitPos, InitVel, PerturbedControls, PPos, PVel);
		Grad.Y = (ComputeCost(PPos, PVel, PerturbedControls, ReferencePoints, Obstacles) - BaseCost) / Eps;
		PerturbedControls[k].Y = Controls[k].Y;

		// Z 分量
		PerturbedControls[k].Z = Controls[k].Z + Eps;
		ForwardSimulate(InitPos, InitVel, PerturbedControls, PPos, PVel);
		Grad.Z = (ComputeCost(PPos, PVel, PerturbedControls, ReferencePoints, Obstacles) - BaseCost) / Eps;
		PerturbedControls[k].Z = Controls[k].Z;

		OutGradient[k] = Grad;
	}
}

// ========== 投影到可行域 ==========
void UNMPCAvoidance::ProjectControls(TArray<FVector>& Controls, const FVector& InitVel) const
{
	const float dt = Config.GetDt();
	FVector Vel = InitVel;

	for (int32 k = 0; k < Controls.Num(); ++k)
	{
		// 加速度约束: ||u|| ≤ MaxAcceleration
		float AccMag = Controls[k].Size();
		if (AccMag > Config.MaxAcceleration)
		{
			Controls[k] = Controls[k] * (Config.MaxAcceleration / AccMag);
		}

		// 速度约束: 各轴独立限幅
		FVector NextVel = Vel + Controls[k] * dt;
		NextVel.X = FMath::Clamp(NextVel.X, -Config.MaxVelocity, Config.MaxVelocity);
		NextVel.Y = FMath::Clamp(NextVel.Y, -Config.MaxVelocity, Config.MaxVelocity);
		NextVel.Z = FMath::Clamp(NextVel.Z, -Config.MaxVelocity, Config.MaxVelocity);
		Controls[k] = (NextVel - Vel) / dt;

		Vel = Vel + Controls[k] * dt;
	}
}

// ========== 温启动 ==========
void UNMPCAvoidance::WarmStart()
{
	if (!bHasPreviousControls || PreviousControls.Num() < 2)
	{
		return;
	}

	// 左移一步, 末尾补零
	for (int32 i = 0; i < PreviousControls.Num() - 1; ++i)
	{
		PreviousControls[i] = PreviousControls[i + 1];
	}
	PreviousControls.Last() = FVector::ZeroVector;
}

// ========== 主求解入口 ==========
FNMPCAvoidanceResult UNMPCAvoidance::ComputeAvoidance(
	const FVector& CurrentPosition,
	const FVector& CurrentVelocity,
	const TArray<FVector>& ReferencePoints,
	const TArray<FObstacleInfo>& Obstacles)
{
	FNMPCAvoidanceResult Result;
	const int32 N = Config.PredictionSteps;
	const float dt = Config.GetDt();

	// 检查是否已到达目标
	if (ReferencePoints.Num() > 0)
	{
		float DistToGoal = FVector::Dist(CurrentPosition, ReferencePoints.Last());
		if (DistToGoal < Config.GoalReachedThreshold)
		{
			Result.CorrectedTarget = CurrentPosition;
			Result.CorrectedDirection = FVector::ZeroVector;
			Result.bNeedsCorrection = false;
			Result.bStuck = false;
			return Result;
		}
	}

	// 初始化控制序列
	TArray<FVector> Controls;
	Controls.SetNum(N);

	if (bHasPreviousControls && PreviousControls.Num() == N
		&& ConsecutiveCostRiseCount < Config.WarmStartResetThreshold)
	{
		// 温启动
		WarmStart();
		Controls = PreviousControls;
	}
	else
	{
		for (int32 k = 0; k < N; ++k)
		{
			Controls[k] = FVector::ZeroVector;
		}
		ConsecutiveCostRiseCount = 0;
	}

	// 确保参考点数量足够
	TArray<FVector> RefPoints = ReferencePoints;
	while (RefPoints.Num() < N + 1)
	{
		RefPoints.Add(RefPoints.Num() > 0 ? RefPoints.Last() : CurrentPosition);
	}

	// 投影梯度下降求解
	float CurrentCost = MAX_FLT;
	for (int32 Iter = 0; Iter < Config.MaxIterations; ++Iter)
	{
		// 投影到可行域
		ProjectControls(Controls, CurrentVelocity);

		// 前向仿真
		TArray<FVector> Positions, Velocities;
		ForwardSimulate(CurrentPosition, CurrentVelocity, Controls, Positions, Velocities);

		// 计算代价
		float Cost = ComputeCost(Positions, Velocities, Controls, RefPoints, Obstacles);

		// 收敛检查
		if (FMath::Abs(CurrentCost - Cost) < Config.ConvergenceTolerance && Iter > 0)
		{
			CurrentCost = Cost;
			break;
		}
		CurrentCost = Cost;

		// 计算梯度
		TArray<FVector> Gradient;
		ComputeGradient(CurrentPosition, CurrentVelocity, Controls, RefPoints, Obstacles, Gradient);

		// 回溯线搜索
		float StepSize = Config.InitialStepSize;
		for (int32 BT = 0; BT < Config.MaxBacktrackSteps; ++BT)
		{
			TArray<FVector> TrialControls;
			TrialControls.SetNum(N);
			for (int32 k = 0; k < N; ++k)
			{
				TrialControls[k] = Controls[k] - Gradient[k] * StepSize;
			}

			ProjectControls(TrialControls, CurrentVelocity);

			TArray<FVector> TrialPos, TrialVel;
			ForwardSimulate(CurrentPosition, CurrentVelocity, TrialControls, TrialPos, TrialVel);
			float TrialCost = ComputeCost(TrialPos, TrialVel, TrialControls, RefPoints, Obstacles);

			if (TrialCost < CurrentCost)
			{
				Controls = TrialControls;
				CurrentCost = TrialCost;
				break;
			}

			StepSize *= Config.BacktrackFactor;
		}
	}

	// 横向扰动: 若代价仍高且有近距障碍物，尝试垂直于障碍物方向的扰动
	if (Obstacles.Num() > 0)
	{
		// 找最近障碍物
		float MinDist = MAX_FLT;
		FVector ClosestObsDir = FVector::ZeroVector;
		for (const FObstacleInfo& Obs : Obstacles)
		{
			float D = CalculateDistanceToObstacle(CurrentPosition, Obs);
			if (D < MinDist)
			{
				MinDist = D;
				ClosestObsDir = (Obs.Center - CurrentPosition).GetSafeNormal();
			}
		}

		if (MinDist < Config.ObstacleInfluenceDistance)
		{
			// 生成 4 个垂直于障碍物方向的扰动候选
			FVector Up = FVector::UpVector;
			FVector Lateral1 = FVector::CrossProduct(ClosestObsDir, Up).GetSafeNormal();
			if (Lateral1.IsNearlyZero())
			{
				Lateral1 = FVector::CrossProduct(ClosestObsDir, FVector::RightVector).GetSafeNormal();
			}
			FVector Lateral2 = FVector::CrossProduct(ClosestObsDir, Lateral1).GetSafeNormal();

			FVector Perturbations[4] = { Lateral1, -Lateral1, Lateral2, -Lateral2 };

			for (const FVector& Perturb : Perturbations)
			{
				TArray<FVector> PertControls;
				PertControls.SetNum(N);
				for (int32 k = 0; k < N; ++k)
				{
					PertControls[k] = Controls[k] + Perturb * Config.LateralPerturbationMagnitude;
				}
				ProjectControls(PertControls, CurrentVelocity);

				TArray<FVector> PertPos, PertVel;
				ForwardSimulate(CurrentPosition, CurrentVelocity, PertControls, PertPos, PertVel);
				float PertCost = ComputeCost(PertPos, PertVel, PertControls, RefPoints, Obstacles);

				if (PertCost < CurrentCost)
				{
					Controls = PertControls;
					CurrentCost = PertCost;
				}
			}
		}
	}

	// 最终投影
	ProjectControls(Controls, CurrentVelocity);

	// 更新连续代价上升计数
	if (CurrentCost >= PreviousTotalCost)
	{
		++ConsecutiveCostRiseCount;
	}
	else
	{
		ConsecutiveCostRiseCount = 0;
	}
	PreviousTotalCost = CurrentCost;

	// 保存控制序列 (温启动)
	PreviousControls = Controls;
	bHasPreviousControls = true;

	// 最终前向仿真得到预测轨迹
	TArray<FVector> FinalPositions, FinalVelocities;
	ForwardSimulate(CurrentPosition, CurrentVelocity, Controls, FinalPositions, FinalVelocities);

	// 填充结果
	Result.TotalCost = CurrentCost;

	// 第一步控制输入 → 修正目标
	FVector FirstControl = Controls.Num() > 0 ? Controls[0] : FVector::ZeroVector;
	FVector NextVel = CurrentVelocity + FirstControl * dt;
	FVector NextPos = CurrentPosition + NextVel * dt;
	Result.CorrectedTarget = NextPos;

	// 修正方向
	FVector CorrDir = NextPos - CurrentPosition;
	const float CorrectionDistance = CorrDir.Size();
	Result.CorrectedDirection = CorrDir.IsNearlyZero() ? FVector::ForwardVector : CorrDir.GetSafeNormal();

	// 力分量映射 (可视化用)
	Result.TotalForce = FirstControl;
	if (RefPoints.Num() > 0)
	{
		FVector ToRef = RefPoints[0] - CurrentPosition;
		Result.AttractiveForce = ToRef.IsNearlyZero() ? FVector::ZeroVector : ToRef.GetSafeNormal() * FirstControl.Size();
		Result.RepulsiveForce = FirstControl - Result.AttractiveForce;
	}

	// 判断是否需要纠偏：障碍物代价超过死区且纠偏位移足够大
	float CurrentObstacleCost = 0.0f;
	for (const FObstacleInfo& Obs : Obstacles)
	{
		CurrentObstacleCost += ComputeObstacleCost(CurrentPosition, Obs);
	}
	const bool bHasObstacleCost = CurrentObstacleCost > Config.ObstacleCostDeadband;
	const bool bHasMeaningfulCorrection = CorrectionDistance >= Config.MinCorrectionDistance;
	Result.bNeedsCorrection = bHasObstacleCost && bHasMeaningfulCorrection;

	// 卡死判定：控制极小或代价持续恶化且前进不足
	const float FirstControlMagnitude = FirstControl.Size();
	const bool bLowControlStuck = bHasObstacleCost && FirstControlMagnitude < Config.StuckForceThreshold;
	const float SaturationRatio =
		Config.MaxAcceleration > KINDA_SMALL_NUMBER ? (FirstControlMagnitude / Config.MaxAcceleration) : 0.0f;

	const bool bControlSaturated =
		Config.MaxAcceleration > KINDA_SMALL_NUMBER &&
		FirstControlMagnitude >= Config.MaxAcceleration * Config.SaturationAccelRatio;
	const bool bCostRiseStuck =
		bHasObstacleCost &&
		ConsecutiveCostRiseCount >= Config.CostRiseStuckThreshold &&
		bControlSaturated &&
		CorrectionDistance < Config.MinProgressPerSolve;

	Result.bStuck = bLowControlStuck || bCostRiseStuck;
	const TCHAR* StuckReason = TEXT("None");
	if (bLowControlStuck)
	{
		StuckReason = TEXT("LowControlWithObstacle");
	}
	else if (bCostRiseStuck)
	{
		StuckReason = TEXT("CostRiseSaturationNoProgress");
	}

	// 填充预测轨迹 (可视化用)
	Result.PredictedTrajectory.SetNum(N + 1);
	for (int32 k = 0; k <= N; ++k)
	{
		FNMPCPredictionStep& Step = Result.PredictedTrajectory[k];
		Step.Position = FinalPositions[k];
		Step.Velocity = FinalVelocities[k];
		Step.ControlInput = (k < N) ? Controls[k] : FVector::ZeroVector;

		// 计算该步的障碍物代价
		Step.ObstacleCost = 0.0f;
		for (const FObstacleInfo& Obs : Obstacles)
		{
			FObstacleInfo PredObs = PredictObstacle(Obs, k * dt);
			Step.ObstacleCost += ComputeObstacleCost(FinalPositions[k], PredObs);
		}
	}

	UE_LOG(LogUAVPlanning, Log, TEXT("[NMPC] Cost=%.1f ObsCost=%.3f RiseCount=%d Progress=%.1f SatRatio=%.2f Control0=(%.1f,%.1f,%.1f) Saturated=%s NeedsCorr=%s Stuck=%s Reason=%s"),
		CurrentCost,
		CurrentObstacleCost,
		ConsecutiveCostRiseCount,
		CorrectionDistance,
		SaturationRatio,
		FirstControl.X, FirstControl.Y, FirstControl.Z,
		bControlSaturated ? TEXT("Y") : TEXT("N"),
		Result.bNeedsCorrection ? TEXT("Y") : TEXT("N"),
		Result.bStuck ? TEXT("Y") : TEXT("N"),
		StuckReason);

	return Result;
}
