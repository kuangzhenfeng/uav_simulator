// Copyright Epic Games, Inc. All Rights Reserved.

#include "NMPCAvoidance.h"
#include "uav_simulator/Debug/UAVLogConfig.h"
#include "uav_simulator/Utility/Filter.h"

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
float UNMPCAvoidance::CalculateCylinderDistance(const FVector& Point, const FObstacleInfo& Obstacle) const
{
	FVector LocalPoint = Obstacle.Rotation.UnrotateVector(Point - Obstacle.Center);
	float HorizontalDist = FVector2D(LocalPoint.X, LocalPoint.Y).Size();
	float VerticalDist = FMath::Abs(LocalPoint.Z);

	float HorizontalPen = HorizontalDist - Obstacle.Extents.X;
	float VerticalPen = VerticalDist - Obstacle.Extents.Z;

	// 两个方向都在内部
	if (HorizontalPen < 0 && VerticalPen < 0)
	{
		return FMath::Max(HorizontalPen, VerticalPen) - Obstacle.SafetyMargin;
	}

	// 仅水平方向在内部
	if (HorizontalPen < 0)
	{
		return VerticalPen - Obstacle.SafetyMargin;
	}

	// 仅垂直方向在内部
	if (VerticalPen < 0)
	{
		return HorizontalPen - Obstacle.SafetyMargin;
	}

	// 两个方向都在外部
	return FMath::Sqrt(HorizontalPen * HorizontalPen + VerticalPen * VerticalPen) - Obstacle.SafetyMargin;
}

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

			if (LocalPoint.Equals(ClosestPoint))
			{
				// 点在 Box 内部：返回负的穿透深度（到最近面的距离）
				float MinPen = FMath::Min3(
					Obstacle.Extents.X - FMath::Abs(LocalPoint.X),
					Obstacle.Extents.Y - FMath::Abs(LocalPoint.Y),
					Obstacle.Extents.Z - FMath::Abs(LocalPoint.Z));
				return -MinPen - Obstacle.SafetyMargin;
			}
			return FVector::Dist(LocalPoint, ClosestPoint) - Obstacle.SafetyMargin;
		}

	case EObstacleType::Cylinder:
		return CalculateCylinderDistance(Point, Obstacle);

	default:
		return FVector::Dist(Point, Obstacle.Center) - Obstacle.Extents.GetMax() - Obstacle.SafetyMargin;
	}
}

// ========== 有符号距离梯度 ==========
FVector UNMPCAvoidance::ComputeDistanceGradient(const FVector& Point, const FObstacleInfo& Obstacle) const
{
	switch (Obstacle.Type)
	{
	case EObstacleType::Sphere:
		{
			FVector Delta = Point - Obstacle.Center;
			float Dist = Delta.Size();
			if (Dist < KINDA_SMALL_NUMBER)
			{
				// 在球心处，梯度方向不确定，默认返回上方向
				return FVector::UpVector;
			}
			return Delta / Dist;
		}

	case EObstacleType::Box:
		{
			FVector LocalPoint = Obstacle.Rotation.UnrotateVector(Point - Obstacle.Center);

			// 找最近表面点
			FVector Clamped;
			Clamped.X = FMath::Clamp(LocalPoint.X, -Obstacle.Extents.X, Obstacle.Extents.X);
			Clamped.Y = FMath::Clamp(LocalPoint.Y, -Obstacle.Extents.Y, Obstacle.Extents.Y);
			Clamped.Z = FMath::Clamp(LocalPoint.Z, -Obstacle.Extents.Z, Obstacle.Extents.Z);

			if (LocalPoint.Equals(Clamped, KINDA_SMALL_NUMBER))
			{
				// 点在 Box 内部：沿最小穿透轴方向推出
				float PenX = Obstacle.Extents.X - FMath::Abs(LocalPoint.X);
				float PenY = Obstacle.Extents.Y - FMath::Abs(LocalPoint.Y);
				float PenZ = Obstacle.Extents.Z - FMath::Abs(LocalPoint.Z);

				FVector LocalGrad;
				if (PenX <= PenY && PenX <= PenZ)
					LocalGrad = FVector(LocalPoint.X > 0 ? 1.0f : -1.0f, 0.0f, 0.0f);
				else if (PenY <= PenX && PenY <= PenZ)
					LocalGrad = FVector(0.0f, LocalPoint.Y > 0 ? 1.0f : -1.0f, 0.0f);
				else
					LocalGrad = FVector(0.0f, 0.0f, LocalPoint.Z > 0 ? 1.0f : -1.0f);

				return Obstacle.Rotation.RotateVector(LocalGrad).GetSafeNormal();
			}

			// 点在 Box 外部：梯度 = (Point - ClosestPoint).GetSafeNormal()
			FVector LocalGrad = (LocalPoint - Clamped).GetSafeNormal();
			return Obstacle.Rotation.RotateVector(LocalGrad);
		}

	case EObstacleType::Cylinder:
		{
			FVector LocalPoint = Obstacle.Rotation.UnrotateVector(Point - Obstacle.Center);
			float HorizontalDist = FVector2D(LocalPoint.X, LocalPoint.Y).Size();
			float VerticalDist = FMath::Abs(LocalPoint.Z);

			float HorizontalPen = HorizontalDist - Obstacle.Extents.X;
			float VerticalPen = VerticalDist - Obstacle.Extents.Z;

			FVector LocalGrad;

			// 两个方向都在内部
			if (HorizontalPen < 0 && VerticalPen < 0)
			{
				// 沿最小穿透方向退出
				if (FMath::Abs(HorizontalPen) <= FMath::Abs(VerticalPen))
				{
					// 水平方向穿透更浅，沿水平推出
					if (HorizontalDist > KINDA_SMALL_NUMBER)
						LocalGrad = FVector(LocalPoint.X, LocalPoint.Y, 0.0f).GetSafeNormal();
					else
						LocalGrad = FVector(1.0f, 0.0f, 0.0f);
				}
				else
				{
					// 垂直方向穿透更浅，沿垂直推出
					LocalGrad = FVector(0.0f, 0.0f, LocalPoint.Z > 0 ? 1.0f : -1.0f);
				}
			}
			// 仅水平方向在内部
			else if (HorizontalPen < 0)
			{
				LocalGrad = FVector(0.0f, 0.0f, LocalPoint.Z > 0 ? 1.0f : -1.0f);
			}
			// 仅垂直方向在内部
			else if (VerticalPen < 0)
			{
				if (HorizontalDist > KINDA_SMALL_NUMBER)
					LocalGrad = FVector(LocalPoint.X, LocalPoint.Y, 0.0f).GetSafeNormal();
				else
					LocalGrad = FVector(1.0f, 0.0f, 0.0f);
			}
			// 两个方向都在外部
			else
			{
				FVector2D HDir(LocalPoint.X, LocalPoint.Y);
				if (HDir.Size() > KINDA_SMALL_NUMBER)
					HDir.Normalize();
				else
					HDir = FVector2D(1.0f, 0.0f);

				float VSign = LocalPoint.Z > 0 ? 1.0f : -1.0f;

				// 梯度 = (HorizontalPen, VerticalPen) 方向的归一化
				FVector Grad3D(HDir.X * HorizontalPen, HDir.Y * HorizontalPen, VSign * VerticalPen);
				LocalGrad = Grad3D.GetSafeNormal();
			}

			return Obstacle.Rotation.RotateVector(LocalGrad);
		}

	default:
		{
			FVector Delta = Point - Obstacle.Center;
			float Dist = Delta.Size();
			if (Dist < KINDA_SMALL_NUMBER)
				return FVector::UpVector;
			return Delta / Dist;
		}
	}
}

// ========== 障碍物代价 ==========
float UNMPCAvoidance::ComputeObstacleCost(const FVector& Position, const FObstacleInfo& Obstacle) const
{
	float Distance = CalculateDistanceToObstacle(Position, Obstacle);

	if (Distance > Config.Obstacle.ObstacleInfluenceDistance)
	{
		return 0.0f;
	}

	if (Config.Obstacle.bUseSmoothHinge)
	{
		// Smooth hinge: (1/β)·log(1+exp(β·(d_safe - d)))
		float Beta = Config.Obstacle.SmoothHingeBeta;
		float Arg = Beta * (Config.Obstacle.ObstacleSafeDistance - Distance);
		// 防止数值溢出
		Arg = FMath::Clamp(Arg, -50.0f, 50.0f);
		float RawCost = (1.0f / Beta) * FMath::Loge(1.0f + FMath::Exp(Arg));

		// smoothstep 衰减到影响距离边界
		if (Distance > Config.Obstacle.ObstacleSafeDistance)
		{
			float T = (Distance - Config.Obstacle.ObstacleSafeDistance) /
				(Config.Obstacle.ObstacleInfluenceDistance - Config.Obstacle.ObstacleSafeDistance);
			T = FMath::Clamp(T, 0.0f, 1.0f);
			float Decay = 1.0f - T * T * (3.0f - 2.0f * T);
			RawCost *= Decay;
		}

		return FMath::Min(RawCost, Config.Obstacle.MaxObstacleCostPerStep);
	}
	else
	{
		// 指数势垒 (旧实现)
		float Exponent = -Config.Obstacle.ObstacleAlpha * (Distance - Config.Obstacle.ObstacleSafeDistance);
		const float ClampMax = 20.0f;
		float SoftMax = Config.Obstacle.MaxObstacleCostPerStep;

		if (Exponent <= ClampMax)
		{
			float RawCost = FMath::Exp(FMath::Max(Exponent, -20.0f));
			return SoftMax * FMath::Loge(1.0f + RawCost / SoftMax);
		}

		float BaseCost = SoftMax * FMath::Loge(1.0f + FMath::Exp(ClampMax) / SoftMax);
		float LinearSlope = Config.Obstacle.ObstacleAlpha * SoftMax;
		return BaseCost + LinearSlope * (Exponent - ClampMax);
	}
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

// ========== Frenet 坐标分解辅助方法 ==========
void UNMPCAvoidance::ComputeFrenetCoordinates(
	const FVector& Position,
	const FVector& RefPoint,
	const FVector& NextRefPoint,
	FVector& OutTangent,
	float& OutParallel,
	FVector& OutPerpendicular) const
{
	FVector Tangent = (NextRefPoint - RefPoint);
	float SegLen = Tangent.Size();
	if (SegLen < KINDA_SMALL_NUMBER)
	{
		// 参考点重合时退化为 L2 距离，避免零误差导致求解器丢失跟踪约束
		OutTangent = FVector::ForwardVector;
		FVector Error = Position - RefPoint;
		OutParallel = 0.0f;
		OutPerpendicular = Error;
		return;
	}
	OutTangent = Tangent / SegLen;
	FVector Error = Position - RefPoint;
	OutParallel = FVector::DotProduct(Error, OutTangent);
	OutPerpendicular = Error - OutTangent * OutParallel;
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

	// 使用 Frenet 分解 + 平方误差的代价
	float MinObsDist = MAX_FLT;
	float PathProgress = 0.0f;

	for (int32 k = 0; k < N; ++k)
	{
		// Frenet 坐标分解
		if (k < ReferencePoints.Num() && k + 1 < ReferencePoints.Num())
		{
			FVector Tangent;
			float EParallel;
			FVector EPerp;
			ComputeFrenetCoordinates(Positions[k], ReferencePoints[k], ReferencePoints[k + 1],
				Tangent, EParallel, EPerp);

			// 横向跟踪代价 (平方)
			Cost += Config.Cost.WeightLateral * EPerp.SizeSquared();

			// 纵向滞后代价 (平方)
			Cost += Config.Cost.WeightLag * EParallel * EParallel;
		}
		else if (k < ReferencePoints.Num())
		{
			// 兜底：L2 平方距离
			Cost += Config.Cost.WeightReference * FVector::DistSquared(Positions[k], ReferencePoints[k]);
		}

		// 速度跟踪代价 (平方)
		if (k + 1 < ReferencePoints.Num() && k < ReferencePoints.Num())
		{
			FVector DesiredVel = (ReferencePoints[k + 1] - ReferencePoints[k]) / FMath::Max(dt, KINDA_SMALL_NUMBER);
			Cost += Config.Cost.WeightVelocity * FVector::DistSquared(Velocities[k], DesiredVel);
		}

		// 控制代价
		Cost += Config.Cost.WeightControl * Controls[k].SizeSquared();

		// 时序一致性代价
		if (StuckEscapeCount <= 0 && bHasPreviousControls && k < PreviousControls.Num())
		{
			Cost += Config.Cost.WeightTemporalConsistency * (Controls[k] - PreviousControls[k]).SizeSquared();
		}

		// 路径进度
		if (k + 1 < Positions.Num())
		{
			FVector Step = Positions[k + 1] - Positions[k];
			if (k < ReferencePoints.Num() && k + 1 < ReferencePoints.Num())
			{
				FVector Tangent = (ReferencePoints[k + 1] - ReferencePoints[k]).GetSafeNormal();
				float AlongTrack = FVector::DotProduct(Step, Tangent);
				PathProgress += FMath::Max(0.0f, AlongTrack);

				// 反向运动惩罚
				if (AlongTrack < 0.0f)
				{
					Cost += Config.Cost.WeightReverse * AlongTrack * AlongTrack;
				}
			}
		}

		// 障碍物代价
		const float EscapeObsScale = (StuckEscapeCount > 0) ? 0.15f : 1.0f;
		float TimeDiscount = FMath::Pow(0.85f, (float)k);
		for (const FObstacleInfo& Obstacle : Obstacles)
		{
			FObstacleInfo PredObs = PredictObstacle(Obstacle, k * dt);
			float Dist = CalculateDistanceToObstacle(Positions[k], PredObs);
			MinObsDist = FMath::Min(MinObsDist, Dist);
			float ObsCost = ComputeObstacleCost(Positions[k], PredObs);
			Cost += Config.Cost.WeightObstacle * EscapeObsScale * TimeDiscount * ObsCost * ObsCost;
		}
	}

	// 终端代价 (平方)
	if (ReferencePoints.Num() > N)
	{
		Cost += Config.Cost.WeightTerminal * FVector::DistSquared(Positions[N], ReferencePoints[N]);
	}

	// 终端障碍物代价
	const float TermEscapeObsScale = (StuckEscapeCount > 0) ? 0.15f : 1.0f;
	float TermDiscount = FMath::Pow(0.85f, (float)N);
	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		FObstacleInfo PredObs = PredictObstacle(Obstacle, N * dt);
		float Dist = CalculateDistanceToObstacle(Positions[N], PredObs);
		MinObsDist = FMath::Min(MinObsDist, Dist);
		float ObsCost = ComputeObstacleCost(Positions[N], PredObs);
		Cost += Config.Cost.WeightObstacle * TermEscapeObsScale * TermDiscount * ObsCost * ObsCost;
	}

	// 路径进度惩罚 (soft-one-sided)
	float TargetProgress = 0.0f;
	for (int32 k = 0; k < N && k + 1 < ReferencePoints.Num(); ++k)
	{
		TargetProgress += FVector::Dist(ReferencePoints[k], ReferencePoints[k + 1]);
	}
	float ProgressDeficit = FMath::Max(0.0f, TargetProgress - PathProgress);
	Cost += Config.Cost.WeightProgress * ProgressDeficit * ProgressDeficit;

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
	const float Eps = Config.Solver.FiniteDiffEpsilon;

	OutGradient.SetNum(N);

	// 中心差分: (f(x+eps) - f(x-eps)) / (2*eps)，精度 O(eps²)
	const float TwoEps = 2.0f * Eps;
	TArray<FVector> PerturbedControls = Controls;
	for (int32 k = 0; k < N; ++k)
	{
		FVector Grad = FVector::ZeroVector;
		TArray<FVector> PPos, PVel, MPos, MVel;

		// X 分量
		PerturbedControls[k].X = Controls[k].X + Eps;
		ForwardSimulate(InitPos, InitVel, PerturbedControls, PPos, PVel);
		float CostPlus = ComputeCost(PPos, PVel, PerturbedControls, ReferencePoints, Obstacles);
		PerturbedControls[k].X = Controls[k].X - Eps;
		ForwardSimulate(InitPos, InitVel, PerturbedControls, MPos, MVel);
		Grad.X = (CostPlus - ComputeCost(MPos, MVel, PerturbedControls, ReferencePoints, Obstacles)) / TwoEps;
		PerturbedControls[k].X = Controls[k].X;

		// Y 分量
		PerturbedControls[k].Y = Controls[k].Y + Eps;
		ForwardSimulate(InitPos, InitVel, PerturbedControls, PPos, PVel);
		CostPlus = ComputeCost(PPos, PVel, PerturbedControls, ReferencePoints, Obstacles);
		PerturbedControls[k].Y = Controls[k].Y - Eps;
		ForwardSimulate(InitPos, InitVel, PerturbedControls, MPos, MVel);
		Grad.Y = (CostPlus - ComputeCost(MPos, MVel, PerturbedControls, ReferencePoints, Obstacles)) / TwoEps;
		PerturbedControls[k].Y = Controls[k].Y;

		// Z 分量
		PerturbedControls[k].Z = Controls[k].Z + Eps;
		ForwardSimulate(InitPos, InitVel, PerturbedControls, PPos, PVel);
		CostPlus = ComputeCost(PPos, PVel, PerturbedControls, ReferencePoints, Obstacles);
		PerturbedControls[k].Z = Controls[k].Z - Eps;
		ForwardSimulate(InitPos, InitVel, PerturbedControls, MPos, MVel);
		Grad.Z = (CostPlus - ComputeCost(MPos, MVel, PerturbedControls, ReferencePoints, Obstacles)) / TwoEps;
		PerturbedControls[k].Z = Controls[k].Z;

		OutGradient[k] = Grad;
	}
}

// ========== Dykstra 单步投影 ==========
void UNMPCAvoidance::DykstraProjectStep(FVector& Control, const FVector& CurrentVel, float dt) const
{
	// 两个约束集:
	// C1 = { u : ||u|| <= MaxAcceleration }
	// C2 = { u : ||v + dt*u|| <= MaxVelocity }
	// 交替投影 + Dykstra 校正增量

	FVector U = Control;
	FVector P1 = FVector::ZeroVector; // C1 的校正增量
	FVector P2 = FVector::ZeroVector; // C2 的校正增量

	for (int32 Iter = 0; Iter < Config.Actuator.MaxProjectionIterations; ++Iter)
	{
		FVector U_Prev = U;

		// 投影到 C1 (加速度球)
		FVector Y1 = U + P1;
		float Y1Mag = Y1.Size();
		if (Y1Mag > Config.Actuator.MaxAcceleration)
		{
			Y1 = Y1 * (Config.Actuator.MaxAcceleration / Y1Mag);
		}
		P1 = U + P1 - Y1;
		U = Y1;

		// 投影到 C2 (速度球)
		FVector Y2 = U + P2;
		FVector NextVel = CurrentVel + Y2 * dt;
		float VelMag = NextVel.Size();
		if (VelMag > Config.Actuator.MaxVelocity)
		{
			// 优先生成制动控制
			if (CurrentVel.Size() > Config.Actuator.MaxVelocity)
			{
				// 已超速：最大制动
				FVector BrakeDir = -CurrentVel.GetSafeNormal();
				float BrakeAccel = FMath::Min(Config.Actuator.MaxAcceleration,
					(CurrentVel.Size() - Config.Actuator.MaxVelocity) / dt);
				NextVel = CurrentVel + BrakeDir * BrakeAccel * dt;
				Y2 = (NextVel - CurrentVel) / dt;
			}
			else
			{
				NextVel = NextVel * (Config.Actuator.MaxVelocity / VelMag);
				Y2 = (NextVel - CurrentVel) / dt;
			}
		}
		P2 = U + P2 - Y2;
		U = Y2;

		// 收敛检查
		if ((U - U_Prev).SizeSquared() < Config.Actuator.ProjectionTolerance * Config.Actuator.ProjectionTolerance)
		{
			break;
		}
	}

	Control = U;
}

// ========== 投影到可行域 ==========
void UNMPCAvoidance::ProjectControls(TArray<FVector>& Controls, const FVector& InitVel) const
{
	const float dt = Config.GetDt();
	FVector Vel = InitVel;

	for (int32 k = 0; k < Controls.Num(); ++k)
	{
		if (Config.Actuator.bUseDykstraProjection)
		{
			DykstraProjectStep(Controls[k], Vel, dt);
		}
		else
		{
			// 旧顺序投影（保留用于 A/B 对比）
			float AccMag = Controls[k].Size();
			if (AccMag > Config.Actuator.MaxAcceleration)
			{
				Controls[k] = Controls[k] * (Config.Actuator.MaxAcceleration / AccMag);
			}

			FVector NextVel = Vel + Controls[k] * dt;
			float VelMag = NextVel.Size();
			if (VelMag > Config.Actuator.MaxVelocity)
			{
				NextVel = NextVel * (Config.Actuator.MaxVelocity / VelMag);
				Controls[k] = (NextVel - Vel) / dt;
			}
		}

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

// ========== WarmStart 初始化辅助方法 ==========

bool UNMPCAvoidance::ShouldUseWarmStart() const
{
	if (!bHasPreviousControls || PreviousControls.Num() != Config.Solver.PredictionSteps)
	{
		return false;
	}

	// 若上一次的首步控制量极小，跳过温启动，避免零控制循环
	float FirstControlMag = PreviousControls[0].Size();
	if (FirstControlMag < Config.Init.StuckForceThreshold * 2.0f)
	{
		return false;
	}

	return true;
}

FVector UNMPCAvoidance::ComputeInitialControl(const TArray<FVector>& ReferencePoints) const
{
	if (ReferencePoints.Num() < 2)
	{
		return FVector::ZeroVector;
	}

	FVector RefDir = (ReferencePoints[1] - ReferencePoints[0]).GetSafeNormal();
	// 参考点退化时（相邻点重合），用终点方向兜底
	if (RefDir.IsNearlyZero())
	{
		RefDir = (ReferencePoints.Last() - ReferencePoints[0]).GetSafeNormal();
	}

	if (!RefDir.IsNearlyZero())
	{
		return RefDir * Config.Actuator.MaxAcceleration * 0.85f;
	}

	return FVector::ZeroVector;
}

void UNMPCAvoidance::InitializeControls(
	TArray<FVector>& OutControls,
	const TArray<FVector>& ReferencePoints)
{
	const int32 N = Config.Solver.PredictionSteps;
	OutControls.SetNum(N);

	if (ShouldUseWarmStart())
	{
		WarmStart();
		OutControls = PreviousControls;
	}
	else
	{
		FVector InitControl = ComputeInitialControl(ReferencePoints);
		for (int32 k = 0; k < N; ++k)
		{
			OutControls[k] = InitControl;
		}
	}
}

// ========== 卡死检测辅助方法 ==========

void UNMPCAvoidance::ResetStuckDetection(const FVector& CurrentPosition)
{
	SlowProgressCount = 0;
	StuckCheckPosition = CurrentPosition;
}

bool UNMPCAvoidance::DetectPositionStuck(const FVector& CurrentPosition, bool bNeedsCorrection)
{
	const float DistFromCheck = FVector::Dist(CurrentPosition, StuckCheckPosition);
	const bool bHasObstacles = bNeedsCorrection;

	// 降低位移阈值到10cm,延长时间窗口到50帧(~5s),避免误判正常减速避障
	if (DistFromCheck < 10.0f && bHasObstacles)
	{
		++SlowProgressCount;
	}
	else
	{
		ResetStuckDetection(CurrentPosition);
	}

	const bool bStuck = SlowProgressCount > 50; // ~5s at 0.1s interval
	if (bStuck)
	{
		ResetStuckDetection(CurrentPosition);
	}

	return bStuck;
}

void UNMPCAvoidance::ResetOscillationDetection(const FVector& CurrentPosition)
{
	bOscillationInitialized = false;
	OscillationSolveCount = 0;
	OscillationPathLength = 0.0f;
	OscillationAnchor = CurrentPosition;
	OscillationPrevPos = CurrentPosition;
}

void UNMPCAvoidance::UpdateOscillationMetrics(const FVector& CurrentPosition)
{
	OscillationPathLength += FVector::Dist(CurrentPosition, OscillationPrevPos);
	OscillationPrevPos = CurrentPosition;
	++OscillationSolveCount;
}

bool UNMPCAvoidance::DetectOscillationStuck(const FVector& CurrentPosition, bool bNeedsCorrection)
{
	if (!bNeedsCorrection)
	{
		ResetOscillationDetection(CurrentPosition);
		return false;
	}

	if (!bOscillationInitialized)
	{
		OscillationAnchor = CurrentPosition;
		OscillationPrevPos = CurrentPosition;
		OscillationPathLength = 0.0f;
		OscillationSolveCount = 0;
		bOscillationInitialized = true;
		return false;
	}

	UpdateOscillationMetrics(CurrentPosition);

	// 每 50 次求解（~5s）评估一次,延长窗口减少误判
	if (OscillationSolveCount >= 50)
	{
		const float NetDisplacement = FVector::Dist(CurrentPosition, OscillationAnchor);
		// 净位移 < 累计路程的 8%，且累计路程足够大（排除静止），判定为振荡
		const bool bStuck = OscillationPathLength > 500.0f && NetDisplacement < OscillationPathLength * 0.08f;

		// 重置窗口
		OscillationAnchor = CurrentPosition;
		OscillationPathLength = 0.0f;
		OscillationSolveCount = 0;

		return bStuck;
	}

	return false;
}

// ========== 控制 Knot 插值 ==========
void UNMPCAvoidance::InterpolateFromKnots(
	TArray<FVector>& OutControls,
	const TArray<FVector>& Knots,
	const TArray<int32>& KnotIndices) const
{
	const int32 N = Config.Solver.PredictionSteps;
	OutControls.SetNum(N);

	for (int32 k = 0; k < N; ++k)
	{
		// 找到 k 所在的 knot 区间
		int32 SegStart = 0;
		int32 SegEnd = Knots.Num() - 1;
		for (int32 s = 0; s < KnotIndices.Num() - 1; ++s)
		{
			if (k >= KnotIndices[s] && k < KnotIndices[s + 1])
			{
				SegStart = s;
				SegEnd = s + 1;
				break;
			}
		}

		int32 IdxStart = KnotIndices[SegStart];
		int32 IdxEnd = KnotIndices[SegEnd];
		float Alpha = (IdxEnd > IdxStart) ?
			static_cast<float>(k - IdxStart) / static_cast<float>(IdxEnd - IdxStart) : 0.0f;
		Alpha = FMath::Clamp(Alpha, 0.0f, 1.0f);

		OutControls[k] = Knots[SegStart] * (1.0f - Alpha) + Knots[SegEnd] * Alpha;
	}
}

// ========== 多初值候选生成 ==========
TArray<FInitCandidate> UNMPCAvoidance::GenerateCandidates(
	const TArray<FVector>& ReferencePoints,
	const FVector& CurrentVelocity) const
{
	const int32 N = Config.Solver.PredictionSteps;
	TArray<FInitCandidate> Candidates;
	Candidates.Reserve(7);

		// 参考方向：扫描非零参考段，回退到当前速度方向
		FVector RefDir = FVector::ZeroVector;
		for (int32 i = 0; i + 1 < ReferencePoints.Num(); ++i)
		{
			FVector Seg = ReferencePoints[i + 1] - ReferencePoints[i];
			if (Seg.SizeSquared() > 1.0f) // 1cm 阈值
			{
				RefDir = Seg.GetSafeNormal();
				break;
			}
		}
		// 回退：使用当前速度方向或默认前向
		if (RefDir.IsNearlyZero())
		{
			RefDir = CurrentVelocity.SizeSquared() > 100.0f ?
				CurrentVelocity.GetSafeNormal() : FVector::ForwardVector;
		}
		FVector Up = FVector::UpVector;
		FVector Left = FVector::CrossProduct(Up, RefDir).GetSafeNormal();
		if (Left.IsNearlyZero()) Left = FVector::CrossProduct(FVector::RightVector, RefDir).GetSafeNormal();
		FVector LateralUp = FVector::CrossProduct(RefDir, Left).GetSafeNormal();

	float NominalAccel = Config.Actuator.MaxAcceleration * 0.6f;
	float LateralAccel = Config.Actuator.MaxAcceleration * 0.4f;

	auto MakeCandidate = [&](EInitCandidateType Type, const TArray<FVector>& Knots,
		const TArray<int32>& Indices) -> FInitCandidate {
		FInitCandidate C;
		C.Type = Type;
		InterpolateFromKnots(C.Controls, Knots, Indices);
		return C;
	};

	// 3-knot 布局: [0] = start, [N/3] = mid, [2N/3] = end
	int32 K1 = FMath::Max(1, N / 3);
	int32 K2 = FMath::Min(N - 1, 2 * N / 3);
	TArray<int32> Indices = {0, K1, K2, N - 1};

	// Warm: 如果有上一帧控制
	if (bHasPreviousControls && PreviousControls.Num() == N)
	{
		FInitCandidate Warm;
		Warm.Type = EInitCandidateType::Warm;
		Warm.Controls = PreviousControls;
		// 左移一步，末尾补零
		for (int32 k = 0; k < N - 1; ++k)
		{
			Warm.Controls[k] = PreviousControls[k + 1];
		}
		Warm.Controls.Last() = FVector::ZeroVector;
		Candidates.Add(Warm);
	}

	// Nominal: 沿参考方向加速
	{
		TArray<FVector> Knots = {
			RefDir * NominalAccel,
			RefDir * NominalAccel,
			RefDir * NominalAccel * 0.5f,
			FVector::ZeroVector
		};
		Candidates.Add(MakeCandidate(EInitCandidateType::Nominal, Knots, Indices));
	}

	// Left / Right: 水平横向绕行
	{
		TArray<FVector> LeftKnots = {
			RefDir * NominalAccel + Left * LateralAccel,
			RefDir * NominalAccel + Left * LateralAccel,
			RefDir * NominalAccel * 0.5f,
			FVector::ZeroVector
		};
		Candidates.Add(MakeCandidate(EInitCandidateType::Left, LeftKnots, Indices));

		TArray<FVector> RightKnots = {
			RefDir * NominalAccel - Left * LateralAccel,
			RefDir * NominalAccel - Left * LateralAccel,
			RefDir * NominalAccel * 0.5f,
			FVector::ZeroVector
		};
		Candidates.Add(MakeCandidate(EInitCandidateType::Right, RightKnots, Indices));
	}

	// Up / Down: 垂直绕行
	{
		TArray<FVector> UpKnots = {
			RefDir * NominalAccel + LateralUp * LateralAccel,
			RefDir * NominalAccel + LateralUp * LateralAccel,
			RefDir * NominalAccel * 0.5f,
			FVector::ZeroVector
		};
		Candidates.Add(MakeCandidate(EInitCandidateType::Up, UpKnots, Indices));

		TArray<FVector> DownKnots = {
			RefDir * NominalAccel - LateralUp * LateralAccel,
			RefDir * NominalAccel - LateralUp * LateralAccel,
			RefDir * NominalAccel * 0.5f,
			FVector::ZeroVector
		};
		Candidates.Add(MakeCandidate(EInitCandidateType::Down, DownKnots, Indices));
	}

	// Brake: 减速
	{
		FVector BrakeDir = CurrentVelocity.Size() > 10.0f ?
			-CurrentVelocity.GetSafeNormal() : -RefDir;
		TArray<FVector> BrakeKnots = {
			BrakeDir * NominalAccel,
			FVector::ZeroVector,
			FVector::ZeroVector,
			FVector::ZeroVector
		};
		Candidates.Add(MakeCandidate(EInitCandidateType::Brake, BrakeKnots, Indices));
	}

	return Candidates;
}

// ========== 主求解入口 ==========
FNMPCAvoidanceResult UNMPCAvoidance::ComputeAvoidance(
	const FVector& CurrentPosition,
	const FVector& CurrentVelocity,
	const TArray<FVector>& ReferencePoints,
	const TArray<FObstacleInfo>& Obstacles)
{
	const double SolveStartTime = FPlatformTime::Seconds();
	FNMPCAvoidanceResult Result;
	float InitialCost = MAX_FLT;
	const int32 N = Config.Solver.PredictionSteps;
	const float dt = Config.GetDt();

	// 确保参考点数量足够
	TArray<FVector> RefPoints = ReferencePoints;
	while (RefPoints.Num() < N + 1)
	{
		const FVector FillPoint = RefPoints.Num() > 0 ? RefPoints.Last() : CurrentPosition;
		RefPoints.Add(FillPoint);
	}

	// ---- 多初值候选生成 + 评估 ----
	TArray<FInitCandidate> Candidates = GenerateCandidates(RefPoints, CurrentVelocity);

	// 逃逸模式下仍用旧初始化（候选方案可能在卡死位置都差不多）
	if (StuckEscapeCount <= 0)
	{
		// 对每个候选做 rollout + 投影 + 代价评估
		for (FInitCandidate& Cand : Candidates)
		{
			ProjectControls(Cand.Controls, CurrentVelocity);
			TArray<FVector> CandPos, CandVel;
			ForwardSimulate(CurrentPosition, CurrentVelocity, Cand.Controls, CandPos, CandVel);
			Cand.Cost = ComputeCost(CandPos, CandVel, Cand.Controls, RefPoints, Obstacles);

			// 计算候选最小净空
			Cand.MinClearance = MAX_FLT;
			for (int32 k = 0; k <= N; ++k)
			{
				for (const FObstacleInfo& Obs : Obstacles)
				{
					FObstacleInfo PredObs = PredictObstacle(Obs, k * dt);
					float D = CalculateDistanceToObstacle(CandPos[k], PredObs);
					Cand.MinClearance = FMath::Min(Cand.MinClearance, D);
				}
			}

			// 路径进度
			if (CandPos.Num() > 0)
			{
				Cand.PathProgress = FVector::Dist(CandPos.Last(), CurrentPosition);
			}
		}

		// 按代价排序，选出 Top-3 做短 PGD
		Candidates.Sort([](const FInitCandidate& A, const FInitCandidate& B)
		{
			// 净空为负（碰撞）的排到最后
			bool bAValid = A.MinClearance > 0.0f;
			bool bBValid = B.MinClearance > 0.0f;
			if (bAValid != bBValid) return bAValid;
			return A.Cost < B.Cost;
		});

		const int32 ShortPGDIter = 3;
		const int32 TopK = FMath::Min(3, Candidates.Num());
		for (int32 ti = 0; ti < TopK; ++ti)
		{
			FInitCandidate& Cand = Candidates[ti];
			TArray<FVector>& CControls = Cand.Controls;

			for (int32 SI = 0; SI < ShortPGDIter; ++SI)
			{
				ProjectControls(CControls, CurrentVelocity);
				TArray<FVector> SPgdPos, SPgdVel;
				ForwardSimulate(CurrentPosition, CurrentVelocity, CControls, SPgdPos, SPgdVel);
				float SCost = ComputeCost(SPgdPos, SPgdVel, CControls, RefPoints, Obstacles);

				TArray<FVector> SGrad;
				ComputeGradient(CurrentPosition, CurrentVelocity, CControls, RefPoints, Obstacles, SGrad);

				// 单步回溯线搜索
				float SStep = Config.Solver.InitialStepSize;
				for (int32 BT = 0; BT < 4; ++BT)
				{
					TArray<FVector> TrialCtrl;
					TrialCtrl.SetNum(N);
					for (int32 k = 0; k < N; ++k)
					{
						TrialCtrl[k] = CControls[k] - SGrad[k] * SStep;
						}
					ProjectControls(TrialCtrl, CurrentVelocity);

					TArray<FVector> TPos, TVel;
					ForwardSimulate(CurrentPosition, CurrentVelocity, TrialCtrl, TPos, TVel);
					float TCost = ComputeCost(TPos, TVel, TrialCtrl, RefPoints, Obstacles);

					if (TCost < SCost)
					{
						CControls = TrialCtrl;
						Cand.Cost = TCost;
						break;
					}
					SStep *= Config.Solver.BacktrackFactor;
				}
			}
		}

			// 短 PGD 后重新 rollout 计算净空（PGD 可能将轨迹推向障碍物）
			for (int32 ti = 0; ti < TopK; ++ti)
			{
				FInitCandidate& Cand = Candidates[ti];
				ProjectControls(Cand.Controls, CurrentVelocity);
				TArray<FVector> RecalcPos, RecalcVel;
				ForwardSimulate(CurrentPosition, CurrentVelocity, Cand.Controls, RecalcPos, RecalcVel);

				Cand.MinClearance = MAX_FLT;
				for (int32 k = 0; k <= N && k < RecalcPos.Num(); ++k)
				{
					for (const FObstacleInfo& Obs : Obstacles)
					{
						FObstacleInfo PredObs = PredictObstacle(Obs, k * dt);
						float D = CalculateDistanceToObstacle(RecalcPos[k], PredObs);
						Cand.MinClearance = FMath::Min(Cand.MinClearance, D);
					}
				}
			}

			// Homotopy 软偏好：匹配当前绕行侧的候选获得 0.95 代价折扣
			if (CurrentHomotopy != EAvoidanceHomotopy::None)
			{
				for (FInitCandidate& Cand : Candidates)
				{
					bool bMatchesHomotopy = false;
					if (CurrentHomotopy == EAvoidanceHomotopy::Left && Cand.Type == EInitCandidateType::Left)
						bMatchesHomotopy = true;
					else if (CurrentHomotopy == EAvoidanceHomotopy::Right && Cand.Type == EInitCandidateType::Right)
						bMatchesHomotopy = true;
					if (bMatchesHomotopy)
						Cand.Cost *= 0.95f;
				}
			}

			// 重新排序：净空为负（碰撞）的排到最后，其余按代价
			Candidates.Sort([](const FInitCandidate& A, const FInitCandidate& B)
			{
				bool bAValid = A.MinClearance > 0.0f;
				bool bBValid = B.MinClearance > 0.0f;
				if (bAValid != bBValid) return bAValid;
				return A.Cost < B.Cost;
			});
		}

	// 最终控制序列：多初值最优 or 旧初始化（逃逸模式）
	TArray<FVector> Controls;
	FString SelectedInitType = TEXT("Legacy");
	if (StuckEscapeCount <= 0 && Candidates.Num() > 0)
	{
		Controls = Candidates[0].Controls;
		SelectedInitType = FString::Printf(TEXT("MultiInit_%s"),
			Candidates[0].Type == EInitCandidateType::Warm ? TEXT("Warm") :
			Candidates[0].Type == EInitCandidateType::Nominal ? TEXT("Nom") :
			Candidates[0].Type == EInitCandidateType::Left ? TEXT("Left") :
			Candidates[0].Type == EInitCandidateType::Right ? TEXT("Right") :
			Candidates[0].Type == EInitCandidateType::Up ? TEXT("Up") :
			Candidates[0].Type == EInitCandidateType::Down ? TEXT("Down") :
			Candidates[0].Type == EInitCandidateType::Brake ? TEXT("Brake") : TEXT("?"));
	}
	else
	{
		InitializeControls(Controls, ReferencePoints);
		SelectedInitType = TEXT("Legacy");
	}

	// 逃逸模式递减
	if (StuckEscapeCount > 0)
	{
		--StuckEscapeCount;
	}

	// 投影梯度下降求解
	float CurrentCost = MAX_FLT;
	int32 FinalIter = 0;
	bool bConverged = false;
	float CostAtIterStart = MAX_FLT; // 本轮迭代开始时的代价
	for (int32 Iter = 0; Iter < Config.Solver.MaxIterations; ++Iter)
	{
		FinalIter = Iter + 1;

		// 计算梯度
		TArray<FVector> Gradient;
		{
			// 投影到可行域
			ProjectControls(Controls, CurrentVelocity);

			// 前向仿真
			TArray<FVector> Positions, Velocities;
			ForwardSimulate(CurrentPosition, CurrentVelocity, Controls, Positions, Velocities);

			// 计算代价
			CurrentCost = ComputeCost(Positions, Velocities, Controls, RefPoints, Obstacles);
		if (InitialCost >= MAX_FLT) { InitialCost = CurrentCost; }

			// 收敛检查：比较本轮代价与上一轮开始时的代价
			if (Iter >= 3 && FMath::Abs(CostAtIterStart) > KINDA_SMALL_NUMBER)
			{
				float RelativeChange = FMath::Abs(CostAtIterStart - CurrentCost) / FMath::Abs(CostAtIterStart);
				if (RelativeChange < 1e-4f)
				{
					bConverged = true;
					break;
				}
			}
			CostAtIterStart = CurrentCost;

			// 计算梯度
			ComputeGradient(CurrentPosition, CurrentVelocity, Controls, RefPoints, Obstacles, Gradient);
		}

		// 回溯线搜索
		float StepSize = Config.Solver.InitialStepSize;
		bool bImproved = false;
		for (int32 BT = 0; BT < Config.Solver.MaxBacktrackSteps; ++BT)
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
				bImproved = true;
				break;
			}

			StepSize *= Config.Solver.BacktrackFactor;
		}

		// 线搜索连续未改进：已处于局部最优，提前退出
		if (!bImproved)
		{
			bConverged = true;
			break;
		}
	}

	// 最近障碍物扫描（横向扰动 + 诊断共用）
	float NearestObsDist = MAX_FLT;
	FVector NearestObsDir = FVector::ZeroVector;
	for (const FObstacleInfo& Obs : Obstacles)
	{
		float D = CalculateDistanceToObstacle(CurrentPosition, Obs);
		if (D < NearestObsDist)
		{
			NearestObsDist = D;
			NearestObsDir = (Obs.Center - CurrentPosition).GetSafeNormal();
		}
	}

	// 最终投影
	ProjectControls(Controls, CurrentVelocity);

	// ---- Homotopy 绕行侧分类 ----
	{
		// 找最近的关键障碍物
		int32 KeyObsID = -1;
		float KeyObsDist = Config.Obstacle.ObstacleInfluenceDistance;
		for (const FObstacleInfo& Obs : Obstacles)
		{
			float D = CalculateDistanceToObstacle(CurrentPosition, Obs);
			if (D < KeyObsDist)
			{
				KeyObsDist = D;
				KeyObsID = Obs.ObstacleID;
			}
		}

		if (KeyObsID < 0)
		{
			// 无近距离障碍物，清除 homotopy
			if (CurrentHomotopy != EAvoidanceHomotopy::None)
			{
				CurrentHomotopy = EAvoidanceHomotopy::None;
				HomotopyCost = MAX_FLT;
				HomotopyObstacleID = -1;
			}
		}
		else
		{
			// 前向仿真得到最终轨迹用于分类
			TArray<FVector> HomoPos, HomoVel;
			ForwardSimulate(CurrentPosition, CurrentVelocity, Controls, HomoPos, HomoVel);

			// 计算预测轨迹相对于参考路径的平均横向偏移
			float SumLateral = 0.0f;
			int32 Count = 0;
			for (int32 k = 1; k <= N && k < HomoPos.Num() && k < RefPoints.Num(); ++k)
			{
				FVector ToPos = HomoPos[k] - RefPoints[k];
				FVector RefDir = (RefPoints[FMath::Min(k + 1, RefPoints.Num() - 1)] - RefPoints[k]).GetSafeNormal();
				if (!RefDir.IsNearlyZero())
				{
					FVector Right = FVector::CrossProduct(FVector::UpVector, RefDir).GetSafeNormal();
					if (Right.IsNearlyZero())
						Right = FVector::CrossProduct(FVector::RightVector, RefDir).GetSafeNormal();
					SumLateral += FVector::DotProduct(ToPos, Right);
					++Count;
				}
			}
			float AvgLateral = (Count > 0) ? SumLateral / Count : 0.0f;

			EAvoidanceHomotopy NewHomo = EAvoidanceHomotopy::None;
			if (FMath::Abs(AvgLateral) > 50.0f) // 50cm 阈值
			{
				NewHomo = (AvgLateral > 0.0f) ? EAvoidanceHomotopy::Right : EAvoidanceHomotopy::Left;
			}

			// 迟滞切换：需要 20% 代价改善
			if (CurrentHomotopy == EAvoidanceHomotopy::None || HomotopyObstacleID != KeyObsID)
			{
				CurrentHomotopy = NewHomo;
				HomotopyCost = CurrentCost;
				HomotopyObstacleID = KeyObsID;
			}
			else if (NewHomo != CurrentHomotopy)
			{
				// 切换需要显著代价改善 (>20%)
				if (HomotopyCost > KINDA_SMALL_NUMBER && CurrentCost < HomotopyCost * 0.8f)
				{
					CurrentHomotopy = NewHomo;
					HomotopyCost = CurrentCost;
				}
			}
			else
			{
				// 同方向，更新代价
				HomotopyCost = FMath::Min(HomotopyCost, CurrentCost);
			}
		}
	}

	// ---- 同次优化内失败检测（仅设置标志，后续不再覆盖）----
	{
		// NaN/Inf 代价
		if (!FMath::IsFinite(CurrentCost) || CurrentCost != CurrentCost)
		{
			Result.Diagnostics.bNaNOrInf = true;
		}

		// 达到最大迭代次数未收敛
		if (!bConverged && FinalIter >= Config.Solver.MaxIterations)
		{
			Result.Diagnostics.bMaxIterReached = true;
			Result.Diagnostics.bLineSearchFailed = true;
		}
	}

	// 保存控制序列 (温启动)：逃逸期间禁用，每帧重新求解
	if (StuckEscapeCount <= 0)
	{
		PreviousControls = Controls;
		bHasPreviousControls = true;
	}

	// 最终前向仿真得到预测轨迹
	TArray<FVector> FinalPositions, FinalVelocities;
	ForwardSimulate(CurrentPosition, CurrentVelocity, Controls, FinalPositions, FinalVelocities);


	// 从全时域预测轨迹计算最小净空（包含动态障碍物预测）
	float PredictedMinClearance = MAX_FLT;
	for (int32 k = 0; k <= N && k < FinalPositions.Num(); ++k)
	{
		for (const FObstacleInfo& Obs : Obstacles)
		{
			FObstacleInfo PredObs = PredictObstacle(Obs, k * dt);
			float D = CalculateDistanceToObstacle(FinalPositions[k], PredObs);
			PredictedMinClearance = FMath::Min(PredictedMinClearance, D);
		}
	}

	// 基于预测净空的失败检测
	if (bConverged && PredictedMinClearance < Config.Obstacle.ObstacleSafeDistance * 0.5f)
	{
		Result.Diagnostics.bClearanceInsufficient = true;
	}
	// 填充结果
	Result.TotalCost = CurrentCost;

	// 修正目标：使用前瞻步
	FVector FirstControl = Controls.Num() > 0 ? Controls[0] : FVector::ZeroVector;
	const int32 LookaheadIdx = FMath::Clamp(Config.Init.CorrectionLookaheadSteps, 1, N);
	FVector NextPos = FinalPositions.IsValidIndex(LookaheadIdx) ? FinalPositions[LookaheadIdx] : CurrentPosition;
	Result.CorrectedTarget = NextPos;
	Result.OptimalAcceleration = FirstControl;

	// 修正方向
	FVector CorrDir = NextPos - CurrentPosition;
	const float CorrectionDistance = CorrDir.Size();
	Result.CorrectedDirection = CorrDir.IsNearlyZero() ? FVector::ForwardVector : CorrDir.GetSafeNormal();

	// 斥力分量 (可视化用)
	if (RefPoints.Num() > 0)
	{
		FVector ToRef = (RefPoints[0] - CurrentPosition).GetSafeNormal();
		FVector Attractive = ToRef * FirstControl.Size();
		Result.RepulsiveForce = FirstControl - Attractive;
	}

	// 填充预测轨迹
	Result.PredictedTrajectory.SetNum(N + 1);
	for (int32 k = 0; k <= N; ++k)
	{
		FNMPCPredictionStep& Step = Result.PredictedTrajectory[k];
		Step.Position = FinalPositions[k];
		Step.Velocity = FinalVelocities[k];
		Step.ControlInput = (k < N) ? Controls[k] : FVector::ZeroVector;

		Step.ObstacleCost = 0.0f;
		for (const FObstacleInfo& Obs : Obstacles)
		{
			FObstacleInfo PredObs = PredictObstacle(Obs, k * dt);
			Step.ObstacleCost += ComputeObstacleCost(FinalPositions[k], PredObs);
		}
	}

	// 当前位置障碍物代价
	float CurrentObstacleCost = Result.PredictedTrajectory[0].ObstacleCost;
	// 预测轨迹最大障碍物代价
	float MaxPredictedObsCost = 0.0f;
	for (int32 k = 0; k <= N; ++k)
	{
		MaxPredictedObsCost = FMath::Max(MaxPredictedObsCost, Result.PredictedTrajectory[k].ObstacleCost);
	}

	// EMA 平滑 MaxHorizonObs
	SmoothedMaxHorizonObs = 0.85f * SmoothedMaxHorizonObs + 0.15f * MaxPredictedObsCost;

	// 纠偏判断
	const bool bCurrentHasObs = CurrentObstacleCost > Config.Obstacle.ObstacleCostDeadband;
	const bool bHorizonHasObs = SmoothedMaxHorizonObs > Config.Obstacle.ObstacleCostDeadband * 15.0f;
	const bool bHasMeaningfulCorrection = CorrectionDistance >= Config.Init.MinCorrectionDistance;
	const bool bRawNeedsCorrection = bCurrentHasObs || (bHorizonHasObs && bHasMeaningfulCorrection);

	// 滞后：触发后保持至少 100 帧
	if (bRawNeedsCorrection)
	{
		NeedsCorrectionHoldCount = 40;
	}
	else if (NeedsCorrectionHoldCount > 0)
	{
		--NeedsCorrectionHoldCount;
	}
	Result.bNeedsCorrection = (NeedsCorrectionHoldCount > 0);

	// 卡死判定
	const float FirstControlMagnitude = FirstControl.Size();
	const float CurrentSpeed = CurrentVelocity.Size();
	const bool bLowControlStuck = bCurrentHasObs
		&& FirstControlMagnitude < Config.Init.StuckForceThreshold
		&& CurrentSpeed < 50.0f;
	const float SaturationRatio =
		Config.Actuator.MaxAcceleration > KINDA_SMALL_NUMBER ? (FirstControlMagnitude / Config.Actuator.MaxAcceleration) : 0.0f;

	// 位置基准卡死检测
	const bool bPositionStuck = DetectPositionStuck(CurrentPosition, Result.bNeedsCorrection);

	// 振荡卡死检测
	const bool bOscillationStuck = DetectOscillationStuck(CurrentPosition, Result.bNeedsCorrection);

	Result.bStuck = bLowControlStuck || bPositionStuck || bOscillationStuck;
	const TCHAR* StuckReason = TEXT("None");
	if (bLowControlStuck)
	{
		StuckReason = TEXT("LowControlWithObstacle");
	}
	else if (bPositionStuck)
	{
		StuckReason = TEXT("PositionStuck");
	}
	else if (bOscillationStuck)
	{
		StuckReason = TEXT("OscillationStuck");
	}

	// 卡死时重置温启动并进入逃逸模式
	if ((bLowControlStuck || bPositionStuck || bOscillationStuck) && NearestObsDist < Config.Obstacle.ObstacleInfluenceDistance)
	{
		bHasPreviousControls = false;
		StuckEscapeCount = 30;
	}

	// 状态转换时用 Log 级别，常规帧用节流
	const bool bStateChanged =
		(Result.bNeedsCorrection != bPrevNeedsCorrection) ||
		(Result.bStuck != bPrevStuck);

	if (bStateChanged)
	{
		const TCHAR* HomoStr = CurrentHomotopy == EAvoidanceHomotopy::None ? TEXT("None") :
				CurrentHomotopy == EAvoidanceHomotopy::Left ? TEXT("L") :
				CurrentHomotopy == EAvoidanceHomotopy::Right ? TEXT("R") :
				CurrentHomotopy == EAvoidanceHomotopy::Above ? TEXT("U") : TEXT("D");
			UE_LOG(LogUAVPlanning, Log, TEXT("[NMPC] >>> State changed: NeedsCorr %s->%s, Stuck %s->%s Homo=%s | Pos=(%.0f,%.0f,%.0f) Speed=%.0f Obs=%d NearDist=%.0f Cost=%.1f ObsCost=%.3f MaxHorizonObs=%.3f Progress=%.1f SatRatio=%.2f Iter=%d Conv=%s Reason=%s"),
				bPrevNeedsCorrection ? TEXT("Y") : TEXT("N"),
				Result.bNeedsCorrection ? TEXT("Y") : TEXT("N"),
				bPrevStuck ? TEXT("Y") : TEXT("N"),
				Result.bStuck ? TEXT("Y") : TEXT("N"),
				HomoStr,
				CurrentPosition.X, CurrentPosition.Y, CurrentPosition.Z,
				CurrentVelocity.Size(),
				Obstacles.Num(), NearestObsDist,
				CurrentCost, CurrentObstacleCost, MaxPredictedObsCost,
				CorrectionDistance, SaturationRatio,
				FinalIter, bConverged ? TEXT("Y") : TEXT("N"),
				StuckReason);
	}
	else
	{
		UE_LOG_THROTTLE(1.0, LogUAVPlanning, Log, TEXT("[NMPC] Pos=(%.0f,%.0f,%.0f) Speed=%.0f Cost=%.1f ObsCost=%.3f Progress=%.1f SatRatio=%.2f Iter=%d NeedsCorr=%s Stuck=%s Reason=%s"),
				CurrentPosition.X, CurrentPosition.Y, CurrentPosition.Z,
				CurrentVelocity.Size(),
				CurrentCost, CurrentObstacleCost,
				CorrectionDistance, SaturationRatio,
				FinalIter,
				Result.bNeedsCorrection ? TEXT("Y") : TEXT("N"),
				Result.bStuck ? TEXT("Y") : TEXT("N"),
				StuckReason);
	}


	// 填充诊断信息
	Result.Diagnostics.InitialCost = InitialCost;
	Result.Diagnostics.FinalCost = CurrentCost;
	if (InitialCost > KINDA_SMALL_NUMBER && InitialCost < MAX_FLT)
	{
		Result.Diagnostics.RelativeCostDecrease = (InitialCost - CurrentCost) / InitialCost;
	}
	Result.Diagnostics.Iterations = FinalIter;
	Result.Diagnostics.InitType = SelectedInitType;
	Result.Diagnostics.MinPredictedClearance = PredictedMinClearance;
	Result.Diagnostics.PredictedPathProgress = CorrectionDistance;
	if (bConverged && CorrectionDistance < Config.Init.MinCorrectionDistance * 0.5f)
	{
		Result.Diagnostics.bProgressInsufficient = true;
	}
	Result.Diagnostics.SolveTimeMs = static_cast<float>((FPlatformTime::Seconds() - SolveStartTime) * 1000.0);

	// 结构化求解日志
	UE_LOG(LogUAVMetrics, Log, TEXT("[NMPC_SOLVE] Init=%s J0=%.1f J=%.1f Decrease=%.4f Clearance=%.0f Progress=%.0f Iter=%d LSFail=%d Ms=%.2f"),
		*Result.Diagnostics.InitType,
		InitialCost, CurrentCost, Result.Diagnostics.RelativeCostDecrease,
		PredictedMinClearance, CorrectionDistance, FinalIter,
		Result.Diagnostics.BacktrackFailCount,
		Result.Diagnostics.SolveTimeMs);

	bPrevNeedsCorrection = Result.bNeedsCorrection;
	bPrevStuck = Result.bStuck;

	return Result;
}
