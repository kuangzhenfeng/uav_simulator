// Copyright Epic Games, Inc. All Rights Reserved.

#include "LocalAvoidance.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

ULocalAvoidance::ULocalAvoidance()
{
}

FLocalAvoidanceResult ULocalAvoidance::ComputeAvoidance(
	const FVector& CurrentPosition,
	const FVector& TargetPosition,
	const TArray<FObstacleInfo>& Obstacles,
	const FVector& CurrentVelocity) const
{
	FLocalAvoidanceResult Result;

	// 计算引力
	Result.AttractiveForce = ComputeAttractiveForce(CurrentPosition, TargetPosition);

	// 计算所有障碍物的斥力之和
	Result.RepulsiveForce = FVector::ZeroVector;
	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		FVector Force = ComputeRepulsiveForce(CurrentPosition, Obstacle);
		if (!Force.IsNearlyZero())
		{
			Result.RepulsiveForce += Force;
			Result.bNeedsCorrection = true;
		}
	}

	// 合力
	Result.TotalForce = Result.AttractiveForce + Result.RepulsiveForce;

	// 检查是否到达目标
	float DistToGoal = FVector::Dist(CurrentPosition, TargetPosition);
	if (DistToGoal < GoalReachedThreshold)
	{
		Result.CorrectedDirection = FVector::ZeroVector;
		Result.bStuck = false;
		return Result;
	}

	// 检查局部极小值 (合力接近零但未到达目标)
	if (Result.TotalForce.Size() < StuckForceThreshold && Result.bNeedsCorrection)
	{
		Result.bStuck = true;

		// 逃逸策略：添加垂直于引力方向的扰动
		FVector AttDir = Result.AttractiveForce.GetSafeNormal();
		FVector Perturbation = FVector::CrossProduct(AttDir, FVector::UpVector);
		if (Perturbation.IsNearlyZero())
		{
			Perturbation = FVector::CrossProduct(AttDir, FVector::RightVector);
		}
		Result.TotalForce = Result.AttractiveForce + Perturbation.GetSafeNormal() * AttractiveGain * 100.0f;

		UE_LOG(LogUAVPlanning, Warning, TEXT("[LocalAvoidance] Stuck detected! Adding perturbation. DistToGoal=%.1f"), DistToGoal);
	}

	// 归一化得到修正方向
	if (!Result.TotalForce.IsNearlyZero())
	{
		Result.CorrectedDirection = Result.TotalForce.GetSafeNormal();
	}
	else
	{
		// 合力为零时保持当前速度方向
		Result.CorrectedDirection = CurrentVelocity.IsNearlyZero()
			? (TargetPosition - CurrentPosition).GetSafeNormal()
			: CurrentVelocity.GetSafeNormal();
	}

	UE_LOG(LogUAVPlanning, Verbose, TEXT("[LocalAvoidance] Att=(%.1f,%.1f,%.1f) Rep=(%.1f,%.1f,%.1f) Total=(%.1f,%.1f,%.1f) NeedsCorr=%s Stuck=%s"),
		Result.AttractiveForce.X, Result.AttractiveForce.Y, Result.AttractiveForce.Z,
		Result.RepulsiveForce.X, Result.RepulsiveForce.Y, Result.RepulsiveForce.Z,
		Result.TotalForce.X, Result.TotalForce.Y, Result.TotalForce.Z,
		Result.bNeedsCorrection ? TEXT("Y") : TEXT("N"),
		Result.bStuck ? TEXT("Y") : TEXT("N"));

	return Result;
}

FVector ULocalAvoidance::ComputeAttractiveForce(const FVector& CurrentPosition, const FVector& TargetPosition) const
{
	FVector Direction = TargetPosition - CurrentPosition;
	float Distance = Direction.Size();

	if (Distance < KINDA_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}

	// 线性引力: F_att = K_att * (Target - Current)
	return Direction.GetSafeNormal() * AttractiveGain * FMath::Min(Distance, 1000.0f);
}

FVector ULocalAvoidance::ComputeRepulsiveForce(const FVector& CurrentPosition, const FObstacleInfo& Obstacle) const
{
	float Distance = CalculateDistanceToObstacle(CurrentPosition, Obstacle);

	// 超出影响距离，不产生斥力
	if (Distance > InfluenceDistance || Distance < KINDA_SMALL_NUMBER)
	{
		if (Distance < KINDA_SMALL_NUMBER && Distance > -InfluenceDistance)
		{
			// 在障碍物内部，产生最大斥力
			FVector RepDir = CalculateRepulsionDirection(CurrentPosition, Obstacle);
			FVector Force = RepDir * MaxRepulsiveForce;
			UE_LOG(LogUAVPlanning, Warning, TEXT("[LocalAvoidance] Inside obstacle ID=%d! MaxForce applied. Dist=%.1f"),
				Obstacle.ObstacleID, Distance);
			return Force;
		}
		return FVector::ZeroVector;
	}

	// 斥力公式: F_rep = K_rep * (1/d - 1/d0) * (1/d^2) * direction
	// d = distance to obstacle, d0 = influence distance
	float InvDist = 1.0f / Distance;
	float InvInfluence = 1.0f / InfluenceDistance;
	float Magnitude = RepulsiveGain * (InvDist - InvInfluence) * (InvDist * InvDist);

	// 限制最大斥力
	Magnitude = FMath::Min(Magnitude, MaxRepulsiveForce);

	FVector RepDir = CalculateRepulsionDirection(CurrentPosition, Obstacle);
	return RepDir * Magnitude;
}

float ULocalAvoidance::CalculateDistanceToObstacle(const FVector& Point, const FObstacleInfo& Obstacle) const
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

FVector ULocalAvoidance::CalculateRepulsionDirection(const FVector& Point, const FObstacleInfo& Obstacle) const
{
	FVector Direction;

	switch (Obstacle.Type)
	{
	case EObstacleType::Sphere:
		Direction = Point - Obstacle.Center;
		break;

	case EObstacleType::Box:
		{
			FVector LocalPoint = Obstacle.Rotation.UnrotateVector(Point - Obstacle.Center);
			FVector ClosestPoint;
			ClosestPoint.X = FMath::Clamp(LocalPoint.X, -Obstacle.Extents.X, Obstacle.Extents.X);
			ClosestPoint.Y = FMath::Clamp(LocalPoint.Y, -Obstacle.Extents.Y, Obstacle.Extents.Y);
			ClosestPoint.Z = FMath::Clamp(LocalPoint.Z, -Obstacle.Extents.Z, Obstacle.Extents.Z);
			Direction = Obstacle.Rotation.RotateVector(LocalPoint - ClosestPoint);
		}
		break;

	case EObstacleType::Cylinder:
		{
			FVector LocalPoint = Point - Obstacle.Center;
			float HorizontalDist = FVector2D(LocalPoint.X, LocalPoint.Y).Size();

			if (HorizontalDist > KINDA_SMALL_NUMBER)
			{
				Direction = FVector(LocalPoint.X, LocalPoint.Y, 0.0f);
			}
			else
			{
				Direction = FVector(1.0f, 0.0f, 0.0f);
			}
		}
		break;

	default:
		Direction = Point - Obstacle.Center;
		break;
	}

	return Direction.IsNearlyZero() ? FVector::UpVector : Direction.GetSafeNormal();
}
