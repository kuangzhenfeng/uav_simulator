#include "ObstacleGeometry.h"
#include "LandscapeProxy.h"

static float CalculateCylinderDistance(const FVector& Point, const FObstacleInfo& Obstacle)
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

float ObstacleGeometry::SignedDistance(const FVector& Point, const FObstacleInfo& Obstacle)
{
	switch (Obstacle.Type)
	{
	case EObstacleType::Terrain:
		{
			const ALandscapeProxy* Landscape = Cast<ALandscapeProxy>(Obstacle.LinkedActor.Get());
			const TOptional<float> Height = Landscape ? Landscape->GetHeightAtLocation(Point) : TOptional<float>();
			// 地形约束使用真实高度场的垂直净空，不把山地包围盒当成实体。
			return Height.IsSet() ? Point.Z - Height.GetValue() - Obstacle.SafetyMargin : MAX_flt;
		}

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
FVector ObstacleGeometry::Gradient(const FVector& Point, const FObstacleInfo& Obstacle)
{
	switch (Obstacle.Type)
	{
	case EObstacleType::Terrain:
		{
			const ALandscapeProxy* Landscape = Cast<ALandscapeProxy>(Obstacle.LinkedActor.Get());
			if (!Landscape) return FVector::ZeroVector;
			const float Step = FMath::Max(10.0f, static_cast<float>(Landscape->GetActorScale3D().GetAbsMin()) * 0.25f);
			const TOptional<float> H = Landscape->GetHeightAtLocation(Point);
			if (!H.IsSet()) return FVector::ZeroVector;
			auto Slope = [&](const FVector& Axis)
			{
				const TOptional<float> Plus = Landscape->GetHeightAtLocation(Point + Axis * Step);
				const TOptional<float> Minus = Landscape->GetHeightAtLocation(Point - Axis * Step);
				if (Plus.IsSet() && Minus.IsSet()) return (Plus.GetValue() - Minus.GetValue()) / (2.0f * Step);
				if (Plus.IsSet()) return (Plus.GetValue() - H.GetValue()) / Step;
				if (Minus.IsSet()) return (H.GetValue() - Minus.GetValue()) / Step;
				return 0.0f;
			};
			return FVector(-Slope(FVector::ForwardVector), -Slope(FVector::RightVector), 1.0f);
		}

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

