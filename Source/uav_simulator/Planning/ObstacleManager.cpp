// Copyright Epic Games, Inc. All Rights Reserved.

#include "ObstacleManager.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UObstacleManager::UObstacleManager()
{
	PrimaryComponentTick.bCanEverTick = true;
	NextObstacleID = 1;
	DefaultSafetyMargin = 50.0f;
	bAutoUpdateDynamicObstacles = true;
	CollisionWarningDistance = 200.0f;
	bShowDebug = false;
}

void UObstacleManager::BeginPlay()
{
	Super::BeginPlay();
}

void UObstacleManager::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bAutoUpdateDynamicObstacles)
	{
		UpdateDynamicObstacles();
	}

	if (bShowDebug)
	{
		DrawDebugObstacles();
	}
}

int32 UObstacleManager::RegisterObstacle(const FObstacleInfo& Obstacle)
{
	FObstacleInfo NewObstacle = Obstacle;
	NewObstacle.ObstacleID = NextObstacleID++;

	Obstacles.Add(NewObstacle);
	OnObstacleDetected.Broadcast(NewObstacle);

	UE_LOG(LogUAVPlanning, Log, TEXT("Obstacle registered: ID=%d, Type=%d, Center=%s, Extents=%s"),
		NewObstacle.ObstacleID,
		(int32)NewObstacle.Type,
		*NewObstacle.Center.ToString(),
		*NewObstacle.Extents.ToString());

	return NewObstacle.ObstacleID;
}

int32 UObstacleManager::RegisterObstacleFromActor(AActor* Actor, EObstacleType Type, float SafetyMargin)
{
	if (!Actor)
	{
		return -1;
	}

	FObstacleInfo Obstacle;
	Obstacle.Type = Type;
	Obstacle.Center = Actor->GetActorLocation();
	Obstacle.Rotation = Actor->GetActorRotation();
	Obstacle.SafetyMargin = SafetyMargin;
	Obstacle.LinkedActor = Actor;

	// 获取Actor的边界
	FVector Origin, BoxExtent;
	Actor->GetActorBounds(false, Origin, BoxExtent);

	switch (Type)
	{
	case EObstacleType::Sphere:
		Obstacle.Extents = FVector(BoxExtent.GetMax());
		break;

	case EObstacleType::Box:
		Obstacle.Extents = BoxExtent;
		break;

	case EObstacleType::Cylinder:
		Obstacle.Extents = FVector(FMath::Max(BoxExtent.X, BoxExtent.Y), 0.0f, BoxExtent.Z);
		break;

	default:
		Obstacle.Extents = BoxExtent;
		break;
	}

	return RegisterObstacle(Obstacle);
}

bool UObstacleManager::RemoveObstacle(int32 ObstacleID)
{
	for (int32 i = 0; i < Obstacles.Num(); ++i)
	{
		if (Obstacles[i].ObstacleID == ObstacleID)
		{
			Obstacles.RemoveAt(i);
			return true;
		}
	}
	return false;
}

void UObstacleManager::ClearAllObstacles()
{
	Obstacles.Empty();
}

bool UObstacleManager::GetObstacle(int32 ObstacleID, FObstacleInfo& OutObstacle) const
{
	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		if (Obstacle.ObstacleID == ObstacleID)
		{
			OutObstacle = Obstacle;
			return true;
		}
	}
	return false;
}

bool UObstacleManager::CheckCollision(const FVector& Point, float Radius) const
{
	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		if (IsPointInObstacle(Point, Obstacle, Radius))
		{
			UE_LOG(LogUAVPlanning, Verbose, TEXT("Collision detected: Point %s collides with Obstacle %d"),
				*Point.ToString(), Obstacle.ObstacleID);
			return true;
		}
	}
	return false;
}

bool UObstacleManager::CheckLineCollision(const FVector& Start, const FVector& End, float Radius) const
{
	FVector Direction = End - Start;
	float Distance = Direction.Size();

	if (Distance < KINDA_SMALL_NUMBER)
	{
		return CheckCollision(Start, Radius);
	}

	Direction.Normalize();

	// 沿线段采样
	float StepSize = FMath::Min(50.0f, Distance * 0.1f);
	int32 NumSteps = FMath::CeilToInt(Distance / StepSize);

	for (int32 i = 0; i <= NumSteps; ++i)
	{
		float t = (float)i / (float)NumSteps;
		FVector SamplePoint = FMath::Lerp(Start, End, t);

		if (CheckCollision(SamplePoint, Radius))
		{
			return true;
		}
	}

	return false;
}

float UObstacleManager::GetDistanceToNearestObstacle(const FVector& Point, FObstacleInfo& OutNearestObstacle) const
{
	float MinDistance = FLT_MAX;
	int32 NearestIndex = -1;

	for (int32 i = 0; i < Obstacles.Num(); ++i)
	{
		float Distance = CalculateDistanceToObstacle(Point, Obstacles[i]);
		if (Distance < MinDistance)
		{
			MinDistance = Distance;
			NearestIndex = i;
		}
	}

	if (NearestIndex >= 0)
	{
		OutNearestObstacle = Obstacles[NearestIndex];
	}

	return MinDistance;
}

TArray<FObstacleInfo> UObstacleManager::GetObstaclesInRange(const FVector& Center, float Radius) const
{
	TArray<FObstacleInfo> Result;

	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		float Distance = CalculateDistanceToObstacle(Center, Obstacle);
		if (Distance <= Radius)
		{
			Result.Add(Obstacle);
		}
	}

	return Result;
}

void UObstacleManager::ScanForObstacles(const FVector& ScanCenter, float ScanRadius, const TArray<TSubclassOf<AActor>>& ActorClasses)
{
	if (!GetWorld())
	{
		return;
	}

	for (const TSubclassOf<AActor>& ActorClass : ActorClasses)
	{
		for (TActorIterator<AActor> It(GetWorld(), ActorClass); It; ++It)
		{
			AActor* Actor = *It;
			if (Actor && Actor != GetOwner())
			{
				float Distance = FVector::Dist(Actor->GetActorLocation(), ScanCenter);
				if (Distance <= ScanRadius)
				{
					// 检查是否已经注册
					bool bAlreadyRegistered = false;
					for (const FObstacleInfo& Obstacle : Obstacles)
					{
						if (Obstacle.LinkedActor.Get() == Actor)
						{
							bAlreadyRegistered = true;
							break;
						}
					}

					if (!bAlreadyRegistered)
					{
						RegisterObstacleFromActor(Actor, EObstacleType::Box, DefaultSafetyMargin);
					}
				}
			}
		}
	}
}

void UObstacleManager::UpdateDynamicObstacles()
{
	for (FObstacleInfo& Obstacle : Obstacles)
	{
		if (Obstacle.bIsDynamic && Obstacle.LinkedActor.IsValid())
		{
			AActor* Actor = Obstacle.LinkedActor.Get();
			Obstacle.Center = Actor->GetActorLocation();
			Obstacle.Rotation = Actor->GetActorRotation();
		}
	}
}

float UObstacleManager::CalculateDistanceToObstacle(const FVector& Point, const FObstacleInfo& Obstacle) const
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

			float Distance = FVector::Dist(LocalPoint, ClosestPoint);
			return Distance - Obstacle.SafetyMargin;
		}

	case EObstacleType::Cylinder:
		{
			FVector LocalPoint = Point - Obstacle.Center;
			float HorizontalDist = FVector2D(LocalPoint.X, LocalPoint.Y).Size();
			float VerticalDist = FMath::Abs(LocalPoint.Z);

			float HorizontalPenetration = HorizontalDist - Obstacle.Extents.X;
			float VerticalPenetration = VerticalDist - Obstacle.Extents.Z;

			if (HorizontalPenetration < 0 && VerticalPenetration < 0)
			{
				return FMath::Max(HorizontalPenetration, VerticalPenetration) - Obstacle.SafetyMargin;
			}
			else if (HorizontalPenetration < 0)
			{
				return VerticalPenetration - Obstacle.SafetyMargin;
			}
			else if (VerticalPenetration < 0)
			{
				return HorizontalPenetration - Obstacle.SafetyMargin;
			}
			else
			{
				return FMath::Sqrt(HorizontalPenetration * HorizontalPenetration +
								   VerticalPenetration * VerticalPenetration) - Obstacle.SafetyMargin;
			}
		}

	default:
		return FVector::Dist(Point, Obstacle.Center) - Obstacle.Extents.GetMax() - Obstacle.SafetyMargin;
	}
}

bool UObstacleManager::IsPointInObstacle(const FVector& Point, const FObstacleInfo& Obstacle, float Radius) const
{
	float TotalRadius = Radius + Obstacle.SafetyMargin;

	switch (Obstacle.Type)
	{
	case EObstacleType::Sphere:
		return FVector::Dist(Point, Obstacle.Center) < (Obstacle.Extents.X + TotalRadius);

	case EObstacleType::Box:
		{
			FVector LocalPoint = Obstacle.Rotation.UnrotateVector(Point - Obstacle.Center);
			FVector ExpandedExtents = Obstacle.Extents + FVector(TotalRadius);

			return FMath::Abs(LocalPoint.X) < ExpandedExtents.X &&
				   FMath::Abs(LocalPoint.Y) < ExpandedExtents.Y &&
				   FMath::Abs(LocalPoint.Z) < ExpandedExtents.Z;
		}

	case EObstacleType::Cylinder:
		{
			FVector LocalPoint = Point - Obstacle.Center;
			float HorizontalDist = FVector2D(LocalPoint.X, LocalPoint.Y).Size();

			return HorizontalDist < (Obstacle.Extents.X + TotalRadius) &&
				   FMath::Abs(LocalPoint.Z) < (Obstacle.Extents.Z + TotalRadius);
		}

	default:
		return false;
	}
}

void UObstacleManager::DrawDebugObstacles() const
{
	if (!GetWorld())
	{
		return;
	}

	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		FColor Color = Obstacle.bIsDynamic ? FColor::Orange : FColor::Red;

		switch (Obstacle.Type)
		{
		case EObstacleType::Sphere:
			DrawDebugSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.X, 16, Color, false, -1.0f, 0, 2.0f);
			DrawDebugSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.X + Obstacle.SafetyMargin, 16,
				FColor(Color.R, Color.G, Color.B, 128), false, -1.0f, 0, 1.0f);
			break;

		case EObstacleType::Box:
			DrawDebugBox(GetWorld(), Obstacle.Center, Obstacle.Extents, Obstacle.Rotation.Quaternion(),
				Color, false, -1.0f, 0, 2.0f);
			break;

		case EObstacleType::Cylinder:
			// 近似用多条线绘制圆柱
			{
				int32 NumSegments = 16;
				for (int32 i = 0; i < NumSegments; ++i)
				{
					float Angle1 = (float)i / NumSegments * 2.0f * PI;
					float Angle2 = (float)(i + 1) / NumSegments * 2.0f * PI;

					FVector P1Top = Obstacle.Center + FVector(FMath::Cos(Angle1) * Obstacle.Extents.X,
						FMath::Sin(Angle1) * Obstacle.Extents.X, Obstacle.Extents.Z);
					FVector P2Top = Obstacle.Center + FVector(FMath::Cos(Angle2) * Obstacle.Extents.X,
						FMath::Sin(Angle2) * Obstacle.Extents.X, Obstacle.Extents.Z);
					FVector P1Bottom = Obstacle.Center + FVector(FMath::Cos(Angle1) * Obstacle.Extents.X,
						FMath::Sin(Angle1) * Obstacle.Extents.X, -Obstacle.Extents.Z);
					FVector P2Bottom = Obstacle.Center + FVector(FMath::Cos(Angle2) * Obstacle.Extents.X,
						FMath::Sin(Angle2) * Obstacle.Extents.X, -Obstacle.Extents.Z);

					DrawDebugLine(GetWorld(), P1Top, P2Top, Color, false, -1.0f, 0, 2.0f);
					DrawDebugLine(GetWorld(), P1Bottom, P2Bottom, Color, false, -1.0f, 0, 2.0f);
					DrawDebugLine(GetWorld(), P1Top, P1Bottom, Color, false, -1.0f, 0, 2.0f);
				}
			}
			break;

		default:
			break;
		}
	}
}
