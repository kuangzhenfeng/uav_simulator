// Copyright Epic Games, Inc. All Rights Reserved.

#include "PathPlanner.h"
#include "DrawDebugHelpers.h"

UPathPlanner::UPathPlanner()
{
	PrimaryComponentTick.bCanEverTick = true;
	LastPlanningTimeMs = 0.0f;
}

void UPathPlanner::BeginPlay()
{
	Super::BeginPlay();
}

void UPathPlanner::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

bool UPathPlanner::PlanPath(const FVector& Start, const FVector& Goal, TArray<FVector>& OutPath)
{
	// 基类默认实现：直线路径
	OutPath.Empty();
	OutPath.Add(Start);
	OutPath.Add(Goal);
	LastPath = OutPath;
	return true;
}

void UPathPlanner::SetObstacles(const TArray<FObstacleInfo>& InObstacles)
{
	Obstacles = InObstacles;
}

void UPathPlanner::AddObstacle(const FObstacleInfo& Obstacle)
{
	Obstacles.Add(Obstacle);
}

void UPathPlanner::ClearObstacles()
{
	Obstacles.Empty();
}

bool UPathPlanner::CheckCollision(const FVector& Point, float Radius) const
{
	float TotalRadius = Radius + UAVCollisionRadius + PlanningConfig.SafetyMargin;

	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		if (IsPointInObstacle(Point, Obstacle, TotalRadius))
		{
			return true;
		}
	}
	return false;
}

bool UPathPlanner::CheckLineCollision(const FVector& Start, const FVector& End, float Radius) const
{
	// 沿线段采样检测碰撞
	FVector Direction = End - Start;
	float Distance = Direction.Size();

	if (Distance < KINDA_SMALL_NUMBER)
	{
		return CheckCollision(Start, Radius);
	}

	Direction.Normalize();

	// 采样步长
	float StepSize = FMath::Min(PlanningConfig.GridResolution * 0.5f, Distance * 0.1f);
	StepSize = FMath::Max(StepSize, 10.0f); // 最小10cm

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

bool UPathPlanner::IsPointInObstacle(const FVector& Point, const FObstacleInfo& Obstacle, float Radius) const
{
	float TotalRadius = Radius + Obstacle.SafetyMargin;

	switch (Obstacle.Type)
	{
	case EObstacleType::Sphere:
		{
			float Distance = FVector::Dist(Point, Obstacle.Center);
			return Distance < (Obstacle.Extents.X + TotalRadius);
		}

	case EObstacleType::Box:
		{
			// 将点转换到障碍物局部坐标系
			FVector LocalPoint = Obstacle.Rotation.UnrotateVector(Point - Obstacle.Center);
			FVector ExpandedExtents = Obstacle.Extents + FVector(TotalRadius);

			return FMath::Abs(LocalPoint.X) < ExpandedExtents.X &&
				   FMath::Abs(LocalPoint.Y) < ExpandedExtents.Y &&
				   FMath::Abs(LocalPoint.Z) < ExpandedExtents.Z;
		}

	case EObstacleType::Cylinder:
		{
			// Extents.X = 半径, Extents.Z = 半高
			FVector LocalPoint = Point - Obstacle.Center;
			float HorizontalDist = FVector2D(LocalPoint.X, LocalPoint.Y).Size();

			return HorizontalDist < (Obstacle.Extents.X + TotalRadius) &&
				   FMath::Abs(LocalPoint.Z) < (Obstacle.Extents.Z + TotalRadius);
		}

	default:
		return false;
	}
}

void UPathPlanner::SimplifyPath(TArray<FVector>& Path) const
{
	if (Path.Num() <= 2)
	{
		return;
	}

	TArray<FVector> SimplifiedPath;
	SimplifiedPath.Add(Path[0]);

	int32 CurrentIndex = 0;

	while (CurrentIndex < Path.Num() - 1)
	{
		// 找到从当前点能直接到达的最远点
		int32 FarthestVisible = CurrentIndex + 1;

		for (int32 i = Path.Num() - 1; i > CurrentIndex + 1; --i)
		{
			if (!CheckLineCollision(Path[CurrentIndex], Path[i], 0.0f))
			{
				FarthestVisible = i;
				break;
			}
		}

		SimplifiedPath.Add(Path[FarthestVisible]);
		CurrentIndex = FarthestVisible;
	}

	Path = SimplifiedPath;
}

void UPathPlanner::VisualizeResult(float Duration)
{
	if (!GetWorld() || LastPath.Num() < 2)
	{
		return;
	}

	DrawDebugPath(LastPath, FColor::Green, Duration, 3.0f);

	// 绘制起点和终点
	if (LastPath.Num() > 0)
	{
		DrawDebugSphere(GetWorld(), LastPath[0], 20.0f, 12, FColor::Blue, false, Duration);
		DrawDebugSphere(GetWorld(), LastPath.Last(), 20.0f, 12, FColor::Red, false, Duration);
	}
}

void UPathPlanner::DrawDebugPath(const TArray<FVector>& Path, const FColor& Color, float Duration, float Thickness) const
{
	if (!GetWorld())
	{
		return;
	}

	for (int32 i = 1; i < Path.Num(); ++i)
	{
		DrawDebugLine(GetWorld(), Path[i - 1], Path[i], Color, false, Duration, 0, Thickness);
		DrawDebugPoint(GetWorld(), Path[i], 10.0f, Color, false, Duration);
	}
}
