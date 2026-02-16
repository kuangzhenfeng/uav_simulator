// Copyright Epic Games, Inc. All Rights Reserved.

#include "PlanningVisualizer.h"
#include "DrawDebugHelpers.h"

UPlanningVisualizer::UPlanningVisualizer()
{
	PrimaryComponentTick.bCanEverTick = true;
	bEnableVisualization = true;
}

void UPlanningVisualizer::BeginPlay()
{
	Super::BeginPlay();
}

void UPlanningVisualizer::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bEnableVisualization)
	{
		DrawPersistentData();
	}
}

void UPlanningVisualizer::DrawPath(const TArray<FVector>& Path, FColor Color, float Duration, float Thickness)
{
	if (!bEnableVisualization || !bShowPath || !GetWorld() || Path.Num() < 2)
	{
		return;
	}

	// 绘制路径线段
	for (int32 i = 1; i < Path.Num(); ++i)
	{
		DrawDebugLine(GetWorld(), Path[i - 1], Path[i], Color, false, Duration, 0, Thickness);
	}

	// 绘制路径点
	for (int32 i = 0; i < Path.Num(); ++i)
	{
		float PointSize = (i == 0 || i == Path.Num() - 1) ? 15.0f : 8.0f;
		DrawDebugPoint(GetWorld(), Path[i], PointSize, Color, false, Duration);
	}

	// 绘制起点和终点标记
	if (Path.Num() > 0)
	{
		DrawDebugSphere(GetWorld(), Path[0], 25.0f, 8, FColor::Green, false, Duration, 0, 2.0f);
		DrawDebugSphere(GetWorld(), Path.Last(), 25.0f, 8, FColor::Red, false, Duration, 0, 2.0f);
	}
}

void UPlanningVisualizer::DrawTrajectory(const FTrajectory& Trajectory, FColor Color, float Duration, bool bShowVelocity)
{
	if (!bEnableVisualization || !bShowTrajectory || !GetWorld() || !Trajectory.bIsValid || Trajectory.Points.Num() < 2)
	{
		return;
	}

	// 绘制轨迹曲线
	for (int32 i = 1; i < Trajectory.Points.Num(); ++i)
	{
		DrawDebugLine(GetWorld(), Trajectory.Points[i - 1].Position, Trajectory.Points[i].Position,
			Color, false, Duration, 0, LineThickness);
	}

	// 绘制速度向量
	if (bShowVelocity)
	{
		int32 VelocityStep = FMath::Max(1, Trajectory.Points.Num() / 20); // 最多显示20个速度向量
		for (int32 i = 0; i < Trajectory.Points.Num(); i += VelocityStep)
		{
			const FTrajectoryPoint& Point = Trajectory.Points[i];
			if (!Point.Velocity.IsNearlyZero())
			{
				FVector VelocityEnd = Point.Position + Point.Velocity * 0.2f; // 缩放速度向量
				DrawDebugDirectionalArrow(GetWorld(), Point.Position, VelocityEnd, 30.0f,
					FColor::Yellow, false, Duration, 0, 1.5f);
			}
		}
	}

	// 绘制起点和终点
	DrawDebugSphere(GetWorld(), Trajectory.Points[0].Position, 20.0f, 8, FColor::Green, false, Duration, 0, 2.0f);
	DrawDebugSphere(GetWorld(), Trajectory.Points.Last().Position, 20.0f, 8, FColor::Red, false, Duration, 0, 2.0f);
}

void UPlanningVisualizer::DrawTrackingPoint(const FTrajectoryPoint& Point, float Radius, FColor Color)
{
	if (!bEnableVisualization || !GetWorld())
	{
		return;
	}

	// 绘制当前位置球体
	DrawDebugSphere(GetWorld(), Point.Position, Radius, 12, Color, false, -1.0f, 0, 2.0f);

	// 绘制速度方向
	if (!Point.Velocity.IsNearlyZero())
	{
		FVector VelocityEnd = Point.Position + Point.Velocity.GetSafeNormal() * (Radius * 3.0f);
		DrawDebugDirectionalArrow(GetWorld(), Point.Position, VelocityEnd, 20.0f, FColor::Cyan, false, -1.0f, 0, 2.0f);
	}
}

void UPlanningVisualizer::DrawObstacles(const TArray<FObstacleInfo>& Obstacles, FColor Color, float Duration)
{
	if (!bEnableVisualization || !bShowObstacles)
	{
		return;
	}

	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		DrawObstacle(Obstacle, Color, Duration);
	}
}

void UPlanningVisualizer::DrawObstacle(const FObstacleInfo& Obstacle, FColor Color, float Duration)
{
	if (!bEnableVisualization || !bShowObstacles || !GetWorld())
	{
		return;
	}

	FColor SafetyColor = FColor(Color.R, Color.G, Color.B, 100);

	switch (Obstacle.Type)
	{
	case EObstacleType::Sphere:
		DrawDebugSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.X, 16, Color, false, Duration, 0, 2.0f);
		// 绘制安全边距
		DrawDebugSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.X + Obstacle.SafetyMargin, 16,
			SafetyColor, false, Duration, 0, 1.0f);
		break;

	case EObstacleType::Box:
		DrawDebugBox(GetWorld(), Obstacle.Center, Obstacle.Extents, Obstacle.Rotation.Quaternion(),
			Color, false, Duration, 0, 2.0f);
		break;

	case EObstacleType::Cylinder:
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

				DrawDebugLine(GetWorld(), P1Top, P2Top, Color, false, Duration, 0, 2.0f);
				DrawDebugLine(GetWorld(), P1Bottom, P2Bottom, Color, false, Duration, 0, 2.0f);
				DrawDebugLine(GetWorld(), P1Top, P1Bottom, Color, false, Duration, 0, 2.0f);
			}
		}
		break;

	default:
		DrawDebugSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.GetMax(), 16, Color, false, Duration, 0, 2.0f);
		break;
	}
}

void UPlanningVisualizer::DrawSearchBounds(const FVector& MinBounds, const FVector& MaxBounds, FColor Color, float Duration)
{
	if (!bEnableVisualization || !GetWorld())
	{
		return;
	}

	FVector Center = (MinBounds + MaxBounds) * 0.5f;
	FVector Extent = (MaxBounds - MinBounds) * 0.5f;

	DrawDebugBox(GetWorld(), Center, Extent, Color, false, Duration, 0, 1.0f);
}

void UPlanningVisualizer::DrawWaypoints(const TArray<FVector>& Waypoints, float Radius, FColor Color, float Duration)
{
	if (!bEnableVisualization || !bShowWaypoints || !GetWorld())
	{
		return;
	}

	for (int32 i = 0; i < Waypoints.Num(); ++i)
	{
		DrawDebugSphere(GetWorld(), Waypoints[i], Radius, 12, Color, false, Duration, 0, 2.0f);

		// 绘制航点编号
		FString Label = FString::Printf(TEXT("%d"), i);
		DrawDebugString(GetWorld(), Waypoints[i] + FVector(0, 0, Radius + 10.0f), Label, nullptr, Color, Duration);
	}
}

void UPlanningVisualizer::ClearVisualization()
{
	if (GetWorld())
	{
		FlushPersistentDebugLines(GetWorld());
	}
}

void UPlanningVisualizer::SetPersistentPath(const TArray<FVector>& Path)
{
	PersistentPath = Path;
}

void UPlanningVisualizer::SetPersistentTrajectory(const FTrajectory& Trajectory)
{
	PersistentTrajectory = Trajectory;
}

void UPlanningVisualizer::ClearPersistentData()
{
	PersistentPath.Empty();
	PersistentTrajectory.Clear();
	bHasPersistentAvoidance = false;
}

void UPlanningVisualizer::DrawLocalAvoidance(const FVector& Position, const FLocalAvoidanceResult& Result, float ForceScale)
{
	if (!bEnableVisualization || !bShowLocalAvoidance || !GetWorld())
	{
		return;
	}

	// 绘制引力向量（绿色）
	if (!Result.AttractiveForce.IsNearlyZero())
	{
		FVector AttEnd = Position + Result.AttractiveForce * ForceScale;
		DrawDebugDirectionalArrow(GetWorld(), Position, AttEnd, 20.0f, FColor::Green, false, -1.0f, 0, 2.0f);
	}

	// 绘制斥力向量（红色）
	if (!Result.RepulsiveForce.IsNearlyZero())
	{
		FVector RepEnd = Position + Result.RepulsiveForce * ForceScale;
		DrawDebugDirectionalArrow(GetWorld(), Position, RepEnd, 20.0f, FColor::Red, false, -1.0f, 0, 2.0f);
	}

	// 绘制合力向量（黄色，加粗）
	if (!Result.TotalForce.IsNearlyZero())
	{
		FVector TotalEnd = Position + Result.TotalForce * ForceScale;
		DrawDebugDirectionalArrow(GetWorld(), Position, TotalEnd, 25.0f, FColor::Yellow, false, -1.0f, 0, 3.0f);
	}

	// 绘制修正方向（青色，较长箭头表示实际飞行方向）
	if (Result.bNeedsCorrection && !Result.CorrectedDirection.IsNearlyZero())
	{
		FVector CorrEnd = Position + Result.CorrectedDirection * 200.0f;
		DrawDebugDirectionalArrow(GetWorld(), Position, CorrEnd, 30.0f, FColor::Cyan, false, -1.0f, 0, 2.5f);
	}

	// Stuck 状态标记（紫色闪烁球）
	if (Result.bStuck)
	{
		DrawDebugSphere(GetWorld(), Position, 50.0f, 8, FColor::Purple, false, -1.0f, 0, 3.0f);
	}
}

void UPlanningVisualizer::SetPersistentLocalAvoidance(const FVector& Position, const FLocalAvoidanceResult& Result)
{
	PersistentAvoidancePosition = Position;
	PersistentAvoidanceResult = Result;
	bHasPersistentAvoidance = true;
}

void UPlanningVisualizer::ClearPersistentLocalAvoidance()
{
	bHasPersistentAvoidance = false;
}

void UPlanningVisualizer::DrawPersistentData()
{
	if (PersistentPath.Num() >= 2)
	{
		DrawPath(PersistentPath, PathColor, -1.0f, LineThickness);
	}

	if (PersistentTrajectory.bIsValid && PersistentTrajectory.Points.Num() >= 2)
	{
		DrawTrajectory(PersistentTrajectory, TrajectoryColor, -1.0f, false);
	}

	if (bHasPersistentAvoidance)
	{
		DrawLocalAvoidance(PersistentAvoidancePosition, PersistentAvoidanceResult);
	}
}
