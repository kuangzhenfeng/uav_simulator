// Copyright Epic Games, Inc. All Rights Reserved.

#include "DebugVisualizer.h"
#include "DrawDebugHelpers.h"

UDebugVisualizer::UDebugVisualizer()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UDebugVisualizer::BeginPlay()
{
	Super::BeginPlay();
	TrajectoryHistory.Reserve(MaxTrajectoryPoints);
}

void UDebugVisualizer::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// 绘制持久化轨迹
	if (bShowPlannedPath)
	{
		DrawPersistentTrajectory();
	}
}

void UDebugVisualizer::DrawUAVState(const FUAVState& State, const FVector& ActorLocation)
{
	if (!bShowDebugInfo || !GetWorld())
		return;

	// 绘制坐标轴（机体坐标系）
	FQuat Orientation = State.Rotation.Quaternion();
	FVector XAxis = Orientation.RotateVector(FVector(100.0f, 0.0f, 0.0f));
	FVector YAxis = Orientation.RotateVector(FVector(0.0f, 100.0f, 0.0f));
	FVector ZAxis = Orientation.RotateVector(FVector(0.0f, 0.0f, 100.0f));

	DrawDebugLine(GetWorld(), ActorLocation, ActorLocation + XAxis, FColor::Red, false, -1.0f, 0, 3.0f);
	DrawDebugLine(GetWorld(), ActorLocation, ActorLocation + YAxis, FColor::Green, false, -1.0f, 0, 3.0f);
	DrawDebugLine(GetWorld(), ActorLocation, ActorLocation + ZAxis, FColor::Blue, false, -1.0f, 0, 3.0f);

	// 绘制速度矢量
	if (!State.Velocity.IsNearlyZero())
	{
		FVector VelocityEnd = ActorLocation + State.Velocity * 0.5f;
		DrawDebugDirectionalArrow(GetWorld(), ActorLocation, VelocityEnd, 50.0f, FColor::Yellow, false, -1.0f, 0, 2.0f);
	}
}

void UDebugVisualizer::DrawTrajectoryHistory(const FVector& CurrentPosition)
{
	if (!bShowTrajectory || !GetWorld())
		return;

	// 添加当前位置到历史
	TrajectoryHistory.Add(CurrentPosition);

	// 限制历史长度
	if (TrajectoryHistory.Num() > MaxTrajectoryPoints)
	{
		TrajectoryHistory.RemoveAt(0);
	}

	// 绘制轨迹线
	for (int32 i = 1; i < TrajectoryHistory.Num(); i++)
	{
		DrawDebugLine(GetWorld(), TrajectoryHistory[i - 1], TrajectoryHistory[i],
			FColor::Cyan, false, -1.0f, 0, TrajectoryThickness);
	}
}

void UDebugVisualizer::ClearTrajectoryHistory()
{
	TrajectoryHistory.Empty();
}

void UDebugVisualizer::DrawPlannedPath(const TArray<FVector>& Path, FColor Color, float Duration)
{
	if (!bShowPlannedPath || !GetWorld() || Path.Num() < 2)
		return;

	// 绘制路径线段
	for (int32 i = 1; i < Path.Num(); ++i)
	{
		DrawDebugLine(GetWorld(), Path[i - 1], Path[i], Color, false, Duration, 0, 3.0f);
		DrawDebugPoint(GetWorld(), Path[i], 10.0f, Color, false, Duration);
	}

	// 绘制起点和终点
	DrawDebugSphere(GetWorld(), Path[0], 25.0f, 8, FColor::Green, false, Duration, 0, 2.0f);
	DrawDebugSphere(GetWorld(), Path.Last(), 25.0f, 8, FColor::Red, false, Duration, 0, 2.0f);
}

void UDebugVisualizer::DrawOptimizedTrajectory(const FTrajectory& Trajectory, FColor Color, float Duration)
{
	if (!bShowPlannedPath || !GetWorld() || !Trajectory.bIsValid || Trajectory.Points.Num() < 2)
		return;

	// 绘制轨迹曲线
	for (int32 i = 1; i < Trajectory.Points.Num(); ++i)
	{
		DrawDebugLine(GetWorld(), Trajectory.Points[i - 1].Position, Trajectory.Points[i].Position,
			Color, false, Duration, 0, TrajectoryThickness);
	}

	// 每隔一定数量的点绘制速度向量
	int32 VelocityStep = FMath::Max(1, Trajectory.Points.Num() / 15);
	for (int32 i = 0; i < Trajectory.Points.Num(); i += VelocityStep)
	{
		const FTrajectoryPoint& Point = Trajectory.Points[i];
		if (!Point.Velocity.IsNearlyZero())
		{
			FVector VelocityEnd = Point.Position + Point.Velocity * 0.15f;
			DrawDebugDirectionalArrow(GetWorld(), Point.Position, VelocityEnd, 20.0f,
				FColor::Yellow, false, Duration, 0, 1.5f);
		}
	}

	// 绘制起点和终点
	DrawDebugSphere(GetWorld(), Trajectory.Points[0].Position, 20.0f, 8, FColor::Green, false, Duration, 0, 2.0f);
	DrawDebugSphere(GetWorld(), Trajectory.Points.Last().Position, 20.0f, 8, FColor::Red, false, Duration, 0, 2.0f);
}

void UDebugVisualizer::DrawTrackingState(const FTrajectoryPoint& DesiredState, const FVector& CurrentPosition)
{
	if (!bShowTrackingState || !GetWorld())
		return;

	// 绘制期望位置
	DrawDebugSphere(GetWorld(), DesiredState.Position, 15.0f, 8, FColor::Yellow, false, -1.0f, 0, 2.0f);

	// 绘制从当前位置到期望位置的线
	DrawDebugLine(GetWorld(), CurrentPosition, DesiredState.Position, FColor::Orange, false, -1.0f, 0, 1.5f);

	// 绘制期望速度方向
	if (!DesiredState.Velocity.IsNearlyZero())
	{
		FVector VelocityEnd = DesiredState.Position + DesiredState.Velocity.GetSafeNormal() * 50.0f;
		DrawDebugDirectionalArrow(GetWorld(), DesiredState.Position, VelocityEnd, 15.0f,
			FColor::Cyan, false, -1.0f, 0, 2.0f);
	}

	// 显示跟踪误差
	float TrackingError = FVector::Dist(CurrentPosition, DesiredState.Position);
	FString ErrorText = FString::Printf(TEXT("Error: %.1f cm"), TrackingError);
	DrawDebugString(GetWorld(), DesiredState.Position + FVector(0, 0, 30), ErrorText, nullptr,
		TrackingError > 50.0f ? FColor::Red : FColor::Green, -1.0f, true);
}

void UDebugVisualizer::DrawObstacles(const TArray<FObstacleInfo>& Obstacles, FColor Color, float Duration)
{
	if (!GetWorld())
		return;

	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		FColor ObstacleColor = Obstacle.bIsDynamic ? FColor::Orange : Color;

		switch (Obstacle.Type)
		{
		case EObstacleType::Sphere:
			DrawDebugSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.X, 16, ObstacleColor, false, Duration, 0, 2.0f);
			break;

		case EObstacleType::Box:
			DrawDebugBox(GetWorld(), Obstacle.Center, Obstacle.Extents, Obstacle.Rotation.Quaternion(),
				ObstacleColor, false, Duration, 0, 2.0f);
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

					DrawDebugLine(GetWorld(), P1Top, P2Top, ObstacleColor, false, Duration, 0, 2.0f);
					DrawDebugLine(GetWorld(), P1Top, P1Bottom, ObstacleColor, false, Duration, 0, 2.0f);
				}
			}
			break;

		default:
			DrawDebugSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.GetMax(), 16, ObstacleColor, false, Duration, 0, 2.0f);
			break;
		}
	}
}

void UDebugVisualizer::DrawWaypoints(const TArray<FVector>& Waypoints, float Radius, FColor Color, float Duration)
{
	if (!GetWorld())
		return;

	for (int32 i = 0; i < Waypoints.Num(); ++i)
	{
		DrawDebugSphere(GetWorld(), Waypoints[i], Radius, 12, Color, false, Duration, 0, 2.0f);

		// 绘制航点编号
		FString Label = FString::Printf(TEXT("%d"), i);
		DrawDebugString(GetWorld(), Waypoints[i] + FVector(0, 0, Radius + 10.0f), Label, nullptr, Color, Duration);

		// 绘制连接线
		if (i > 0)
		{
			DrawDebugLine(GetWorld(), Waypoints[i - 1], Waypoints[i], Color, false, Duration, 0, 1.5f);
		}
	}
}

void UDebugVisualizer::SetPersistentTrajectory(const FTrajectory& Trajectory)
{
	PersistentTrajectory = Trajectory;
}

void UDebugVisualizer::ClearPersistentTrajectory()
{
	PersistentTrajectory.Clear();
}

void UDebugVisualizer::DrawPersistentTrajectory()
{
	if (!PersistentTrajectory.bIsValid || PersistentTrajectory.Points.Num() < 2)
		return;

	DrawOptimizedTrajectory(PersistentTrajectory, FColor::Blue, -1.0f);
}
