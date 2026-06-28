// Copyright Epic Games, Inc. All Rights Reserved.

#include "DebugVisualizer.h"
#include "DebugDrawBuffer.h"
#include "../uav_simulator.h"
#include "../Core/UAVPawn.h"
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
	SCOPE_CYCLE_COUNTER(STAT_DebugVisualizer);

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

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);

	// body_axes / velocity 都派生自飞机当前位置, 携带 AgentID 让前端绑到 agent.group
	const AUAVPawn* OwnerPawn = Cast<AUAVPawn>(GetOwner());
	const int32 AgentID = OwnerPawn ? OwnerPawn->GetAgentID() : -1;

	// 绘制坐标轴（机体坐标系）
	FQuat Orientation = State.Rotation.Quaternion();
	FVector XAxis = Orientation.RotateVector(FVector(100.0f, 0.0f, 0.0f));
	FVector YAxis = Orientation.RotateVector(FVector(0.0f, 100.0f, 0.0f));
	FVector ZAxis = Orientation.RotateVector(FVector(0.0f, 0.0f, 100.0f));

	Buffer->AddLine(GetWorld(), ActorLocation, ActorLocation + XAxis, FColor::Red, 3.0f, -1.0f, AgentID, TEXT("body_axes"));
	Buffer->AddLine(GetWorld(), ActorLocation, ActorLocation + YAxis, FColor::Green, 3.0f, -1.0f, AgentID, TEXT("body_axes"));
	Buffer->AddLine(GetWorld(), ActorLocation, ActorLocation + ZAxis, FColor::Blue, 3.0f, -1.0f, AgentID, TEXT("body_axes"));

	// 绘制速度矢量
	if (!State.Velocity.IsNearlyZero())
	{
		FVector VelocityEnd = ActorLocation + State.Velocity * 0.5f;
		Buffer->AddArrow(GetWorld(), ActorLocation, VelocityEnd, 50.0f, FColor::Yellow, 2.0f, -1.0f, AgentID, TEXT("velocity"));
	}
}

void UDebugVisualizer::DrawTrajectoryHistory(const FVector& CurrentPosition)
{
	if (!bShowTrajectory || !GetWorld())
		return;

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);

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
		Buffer->AddLine(GetWorld(), TrajectoryHistory[i - 1], TrajectoryHistory[i],
			FColor::Cyan, TrajectoryThickness, -1.0f, -1, TEXT("history_trail"));
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

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);

	// 绘制路径线段
	for (int32 i = 1; i < Path.Num(); ++i)
	{
		Buffer->AddLine(GetWorld(), Path[i - 1], Path[i], Color, 3.0f, Duration, -1, TEXT("planner_path"));
		Buffer->AddPoint(GetWorld(), Path[i], 10.0f, Color, Duration, -1, TEXT("planner_path"));
	}

	// 绘制起点和终点
	Buffer->AddSphere(GetWorld(), Path[0], 25.0f, FColor::Green, Duration, -1, TEXT("planner_path"));
	Buffer->AddSphere(GetWorld(), Path.Last(), 25.0f, FColor::Red, Duration, -1, TEXT("planner_path"));
}

void UDebugVisualizer::DrawOptimizedTrajectory(const FTrajectory& Trajectory, FColor Color, float Duration)
{
	if (!bShowPlannedPath || !GetWorld() || !Trajectory.bIsValid || Trajectory.Points.Num() < 2)
		return;

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);

	// 绘制轨迹曲线
	for (int32 i = 1; i < Trajectory.Points.Num(); ++i)
	{
		Buffer->AddLine(GetWorld(), Trajectory.Points[i - 1].Position, Trajectory.Points[i].Position,
			Color, TrajectoryThickness, Duration, -1, TEXT("trajectory"));
	}

	// 每隔一定数量的点绘制速度向量
	int32 VelocityStep = FMath::Max(1, Trajectory.Points.Num() / 15);
	for (int32 i = 0; i < Trajectory.Points.Num(); i += VelocityStep)
	{
		const FTrajectoryPoint& Point = Trajectory.Points[i];
		if (!Point.Velocity.IsNearlyZero())
		{
			FVector VelocityEnd = Point.Position + Point.Velocity * 0.15f;
			Buffer->AddArrow(GetWorld(), Point.Position, VelocityEnd, 20.0f,
				FColor::Yellow, 1.5f, Duration, -1, TEXT("trajectory"));
		}
	}

	// 绘制起点和终点
	Buffer->AddSphere(GetWorld(), Trajectory.Points[0].Position, 20.0f, FColor::Green, Duration, -1, TEXT("trajectory"));
	Buffer->AddSphere(GetWorld(), Trajectory.Points.Last().Position, 20.0f, FColor::Red, Duration, -1, TEXT("trajectory"));
}

void UDebugVisualizer::DrawTrackingState(const FTrajectoryPoint& DesiredState, const FVector& CurrentPosition)
{
	if (!bShowTrackingState || !GetWorld())
		return;

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);

	// 绘制期望位置
	Buffer->AddSphere(GetWorld(), DesiredState.Position, 15.0f, FColor::Yellow, -1.0f, -1, TEXT("tracking"));

	// 绘制从当前位置到期望位置的线
	Buffer->AddLine(GetWorld(), CurrentPosition, DesiredState.Position, FColor::Orange, 1.5f, -1.0f, -1, TEXT("tracking"));

	// 绘制期望速度方向
	if (!DesiredState.Velocity.IsNearlyZero())
	{
		FVector VelocityEnd = DesiredState.Position + DesiredState.Velocity.GetSafeNormal() * 50.0f;
		Buffer->AddArrow(GetWorld(), DesiredState.Position, VelocityEnd, 15.0f,
			FColor::Cyan, 2.0f, -1.0f, -1, TEXT("tracking"));
	}

	// 显示跟踪误差（降低文字绘制频率）
	constexpr float TextDrawInterval = 0.5f;
	TrackingTextTimer += GetWorld()->GetDeltaSeconds();
	if (TrackingTextTimer >= TextDrawInterval)
	{
		TrackingTextTimer = 0.f;
		float TrackingError = FVector::Dist(CurrentPosition, DesiredState.Position);
		FString ErrorText = FString::Printf(TEXT("Error: %.1f cm"), TrackingError);
		// 携带 AgentID 让可视化前端把文本绑定到对应飞机 (sprite 作为子节点跟随)
		const int32 AgentID = [this]() -> int32 {
			if (const AUAVPawn* Pawn = Cast<AUAVPawn>(GetOwner()))
			{
				return Pawn->GetAgentID();
			}
			return -1;
		}();
		Buffer->AddText(GetWorld(), DesiredState.Position + FVector(0, 0, 30), ErrorText,
			TrackingError > 50.0f ? FColor::Red : FColor::Green, TextDrawInterval, AgentID, TEXT("label"));
	}
}

void UDebugVisualizer::DrawObstacles(const TArray<FObstacleInfo>& Obstacles, FColor Color, float Duration)
{
	if (!GetWorld())
		return;

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);

	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		FColor ObstacleColor = Obstacle.bIsDynamic ? FColor::Orange : Color;

		switch (Obstacle.Type)
		{
		case EObstacleType::Sphere:
			Buffer->AddSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.X, ObstacleColor, Duration, -1, TEXT("obstacle"));
			break;

		case EObstacleType::Box:
			Buffer->AddBox(GetWorld(), Obstacle.Center, Obstacle.Extents, Obstacle.Rotation.Quaternion(),
				ObstacleColor, Duration, -1, TEXT("obstacle"));
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

					Buffer->AddLine(GetWorld(), P1Top, P2Top, ObstacleColor, 2.0f, Duration, -1, TEXT("obstacle"));
					Buffer->AddLine(GetWorld(), P1Top, P1Bottom, ObstacleColor, 2.0f, Duration, -1, TEXT("obstacle"));
				}
			}
			break;

		default:
			Buffer->AddSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.GetMax(), ObstacleColor, Duration, -1, TEXT("obstacle"));
			break;
		}
	}
}

void UDebugVisualizer::DrawWaypoints(const TArray<FVector>& Waypoints, float Radius, FColor Color, float Duration)
{
	if (!GetWorld())
		return;

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);

	for (int32 i = 0; i < Waypoints.Num(); ++i)
	{
		Buffer->AddSphere(GetWorld(), Waypoints[i], Radius, Color, Duration, -1, TEXT("waypoint"));

		// 绘制航点编号
		FString Label = FString::Printf(TEXT("%d"), i);
		Buffer->AddText(GetWorld(), Waypoints[i] + FVector(0, 0, Radius + 10.0f), Label, Color, Duration, -1, TEXT("waypoint"));

		// 绘制连接线
		if (i > 0)
		{
			Buffer->AddLine(GetWorld(), Waypoints[i - 1], Waypoints[i], Color, 1.5f, Duration, -1, TEXT("waypoint"));
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
