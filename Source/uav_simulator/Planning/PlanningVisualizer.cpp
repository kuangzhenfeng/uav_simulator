// Copyright Epic Games, Inc. All Rights Reserved.

#include "PlanningVisualizer.h"
#include "../uav_simulator.h"
#include "DrawDebugHelpers.h"
#include "../Debug/DebugDrawBuffer.h"

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
	SCOPE_CYCLE_COUNTER(STAT_PlanningVisualizer);

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

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);
	int32 AgentID = -1;

	// 绘制路径线段
	for (int32 i = 1; i < Path.Num(); ++i)
	{
		Buffer->AddLine(GetWorld(), Path[i - 1], Path[i], Color, Thickness, Duration, AgentID, TEXT("path"));
	}

	// 绘制路径点
	for (int32 i = 0; i < Path.Num(); ++i)
	{
		float PointSize = (i == 0 || i == Path.Num() - 1) ? 15.0f : 8.0f;
		Buffer->AddPoint(GetWorld(), Path[i], PointSize, Color, Duration, AgentID, TEXT("path"));
	}

	// 绘制起点和终点标记
	if (Path.Num() > 0)
	{
		Buffer->AddSphere(GetWorld(), Path[0], 25.0f, FColor::Green, Duration, AgentID, TEXT("path"));
		Buffer->AddSphere(GetWorld(), Path.Last(), 25.0f, FColor::Red, Duration, AgentID, TEXT("path"));
	}
}

void UPlanningVisualizer::DrawTrajectory(const FTrajectory& Trajectory, FColor Color, float Duration, bool bShowVelocity)
{
	if (!bEnableVisualization || !bShowTrajectory || !GetWorld() || !Trajectory.bIsValid || Trajectory.Points.Num() < 2)
	{
		return;
	}

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);
	int32 AgentID = -1;

	// 绘制轨迹曲线
	for (int32 i = 1; i < Trajectory.Points.Num(); ++i)
	{
		Buffer->AddLine(GetWorld(), Trajectory.Points[i - 1].Position, Trajectory.Points[i].Position,
			Color, LineThickness, Duration, AgentID, TEXT("trajectory"));
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
				Buffer->AddArrow(GetWorld(), Point.Position, VelocityEnd, 30.0f,
					FColor::Yellow, 1.5f, Duration, AgentID, TEXT("trajectory"));
			}
		}
	}

	// 绘制起点和终点
	Buffer->AddSphere(GetWorld(), Trajectory.Points[0].Position, 20.0f, FColor::Green, Duration, AgentID, TEXT("trajectory"));
	Buffer->AddSphere(GetWorld(), Trajectory.Points.Last().Position, 20.0f, FColor::Red, Duration, AgentID, TEXT("trajectory"));
}

void UPlanningVisualizer::DrawTrackingPoint(const FTrajectoryPoint& Point, float Radius, FColor Color)
{
	if (!bEnableVisualization || !GetWorld())
	{
		return;
	}

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);
	int32 AgentID = -1;

	// 绘制当前位置球体
	Buffer->AddSphere(GetWorld(), Point.Position, Radius, Color, -1.0f, AgentID, TEXT("tracking"));

	// 绘制速度方向
	if (!Point.Velocity.IsNearlyZero())
	{
		FVector VelocityEnd = Point.Position + Point.Velocity.GetSafeNormal() * (Radius * 3.0f);
		Buffer->AddArrow(GetWorld(), Point.Position, VelocityEnd, 20.0f, FColor::Cyan, 2.0f, -1.0f, AgentID, TEXT("tracking"));
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

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);
	int32 AgentID = -1;

	FColor SafetyColor = FColor(Color.R, Color.G, Color.B, 100);

	switch (Obstacle.Type)
	{
	case EObstacleType::Sphere:
		Buffer->AddSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.X, Color, Duration, AgentID, TEXT("obstacle"));
		// 绘制安全边距
		Buffer->AddSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.X + Obstacle.SafetyMargin, SafetyColor, Duration, AgentID, TEXT("obstacle"));
		break;

	case EObstacleType::Box:
		Buffer->AddBox(GetWorld(), Obstacle.Center, Obstacle.Extents, Obstacle.Rotation.Quaternion(),
			Color, Duration, AgentID, TEXT("obstacle"));
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

				Buffer->AddLine(GetWorld(), P1Top, P2Top, Color, 2.0f, Duration, AgentID, TEXT("obstacle"));
				Buffer->AddLine(GetWorld(), P1Bottom, P2Bottom, Color, 2.0f, Duration, AgentID, TEXT("obstacle"));
				Buffer->AddLine(GetWorld(), P1Top, P1Bottom, Color, 2.0f, Duration, AgentID, TEXT("obstacle"));
			}
		}
		break;

	default:
		Buffer->AddSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.GetMax(), Color, Duration, AgentID, TEXT("obstacle"));
		break;
	}
}

void UPlanningVisualizer::DrawSearchBounds(const FVector& MinBounds, const FVector& MaxBounds, FColor Color, float Duration)
{
	if (!bEnableVisualization || !GetWorld())
	{
		return;
	}

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);
	int32 AgentID = -1;

	FVector Center = (MinBounds + MaxBounds) * 0.5f;
	FVector Extent = (MaxBounds - MinBounds) * 0.5f;

	Buffer->AddBox(GetWorld(), Center, Extent, FQuat::Identity, Color, Duration, AgentID, TEXT("search_bounds"));
}

void UPlanningVisualizer::DrawWaypoints(const TArray<FVector>& Waypoints, float Radius, FColor Color, float Duration)
{
	if (!bEnableVisualization || !bShowWaypoints || !GetWorld())
	{
		return;
	}

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);
	int32 AgentID = -1;

	for (int32 i = 0; i < Waypoints.Num(); ++i)
	{
		Buffer->AddSphere(GetWorld(), Waypoints[i], Radius, Color, Duration, AgentID, TEXT("waypoint"));

		// 绘制航点编号
		FString Label = FString::Printf(TEXT("%d"), i);
		Buffer->AddText(GetWorld(), Waypoints[i] + FVector(0, 0, Radius + 10.0f), Label, Color, Duration, AgentID, TEXT("waypoint"));
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

bool UPlanningVisualizer::GetPersistentPath(TArray<FVector>& OutPath) const
{
	if (PersistentPath.Num() < 2)
	{
		return false;
	}
	OutPath = PersistentPath;
	return true;
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

void UPlanningVisualizer::DrawNMPCPrediction(const FVector& Position, const FNMPCAvoidanceResult& Result)
{
	if (!bEnableVisualization || !bShowNMPCPrediction || !GetWorld())
	{
		return;
	}

	UDebugDrawBuffer* Buffer = UDebugDrawBuffer::Get(this);
	int32 AgentID = -1;

	// 绘制预测轨迹线（橙色）
	if (Result.PredictedTrajectory.Num() >= 2)
	{
		for (int32 i = 1; i < Result.PredictedTrajectory.Num(); ++i)
		{
			const FNMPCPredictionStep& Prev = Result.PredictedTrajectory[i - 1];
			const FNMPCPredictionStep& Curr = Result.PredictedTrajectory[i];
			Buffer->AddLine(GetWorld(), Prev.Position, Curr.Position, FColor::Orange, 2.5f, -1.0f, AgentID, TEXT("nmpc"));

			// 障碍物代价越高，球体越红越大
			float CostRatio = FMath::Clamp(Curr.ObstacleCost / 10.0f, 0.0f, 1.0f);
			uint8 R = FMath::Lerp(255, 255, CostRatio);
			uint8 G = FMath::Lerp(165, 0, CostRatio);
			uint8 B = 0;
			float Radius = FMath::Lerp(5.0f, 20.0f, CostRatio);
			Buffer->AddPoint(GetWorld(), Curr.Position, Radius, FColor(R, G, B), -1.0f, AgentID, TEXT("nmpc"));
		}

		// 绘制控制输入箭头（青色，每隔几步显示一次）
		int32 Step = FMath::Max(1, Result.PredictedTrajectory.Num() / 5);
		for (int32 i = 0; i < Result.PredictedTrajectory.Num(); i += Step)
		{
			const FNMPCPredictionStep& PredStep = Result.PredictedTrajectory[i];
			if (!PredStep.ControlInput.IsNearlyZero())
			{
				FVector ArrowEnd = PredStep.Position + PredStep.ControlInput * 0.5f;
				Buffer->AddArrow(GetWorld(), PredStep.Position, ArrowEnd, 15.0f, FColor::Cyan, 1.5f, -1.0f, AgentID, TEXT("nmpc"));
			}
		}
	}

	// 绘制修正目标（黄色球）
	if (Result.bNeedsCorrection)
	{
		Buffer->AddSphere(GetWorld(), Result.CorrectedTarget, 25.0f, FColor::Yellow, -1.0f, AgentID, TEXT("nmpc"));
		Buffer->AddArrow(GetWorld(), Position, Result.CorrectedTarget, 25.0f, FColor::Yellow, 2.5f, -1.0f, AgentID, TEXT("nmpc"));
	}

	// Stuck 状态标记（紫色球）
	if (Result.bStuck)
	{
		Buffer->AddSphere(GetWorld(), Position, 50.0f, FColor::Purple, -1.0f, AgentID, TEXT("nmpc"));
	}
}

void UPlanningVisualizer::SetPersistentNMPCPrediction(const FVector& Position, const FNMPCAvoidanceResult& Result)
{
	PersistentAvoidancePosition = Position;
	PersistentAvoidanceResult = Result;
	bHasPersistentAvoidance = true;
}

void UPlanningVisualizer::ClearPersistentNMPCPrediction()
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
		DrawNMPCPrediction(PersistentAvoidancePosition, PersistentAvoidanceResult);
	}
}
