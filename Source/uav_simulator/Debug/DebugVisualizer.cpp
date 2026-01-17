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
