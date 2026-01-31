// Copyright Epic Games, Inc. All Rights Reserved.

#include "RRTPathPlanner.h"
#include "DrawDebugHelpers.h"

URRTPathPlanner::URRTPathPlanner()
{
	StepSize = 100.0f;
	MaxIterations = 5000;
	GoalBias = 0.1f;
	GoalThreshold = 100.0f;
	bUseRRTStar = false;
	NeighborhoodRadius = 200.0f;
}

bool URRTPathPlanner::PlanPath(const FVector& Start, const FVector& Goal, TArray<FVector>& OutPath)
{
	double StartTime = FPlatformTime::Seconds();

	OutPath.Empty();
	LastPath.Empty();
	Tree.Empty();

	GoalPosition = Goal;

	// 自动计算搜索边界
	AutoComputeSearchBounds(Start, Goal, 500.0f);

	// 检查起点和终点是否有效
	if (CheckCollision(Start, 0.0f))
	{
		UE_LOG(LogTemp, Warning, TEXT("RRT PathPlanner: Start position is blocked"));
		return false;
	}

	if (CheckCollision(Goal, 0.0f))
	{
		UE_LOG(LogTemp, Warning, TEXT("RRT PathPlanner: Goal position is blocked"));
		return false;
	}

	// 添加起始节点
	Tree.Add(FRRTNode(Start, -1, 0.0f));

	int32 GoalNodeIndex = -1;

	for (int32 Iteration = 0; Iteration < MaxIterations; ++Iteration)
	{
		// 随机采样
		FVector RandomPoint = SampleRandomPoint();

		// 找到最近节点
		int32 NearestIndex = FindNearestNode(RandomPoint);
		if (NearestIndex < 0)
		{
			continue;
		}

		// 向随机点方向扩展
		FVector NewPoint = Steer(Tree[NearestIndex].Position, RandomPoint);

		// 检查新点是否有效
		if (CheckCollision(NewPoint, 0.0f))
		{
			continue;
		}

		// 检查路径是否有效
		if (CheckLineCollision(Tree[NearestIndex].Position, NewPoint, 0.0f))
		{
			continue;
		}

		int32 ParentIndex = NearestIndex;
		float NewCost = Tree[NearestIndex].Cost + FVector::Dist(Tree[NearestIndex].Position, NewPoint);

		// RRT* 优化：选择最佳父节点
		if (bUseRRTStar)
		{
			TArray<int32> NearbyNodes = FindNearbyNodes(NewPoint, NeighborhoodRadius);
			int32 BestParent = ChooseBestParent(NewPoint, NearbyNodes);
			if (BestParent >= 0)
			{
				ParentIndex = BestParent;
				NewCost = Tree[ParentIndex].Cost + FVector::Dist(Tree[ParentIndex].Position, NewPoint);
			}
		}

		// 添加新节点
		int32 NewNodeIndex = Tree.Add(FRRTNode(NewPoint, ParentIndex, NewCost));

		// RRT* 优化：重新连线
		if (bUseRRTStar)
		{
			TArray<int32> NearbyNodes = FindNearbyNodes(NewPoint, NeighborhoodRadius);
			Rewire(NewNodeIndex, NearbyNodes);
		}

		// 检查是否到达目标
		float DistToGoal = FVector::Dist(NewPoint, Goal);
		if (DistToGoal <= GoalThreshold)
		{
			// 尝试直接连接到目标
			if (!CheckLineCollision(NewPoint, Goal, 0.0f))
			{
				float FinalCost = NewCost + DistToGoal;
				GoalNodeIndex = Tree.Add(FRRTNode(Goal, NewNodeIndex, FinalCost));
				break;
			}
		}
	}

	bool bSuccess = (GoalNodeIndex >= 0);

	if (bSuccess)
	{
		// 重建路径
		ReconstructPath(GoalNodeIndex, OutPath);

		// 简化路径
		SimplifyPath(OutPath);

		LastPath = OutPath;
	}

	double EndTime = FPlatformTime::Seconds();
	LastPlanningTimeMs = (EndTime - StartTime) * 1000.0;

	if (bShowDebug)
	{
		UE_LOG(LogTemp, Log, TEXT("RRT PathPlanner: %s, Tree size: %d, Path length: %d, Time: %.2f ms"),
			bSuccess ? TEXT("Success") : TEXT("Failed"),
			Tree.Num(),
			OutPath.Num(),
			LastPlanningTimeMs);
	}

	return bSuccess;
}

void URRTPathPlanner::SetSearchBounds(const FVector& InMinBounds, const FVector& InMaxBounds)
{
	MinBounds = InMinBounds;
	MaxBounds = InMaxBounds;
}

void URRTPathPlanner::AutoComputeSearchBounds(const FVector& Start, const FVector& Goal, float Padding)
{
	FVector Min, Max;
	Min.X = FMath::Min(Start.X, Goal.X) - Padding;
	Min.Y = FMath::Min(Start.Y, Goal.Y) - Padding;
	Min.Z = FMath::Min(Start.Z, Goal.Z) - Padding;
	Max.X = FMath::Max(Start.X, Goal.X) + Padding;
	Max.Y = FMath::Max(Start.Y, Goal.Y) + Padding;
	Max.Z = FMath::Max(Start.Z, Goal.Z) + Padding;

	SetSearchBounds(Min, Max);
}

FVector URRTPathPlanner::SampleRandomPoint() const
{
	// 目标偏置采样
	if (FMath::FRand() < GoalBias)
	{
		return GoalPosition;
	}

	// 在边界内随机采样
	return FVector(
		FMath::RandRange(MinBounds.X, MaxBounds.X),
		FMath::RandRange(MinBounds.Y, MaxBounds.Y),
		FMath::RandRange(MinBounds.Z, MaxBounds.Z)
	);
}

int32 URRTPathPlanner::FindNearestNode(const FVector& Point) const
{
	int32 NearestIndex = -1;
	float NearestDist = FLT_MAX;

	for (int32 i = 0; i < Tree.Num(); ++i)
	{
		float Dist = FVector::DistSquared(Tree[i].Position, Point);
		if (Dist < NearestDist)
		{
			NearestDist = Dist;
			NearestIndex = i;
		}
	}

	return NearestIndex;
}

FVector URRTPathPlanner::Steer(const FVector& From, const FVector& To) const
{
	FVector Direction = To - From;
	float Distance = Direction.Size();

	if (Distance <= StepSize)
	{
		return To;
	}

	Direction.Normalize();
	return From + Direction * StepSize;
}

TArray<int32> URRTPathPlanner::FindNearbyNodes(const FVector& Point, float Radius) const
{
	TArray<int32> NearbyNodes;
	float RadiusSquared = Radius * Radius;

	for (int32 i = 0; i < Tree.Num(); ++i)
	{
		if (FVector::DistSquared(Tree[i].Position, Point) <= RadiusSquared)
		{
			NearbyNodes.Add(i);
		}
	}

	return NearbyNodes;
}

int32 URRTPathPlanner::ChooseBestParent(const FVector& NewPoint, const TArray<int32>& NearbyNodes) const
{
	int32 BestParent = -1;
	float BestCost = FLT_MAX;

	for (int32 NodeIndex : NearbyNodes)
	{
		float EdgeCost = FVector::Dist(Tree[NodeIndex].Position, NewPoint);
		float TotalCost = Tree[NodeIndex].Cost + EdgeCost;

		if (TotalCost < BestCost)
		{
			// 检查路径是否有效
			if (!CheckLineCollision(Tree[NodeIndex].Position, NewPoint, 0.0f))
			{
				BestCost = TotalCost;
				BestParent = NodeIndex;
			}
		}
	}

	return BestParent;
}

void URRTPathPlanner::Rewire(int32 NewNodeIndex, const TArray<int32>& NearbyNodes)
{
	const FRRTNode& NewNode = Tree[NewNodeIndex];

	for (int32 NodeIndex : NearbyNodes)
	{
		if (NodeIndex == NewNodeIndex || NodeIndex == NewNode.ParentIndex)
		{
			continue;
		}

		float EdgeCost = FVector::Dist(NewNode.Position, Tree[NodeIndex].Position);
		float NewCost = NewNode.Cost + EdgeCost;

		if (NewCost < Tree[NodeIndex].Cost)
		{
			// 检查路径是否有效
			if (!CheckLineCollision(NewNode.Position, Tree[NodeIndex].Position, 0.0f))
			{
				Tree[NodeIndex].ParentIndex = NewNodeIndex;
				Tree[NodeIndex].Cost = NewCost;
			}
		}
	}
}

void URRTPathPlanner::ReconstructPath(int32 GoalNodeIndex, TArray<FVector>& OutPath) const
{
	TArray<FVector> ReversePath;
	int32 CurrentIndex = GoalNodeIndex;

	while (CurrentIndex >= 0)
	{
		ReversePath.Add(Tree[CurrentIndex].Position);
		CurrentIndex = Tree[CurrentIndex].ParentIndex;
	}

	// 反转路径
	for (int32 i = ReversePath.Num() - 1; i >= 0; --i)
	{
		OutPath.Add(ReversePath[i]);
	}
}
