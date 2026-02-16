// Copyright Epic Games, Inc. All Rights Reserved.

#include "AStarPathPlanner.h"
#include "DrawDebugHelpers.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UAStarPathPlanner::UAStarPathPlanner()
{
	GridResolution = 50.0f;
}

bool UAStarPathPlanner::PlanPath(const FVector& Start, const FVector& Goal, TArray<FVector>& OutPath)
{
	double StartTime = FPlatformTime::Seconds();

	OutPath.Empty();
	LastPath.Empty();

	UE_LOG(LogUAVPlanning, Warning, TEXT("===== A* Path Planning Started ====="));
	UE_LOG(LogUAVPlanning, Warning, TEXT("Start: %s, Goal: %s"), *Start.ToString(), *Goal.ToString());
	UE_LOG(LogUAVPlanning, Warning, TEXT("Obstacle count: %d, Grid resolution: %.1f, UAVCollisionRadius: %.1f, SafetyMargin: %.1f"),
		Obstacles.Num(), GridResolution, UAVCollisionRadius, PlanningConfig.SafetyMargin);

	// 打印所有障碍物详细信息
	for (int32 i = 0; i < Obstacles.Num(); ++i)
	{
		const FObstacleInfo& Obs = Obstacles[i];
		UE_LOG(LogUAVPlanning, Warning, TEXT("  Obstacle[%d]: ID=%d, Type=%d, Center=%s, Extents=%s, SafetyMargin=%.1f"),
			i, Obs.ObstacleID, (int32)Obs.Type, *Obs.Center.ToString(), *Obs.Extents.ToString(), Obs.SafetyMargin);
	}

	// 自动计算搜索边界
	AutoComputeSearchBounds(Start, Goal, 500.0f);

	// 转换起点和终点到网格坐标
	FIntVector StartGrid = WorldToGrid(Start);
	FIntVector GoalGrid = WorldToGrid(Goal);

	UE_LOG(LogUAVPlanning, Log, TEXT("Start grid: (%d,%d,%d), Goal grid: (%d,%d,%d)"),
		StartGrid.X, StartGrid.Y, StartGrid.Z, GoalGrid.X, GoalGrid.Y, GoalGrid.Z);

	// 检查起点和终点是否有效
	if (!IsValidGridCoord(StartGrid) || IsGridBlocked(StartGrid))
	{
		UE_LOG(LogUAVPlanning, Error, TEXT("Start position invalid or blocked! IsValid=%s, IsBlocked=%s"),
			IsValidGridCoord(StartGrid) ? TEXT("true") : TEXT("false"),
			IsGridBlocked(StartGrid) ? TEXT("true") : TEXT("false"));
		return false;
	}

	if (!IsValidGridCoord(GoalGrid) || IsGridBlocked(GoalGrid))
	{
		UE_LOG(LogUAVPlanning, Error, TEXT("Goal position invalid or blocked! IsValid=%s, IsBlocked=%s"),
			IsValidGridCoord(GoalGrid) ? TEXT("true") : TEXT("false"),
			IsGridBlocked(GoalGrid) ? TEXT("true") : TEXT("false"));
		return false;
	}

	// 节点存储
	TMap<uint32, FAStarNode*> AllNodes;
	TArray<FAStarNode*> OpenList;
	TSet<uint32> ClosedSet;

	// 创建起始节点
	FAStarNode* StartNode = new FAStarNode(StartGrid, GridToWorld(StartGrid));
	StartNode->GCost = 0.0f;
	StartNode->HCost = CalculateHeuristic(StartGrid, GoalGrid);
	AllNodes.Add(GetGridHash(StartGrid), StartNode);
	OpenList.Add(StartNode);

	int32 NodesExplored = 0;
	FAStarNode* GoalNode = nullptr;

	while (OpenList.Num() > 0 && NodesExplored < MaxSearchNodes)
	{
		// 找到F值最小的节点
		int32 BestIndex = 0;
		float BestFCost = OpenList[0]->FCost();
		for (int32 i = 1; i < OpenList.Num(); ++i)
		{
			float FCost = OpenList[i]->FCost();
			if (FCost < BestFCost || (FCost == BestFCost && OpenList[i]->HCost < OpenList[BestIndex]->HCost))
			{
				BestFCost = FCost;
				BestIndex = i;
			}
		}

		FAStarNode* CurrentNode = OpenList[BestIndex];
		OpenList.RemoveAt(BestIndex);

		uint32 CurrentHash = GetGridHash(CurrentNode->GridCoord);
		ClosedSet.Add(CurrentHash);
		NodesExplored++;

		// 检查是否到达目标
		if (CurrentNode->GridCoord == GoalGrid)
		{
			GoalNode = CurrentNode;
			break;
		}

		// 探索邻居
		TArray<FIntVector> Neighbors = GetNeighbors(CurrentNode->GridCoord);
		for (const FIntVector& NeighborCoord : Neighbors)
		{
			uint32 NeighborHash = GetGridHash(NeighborCoord);

			// 跳过已关闭的节点
			if (ClosedSet.Contains(NeighborHash))
			{
				continue;
			}

			// 跳过被阻挡的节点
			if (IsGridBlocked(NeighborCoord))
			{
				continue;
			}

			// 检查移动路径是否被阻挡
			if (CheckLineCollision(CurrentNode->WorldPos, GridToWorld(NeighborCoord), 0.0f))
			{
				continue;
			}

			// 计算移动代价
			FVector NeighborWorld = GridToWorld(NeighborCoord);
			float MoveCost = FVector::Dist(CurrentNode->WorldPos, NeighborWorld);
			float NewGCost = CurrentNode->GCost + MoveCost;

			// 获取或创建邻居节点
			FAStarNode** ExistingNode = AllNodes.Find(NeighborHash);
			FAStarNode* NeighborNode = nullptr;

			if (ExistingNode)
			{
				NeighborNode = *ExistingNode;
				if (NewGCost < NeighborNode->GCost)
				{
					NeighborNode->GCost = NewGCost;
					NeighborNode->Parent = CurrentNode;
				}
			}
			else
			{
				NeighborNode = new FAStarNode(NeighborCoord, NeighborWorld);
				NeighborNode->GCost = NewGCost;
				NeighborNode->HCost = CalculateHeuristic(NeighborCoord, GoalGrid) * HeuristicWeight;
				NeighborNode->Parent = CurrentNode;
				AllNodes.Add(NeighborHash, NeighborNode);
				OpenList.Add(NeighborNode);
			}
		}
	}

	bool bSuccess = (GoalNode != nullptr);

	if (bSuccess)
	{
		// 重建路径
		ReconstructPath(GoalNode, OutPath);

		// 将起点和终点替换为精确位置
		if (OutPath.Num() > 0)
		{
			OutPath[0] = Start;
			OutPath.Last() = Goal;
		}

		// 简化路径
		int32 PathLengthBeforeSimplify = OutPath.Num();
		SimplifyPath(OutPath);
		UE_LOG(LogUAVPlanning, Log, TEXT("Path simplified: %d -> %d points"), PathLengthBeforeSimplify, OutPath.Num());

		LastPath = OutPath;
	}
	else
	{
		UE_LOG(LogUAVPlanning, Error, TEXT("A* path planning failed! NodesExplored: %d, MaxNodes: %d"), NodesExplored, MaxSearchNodes);
	}

	// 清理内存
	for (auto& Pair : AllNodes)
	{
		delete Pair.Value;
	}
	AllNodes.Empty();

	double EndTime = FPlatformTime::Seconds();
	LastPlanningTimeMs = (EndTime - StartTime) * 1000.0;

	UE_LOG(LogUAVPlanning, Warning, TEXT("A* result: %s, NodesExplored: %d, PathPoints: %d, Time: %.2fms"),
		bSuccess ? TEXT("SUCCESS") : TEXT("FAILED"),
		NodesExplored,
		OutPath.Num(),
		LastPlanningTimeMs);

	return bSuccess;
}

void UAStarPathPlanner::SetSearchBounds(const FVector& InMinBounds, const FVector& InMaxBounds)
{
	MinBounds = InMinBounds;
	MaxBounds = InMaxBounds;
	MinGridBounds = WorldToGrid(MinBounds);
	MaxGridBounds = WorldToGrid(MaxBounds);
}

void UAStarPathPlanner::AutoComputeSearchBounds(const FVector& Start, const FVector& Goal, float Padding)
{
	FVector Min, Max;
	Min.X = FMath::Min(Start.X, Goal.X) - Padding;
	Min.Y = FMath::Min(Start.Y, Goal.Y) - Padding;
	Min.Z = FMath::Min(Start.Z, Goal.Z) - Padding;
	Max.X = FMath::Max(Start.X, Goal.X) + Padding;
	Max.Y = FMath::Max(Start.Y, Goal.Y) + Padding;
	Max.Z = FMath::Max(Start.Z, Goal.Z) + Padding;

	UE_LOG(LogUAVPlanning, Log, TEXT("[A*::AutoComputeSearchBounds] MinBounds=%s, MaxBounds=%s, Padding=%.1f"),
		*Min.ToString(), *Max.ToString(), Padding);

	SetSearchBounds(Min, Max);
}

FIntVector UAStarPathPlanner::WorldToGrid(const FVector& WorldPos) const
{
	return FIntVector(
		FMath::FloorToInt(WorldPos.X / GridResolution),
		FMath::FloorToInt(WorldPos.Y / GridResolution),
		FMath::FloorToInt(WorldPos.Z / GridResolution)
	);
}

FVector UAStarPathPlanner::GridToWorld(const FIntVector& GridCoord) const
{
	return FVector(
		(GridCoord.X + 0.5f) * GridResolution,
		(GridCoord.Y + 0.5f) * GridResolution,
		(GridCoord.Z + 0.5f) * GridResolution
	);
}

float UAStarPathPlanner::CalculateHeuristic(const FIntVector& From, const FIntVector& To) const
{
	// 欧几里得距离
	FVector Diff = FVector(To.X - From.X, To.Y - From.Y, To.Z - From.Z);
	return Diff.Size() * GridResolution;
}

TArray<FIntVector> UAStarPathPlanner::GetNeighbors(const FIntVector& GridCoord) const
{
	TArray<FIntVector> Neighbors;

	// 6个正交方向
	const FIntVector Directions6[] = {
		FIntVector(1, 0, 0), FIntVector(-1, 0, 0),
		FIntVector(0, 1, 0), FIntVector(0, -1, 0),
		FIntVector(0, 0, 1), FIntVector(0, 0, -1)
	};

	for (const FIntVector& Dir : Directions6)
	{
		FIntVector Neighbor = GridCoord + Dir;
		if (IsValidGridCoord(Neighbor))
		{
			Neighbors.Add(Neighbor);
		}
	}

	// 2D对角线方向 (12个)
	if (bAllowDiagonalMove)
	{
		const FIntVector Directions12[] = {
			FIntVector(1, 1, 0), FIntVector(1, -1, 0),
			FIntVector(-1, 1, 0), FIntVector(-1, -1, 0),
			FIntVector(1, 0, 1), FIntVector(1, 0, -1),
			FIntVector(-1, 0, 1), FIntVector(-1, 0, -1),
			FIntVector(0, 1, 1), FIntVector(0, 1, -1),
			FIntVector(0, -1, 1), FIntVector(0, -1, -1)
		};

		for (const FIntVector& Dir : Directions12)
		{
			FIntVector Neighbor = GridCoord + Dir;
			if (IsValidGridCoord(Neighbor))
			{
				Neighbors.Add(Neighbor);
			}
		}
	}

	// 3D对角线方向 (8个)
	if (bAllow3DDiagonalMove)
	{
		const FIntVector Directions8[] = {
			FIntVector(1, 1, 1), FIntVector(1, 1, -1),
			FIntVector(1, -1, 1), FIntVector(1, -1, -1),
			FIntVector(-1, 1, 1), FIntVector(-1, 1, -1),
			FIntVector(-1, -1, 1), FIntVector(-1, -1, -1)
		};

		for (const FIntVector& Dir : Directions8)
		{
			FIntVector Neighbor = GridCoord + Dir;
			if (IsValidGridCoord(Neighbor))
			{
				Neighbors.Add(Neighbor);
			}
		}
	}

	return Neighbors;
}

bool UAStarPathPlanner::IsValidGridCoord(const FIntVector& GridCoord) const
{
	return GridCoord.X >= MinGridBounds.X && GridCoord.X <= MaxGridBounds.X &&
		   GridCoord.Y >= MinGridBounds.Y && GridCoord.Y <= MaxGridBounds.Y &&
		   GridCoord.Z >= MinGridBounds.Z && GridCoord.Z <= MaxGridBounds.Z;
}

bool UAStarPathPlanner::IsGridBlocked(const FIntVector& GridCoord) const
{
	FVector WorldPos = GridToWorld(GridCoord);
	return CheckCollision(WorldPos, 0.0f);
}

void UAStarPathPlanner::ReconstructPath(FAStarNode* EndNode, TArray<FVector>& OutPath) const
{
	TArray<FVector> ReversePath;
	FAStarNode* Current = EndNode;

	while (Current != nullptr)
	{
		ReversePath.Add(Current->WorldPos);
		Current = Current->Parent;
	}

	// 反转路径
	for (int32 i = ReversePath.Num() - 1; i >= 0; --i)
	{
		OutPath.Add(ReversePath[i]);
	}
}

uint32 UAStarPathPlanner::GetGridHash(const FIntVector& GridCoord) const
{
	// 简单的哈希函数
	return HashCombine(HashCombine(GetTypeHash(GridCoord.X), GetTypeHash(GridCoord.Y)), GetTypeHash(GridCoord.Z));
}
