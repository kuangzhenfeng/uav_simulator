// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "PathPlanner.h"
#include "AStarPathPlanner.generated.h"

/**
 * A*网格节点
 */
struct FAStarNode
{
	FIntVector GridCoord;	// 网格坐标
	FVector WorldPos;		// 世界坐标
	float GCost;			// 从起点到当前节点的代价
	float HCost;			// 从当前节点到目标的启发式代价
	float FCost() const { return GCost + HCost; }	// 总代价
	FAStarNode* Parent;		// 父节点

	FAStarNode()
		: GridCoord(FIntVector::ZeroValue)
		, WorldPos(FVector::ZeroVector)
		, GCost(FLT_MAX)
		, HCost(0.0f)
		, Parent(nullptr)
	{}

	FAStarNode(const FIntVector& InGridCoord, const FVector& InWorldPos)
		: GridCoord(InGridCoord)
		, WorldPos(InWorldPos)
		, GCost(FLT_MAX)
		, HCost(0.0f)
		, Parent(nullptr)
	{}

	bool operator==(const FAStarNode& Other) const
	{
		return GridCoord == Other.GridCoord;
	}
};

/**
 * A*路径规划器
 * 使用3D网格化搜索空间实现A*算法
 */
UCLASS(ClassGroup=(Planning), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UAStarPathPlanner : public UPathPlanner
{
	GENERATED_BODY()

public:
	UAStarPathPlanner();

	// 重写路径规划接口
	virtual bool PlanPath(const FVector& Start, const FVector& Goal, TArray<FVector>& OutPath) override;

	/**
	 * 设置搜索边界
	 * @param InMinBounds 最小边界
	 * @param InMaxBounds 最大边界
	 */
	UFUNCTION(BlueprintCallable, Category = "A* Path Planning")
	void SetSearchBounds(const FVector& InMinBounds, const FVector& InMaxBounds);

	/**
	 * 自动计算搜索边界
	 * @param Start 起点
	 * @param Goal 终点
	 * @param Padding 边界填充
	 */
	UFUNCTION(BlueprintCallable, Category = "A* Path Planning")
	void AutoComputeSearchBounds(const FVector& Start, const FVector& Goal, float Padding = 1000.0f);

protected:
	// 网格分辨率 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "A* Settings")
	float GridResolution = 50.0f;

	// 是否允许对角线移动
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "A* Settings")
	bool bAllowDiagonalMove = true;

	// 是否允许3D对角线移动
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "A* Settings")
	bool bAllow3DDiagonalMove = true;

	// 最大搜索节点数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "A* Settings")
	int32 MaxSearchNodes = 100000;

	// 启发式权重 (>1 更贪婪, <1 更保守)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "A* Settings")
	float HeuristicWeight = 1.0f;

	// 搜索边界
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "A* Settings")
	FVector MinBounds = FVector(-10000.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "A* Settings")
	FVector MaxBounds = FVector(10000.0f);

private:
	// 将世界坐标转换为网格坐标
	FIntVector WorldToGrid(const FVector& WorldPos) const;

	// 将网格坐标转换为世界坐标
	FVector GridToWorld(const FIntVector& GridCoord) const;

	// 计算启发式代价 (欧几里得距离)
	float CalculateHeuristic(const FIntVector& From, const FIntVector& To) const;

	// 获取邻居节点
	TArray<FIntVector> GetNeighbors(const FIntVector& GridCoord) const;

	// 检查网格坐标是否有效
	bool IsValidGridCoord(const FIntVector& GridCoord) const;

	// 检查网格点是否被障碍物阻挡
	bool IsGridBlocked(const FIntVector& GridCoord) const;

	// 从节点重建路径
	void ReconstructPath(FAStarNode* EndNode, TArray<FVector>& OutPath) const;

	// 用于节点查找的哈希函数
	uint32 GetGridHash(const FIntVector& GridCoord) const;

	// 网格边界（网格坐标）
	FIntVector MinGridBounds;
	FIntVector MaxGridBounds;
};
