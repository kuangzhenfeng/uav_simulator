// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "PathPlanner.h"
#include "RRTPathPlanner.generated.h"

/**
 * RRT树节点
 */
struct FRRTNode
{
	FVector Position;		// 节点位置
	int32 ParentIndex;		// 父节点索引
	float Cost;				// 从起点到该节点的代价 (用于RRT*)

	FRRTNode()
		: Position(FVector::ZeroVector)
		, ParentIndex(-1)
		, Cost(0.0f)
	{}

	FRRTNode(const FVector& InPosition, int32 InParentIndex = -1, float InCost = 0.0f)
		: Position(InPosition)
		, ParentIndex(InParentIndex)
		, Cost(InCost)
	{}
};

/**
 * RRT路径规划器
 * 实现快速随机探索树 (Rapidly-exploring Random Tree) 算法
 * 支持基础RRT和RRT*优化版本
 */
UCLASS(ClassGroup=(Planning), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API URRTPathPlanner : public UPathPlanner
{
	GENERATED_BODY()

public:
	URRTPathPlanner();

	// 重写路径规划接口
	virtual bool PlanPath(const FVector& Start, const FVector& Goal, TArray<FVector>& OutPath) override;

	/**
	 * 设置搜索边界
	 * @param InMinBounds 最小边界
	 * @param InMaxBounds 最大边界
	 */
	UFUNCTION(BlueprintCallable, Category = "RRT Path Planning")
	void SetSearchBounds(const FVector& InMinBounds, const FVector& InMaxBounds);

	/**
	 * 自动计算搜索边界
	 * @param Start 起点
	 * @param Goal 终点
	 * @param Padding 边界填充
	 */
	UFUNCTION(BlueprintCallable, Category = "RRT Path Planning")
	void AutoComputeSearchBounds(const FVector& Start, const FVector& Goal, float Padding = 1000.0f);

protected:
	// 扩展步长 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT Settings")
	float StepSize = 100.0f;

	// 最大迭代次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT Settings")
	int32 MaxIterations = 5000;

	// 目标偏置概率 (0-1)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT Settings")
	float GoalBias = 0.1f;

	// 目标到达阈值 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT Settings")
	float GoalThreshold = 100.0f;

	// 是否使用RRT*优化
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT Settings")
	bool bUseRRTStar = false;

	// RRT* 邻域搜索半径 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT* Settings")
	float NeighborhoodRadius = 200.0f;

	// 搜索边界
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT Settings")
	FVector MinBounds = FVector(-10000.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT Settings")
	FVector MaxBounds = FVector(10000.0f);

private:
	// RRT树节点
	TArray<FRRTNode> Tree;

	// 随机采样一个点
	FVector SampleRandomPoint() const;

	// 找到树中距离给定点最近的节点
	int32 FindNearestNode(const FVector& Point) const;

	// 向目标方向扩展
	FVector Steer(const FVector& From, const FVector& To) const;

	// 找到邻域内的节点 (用于RRT*)
	TArray<int32> FindNearbyNodes(const FVector& Point, float Radius) const;

	// 选择最佳父节点 (用于RRT*)
	int32 ChooseBestParent(const FVector& NewPoint, const TArray<int32>& NearbyNodes) const;

	// 重新连线 (用于RRT*)
	void Rewire(int32 NewNodeIndex, const TArray<int32>& NearbyNodes);

	// 从节点重建路径
	void ReconstructPath(int32 GoalNodeIndex, TArray<FVector>& OutPath) const;

	// 目标位置 (用于采样时的目标偏置)
	FVector GoalPosition;
};
