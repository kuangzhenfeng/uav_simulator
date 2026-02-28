// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "PathPlanner.generated.h"

/**
 * 路径规划基类
 * 定义路径规划的通用接口，供A*、RRT等算法继承实现
 */
UCLASS(Abstract, ClassGroup=(Planning), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UPathPlanner : public UActorComponent
{
	GENERATED_BODY()

public:
	UPathPlanner();

public:
	/**
	 * 规划从起点到终点的路径
	 * @param Start 起点位置 (世界坐标系)
	 * @param Goal 目标位置 (世界坐标系)
	 * @param OutPath 输出路径点序列
	 * @return 是否成功找到路径
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	virtual bool PlanPath(const FVector& Start, const FVector& Goal, TArray<FVector>& OutPath);

	/**
	 * 设置障碍物列表
	 * @param InObstacles 障碍物信息数组
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	virtual void SetObstacles(const TArray<FObstacleInfo>& InObstacles);

	/**
	 * 添加单个障碍物
	 * @param Obstacle 障碍物信息
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	virtual void AddObstacle(const FObstacleInfo& Obstacle);

	/**
	 * 清除所有障碍物
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	virtual void ClearObstacles();

	/**
	 * 检查点是否与障碍物碰撞
	 * @param Point 检查点位置
	 * @param Radius 碰撞检测半径
	 * @return 是否发生碰撞
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	virtual bool CheckCollision(const FVector& Point, float Radius = 0.0f) const;

	/**
	 * 检查线段是否与障碍物碰撞
	 * @param Start 线段起点
	 * @param End 线段终点
	 * @param Radius 碰撞检测半径
	 * @return 是否发生碰撞
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	virtual bool CheckLineCollision(const FVector& Start, const FVector& End, float Radius = 0.0f) const;

	/**
	 * 可视化规划结果
	 * @param Duration 可视化持续时间
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	virtual void VisualizeResult(float Duration = 5.0f);

	/**
	 * 获取上次规划的路径
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	const TArray<FVector>& GetLastPath() const { return LastPath; }

	/**
	 * 获取规划耗时（毫秒）
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	float GetLastPlanningTime() const { return LastPlanningTimeMs; }

	/**
	 * 设置规划配置
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	void SetPlanningConfig(const FPlanningConfig& InConfig) { PlanningConfig = InConfig; }

	/**
	 * 获取规划配置
	 */
	UFUNCTION(BlueprintCallable, Category = "Path Planning")
	const FPlanningConfig& GetPlanningConfig() const { return PlanningConfig; }

protected:
	// 障碍物列表
	UPROPERTY(BlueprintReadOnly, Category = "Path Planning")
	TArray<FObstacleInfo> Obstacles;

	// 规划配置
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Planning")
	FPlanningConfig PlanningConfig;

	// 上次规划的路径
	UPROPERTY(BlueprintReadOnly, Category = "Path Planning")
	TArray<FVector> LastPath;

	// 上次规划耗时（毫秒）
	UPROPERTY(BlueprintReadOnly, Category = "Path Planning")
	float LastPlanningTimeMs;

	// UAV碰撞半径 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Planning")
	float UAVCollisionRadius = 50.0f;

protected:
	/**
	 * 检查点是否在单个障碍物内
	 */
	bool IsPointInObstacle(const FVector& Point, const FObstacleInfo& Obstacle, float Radius) const;

	/**
	 * 简化路径（去除不必要的中间点）
	 */
	void SimplifyPath(TArray<FVector>& Path) const;

	/**
	 * 绘制调试信息
	 */
	void DrawDebugPath(const TArray<FVector>& Path, const FColor& Color, float Duration, float Thickness = 2.0f) const;
};
