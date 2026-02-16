// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "../Planning/LocalAvoidance.h"
#include "PlanningVisualizer.generated.h"

/**
 * 规划可视化器
 * 用于可视化路径规划、轨迹和障碍物
 */
UCLASS(ClassGroup=(Planning), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UPlanningVisualizer : public UActorComponent
{
	GENERATED_BODY()

public:
	UPlanningVisualizer();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/**
	 * 绘制规划路径
	 * @param Path 路径点数组
	 * @param Color 颜色
	 * @param Duration 持续时间
	 * @param Thickness 线条粗细
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void DrawPath(const TArray<FVector>& Path, FColor Color = FColor::Green, float Duration = 5.0f, float Thickness = 3.0f);

	/**
	 * 绘制轨迹
	 * @param Trajectory 轨迹数据
	 * @param Color 颜色
	 * @param Duration 持续时间
	 * @param bShowVelocity 是否显示速度向量
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void DrawTrajectory(const FTrajectory& Trajectory, FColor Color = FColor::Blue, float Duration = 5.0f, bool bShowVelocity = false);

	/**
	 * 绘制当前跟踪点
	 * @param Point 轨迹点
	 * @param Radius 球体半径
	 * @param Color 颜色
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void DrawTrackingPoint(const FTrajectoryPoint& Point, float Radius = 20.0f, FColor Color = FColor::Yellow);

	/**
	 * 绘制障碍物
	 * @param Obstacles 障碍物数组
	 * @param Color 颜色
	 * @param Duration 持续时间
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void DrawObstacles(const TArray<FObstacleInfo>& Obstacles, FColor Color = FColor::Red, float Duration = -1.0f);

	/**
	 * 绘制单个障碍物
	 * @param Obstacle 障碍物信息
	 * @param Color 颜色
	 * @param Duration 持续时间
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void DrawObstacle(const FObstacleInfo& Obstacle, FColor Color = FColor::Red, float Duration = -1.0f);

	/**
	 * 绘制搜索空间边界
	 * @param MinBounds 最小边界
	 * @param MaxBounds 最大边界
	 * @param Color 颜色
	 * @param Duration 持续时间
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void DrawSearchBounds(const FVector& MinBounds, const FVector& MaxBounds, FColor Color = FColor::White, float Duration = 5.0f);

	/**
	 * 绘制航点
	 * @param Waypoints 航点数组
	 * @param Radius 球体半径
	 * @param Color 颜色
	 * @param Duration 持续时间
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void DrawWaypoints(const TArray<FVector>& Waypoints, float Radius = 30.0f, FColor Color = FColor::Magenta, float Duration = 5.0f);

	/**
	 * 清除所有可视化
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void ClearVisualization();

	/**
	 * 设置持久化路径（每帧绘制）
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void SetPersistentPath(const TArray<FVector>& Path);

	/**
	 * 设置持久化轨迹（每帧绘制）
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void SetPersistentTrajectory(const FTrajectory& Trajectory);

	/**
	 * 清除持久化数据
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void ClearPersistentData();

	/**
	 * 绘制局部避障力场向量（引力、斥力、合力）
	 * @param Position 当前位置
	 * @param Result APF 计算结果
	 * @param ForceScale 力向量缩放系数
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void DrawLocalAvoidance(const FVector& Position, const FLocalAvoidanceResult& Result, float ForceScale = 0.5f);

	/**
	 * 设置持久化局部避障数据（每帧绘制）
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void SetPersistentLocalAvoidance(const FVector& Position, const FLocalAvoidanceResult& Result);

	/**
	 * 清除持久化局部避障数据
	 */
	UFUNCTION(BlueprintCallable, Category = "Planning Visualization")
	void ClearPersistentLocalAvoidance();

protected:
	// 是否启用可视化
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	bool bEnableVisualization = true;

	// 是否显示路径
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	bool bShowPath = true;

	// 是否显示轨迹
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	bool bShowTrajectory = true;

	// 是否显示障碍物
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	bool bShowObstacles = true;

	// 是否显示航点
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	bool bShowWaypoints = true;

	// 是否显示局部避障力场
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	bool bShowLocalAvoidance = true;

	// 路径颜色
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	FColor PathColor = FColor::Green;

	// 轨迹颜色
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	FColor TrajectoryColor = FColor::Blue;

	// 障碍物颜色
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	FColor ObstacleColor = FColor::Red;

	// 航点颜色
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	FColor WaypointColor = FColor::Magenta;

	// 线条粗细
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visualization Settings")
	float LineThickness = 2.0f;

private:
	// 持久化路径
	TArray<FVector> PersistentPath;

	// 持久化轨迹
	FTrajectory PersistentTrajectory;

	// 持久化局部避障数据
	FVector PersistentAvoidancePosition;
	FLocalAvoidanceResult PersistentAvoidanceResult;
	bool bHasPersistentAvoidance = false;

	// 绘制持久化数据
	void DrawPersistentData();
};
