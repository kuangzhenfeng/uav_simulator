// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "DebugVisualizer.generated.h"

/**
 * 调试可视化组件
 * 用于实时显示无人机状态、轨迹等调试信息
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UDebugVisualizer : public UActorComponent
{
	GENERATED_BODY()

public:
	UDebugVisualizer();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 绘制无人机状态信息
	UFUNCTION(BlueprintCallable, Category = "Debug")
	void DrawUAVState(const FUAVState& State, const FVector& ActorLocation);

	// 绘制轨迹历史
	UFUNCTION(BlueprintCallable, Category = "Debug")
	void DrawTrajectoryHistory(const FVector& CurrentPosition);

	// 清除轨迹历史
	UFUNCTION(BlueprintCallable, Category = "Debug")
	void ClearTrajectoryHistory();

	// ===== 轨迹规划可视化 =====

	// 绘制规划路径
	UFUNCTION(BlueprintCallable, Category = "Debug|Planning")
	void DrawPlannedPath(const TArray<FVector>& Path, FColor Color = FColor::Green, float Duration = 5.0f);

	// 绘制优化轨迹
	UFUNCTION(BlueprintCallable, Category = "Debug|Planning")
	void DrawOptimizedTrajectory(const FTrajectory& Trajectory, FColor Color = FColor::Blue, float Duration = 5.0f);

	// 绘制当前跟踪状态
	UFUNCTION(BlueprintCallable, Category = "Debug|Planning")
	void DrawTrackingState(const FTrajectoryPoint& DesiredState, const FVector& CurrentPosition);

	// 绘制障碍物
	UFUNCTION(BlueprintCallable, Category = "Debug|Planning")
	void DrawObstacles(const TArray<FObstacleInfo>& Obstacles, FColor Color = FColor::Red, float Duration = -1.0f);

	// 绘制航点
	UFUNCTION(BlueprintCallable, Category = "Debug|Planning")
	void DrawWaypoints(const TArray<FVector>& Waypoints, float Radius = 30.0f, FColor Color = FColor::Magenta, float Duration = 5.0f);

	// 设置持久化轨迹显示
	UFUNCTION(BlueprintCallable, Category = "Debug|Planning")
	void SetPersistentTrajectory(const FTrajectory& Trajectory);

	// 清除持久化轨迹
	UFUNCTION(BlueprintCallable, Category = "Debug|Planning")
	void ClearPersistentTrajectory();

protected:
	// 是否显示调试信息
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug Settings")
	bool bShowDebugInfo = true;

	// 是否显示轨迹历史
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug Settings")
	bool bShowTrajectory = true;

	// 是否显示规划路径
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug Settings")
	bool bShowPlannedPath = true;

	// 是否显示跟踪状态
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug Settings")
	bool bShowTrackingState = true;

	// 轨迹历史最大长度
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug Settings")
	int32 MaxTrajectoryPoints = 500;

	// 轨迹线条粗细
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug Settings")
	float TrajectoryThickness = 2.0f;

private:
	// 轨迹历史点
	TArray<FVector> TrajectoryHistory;

	// 持久化轨迹
	FTrajectory PersistentTrajectory;

	// 跟踪误差文字绘制计时器
	float TrackingTextTimer = 0.f;

	// 绘制持久化轨迹
	void DrawPersistentTrajectory();
};
