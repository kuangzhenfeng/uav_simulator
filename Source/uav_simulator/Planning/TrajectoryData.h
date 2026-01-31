// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "../Core/UAVTypes.h"
#include "TrajectoryData.generated.h"

/**
 * 轨迹数据持有者
 * 用于在黑板中存储轨迹数据
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UTrajectoryData : public UObject
{
	GENERATED_BODY()

public:
	UTrajectoryData();

	// 轨迹数据
	UPROPERTY(BlueprintReadWrite, Category = "Trajectory")
	FTrajectory Trajectory;

	// 设置轨迹
	UFUNCTION(BlueprintCallable, Category = "Trajectory")
	void SetTrajectory(const FTrajectory& InTrajectory);

	// 获取轨迹
	UFUNCTION(BlueprintCallable, Category = "Trajectory")
	const FTrajectory& GetTrajectory() const;

	// 检查轨迹是否有效
	UFUNCTION(BlueprintCallable, Category = "Trajectory")
	bool IsValid() const;

	// 清空轨迹
	UFUNCTION(BlueprintCallable, Category = "Trajectory")
	void Clear();
};
