// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "WaypointsData.generated.h"

/**
 * 航点数据持有者
 * 用于在黑板中存储航点数组
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UWaypointsData : public UObject
{
	GENERATED_BODY()

public:
	UWaypointsData();

	// 航点数组
	UPROPERTY(BlueprintReadWrite, Category = "Waypoints")
	TArray<FVector> Waypoints;

	// 添加航点
	UFUNCTION(BlueprintCallable, Category = "Waypoints")
	void AddWaypoint(const FVector& Waypoint);

	// 设置航点数组
	UFUNCTION(BlueprintCallable, Category = "Waypoints")
	void SetWaypoints(const TArray<FVector>& InWaypoints);

	// 获取航点数组
	UFUNCTION(BlueprintCallable, Category = "Waypoints")
	const TArray<FVector>& GetWaypoints() const;

	// 获取航点数量
	UFUNCTION(BlueprintCallable, Category = "Waypoints")
	int32 GetNumWaypoints() const;

	// 获取指定索引的航点
	UFUNCTION(BlueprintCallable, Category = "Waypoints")
	FVector GetWaypoint(int32 Index) const;

	// 检查是否有有效航点
	UFUNCTION(BlueprintCallable, Category = "Waypoints")
	bool IsValid() const;

	// 清空航点
	UFUNCTION(BlueprintCallable, Category = "Waypoints")
	void Clear();
};
