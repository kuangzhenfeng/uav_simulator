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

protected:
	// 是否显示调试信息
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug Settings")
	bool bShowDebugInfo = true;

	// 是否显示轨迹历史
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug Settings")
	bool bShowTrajectory = true;

	// 轨迹历史最大长度
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug Settings")
	int32 MaxTrajectoryPoints = 500;

	// 轨迹线条粗细
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug Settings")
	float TrajectoryThickness = 2.0f;

private:
	// 轨迹历史点
	TArray<FVector> TrajectoryHistory;
};
