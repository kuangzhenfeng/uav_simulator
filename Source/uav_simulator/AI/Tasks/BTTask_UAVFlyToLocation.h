// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "uav_simulator/Core/UAVTypes.h"
#include "BTTask_UAVFlyToLocation.generated.h"

/**
 * 行为树任务：让UAV飞往指定位置
 */
UCLASS()
class UAV_SIMULATOR_API UBTTask_UAVFlyToLocation : public UBTTaskNode
{
	GENERATED_BODY()

public:
	UBTTask_UAVFlyToLocation();

	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
	virtual FString GetStaticDescription() const override;

protected:
	// 目标位置的黑板键
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector TargetLocationKey;

	// 到达目标的可接受距离
	UPROPERTY(EditAnywhere, Category = "UAV")
	float AcceptableRadius = 100.0f;

	// 飞行速度
	UPROPERTY(EditAnywhere, Category = "UAV")
	float FlySpeed = 500.0f;

	// ===== 路径规划选项 =====

	// 是否使用路径规划
	UPROPERTY(EditAnywhere, Category = "Path Planning")
	bool bUsePathPlanning = false;

	// 路径规划算法
	UPROPERTY(EditAnywhere, Category = "Path Planning", meta = (EditCondition = "bUsePathPlanning"))
	EPathPlanningAlgorithm PathPlanningAlgorithm = EPathPlanningAlgorithm::AStar;

	// 是否优化轨迹
	UPROPERTY(EditAnywhere, Category = "Path Planning", meta = (EditCondition = "bUsePathPlanning"))
	bool bOptimizeTrajectory = true;

	// 最大加速度 (cm/s²)
	UPROPERTY(EditAnywhere, Category = "Path Planning", meta = (EditCondition = "bUsePathPlanning"))
	float MaxAcceleration = 200.0f;

	// 安全边距 (cm)
	UPROPERTY(EditAnywhere, Category = "Path Planning", meta = (EditCondition = "bUsePathPlanning"))
	float SafetyMargin = 50.0f;

private:
	// 缓存的轨迹
	FTrajectory CachedTrajectory;

	// 是否已初始化路径
	bool bPathInitialized = false;
};
