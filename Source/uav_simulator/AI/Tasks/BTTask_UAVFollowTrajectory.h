// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "BehaviorTree/BehaviorTreeTypes.h"
#include "uav_simulator/Core/UAVTypes.h"
#include "BTTask_UAVFollowTrajectory.generated.h"

class UTrajectoryData;
class UWaypointsData;

/**
 * 行为树任务：让UAV跟踪预定义的轨迹
 */
UCLASS()
class UAV_SIMULATOR_API UBTTask_UAVFollowTrajectory : public UBTTaskNode
{
	GENERATED_BODY()

public:
	UBTTask_UAVFollowTrajectory();

	virtual void InitializeFromAsset(UBehaviorTree& Asset) override;
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
	virtual void OnTaskFinished(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, EBTNodeResult::Type TaskResult) override;
	virtual FString GetStaticDescription() const override;

protected:
	// 轨迹数据的黑板键（存储UTrajectoryData对象）
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector TrajectoryKey;

	// 航点数组的黑板键（存储UWaypointsData对象，用于自动生成轨迹）
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector WaypointsKey;

	// 是否使用黑板中的轨迹，否则使用航点生成
	UPROPERTY(EditAnywhere, Category = "Trajectory")
	bool bUseTrajectoryFromBlackboard = false;

	// 最大飞行速度 (cm/s)
	UPROPERTY(EditAnywhere, Category = "Trajectory")
	float MaxVelocity = 500.0f;

	// 最大加速度 (cm/s²)
	UPROPERTY(EditAnywhere, Category = "Trajectory")
	float MaxAcceleration = 200.0f;

	// 完成阈值 - 轨迹进度达到此值时认为完成
	UPROPERTY(EditAnywhere, Category = "Trajectory")
	float CompletionThreshold = 0.98f;

	// 是否在完成后保持位置
	UPROPERTY(EditAnywhere, Category = "Trajectory")
	bool bHoldPositionOnComplete = true;

private:
	// 缓存的轨迹
	FTrajectory CachedTrajectory;
};
