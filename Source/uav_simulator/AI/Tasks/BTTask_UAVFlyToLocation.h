// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
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
};
