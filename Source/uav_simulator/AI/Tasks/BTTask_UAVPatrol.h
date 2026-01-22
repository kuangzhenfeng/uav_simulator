// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "BTTask_UAVPatrol.generated.h"

/**
 * 行为树任务：让UAV在航点之间巡逻
 */
UCLASS()
class UAV_SIMULATOR_API UBTTask_UAVPatrol : public UBTTaskNode
{
	GENERATED_BODY()

public:
	UBTTask_UAVPatrol();

	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual FString GetStaticDescription() const override;

protected:
	// 存储下一个航点索引的黑板键
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector PatrolIndexKey;

	// 存储目标位置的黑板键
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector TargetLocationKey;

	// 巡逻航点列表
	UPROPERTY(EditAnywhere, Category = "UAV")
	TArray<FVector> PatrolPoints;

	// 是否循环巡逻
	UPROPERTY(EditAnywhere, Category = "UAV")
	bool bLoopPatrol = true;
};
