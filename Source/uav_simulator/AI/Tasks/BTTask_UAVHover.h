// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "BTTask_UAVHover.generated.h"

/**
 * 行为树任务：让UAV在当前位置悬停指定时间
 */
UCLASS()
class UAV_SIMULATOR_API UBTTask_UAVHover : public UBTTaskNode
{
	GENERATED_BODY()

public:
	UBTTask_UAVHover();

	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
	virtual FString GetStaticDescription() const override;
	virtual uint16 GetInstanceMemorySize() const override;

protected:
	// 悬停持续时间（秒），0表示无限悬停
	UPROPERTY(EditAnywhere, Category = "UAV")
	float HoverDuration = 3.0f;

	// 悬停高度偏移（相对于当前高度）
	UPROPERTY(EditAnywhere, Category = "UAV")
	float HeightOffset = 0.0f;
};

// 任务内存结构
struct FBTTask_UAVHoverMemory
{
	float ElapsedTime;
	FVector HoverPosition;
};
