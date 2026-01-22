// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTDecorator.h"
#include "BTDecorator_UAVAtLocation.generated.h"

/**
 * 行为树装饰器：检查UAV是否在指定位置
 */
UCLASS()
class UAV_SIMULATOR_API UBTDecorator_UAVAtLocation : public UBTDecorator
{
	GENERATED_BODY()

public:
	UBTDecorator_UAVAtLocation();

protected:
	virtual bool CalculateRawConditionValue(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) const override;
	virtual FString GetStaticDescription() const override;

protected:
	// 目标位置的黑板键
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector TargetLocationKey;

	// 可接受的距离误差
	UPROPERTY(EditAnywhere, Category = "UAV")
	float AcceptableRadius = 100.0f;
};
