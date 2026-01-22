// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTService.h"
#include "BTService_UAVUpdateState.generated.h"

/**
 * 行为树服务：定期更新UAV状态到黑板
 */
UCLASS()
class UAV_SIMULATOR_API UBTService_UAVUpdateState : public UBTService
{
	GENERATED_BODY()

public:
	UBTService_UAVUpdateState();

protected:
	virtual void TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
	virtual FString GetStaticDescription() const override;

protected:
	// 存储当前位置的黑板键
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector CurrentLocationKey;

	// 存储当前速度的黑板键
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector CurrentVelocityKey;

	// 存储当前高度的黑板键
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector CurrentAltitudeKey;
};
