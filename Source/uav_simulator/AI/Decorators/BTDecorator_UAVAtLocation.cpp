// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTDecorator_UAVAtLocation.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"

UBTDecorator_UAVAtLocation::UBTDecorator_UAVAtLocation()
{
	NodeName = TEXT("UAV At Location");

	// 设置黑板键过滤器
	TargetLocationKey.AddVectorFilter(this, GET_MEMBER_NAME_CHECKED(UBTDecorator_UAVAtLocation, TargetLocationKey));
}

bool UBTDecorator_UAVAtLocation::CalculateRawConditionValue(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) const
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (!AIController)
	{
		return false;
	}

	APawn* ControlledPawn = AIController->GetPawn();
	if (!ControlledPawn)
	{
		return false;
	}

	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	if (!BlackboardComp)
	{
		return false;
	}

	FVector TargetLocation = BlackboardComp->GetValueAsVector(TargetLocationKey.SelectedKeyName);
	FVector CurrentLocation = ControlledPawn->GetActorLocation();

	float Distance = FVector::Dist(CurrentLocation, TargetLocation);
	return Distance <= AcceptableRadius;
}

FString UBTDecorator_UAVAtLocation::GetStaticDescription() const
{
	return FString::Printf(TEXT("At %s (Radius: %.1f)"), 
		*TargetLocationKey.SelectedKeyName.ToString(), 
		AcceptableRadius);
}
