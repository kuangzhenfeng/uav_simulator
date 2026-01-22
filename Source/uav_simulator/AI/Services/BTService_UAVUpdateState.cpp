// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTService_UAVUpdateState.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "uav_simulator/Core/UAVPawn.h"

UBTService_UAVUpdateState::UBTService_UAVUpdateState()
{
	NodeName = TEXT("Update UAV State");
	Interval = 0.1f;
	RandomDeviation = 0.0f;

	// 设置黑板键过滤器
	CurrentLocationKey.AddVectorFilter(this, GET_MEMBER_NAME_CHECKED(UBTService_UAVUpdateState, CurrentLocationKey));
	CurrentVelocityKey.AddVectorFilter(this, GET_MEMBER_NAME_CHECKED(UBTService_UAVUpdateState, CurrentVelocityKey));
	CurrentAltitudeKey.AddFloatFilter(this, GET_MEMBER_NAME_CHECKED(UBTService_UAVUpdateState, CurrentAltitudeKey));
}

void UBTService_UAVUpdateState::TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	Super::TickNode(OwnerComp, NodeMemory, DeltaSeconds);

	AAIController* AIController = OwnerComp.GetAIOwner();
	if (!AIController)
	{
		return;
	}

	APawn* ControlledPawn = AIController->GetPawn();
	if (!ControlledPawn)
	{
		return;
	}

	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	if (!BlackboardComp)
	{
		return;
	}

	// 获取UAV状态
	AUAVPawn* UAVPawn = Cast<AUAVPawn>(ControlledPawn);
	if (UAVPawn)
	{
		FUAVState State = UAVPawn->GetUAVState();

		// 更新黑板值
		if (CurrentLocationKey.SelectedKeyName != NAME_None)
		{
			BlackboardComp->SetValueAsVector(CurrentLocationKey.SelectedKeyName, State.Position);
		}

		if (CurrentVelocityKey.SelectedKeyName != NAME_None)
		{
			BlackboardComp->SetValueAsVector(CurrentVelocityKey.SelectedKeyName, State.Velocity);
		}

		if (CurrentAltitudeKey.SelectedKeyName != NAME_None)
		{
			BlackboardComp->SetValueAsFloat(CurrentAltitudeKey.SelectedKeyName, State.Position.Z);
		}
	}
}

FString UBTService_UAVUpdateState::GetStaticDescription() const
{
	return FString::Printf(TEXT("Updates UAV state to blackboard every %.2f seconds"), Interval);
}
