// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTTask_UAVFlyToLocation.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "uav_simulator/Core/UAVPawn.h"

UBTTask_UAVFlyToLocation::UBTTask_UAVFlyToLocation()
{
	NodeName = TEXT("UAV Fly To Location");
	bNotifyTick = true;

	// 设置默认的黑板键过滤器
	TargetLocationKey.AddVectorFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_UAVFlyToLocation, TargetLocationKey));
}

EBTNodeResult::Type UBTTask_UAVFlyToLocation::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (!AIController)
	{
		return EBTNodeResult::Failed;
	}

	APawn* ControlledPawn = AIController->GetPawn();
	if (!ControlledPawn)
	{
		return EBTNodeResult::Failed;
	}

	// 获取目标位置
	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	if (!BlackboardComp)
	{
		return EBTNodeResult::Failed;
	}

	FVector TargetLocation = BlackboardComp->GetValueAsVector(TargetLocationKey.SelectedKeyName);
	
	// 检查是否已经在目标位置
	float Distance = FVector::Dist(ControlledPawn->GetActorLocation(), TargetLocation);
	if (Distance <= AcceptableRadius)
	{
		return EBTNodeResult::Succeeded;
	}

	// 设置UAV的目标位置
	AUAVPawn* UAVPawn = Cast<AUAVPawn>(ControlledPawn);
	if (UAVPawn)
	{
		UAVPawn->SetTargetPosition(TargetLocation);
	}

	return EBTNodeResult::InProgress;
}

void UBTTask_UAVFlyToLocation::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (!AIController)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	APawn* ControlledPawn = AIController->GetPawn();
	if (!ControlledPawn)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	if (!BlackboardComp)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	FVector TargetLocation = BlackboardComp->GetValueAsVector(TargetLocationKey.SelectedKeyName);
	FVector CurrentLocation = ControlledPawn->GetActorLocation();
	
	// 检查是否到达目标
	float Distance = FVector::Dist(CurrentLocation, TargetLocation);
	if (Distance <= AcceptableRadius)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
		return;
	}

	// 持续更新目标位置（以防黑板值改变）
	AUAVPawn* UAVPawn = Cast<AUAVPawn>(ControlledPawn);
	if (UAVPawn)
	{
		UAVPawn->SetTargetPosition(TargetLocation);
	}
}

FString UBTTask_UAVFlyToLocation::GetStaticDescription() const
{
	return FString::Printf(TEXT("Fly to: %s\nAcceptable Radius: %.1f"), 
		*TargetLocationKey.SelectedKeyName.ToString(), 
		AcceptableRadius);
}
