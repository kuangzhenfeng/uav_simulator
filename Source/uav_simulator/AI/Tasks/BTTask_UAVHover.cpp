// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTTask_UAVHover.h"
#include "AIController.h"
#include "uav_simulator/Core/UAVPawn.h"

UBTTask_UAVHover::UBTTask_UAVHover()
{
	NodeName = TEXT("UAV Hover");
	bNotifyTick = true;
}

EBTNodeResult::Type UBTTask_UAVHover::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
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

	// 初始化任务内存
	FBTTask_UAVHoverMemory* Memory = reinterpret_cast<FBTTask_UAVHoverMemory*>(NodeMemory);
	Memory->ElapsedTime = 0.0f;
	Memory->HoverPosition = ControlledPawn->GetActorLocation();
	Memory->HoverPosition.Z += HeightOffset;

	// 设置UAV的目标位置为悬停位置
	AUAVPawn* UAVPawn = Cast<AUAVPawn>(ControlledPawn);
	if (UAVPawn)
	{
		UAVPawn->SetTargetPosition(Memory->HoverPosition);
	}

	// 如果悬停时间为0，表示无限悬停，返回InProgress
	if (HoverDuration <= 0.0f)
	{
		return EBTNodeResult::InProgress;
	}

	return EBTNodeResult::InProgress;
}

void UBTTask_UAVHover::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
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

	FBTTask_UAVHoverMemory* Memory = reinterpret_cast<FBTTask_UAVHoverMemory*>(NodeMemory);

	// 持续设置目标位置以保持悬停
	AUAVPawn* UAVPawn = Cast<AUAVPawn>(ControlledPawn);
	if (UAVPawn)
	{
		UAVPawn->SetTargetPosition(Memory->HoverPosition);
	}

	// 如果悬停时间为0，无限悬停
	if (HoverDuration <= 0.0f)
	{
		return;
	}

	// 更新已过时间
	Memory->ElapsedTime += DeltaSeconds;

	// 检查是否完成悬停
	if (Memory->ElapsedTime >= HoverDuration)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
	}
}

FString UBTTask_UAVHover::GetStaticDescription() const
{
	if (HoverDuration <= 0.0f)
	{
		return TEXT("Hover indefinitely");
	}
	return FString::Printf(TEXT("Hover for %.1f seconds"), HoverDuration);
}

uint16 UBTTask_UAVHover::GetInstanceMemorySize() const
{
	return sizeof(FBTTask_UAVHoverMemory);
}
