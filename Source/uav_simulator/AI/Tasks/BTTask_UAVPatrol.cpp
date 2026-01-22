// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTTask_UAVPatrol.h"
#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"

UBTTask_UAVPatrol::UBTTask_UAVPatrol()
{
	NodeName = TEXT("UAV Get Next Patrol Point");
	bNotifyTick = false;

	// 设置黑板键过滤器
	PatrolIndexKey.AddIntFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_UAVPatrol, PatrolIndexKey));
	TargetLocationKey.AddVectorFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_UAVPatrol, TargetLocationKey));
}

EBTNodeResult::Type UBTTask_UAVPatrol::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	// 检查是否有巡逻点
	if (PatrolPoints.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("BTTask_UAVPatrol: No patrol points defined!"));
		return EBTNodeResult::Failed;
	}

	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	if (!BlackboardComp)
	{
		return EBTNodeResult::Failed;
	}

	// 获取当前巡逻索引
	int32 CurrentIndex = BlackboardComp->GetValueAsInt(PatrolIndexKey.SelectedKeyName);

	// 检查索引是否有效
	if (CurrentIndex >= PatrolPoints.Num())
	{
		if (bLoopPatrol)
		{
			CurrentIndex = 0;
		}
		else
		{
			// 巡逻完成
			return EBTNodeResult::Failed;
		}
	}

	// 设置目标位置
	FVector TargetLocation = PatrolPoints[CurrentIndex];
	BlackboardComp->SetValueAsVector(TargetLocationKey.SelectedKeyName, TargetLocation);

	// 更新索引到下一个航点
	int32 NextIndex = CurrentIndex + 1;
	if (bLoopPatrol && NextIndex >= PatrolPoints.Num())
	{
		NextIndex = 0;
	}
	BlackboardComp->SetValueAsInt(PatrolIndexKey.SelectedKeyName, NextIndex);

	UE_LOG(LogTemp, Log, TEXT("BTTask_UAVPatrol: Moving to patrol point %d at %s"), CurrentIndex, *TargetLocation.ToString());

	return EBTNodeResult::Succeeded;
}

FString UBTTask_UAVPatrol::GetStaticDescription() const
{
	return FString::Printf(TEXT("Patrol Points: %d\nLoop: %s"), 
		PatrolPoints.Num(), 
		bLoopPatrol ? TEXT("Yes") : TEXT("No"));
}
