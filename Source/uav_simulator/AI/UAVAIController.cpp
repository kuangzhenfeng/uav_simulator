// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVAIController.h"
#include "BehaviorTree/BehaviorTree.h"
#include "BehaviorTree/BehaviorTreeComponent.h"
#include "BehaviorTree/BlackboardComponent.h"

// 定义黑板键名称
const FName AUAVAIController::TargetLocationKey = TEXT("TargetLocation");
const FName AUAVAIController::TargetActorKey = TEXT("TargetActor");
const FName AUAVAIController::HomeLocationKey = TEXT("HomeLocation");
const FName AUAVAIController::CurrentStateKey = TEXT("CurrentState");

AUAVAIController::AUAVAIController()
{
	// 创建行为树组件
	BehaviorTreeComp = CreateDefaultSubobject<UBehaviorTreeComponent>(TEXT("BehaviorTreeComponent"));
	
	// 创建黑板组件
	BlackboardComp = CreateDefaultSubobject<UBlackboardComponent>(TEXT("BlackboardComponent"));

	// 设置为主要的Tick组件
	PrimaryActorTick.bCanEverTick = true;
}

void AUAVAIController::BeginPlay()
{
	Super::BeginPlay();
}

void AUAVAIController::OnPossess(APawn* InPawn)
{
	Super::OnPossess(InPawn);

	if (BehaviorTreeAsset && BehaviorTreeAsset->BlackboardAsset)
	{
		// 初始化黑板
		BlackboardComp->InitializeBlackboard(*BehaviorTreeAsset->BlackboardAsset);

		// 设置初始位置为Home位置
		if (InPawn)
		{
			BlackboardComp->SetValueAsVector(HomeLocationKey, InPawn->GetActorLocation());
		}

		// 自动启动行为树
		if (bAutoStartBehaviorTree)
		{
			StartBehaviorTree();
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("UAVAIController: No BehaviorTree or BlackboardAsset assigned!"));
	}
}

void AUAVAIController::OnUnPossess()
{
	// 停止行为树
	StopBehaviorTree();

	Super::OnUnPossess();
}

void AUAVAIController::StartBehaviorTree()
{
	if (BehaviorTreeAsset && BehaviorTreeComp)
	{
		BehaviorTreeComp->StartTree(*BehaviorTreeAsset);
		UE_LOG(LogTemp, Log, TEXT("UAVAIController: Behavior Tree started."));
	}
}

void AUAVAIController::StopBehaviorTree()
{
	if (BehaviorTreeComp)
	{
		BehaviorTreeComp->StopTree();
		UE_LOG(LogTemp, Log, TEXT("UAVAIController: Behavior Tree stopped."));
	}
}

void AUAVAIController::SetTargetLocation(FVector Location)
{
	if (BlackboardComp)
	{
		BlackboardComp->SetValueAsVector(TargetLocationKey, Location);
	}
}

void AUAVAIController::SetTargetActor(AActor* Target)
{
	if (BlackboardComp)
	{
		BlackboardComp->SetValueAsObject(TargetActorKey, Target);
	}
}
