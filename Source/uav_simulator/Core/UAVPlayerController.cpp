// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVPlayerController.h"
#include "UAVPawn.h"
#include "../UI/CameraSwitcherWidget.h"
#include "../MultiAgent/AgentManager.h"
#include "../Debug/UAVLogConfig.h"
#include "Kismet/GameplayStatics.h"
#include "../Debug/DebugVisualizer.h"
#include "../Planning/PlanningVisualizer.h"

DEFINE_LOG_CATEGORY_STATIC(LogUAVPlayerController, Log, All);

AUAVPlayerController::AUAVPlayerController()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
}

void AUAVPlayerController::BeginPlay()
{
	Super::BeginPlay();

	// 缓存默认视角目标
	DefaultViewTarget = GetPawn();

	// 创建视角切换 UI
	SwitcherWidget = CreateWidget<UCameraSwitcherWidget>(this, UCameraSwitcherWidget::StaticClass());
	if (SwitcherWidget)
	{
		SwitcherWidget->AddToViewport(10);

		// 绑定视角切换回调
		SwitcherWidget->OnViewChanged.AddDynamic(this, &AUAVPlayerController::OnViewChanged);
			SwitcherWidget->OnToggleDebug.AddDynamic(this, &AUAVPlayerController::OnToggleDebug);

		// 默认高亮全局视角
		SwitcherWidget->HighlightCurrentView(-1);

		UE_LOG(LogUAVCamera, Log, TEXT("[CameraSwitcher] Widget created and added to viewport"));
	}
	else
	{
		UE_LOG(LogUAVCamera, Error, TEXT("[CameraSwitcher] Failed to create widget"));
	}
}

void AUAVPlayerController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// 定期刷新无人机列表
	UIRefreshAccumulator += DeltaTime;
	if (UIRefreshAccumulator >= UIRefreshInterval)
	{
		UIRefreshAccumulator = 0.0f;

		int32 DroneCount = GetDroneCount();
		if (DroneCount != LastDroneCount && SwitcherWidget)
		{
			SwitcherWidget->RefreshDroneList(DroneCount);
			LastDroneCount = DroneCount;
		}
	}
}

void AUAVPlayerController::OnViewChanged(int32 AgentID)
{
	SwitchView(AgentID);
}

void AUAVPlayerController::SwitchView(int32 AgentID)
{
	if (AgentID == -1)
	{
		// 全局视角：切回默认 Pawn
		if (DefaultViewTarget)
		{
			SetViewTargetWithBlend(DefaultViewTarget, 0.5f);
			CurrentViewAgentID = -1;
			UE_LOG(LogUAVCamera, Log, TEXT("[CameraSwitcher] Switched to Global View"));
		}
	}
	else
	{
		// 跟随无人机视角
		AUAVPawn* DronePawn = GetDroneByAgentID(AgentID);
		if (DronePawn)
		{
			SetViewTargetWithBlend(DronePawn, 0.5f);
			CurrentViewAgentID = AgentID;
			UE_LOG(LogUAVCamera, Log, TEXT("[CameraSwitcher] Switched to UAV #%d"), AgentID);
		}
		else
		{
			UE_LOG(LogUAVCamera, Warning, TEXT("[CameraSwitcher] UAV #%d not found"), AgentID);
		}
	}

	// 更新 UI 高亮
	if (SwitcherWidget)
	{
		SwitcherWidget->HighlightCurrentView(CurrentViewAgentID);
	}
}

void AUAVPlayerController::OnToggleDebug(int32 ToggleIndex)
{
	// 遍历所有 UAVPawn，切换对应组件的可视化属性
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(this, AUAVPawn::StaticClass(), FoundActors);

	for (AActor* Actor : FoundActors)
	{
		AUAVPawn* UAVPawn = Cast<AUAVPawn>(Actor);
		if (!UAVPawn) continue;

		switch (ToggleIndex)
		{
		case 0: // Vis - 规划可视化总开关
			if (UPlanningVisualizer* Vis = UAVPawn->GetPlanningVisualizer())
				Vis->ToggleVisualization();
			break;
		case 1: // Obs - 障碍物绘制
			if (UPlanningVisualizer* Vis = UAVPawn->GetPlanningVisualizer())
				Vis->ToggleShowObstacles();
			break;
		case 2: // Path - 路径绘制
			if (UPlanningVisualizer* Vis = UAVPawn->GetPlanningVisualizer())
				Vis->ToggleShowPath();
			break;
		case 3: // Dbg - UAV 状态调试信息
			if (UDebugVisualizer* Dbg = UAVPawn->GetComponentByClass<UDebugVisualizer>())
				Dbg->ToggleShowDebugInfo();
			break;
		}
	}
}

AUAVPawn* AUAVPlayerController::GetDroneByAgentID(int32 AgentID)
{
	// 优先从 MultiAgentGameMode 获取
	if (AMultiAgentGameMode* MultiGM = GetWorld()->GetAuthGameMode<AMultiAgentGameMode>())
	{
		// AgentManager 没有直接暴露 GetAgent 方法，遍历场景查找
	}

	// 回退：遍历场景中所有 UAVPawn，通过 AgentID 匹配
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(this, AUAVPawn::StaticClass(), FoundActors);

	for (AActor* Actor : FoundActors)
	{
		AUAVPawn* FoundPawn = Cast<AUAVPawn>(Actor);
		if (FoundPawn && FoundPawn->GetAgentID() == AgentID)
		{
			return FoundPawn;
		}
	}

	return nullptr;
}

int32 AUAVPlayerController::GetDroneCount()
{
	// 优先从 MultiAgentGameMode 获取
	if (AMultiAgentGameMode* MultiGM = GetWorld()->GetAuthGameMode<AMultiAgentGameMode>())
	{
		return MultiGM->GetAgentCount();
	}

	// 回退：统计场景中的 UAVPawn 数量
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(this, AUAVPawn::StaticClass(), FoundActors);
	return FoundActors.Num();
}
