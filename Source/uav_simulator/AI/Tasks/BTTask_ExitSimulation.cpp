// Copyright Epic Games, Inc. All Rights Reserved.

#include "BTTask_ExitSimulation.h"
#include "AIController.h"
#include "Misc/App.h"
#include "GenericPlatform/GenericPlatformMisc.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

// 进程内热重载用：置 true 时，已挂起的延迟退出 Timer 触发也不再 RequestExit。
// 由 CancelPendingExit() 置 true，新场景装配前清零，避免旧场景任务完成误杀进程。
static bool gbExitCancelled = false;

UBTTask_ExitSimulation::UBTTask_ExitSimulation()
{
	NodeName = TEXT("Exit Simulation");
	bNotifyTick = false;
}

EBTNodeResult::Type UBTTask_ExitSimulation::ExecuteTask(
	UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	// 非无头模式下直接返回成功，不做任何操作
	if (!FApp::IsUnattended())
	{
		UE_LOG(LogUAVAI, Log,
			TEXT("BTTask_ExitSimulation: Not in unattended mode, skipping exit"));
		return EBTNodeResult::Succeeded;
	}

	UE_LOG(LogUAVAI, Log,
		TEXT("BTTask_ExitSimulation: Mission completed in headless mode, "
			 "requesting exit in %.1f seconds"), ExitDelay);

	GLog->Flush();

	// 通过 TimerManager 延迟退出，给引擎时间完成日志刷新
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (AIController)
	{
		UWorld* World = AIController->GetWorld();
		if (World && ExitDelay > 0.0f)
		{
			RequestDelayedExit(World);
			return EBTNodeResult::Succeeded;
		}
	}

	// 无法获取 World 或延迟为 0，立即退出
	FGenericPlatformMisc::RequestExit(false);
	return EBTNodeResult::Succeeded;
}

void UBTTask_ExitSimulation::RequestDelayedExit(UWorld* World) const
{
	FTimerHandle TimerHandle;
	World->GetTimerManager().SetTimer(
		TimerHandle,
		FTimerDelegate::CreateLambda([]()
		{
			// 热重载可能在该 Timer 挂起期间取消退出；取消则放弃 RequestExit。
			if (gbExitCancelled)
			{
				UE_LOG(LogUAVAI, Log,
					TEXT("BTTask_ExitSimulation: Pending exit cancelled by scenario reload"));
				gbExitCancelled = false; // 消费一次，恢复默认
				return;
			}
			UE_LOG(LogUAVAI, Log,
				TEXT("BTTask_ExitSimulation: Executing graceful shutdown"));
			GLog->Flush();
			FGenericPlatformMisc::RequestExit(false);
		}),
		ExitDelay,
		false
	);
}

void UBTTask_ExitSimulation::CancelPendingExit()
{
	gbExitCancelled = true;
	UE_LOG(LogUAVAI, Log, TEXT("BTTask_ExitSimulation: Pending exit cancel requested (scenario reload)"));
}

FString UBTTask_ExitSimulation::GetStaticDescription() const
{
	return FString::Printf(
		TEXT("Exit simulation in headless mode\nDelay: %.1f s"), ExitDelay);
}
