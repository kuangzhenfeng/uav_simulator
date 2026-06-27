// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "BTTask_ExitSimulation.generated.h"

/**
 * 行为树任务：仿真完成后自动退出进程（仅在无头模式下生效）
 *
 * 放置在 BTTask_UAVFollowTrajectory 之后的 Sequence 中。
 * 当轨迹跟踪完成、BT 执行到此节点时：
 *   - 无头模式：记录日志并请求进程退出
 *   - 有头模式：直接返回 Succeeded（空操作）
 */
UCLASS()
class UAV_SIMULATOR_API UBTTask_ExitSimulation : public UBTTaskNode
{
	GENERATED_BODY()

public:
	UBTTask_ExitSimulation();

	virtual EBTNodeResult::Type ExecuteTask(
		UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual FString GetStaticDescription() const override;

	/**
	 * 取消所有挂起的延迟退出。
	 * 进程内场景热重载前由编排者调用：把退出标志置为取消，使已注册的延迟退出 Timer
	 * 在触发时不再 RequestExit，防止旧场景结束误杀进程、阻断新场景装配。
	 */
	static void CancelPendingExit();

protected:
	// 退出前的延迟（秒），用于确保日志刷新完成
	UPROPERTY(EditAnywhere, Category = "Simulation",
		meta = (ClampMin = "0.0", ClampMax = "10.0"))
	float ExitDelay = 1.0f;

private:
	// 执行延迟退出
	void RequestDelayedExit(UWorld* World) const;
};
