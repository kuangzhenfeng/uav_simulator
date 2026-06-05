// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "MultiAgentTypes.h"
#include "FormationComponent.generated.h"

/**
 * 编队控制组件
 *
 * 管理单个 Agent 的编队偏移量，支持平滑过渡。
 * 编队偏移量相对于 Leader 位置（或质心），由 AgentManager 统一计算并分发。
 */
UCLASS(ClassGroup = (MultiAgent), meta = (BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UFormationComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UFormationComponent();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/**
	 * 设置目标编队偏移量
	 * 由 AgentManager 在编队配置变更时调用
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|Formation")
	void SetTargetOffset(const FVector& NewOffset);

	/**
	 * 获取当前插值后的编队偏移量
	 * 用于混入 NMPC 参考轨迹点
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|Formation")
	FVector GetCurrentOffset() const { return CurrentFormationOffset; }

	/**
	 * 获取目标编队偏移量
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|Formation")
	FVector GetTargetOffset() const { return TargetFormationOffset; }

	/**
	 * 是否正在过渡中
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|Formation")
	bool IsTransitioning() const { return bIsTransitioning; }

	// 过渡速度 (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MultiAgent|Formation")
	float TransitionSpeed = 500.0f;

private:
	// 目标编队偏移量
	FVector TargetFormationOffset = FVector::ZeroVector;

	// 当前编队偏移量（平滑插值后）
	FVector CurrentFormationOffset = FVector::ZeroVector;

	// 是否正在过渡
	bool bIsTransitioning = false;
};
