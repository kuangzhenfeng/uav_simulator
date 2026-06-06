// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "UAVPlayerController.generated.h"

class UCameraSwitcherWidget;
class AUAVPawn;

/**
 * 无人机仿真视角切换控制器
 *
 * 管理视角切换 UI 和相机跟随逻辑。
 * 支持全局视角（默认 Pawn）和无人机跟随视角之间的平滑切换。
 */
UCLASS()
class UAV_SIMULATOR_API AUAVPlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	AUAVPlayerController();

	// 当前跟随的无人机 AgentID（-1 表示全局视角）
	UFUNCTION(BlueprintCallable, Category = "Camera")
	int32 GetCurrentViewAgentID() const { return CurrentViewAgentID; }

	// 切换视角到指定无人机（-1 为全局视角）
	UFUNCTION(BlueprintCallable, Category = "Camera")
	void SwitchView(int32 AgentID);

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

private:
	// 视角切换 UI 面板
	UPROPERTY()
	TObjectPtr<UCameraSwitcherWidget> SwitcherWidget;

	// 默认视角目标（PlayerController 的默认 Pawn）
	UPROPERTY()
	TObjectPtr<AActor> DefaultViewTarget;

	// 当前跟随的无人机 AgentID（-1 表示全局视角）
	int32 CurrentViewAgentID = -1;

	// 上次检测到的无人机数量（用于检测变化并刷新 UI）
	int32 LastDroneCount = -1;

	// UI 刷新间隔（秒）
	float UIRefreshInterval = 1.0f;
	float UIRefreshAccumulator = 0.0f;

	// 视角切换委托回调
	UFUNCTION()
	void OnViewChanged(int32 AgentID);

	// 通过 AgentID 查找无人机
	AUAVPawn* GetDroneByAgentID(int32 AgentID);

	// 获取当前场景中的无人机数量
	int32 GetDroneCount();
};
