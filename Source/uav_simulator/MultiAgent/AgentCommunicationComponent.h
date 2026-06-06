// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "MultiAgentTypes.h"
#include "AgentCommunicationComponent.generated.h"

class AMultiAgentGameMode;

/**
 * Agent 通信组件
 *
 * 负责 Agent 间的状态广播和邻居发现。
 * 第一期实现理想通信（零延迟、零丢包），通过 AgentManager 直接交换数据。
 * 架构预留延迟/丢包模拟接口，后续可扩展。
 */
UCLASS(ClassGroup = (MultiAgent), meta = (BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UAgentCommunicationComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UAgentCommunicationComponent();

	virtual void BeginPlay() override;

	/**
	 * 广播自身状态到所有 Agent
	 * 理想通信模式：直接写入 AgentManager 状态缓存
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|Communication")
	void BroadcastState(const FAgentStateSnapshot& State);

	/**
	 * 发送定向消息给指定 Agent
	 * 理想通信模式：直接写入目标 Agent 的消息队列
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|Communication")
	void SendMessage(int32 ReceiverID, const FAgentMessage& Message);

	/**
	 * 接收邻居 Agent 的状态
	 * @param Radius 查询半径 (cm)
	 * @return 范围内邻居的状态列表
	 */
	UFUNCTION(BlueprintCallable, Category = "MultiAgent|Communication")
	TArray<FAgentStateSnapshot> ReceiveNeighborStates(float Radius);

	// 设置 OwnerAgentID（由 UAVPawn 在注册后调用）
	void SetOwnerAgentID(int32 InAgentID) { OwnerAgentID = InAgentID; }

	// ---- 通信模拟配置（预留，当前未启用）----

	// 是否模拟通信延迟
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MultiAgent|Communication|Simulation")
	bool bSimulateDelay = false;

	// 模拟延迟 (秒)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MultiAgent|Communication|Simulation",
		meta = (EditCondition = "bSimulateDelay"))
	float SimulatedLatency = 0.0f;

	// 模拟丢包率 (0..1)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MultiAgent|Communication|Simulation",
		meta = (EditCondition = "bSimulateDelay"))
	float PacketLossRate = 0.0f;

private:
	// 缓存 AgentManager 引用
	TWeakObjectPtr<AMultiAgentGameMode> CachedGameMode;

	// 本 Agent 的 ID
	int32 OwnerAgentID = -1;

	// 获取 AgentManager（带缓存）
	AMultiAgentGameMode* GetAgentManager() const;
};
