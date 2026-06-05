// Copyright Epic Games, Inc. All Rights Reserved.

#include "AgentCommunicationComponent.h"
#include "AgentManager.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UAgentCommunicationComponent::UAgentCommunicationComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UAgentCommunicationComponent::BeginPlay()
{
	Super::BeginPlay();
	// 预缓存 AgentManager 引用
	CachedGameMode = GetAgentManager();
}

AMultiAgentGameMode* UAgentCommunicationComponent::GetAgentManager() const
{
	if (CachedGameMode.IsValid())
	{
		return CachedGameMode.Get();
	}

	UWorld* World = GetWorld();
	if (World)
	{
		return World->GetAuthGameMode<AMultiAgentGameMode>();
	}
	return nullptr;
}

void UAgentCommunicationComponent::BroadcastState(const FAgentStateSnapshot& State)
{
	// 理想通信模式：直接更新 AgentManager 状态缓存
	// AgentManager 在 Tick 中已通过 RefreshStateCache 更新
	// 此处用于额外的高频手动广播场景
	OwnerAgentID = State.AgentID;
}

void UAgentCommunicationComponent::SendMessage(int32 ReceiverID, const FAgentMessage& Message)
{
	// 理想通信模式：AgentManager 的状态缓存即为共享数据
	// 定向消息暂不实现，通过 AgentManager 的 GetNeighborStates 替代
}

TArray<FAgentStateSnapshot> UAgentCommunicationComponent::ReceiveNeighborStates(float Radius)
{
	AMultiAgentGameMode* GM = GetAgentManager();
	if (!GM)
	{
		return TArray<FAgentStateSnapshot>();
	}
	return GM->GetNeighborStates(OwnerAgentID, Radius);
}
