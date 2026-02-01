// Copyright Epic Games, Inc. All Rights Reserved.

#include "MissionComponent.h"
#include "uav_simulator/Core/UAVPawn.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UMissionComponent::UMissionComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;

	// 初始化状态
	MissionState = EMissionState::Idle;
	CurrentWaypointIndex = 0;
	CurrentLoopCount = 0;
	bPingPongForward = true;
	CurrentHoverTimer = 0.0f;
	bIsHovering = false;
	MissionStartPosition = FVector::ZeroVector;
	bAutoCheckWaypoints = false; // 默认关闭，由行为树控制
}

void UMissionComponent::BeginPlay()
{
	Super::BeginPlay();

	// 如果有预设航点，设置为 Ready 状态
	if (Waypoints.Num() > 0)
	{
		SetMissionState(EMissionState::Ready);
	}
}

void UMissionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// 只在运行状态且启用自动检测时检查航点
	if (bAutoCheckWaypoints && MissionState == EMissionState::Running)
	{
		CheckWaypointReached(DeltaTime);
	}
}

// ==================== 航点管理 ====================

void UMissionComponent::SetWaypoints(const TArray<FVector>& InWaypoints)
{
	Waypoints.Empty();
	Waypoints.Reserve(InWaypoints.Num());

	for (const FVector& Position : InWaypoints)
	{
		Waypoints.Add(FMissionWaypoint(Position));
	}

	// 更新状态
	if (Waypoints.Num() > 0 && MissionState == EMissionState::Idle)
	{
		SetMissionState(EMissionState::Ready);
	}
	else if (Waypoints.Num() == 0)
	{
		SetMissionState(EMissionState::Idle);
	}

	UE_LOG(LogUAVMission, Log, TEXT("SetWaypoints: %d waypoints set"), Waypoints.Num());
}

void UMissionComponent::SetMissionWaypoints(const TArray<FMissionWaypoint>& InWaypoints)
{
	Waypoints = InWaypoints;

	// 更新状态
	if (Waypoints.Num() > 0 && MissionState == EMissionState::Idle)
	{
		SetMissionState(EMissionState::Ready);
	}
	else if (Waypoints.Num() == 0)
	{
		SetMissionState(EMissionState::Idle);
	}

	UE_LOG(LogUAVMission, Log, TEXT("SetMissionWaypoints: %d waypoints set"), Waypoints.Num());
}

void UMissionComponent::AddWaypoint(const FVector& Position, float HoverDuration, float DesiredSpeed)
{
	FMissionWaypoint Waypoint(Position, HoverDuration, DesiredSpeed);
	AddMissionWaypoint(Waypoint);
}

void UMissionComponent::AddMissionWaypoint(const FMissionWaypoint& Waypoint)
{
	Waypoints.Add(Waypoint);

	if (MissionState == EMissionState::Idle)
	{
		SetMissionState(EMissionState::Ready);
	}

	UE_LOG(LogUAVMission, Verbose, TEXT("AddMissionWaypoint: Added waypoint at %s, total: %d"),
		*Waypoint.Position.ToString(), Waypoints.Num());
}

bool UMissionComponent::RemoveWaypoint(int32 Index)
{
	if (!Waypoints.IsValidIndex(Index))
	{
		UE_LOG(LogUAVMission, Warning, TEXT("RemoveWaypoint: Invalid index %d"), Index);
		return false;
	}

	Waypoints.RemoveAt(Index);

	// 调整当前索引
	if (CurrentWaypointIndex >= Waypoints.Num())
	{
		CurrentWaypointIndex = FMath::Max(0, Waypoints.Num() - 1);
	}

	// 更新状态
	if (Waypoints.Num() == 0)
	{
		SetMissionState(EMissionState::Idle);
	}

	return true;
}

void UMissionComponent::ClearWaypoints()
{
	Waypoints.Empty();
	CurrentWaypointIndex = 0;
	CurrentLoopCount = 0;
	bPingPongForward = true;
	SetMissionState(EMissionState::Idle);

	UE_LOG(LogUAVMission, Log, TEXT("ClearWaypoints: All waypoints cleared"));
}

void UMissionComponent::InsertWaypoint(int32 Index, const FMissionWaypoint& Waypoint)
{
	if (Index < 0)
	{
		Index = 0;
	}
	else if (Index > Waypoints.Num())
	{
		Index = Waypoints.Num();
	}

	Waypoints.Insert(Waypoint, Index);

	// 如果插入位置在当前索引之前，需要调整当前索引
	if (Index <= CurrentWaypointIndex && MissionState == EMissionState::Running)
	{
		CurrentWaypointIndex++;
	}

	if (MissionState == EMissionState::Idle)
	{
		SetMissionState(EMissionState::Ready);
	}
}

TArray<FVector> UMissionComponent::GetWaypointPositions() const
{
	TArray<FVector> Positions;
	Positions.Reserve(Waypoints.Num());

	for (const FMissionWaypoint& Waypoint : Waypoints)
	{
		Positions.Add(Waypoint.Position);
	}

	return Positions;
}

bool UMissionComponent::GetCurrentWaypoint(FMissionWaypoint& OutWaypoint) const
{
	if (!Waypoints.IsValidIndex(CurrentWaypointIndex))
	{
		return false;
	}

	OutWaypoint = Waypoints[CurrentWaypointIndex];
	return true;
}

bool UMissionComponent::GetWaypointAt(int32 Index, FMissionWaypoint& OutWaypoint) const
{
	if (!Waypoints.IsValidIndex(Index))
	{
		return false;
	}

	OutWaypoint = Waypoints[Index];
	return true;
}

// ==================== 任务控制 ====================

bool UMissionComponent::StartMission()
{
	if (Waypoints.Num() == 0)
	{
		UE_LOG(LogUAVMission, Warning, TEXT("StartMission: No waypoints set"));
		OnMissionFailed.Broadcast(TEXT("No waypoints set"));
		return false;
	}

	if (MissionState == EMissionState::Running)
	{
		UE_LOG(LogUAVMission, Warning, TEXT("StartMission: Mission already running"));
		return false;
	}

	// 记录起始位置
	AUAVPawn* UAV = GetOwnerUAV();
	if (UAV)
	{
		MissionStartPosition = UAV->GetActorLocation();
	}

	// 重置状态
	CurrentWaypointIndex = 0;
	CurrentLoopCount = 0;
	bPingPongForward = true;
	CurrentHoverTimer = 0.0f;
	bIsHovering = false;

	SetMissionState(EMissionState::Running);
	OnMissionStarted.Broadcast();

	UE_LOG(LogUAVMission, Log, TEXT("StartMission: Mission started with %d waypoints"), Waypoints.Num());
	return true;
}

void UMissionComponent::StopMission()
{
	if (MissionState == EMissionState::Idle)
	{
		return;
	}

	EMissionState PreviousState = MissionState;
	SetMissionState(EMissionState::Idle);

	// 重置状态
	CurrentWaypointIndex = 0;
	CurrentLoopCount = 0;
	bPingPongForward = true;
	CurrentHoverTimer = 0.0f;
	bIsHovering = false;

	UE_LOG(LogUAVMission, Log, TEXT("StopMission: Mission stopped from state %d"), static_cast<int32>(PreviousState));
}

bool UMissionComponent::PauseMission()
{
	if (MissionState != EMissionState::Running)
	{
		UE_LOG(LogUAVMission, Warning, TEXT("PauseMission: Mission not running"));
		return false;
	}

	SetMissionState(EMissionState::Paused);
	OnMissionPaused.Broadcast(CurrentWaypointIndex);

	UE_LOG(LogUAVMission, Log, TEXT("PauseMission: Mission paused at waypoint %d"), CurrentWaypointIndex);
	return true;
}

bool UMissionComponent::ResumeMission()
{
	if (MissionState != EMissionState::Paused)
	{
		UE_LOG(LogUAVMission, Warning, TEXT("ResumeMission: Mission not paused"));
		return false;
	}

	SetMissionState(EMissionState::Running);
	OnMissionResumed.Broadcast(CurrentWaypointIndex);

	UE_LOG(LogUAVMission, Log, TEXT("ResumeMission: Mission resumed at waypoint %d"), CurrentWaypointIndex);
	return true;
}

bool UMissionComponent::AdvanceToNextWaypoint()
{
	if (MissionState != EMissionState::Running && MissionState != EMissionState::Paused)
	{
		return false;
	}

	// 触发当前航点到达事件
	OnWaypointReached.Broadcast(CurrentWaypointIndex);

	// 处理航点到达逻辑
	HandleWaypointReached();

	return true;
}

bool UMissionComponent::GoToWaypoint(int32 Index)
{
	if (!Waypoints.IsValidIndex(Index))
	{
		UE_LOG(LogUAVMission, Warning, TEXT("GoToWaypoint: Invalid index %d"), Index);
		return false;
	}

	CurrentWaypointIndex = Index;
	CurrentHoverTimer = 0.0f;
	bIsHovering = false;

	UE_LOG(LogUAVMission, Log, TEXT("GoToWaypoint: Jumped to waypoint %d"), Index);
	return true;
}

void UMissionComponent::ResetMission()
{
	CurrentWaypointIndex = 0;
	CurrentLoopCount = 0;
	bPingPongForward = true;
	CurrentHoverTimer = 0.0f;
	bIsHovering = false;

	if (Waypoints.Num() > 0)
	{
		SetMissionState(EMissionState::Ready);
	}
	else
	{
		SetMissionState(EMissionState::Idle);
	}

	UE_LOG(LogUAVMission, Log, TEXT("ResetMission: Mission reset"));
}

// ==================== 状态查询 ====================

float UMissionComponent::GetMissionProgress() const
{
	if (Waypoints.Num() == 0)
	{
		return 0.0f;
	}

	if (MissionState == EMissionState::Completed)
	{
		return 1.0f;
	}

	// 基于当前航点索引计算进度
	return static_cast<float>(CurrentWaypointIndex) / static_cast<float>(Waypoints.Num());
}

float UMissionComponent::GetDistanceToCurrentWaypoint() const
{
	AUAVPawn* UAV = GetOwnerUAV();
	if (!UAV || !Waypoints.IsValidIndex(CurrentWaypointIndex))
	{
		return -1.0f;
	}

	return FVector::Dist(UAV->GetActorLocation(), Waypoints[CurrentWaypointIndex].Position);
}

bool UMissionComponent::HasReachedCurrentWaypoint() const
{
	float Distance = GetDistanceToCurrentWaypoint();
	return Distance >= 0.0f && Distance <= Config.WaypointReachThreshold;
}

// ==================== 配置 ====================

void UMissionComponent::SetMissionConfig(const FMissionConfig& InConfig)
{
	Config = InConfig;
	UE_LOG(LogUAVMission, Log, TEXT("SetMissionConfig: Config updated"));
}

void UMissionComponent::SetMissionMode(EMissionMode InMode)
{
	Config.Mode = InMode;
}

void UMissionComponent::SetDefaultSpeed(float InSpeed)
{
	Config.DefaultSpeed = FMath::Clamp(InSpeed, 10.0f, 2000.0f);
}

// ==================== 内部方法 ====================

void UMissionComponent::SetMissionState(EMissionState NewState)
{
	if (MissionState == NewState)
	{
		return;
	}

	EMissionState OldState = MissionState;
	MissionState = NewState;

	OnMissionStateChanged.Broadcast(OldState, NewState);

	UE_LOG(LogUAVMission, Verbose, TEXT("SetMissionState: %d -> %d"),
		static_cast<int32>(OldState), static_cast<int32>(NewState));
}

void UMissionComponent::HandleWaypointReached()
{
	UE_LOG(LogUAVMission, Log, TEXT("HandleWaypointReached: Reached waypoint %d"), CurrentWaypointIndex);

	// 根据任务模式处理下一步
	switch (Config.Mode)
	{
	case EMissionMode::Once:
		{
			// 单次模式：前进到下一个航点
			CurrentWaypointIndex++;
			if (CurrentWaypointIndex >= Waypoints.Num())
			{
				HandleMissionCompleted();
			}
		}
		break;

	case EMissionMode::Loop:
		{
			// 循环模式
			CurrentWaypointIndex++;
			if (CurrentWaypointIndex >= Waypoints.Num())
			{
				if (!HandleLoopLogic())
				{
					HandleMissionCompleted();
				}
				else
				{
					CurrentWaypointIndex = 0;
				}
			}
		}
		break;

	case EMissionMode::PingPong:
		{
			// 往返模式
			if (bPingPongForward)
			{
				CurrentWaypointIndex++;
				if (CurrentWaypointIndex >= Waypoints.Num())
				{
					// 到达终点，反向
					bPingPongForward = false;
					CurrentWaypointIndex = Waypoints.Num() - 2;
					if (CurrentWaypointIndex < 0)
					{
						// 只有一个航点的情况
						if (!HandleLoopLogic())
						{
							HandleMissionCompleted();
						}
						else
						{
							CurrentWaypointIndex = 0;
							bPingPongForward = true;
						}
					}
				}
			}
			else
			{
				CurrentWaypointIndex--;
				if (CurrentWaypointIndex < 0)
				{
					// 回到起点，检查循环
					if (!HandleLoopLogic())
					{
						HandleMissionCompleted();
					}
					else
					{
						CurrentWaypointIndex = 1;
						bPingPongForward = true;
						if (Waypoints.Num() == 1)
						{
							CurrentWaypointIndex = 0;
						}
					}
				}
			}
		}
		break;
	}
}

void UMissionComponent::HandleMissionCompleted()
{
	// 如果需要返回起点
	if (Config.bReturnToStart)
	{
		// 添加返回起点的逻辑（可以通过事件让外部处理）
		UE_LOG(LogUAVMission, Log, TEXT("HandleMissionCompleted: Returning to start position"));
	}

	SetMissionState(EMissionState::Completed);
	OnMissionCompleted.Broadcast(true);

	UE_LOG(LogUAVMission, Log, TEXT("HandleMissionCompleted: Mission completed successfully"));
}

bool UMissionComponent::HandleLoopLogic()
{
	CurrentLoopCount++;

	// -1 表示无限循环
	if (Config.LoopCount < 0)
	{
		UE_LOG(LogUAVMission, Log, TEXT("HandleLoopLogic: Infinite loop, count: %d"), CurrentLoopCount);
		return true;
	}

	// 检查是否还有剩余循环次数
	if (CurrentLoopCount < Config.LoopCount)
	{
		UE_LOG(LogUAVMission, Log, TEXT("HandleLoopLogic: Loop %d/%d"), CurrentLoopCount + 1, Config.LoopCount);
		return true;
	}

	UE_LOG(LogUAVMission, Log, TEXT("HandleLoopLogic: All loops completed"));
	return false;
}

AUAVPawn* UMissionComponent::GetOwnerUAV() const
{
	return Cast<AUAVPawn>(GetOwner());
}

void UMissionComponent::CheckWaypointReached(float DeltaTime)
{
	if (!Waypoints.IsValidIndex(CurrentWaypointIndex))
	{
		return;
	}

	// 如果正在悬停
	if (bIsHovering)
	{
		CurrentHoverTimer += DeltaTime;
		float RequiredHoverTime = Waypoints[CurrentWaypointIndex].HoverDuration;

		if (CurrentHoverTimer >= RequiredHoverTime)
		{
			// 悬停完成，前进到下一个航点
			bIsHovering = false;
			CurrentHoverTimer = 0.0f;
			AdvanceToNextWaypoint();
		}
		return;
	}

	// 检查是否到达当前航点
	if (HasReachedCurrentWaypoint())
	{
		float HoverDuration = Waypoints[CurrentWaypointIndex].HoverDuration;

		if (HoverDuration > 0.0f)
		{
			// 开始悬停
			bIsHovering = true;
			CurrentHoverTimer = 0.0f;
			UE_LOG(LogUAVMission, Log, TEXT("CheckWaypointReached: Starting hover at waypoint %d for %.1f seconds"),
				CurrentWaypointIndex, HoverDuration);
		}
		else
		{
			// 直接前进到下一个航点
			AdvanceToNextWaypoint();
		}
	}
}
