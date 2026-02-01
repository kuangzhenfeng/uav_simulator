// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "MissionTypes.h"
#include "MissionComponent.generated.h"

// 前向声明
class AUAVPawn;

// 委托声明
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnMissionStarted);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnMissionCompleted, bool, bSuccess);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnMissionFailed, FString, Reason);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnWaypointReached, int32, WaypointIndex);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnMissionStateChanged, EMissionState, OldState, EMissionState, NewState);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnMissionPaused, int32, CurrentWaypointIndex);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnMissionResumed, int32, CurrentWaypointIndex);

/**
 * 任务管理组件
 * 统一管理 UAV 的航点数据、任务状态和任务配置
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UMissionComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UMissionComponent();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// ==================== 航点管理 ====================

	/** 设置航点（简单版，仅位置） */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	void SetWaypoints(const TArray<FVector>& InWaypoints);

	/** 设置航点（完整版） */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	void SetMissionWaypoints(const TArray<FMissionWaypoint>& InWaypoints);

	/** 添加单个航点 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	void AddWaypoint(const FVector& Position, float HoverDuration = 0.0f, float DesiredSpeed = 0.0f);

	/** 添加完整航点 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	void AddMissionWaypoint(const FMissionWaypoint& Waypoint);

	/** 移除指定索引的航点 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	bool RemoveWaypoint(int32 Index);

	/** 清除所有航点 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	void ClearWaypoints();

	/** 插入航点到指定位置 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	void InsertWaypoint(int32 Index, const FMissionWaypoint& Waypoint);

	/** 获取航点位置数组（简单版） */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	TArray<FVector> GetWaypointPositions() const;

	/** 获取所有航点（完整版） */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	const TArray<FMissionWaypoint>& GetMissionWaypoints() const { return Waypoints; }

	/** 获取当前航点 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	bool GetCurrentWaypoint(FMissionWaypoint& OutWaypoint) const;

	/** 获取指定索引的航点 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	bool GetWaypointAt(int32 Index, FMissionWaypoint& OutWaypoint) const;

	/** 获取航点数量 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	int32 GetWaypointCount() const { return Waypoints.Num(); }

	/** 检查是否有航点 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Waypoints")
	bool HasWaypoints() const { return Waypoints.Num() > 0; }

	// ==================== 任务控制 ====================

	/** 开始任务 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Control")
	bool StartMission();

	/** 停止任务 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Control")
	void StopMission();

	/** 暂停任务 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Control")
	bool PauseMission();

	/** 恢复任务 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Control")
	bool ResumeMission();

	/** 前进到下一个航点 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Control")
	bool AdvanceToNextWaypoint();

	/** 跳转到指定航点 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Control")
	bool GoToWaypoint(int32 Index);

	/** 重置任务（回到起点，状态变为Ready） */
	UFUNCTION(BlueprintCallable, Category = "Mission|Control")
	void ResetMission();

	// ==================== 状态查询 ====================

	/** 获取任务状态 */
	UFUNCTION(BlueprintCallable, Category = "Mission|State")
	EMissionState GetMissionState() const { return MissionState; }

	/** 检查任务是否正在运行 */
	UFUNCTION(BlueprintCallable, Category = "Mission|State")
	bool IsMissionRunning() const { return MissionState == EMissionState::Running; }

	/** 检查任务是否暂停 */
	UFUNCTION(BlueprintCallable, Category = "Mission|State")
	bool IsMissionPaused() const { return MissionState == EMissionState::Paused; }

	/** 检查任务是否完成 */
	UFUNCTION(BlueprintCallable, Category = "Mission|State")
	bool IsMissionCompleted() const { return MissionState == EMissionState::Completed; }

	/** 获取任务进度 (0.0 - 1.0) */
	UFUNCTION(BlueprintCallable, Category = "Mission|State")
	float GetMissionProgress() const;

	/** 获取当前航点索引 */
	UFUNCTION(BlueprintCallable, Category = "Mission|State")
	int32 GetCurrentWaypointIndex() const { return CurrentWaypointIndex; }

	/** 获取当前循环次数 */
	UFUNCTION(BlueprintCallable, Category = "Mission|State")
	int32 GetCurrentLoopCount() const { return CurrentLoopCount; }

	/** 获取到当前航点的距离 */
	UFUNCTION(BlueprintCallable, Category = "Mission|State")
	float GetDistanceToCurrentWaypoint() const;

	/** 检查是否到达当前航点 */
	UFUNCTION(BlueprintCallable, Category = "Mission|State")
	bool HasReachedCurrentWaypoint() const;

	// ==================== 配置 ====================

	/** 获取任务配置 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Config")
	const FMissionConfig& GetMissionConfig() const { return Config; }

	/** 设置任务配置 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Config")
	void SetMissionConfig(const FMissionConfig& InConfig);

	/** 设置任务模式 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Config")
	void SetMissionMode(EMissionMode InMode);

	/** 设置默认速度 */
	UFUNCTION(BlueprintCallable, Category = "Mission|Config")
	void SetDefaultSpeed(float InSpeed);

	// ==================== 事件委托 ====================

	/** 任务开始事件 */
	UPROPERTY(BlueprintAssignable, Category = "Mission|Events")
	FOnMissionStarted OnMissionStarted;

	/** 任务完成事件 */
	UPROPERTY(BlueprintAssignable, Category = "Mission|Events")
	FOnMissionCompleted OnMissionCompleted;

	/** 任务失败事件 */
	UPROPERTY(BlueprintAssignable, Category = "Mission|Events")
	FOnMissionFailed OnMissionFailed;

	/** 到达航点事件 */
	UPROPERTY(BlueprintAssignable, Category = "Mission|Events")
	FOnWaypointReached OnWaypointReached;

	/** 任务状态改变事件 */
	UPROPERTY(BlueprintAssignable, Category = "Mission|Events")
	FOnMissionStateChanged OnMissionStateChanged;

	/** 任务暂停事件 */
	UPROPERTY(BlueprintAssignable, Category = "Mission|Events")
	FOnMissionPaused OnMissionPaused;

	/** 任务恢复事件 */
	UPROPERTY(BlueprintAssignable, Category = "Mission|Events")
	FOnMissionResumed OnMissionResumed;

protected:
	/** 设置任务状态（内部使用） */
	void SetMissionState(EMissionState NewState);

	/** 处理航点到达 */
	void HandleWaypointReached();

	/** 处理任务完成 */
	void HandleMissionCompleted();

	/** 处理循环逻辑 */
	bool HandleLoopLogic();

	/** 获取拥有者 UAVPawn */
	AUAVPawn* GetOwnerUAV() const;

	/** 检查航点到达（在 Tick 中调用） */
	void CheckWaypointReached(float DeltaTime);

protected:
	/** 航点数组 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Mission|Waypoints")
	TArray<FMissionWaypoint> Waypoints;

	/** 任务配置 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Mission|Config")
	FMissionConfig Config;

	/** 当前任务状态 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mission|State")
	EMissionState MissionState;

	/** 当前航点索引 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mission|State")
	int32 CurrentWaypointIndex;

	/** 当前循环次数 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mission|State")
	int32 CurrentLoopCount;

	/** PingPong 模式下的方向（true = 正向，false = 反向） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mission|State")
	bool bPingPongForward;

	/** 当前航点的悬停计时器 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mission|State")
	float CurrentHoverTimer;

	/** 是否正在悬停 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mission|State")
	bool bIsHovering;

	/** 任务起始位置 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mission|State")
	FVector MissionStartPosition;

	/** 是否启用自动航点检测（在 Tick 中检测是否到达航点） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission|Config")
	bool bAutoCheckWaypoints;
};
