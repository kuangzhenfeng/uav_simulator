// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MissionTypes.generated.h"

/**
 * 任务状态枚举
 */
UENUM(BlueprintType)
enum class EMissionState : uint8
{
	Idle		UMETA(DisplayName = "Idle"),		// 空闲状态
	Ready		UMETA(DisplayName = "Ready"),		// 就绪状态（有航点但未开始）
	Running		UMETA(DisplayName = "Running"),		// 运行中
	Paused		UMETA(DisplayName = "Paused"),		// 暂停
	Completed	UMETA(DisplayName = "Completed"),	// 已完成
	Failed		UMETA(DisplayName = "Failed")		// 失败
};

/**
 * 任务模式枚举
 */
UENUM(BlueprintType)
enum class EMissionMode : uint8
{
	Once		UMETA(DisplayName = "Once"),		// 单次执行
	Loop		UMETA(DisplayName = "Loop"),		// 循环执行
	PingPong	UMETA(DisplayName = "PingPong")		// 往返执行
};

/**
 * 航点数据结构
 */
USTRUCT(BlueprintType)
struct FMissionWaypoint
{
	GENERATED_BODY()

	/** 航点位置 (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
	FVector Position;

	/** 悬停时间 (秒)，0表示不悬停 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission", meta = (ClampMin = "0.0"))
	float HoverDuration;

	/** 期望速度 (cm/s)，0表示使用默认速度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission", meta = (ClampMin = "0.0"))
	float DesiredSpeed;

	/** 期望偏航角 (度)，NaN表示自动计算 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
	float DesiredYaw;

	/** 航点标签，用于标识特殊航点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
	FName Tag;

	FMissionWaypoint()
		: Position(FVector::ZeroVector)
		, HoverDuration(0.0f)
		, DesiredSpeed(0.0f)
		, DesiredYaw(TNumericLimits<float>::Lowest()) // 使用最小值表示"未设置"
		, Tag(NAME_None)
	{
	}

	FMissionWaypoint(const FVector& InPosition)
		: Position(InPosition)
		, HoverDuration(0.0f)
		, DesiredSpeed(0.0f)
		, DesiredYaw(TNumericLimits<float>::Lowest())
		, Tag(NAME_None)
	{
	}

	FMissionWaypoint(const FVector& InPosition, float InHoverDuration, float InDesiredSpeed = 0.0f)
		: Position(InPosition)
		, HoverDuration(InHoverDuration)
		, DesiredSpeed(InDesiredSpeed)
		, DesiredYaw(TNumericLimits<float>::Lowest())
		, Tag(NAME_None)
	{
	}

	/** 检查是否设置了期望偏航角 */
	bool HasDesiredYaw() const
	{
		return DesiredYaw > TNumericLimits<float>::Lowest() + 1.0f;
	}

	/** 检查是否使用默认速度 */
	bool UseDefaultSpeed() const
	{
		return DesiredSpeed <= 0.0f;
	}
};

/**
 * 任务配置
 */
USTRUCT(BlueprintType)
struct FMissionConfig
{
	GENERATED_BODY()

	/** 任务模式 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
	EMissionMode Mode;

	/** 默认飞行速度 (cm/s) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission", meta = (ClampMin = "10.0", ClampMax = "2000.0"))
	float DefaultSpeed;

	/** 最大加速度 (cm/s²) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission", meta = (ClampMin = "10.0", ClampMax = "1000.0"))
	float MaxAcceleration;

	/** 航点到达阈值 (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission", meta = (ClampMin = "1.0", ClampMax = "500.0"))
	float WaypointReachThreshold;

	/** 循环次数，-1表示无限循环（仅在Loop模式下有效） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission", meta = (ClampMin = "-1"))
	int32 LoopCount;

	/** 任务完成后是否返回起点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
	bool bReturnToStart;

	/** 是否启用路径规划（避障） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission")
	bool bEnablePathPlanning;

	// 农业模式参数（ProductType == Agricultural 时有效）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission|Agricultural")
	float StripSpacingM = 4.0f;			// 条带间距（m）

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission|Agricultural")
	float SprayFlowLPerMin = 1.5f;		// 喷洒流量（L/min）

	// 测绘模式参数（ProductType == Mapping 时有效）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission|Mapping")
	float OverlapRatio = 0.8f;			// 航线重叠率

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mission|Mapping")
	float CameraTriggerIntervalM = 5.0f;	// 相机触发间距（m）

	FMissionConfig()
		: Mode(EMissionMode::Once)
		, DefaultSpeed(500.0f)
		, MaxAcceleration(200.0f)
		, WaypointReachThreshold(50.0f)
		, LoopCount(1)
		, bReturnToStart(false)
		, bEnablePathPlanning(false)
	{
	}
};
