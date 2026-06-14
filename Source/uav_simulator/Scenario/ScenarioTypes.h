// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "../Core/UAVTypes.h"          // EObstacleType
#include "../Environment/EnvironmentTypes.h" // FWindConfig
#include "../Mission/MissionTypes.h"   // FMissionWaypoint, EMissionMode
#include "../Core/UAVProductTypes.h"   // EUAVModelID
#include "../Planning/NMPCAvoidance.h" // EMPCType
#include "ScenarioTypes.generated.h"

class AUAVPawn;

// ==================== 场景系统的子资产 ====================

/**
 * 单个障碍物的场景声明。
 * 描述逻辑几何，由 ScenarioLoader 注册到 ObstacleManager 并 Spawn 可视化 Mesh。
 * 运动轨迹字段预留（动态障碍二期启用）。
 */
USTRUCT(BlueprintType)
struct FScenarioObstacleEntry
{
	GENERATED_BODY()

	/** 障碍物几何类型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Obstacle")
	EObstacleType Type = EObstacleType::Sphere;

	/** 中心位置（世界坐标系，cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Obstacle")
	FVector Center = FVector::ZeroVector;

	/** 尺寸（Sphere=半径，Box=半尺寸，Cylinder=半径+高度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Obstacle")
	FVector Extents = FVector(100.0f);

	/** 旋转 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Obstacle")
	FRotator Rotation = FRotator::ZeroRotator;

	/** 安全边距（cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Obstacle", meta = (ClampMin = "0.0"))
	float SafetyMargin = 50.0f;

	/** 是否为动态障碍物（二期启用，MVP 保持 false） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Obstacle")
	bool bIsDynamic = false;

	/** 动态障碍匀速速度（cm/s，二期启用） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Obstacle", meta = (EditCondition = "bIsDynamic"))
	FVector Velocity = FVector::ZeroVector;
};

/**
 * 障碍布局：场景中全部静态/动态障碍的逻辑几何声明。
 * 可被多个 UScenario 复用。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UObstacleLayout : public UDataAsset
{
	GENERATED_BODY()

public:
	/** 障碍物条目列表 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	TArray<FScenarioObstacleEntry> Obstacles;
};

/**
 * 风场档案：封装现有 FWindConfig。
 * 可被多个 UScenario 复用。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UWindProfile : public UDataAsset
{
	GENERATED_BODY()

public:
	/** 风场配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	FWindConfig Config;
};

/**
 * 单架机的场景声明。
 */
USTRUCT(BlueprintType)
struct FScenarioAgentEntry
{
	GENERATED_BODY()

	/** UAV 蓝图子类（未指定时使用默认 BP_UAVPawn_Default，由 Loader 决定） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Agent")
	TSubclassOf<AUAVPawn> UAVClass;

	/** 型号 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Agent")
	EUAVModelID ModelID = EUAVModelID::Agri_AG20;

	/** 初始位置（cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Agent")
	FVector InitialPosition = FVector::ZeroVector;

	/** 初始偏航（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Agent")
	float InitialYaw = 0.0f;

	/** 是否为编队长机（二期编队使用） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Agent")
	bool bIsLeader = false;
};

/**
 * 机队配置：显式 Agents[] 数组 + 预留编队参数。
 * 单机场景数组只有一个元素。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UFleetSetup : public UDataAsset
{
	GENERATED_BODY()

public:
	/** 机队成员显式列表 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	TArray<FScenarioAgentEntry> Agents;
};

/**
 * 任务档案：航点序列 + 任务模式。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UMissionProfile : public UDataAsset
{
	GENERATED_BODY()

public:
	/** 航点序列 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	TArray<FMissionWaypoint> Waypoints;

	/** 任务模式（单次/循环/往返） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	EMissionMode Mode = EMissionMode::Once;
};

/**
 * 验收标准：定义"场景成功"的硬指标阈值。
 * ScenarioEvaluator 据此判定 PASS/FAIL。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UAcceptanceCriteria : public UDataAsset
{
	GENERATED_BODY()

public:
	/** 要求到达全部航点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Acceptance")
	bool bRequireAllWaypoints = true;

	/** 航点到达判据（距航点中心 cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Acceptance", meta = (ClampMin = "0.0"))
	float WaypointArrivalRadius = 300.0f;

	/** 最小净空阈值（障碍表面距离 cm），低于则 FAIL */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Acceptance", meta = (ClampMin = "0.0"))
	float MinClearanceCm = 0.0f;

	/** 最大横向轨迹偏差（cm），超过则 FAIL */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Acceptance", meta = (ClampMin = "0.0"))
	float MaxLateralDeviationCm = 300.0f;

	/** 超时阈值（秒），超过则 FAIL */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Acceptance", meta = (ClampMin = "0.0"))
	float TimeoutSeconds = 60.0f;

	/** 能耗预算（归一化 0~1），超过则 FAIL；0 表示不限制 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Acceptance", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float EnergyBudget = 0.0f;
};

/**
 * 算法覆盖项：可选地指定本场景使用的控制算法与模式。
 * 未设置（bOverride=false）时回退到 UAV 默认。
 */
USTRUCT(BlueprintType)
struct FScenarioAlgorithmOverride
{
	GENERATED_BODY()

	/** 是否覆盖控制模式 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Algorithm")
	bool bOverrideControlMode = false;

	/** 覆盖的控制模式 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Algorithm", meta = (EditCondition = "bOverrideControlMode"))
	EUAVControlMode ControlMode = EUAVControlMode::Trajectory;

	/** 是否覆盖 MPC 类型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Algorithm")
	bool bOverrideMPCType = false;

	/** 覆盖的 MPC 类型 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Algorithm", meta = (EditCondition = "bOverrideMPCType"))
	EMPCType MPCType = EMPCType::Nonlinear;
};

/**
 * 场景资产外壳：组合引用式 DataAsset，声明一次完整仿真。
 * 引用五个可复用子资产 + 一个 RandomSeed + 可选算法覆盖。
 * 运行时由 UScenarioLoader 装配。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UScenario : public UDataAsset
{
	GENERATED_BODY()

public:
	/** 场景可读名称 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	FString Name;

	/** 场景描述 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (MultiLine = true))
	FString Description;

	/** 随机种子（同 seed 同结果） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	int32 RandomSeed = 0;

	/** 障碍布局 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	TSoftObjectPtr<UObstacleLayout> ObstacleLayout;

	/** 风场档案 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	TSoftObjectPtr<UWindProfile> WindProfile;

	/** 机队配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	TSoftObjectPtr<UFleetSetup> FleetSetup;

	/** 任务档案 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	TSoftObjectPtr<UMissionProfile> MissionProfile;

	/** 验收标准 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	TSoftObjectPtr<UAcceptanceCriteria> AcceptanceCriteria;

	/** 可选算法覆盖 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	FScenarioAlgorithmOverride AlgorithmOverride;
};
