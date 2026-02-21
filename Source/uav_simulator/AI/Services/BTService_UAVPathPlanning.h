// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTService.h"
#include "uav_simulator/Core/UAVTypes.h"
#include "BTService_UAVPathPlanning.generated.h"

class UNMPCAvoidance;

/**
 * 行为树服务：后台持续进行路径规划和避障检测
 *
 * 两层架构：
 * - Global Planner: 基于已知障碍物的全局路径规划 (A* / RRT)
 * - Local Planner: 基于 NMPC 的实时局部避障
 */
UCLASS()
class UAV_SIMULATOR_API UBTService_UAVPathPlanning : public UBTService
{
	GENERATED_BODY()

public:
	UBTService_UAVPathPlanning();

	virtual void TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
	virtual FString GetStaticDescription() const override;

protected:
	// ---- 模式选择 ----

	// 是否使用预定义航点模式（从UAVPawn获取航点，逐段 A* 避障）
	UPROPERTY(EditAnywhere, Category = "Path Planning")
	bool bUsePresetWaypoints = true;

	// 目标位置的黑板键（仅在非预定义航点模式下使用）
	UPROPERTY(EditAnywhere, Category = "Blackboard", meta = (EditCondition = "!bUsePresetWaypoints"))
	FBlackboardKeySelector TargetLocationKey;

	// 路径规划算法
	UPROPERTY(EditAnywhere, Category = "Path Planning")
	EPathPlanningAlgorithm PathPlanningAlgorithm = EPathPlanningAlgorithm::AStar;

	// ---- Global Planner 参数 ----

	// 重规划距离阈值 (cm) - 目标移动超过此距离时触发重规划
	UPROPERTY(EditAnywhere, Category = "Global Planner")
	float ReplanningThreshold = 200.0f;

	// 安全边距 (cm)
	UPROPERTY(EditAnywhere, Category = "Global Planner")
	float SafetyMargin = 50.0f;

	// 最大速度 (cm/s)
	UPROPERTY(EditAnywhere, Category = "Global Planner")
	float MaxVelocity = 2000.0f;

	// 最大加速度 (cm/s²)
	UPROPERTY(EditAnywhere, Category = "Global Planner")
	float MaxAcceleration = 400.0f;

	// ---- Local Planner 参数 ----

	// 是否启用局部避障 (NMPC)
	UPROPERTY(EditAnywhere, Category = "Local Planner")
	bool bEnableDynamicAvoidance = true;

	// 碰撞检测距离 (cm) - 检测前方此距离内的障碍物
	UPROPERTY(EditAnywhere, Category = "Local Planner")
	float CollisionCheckDistance = 3000.0f;

	// 碰撞警告距离 (cm) - 触发 NMPC 修正的距离
	UPROPERTY(EditAnywhere, Category = "Local Planner")
	float CollisionWarningDistance = 300.0f;

	// Local Planner 连续失败次数阈值 - 超过此值触发 Global Replan
	UPROPERTY(EditAnywhere, Category = "Local Planner")
	int32 LocalPlannerFailThreshold = 5;

private:
	// 上次目标位置（仅在非预定义航点模式下使用）
	FVector LastTargetLocation;

	// 上次规划时间
	float LastPlanningTime;

	// 是否已处理过预定义航点
	bool bWaypointsProcessed = false;

	// Local Planner stuck 计数（负值为重规划冷却期）
	int32 LocalPlannerStuckCount = 0;

	// 上一帧是否处于 stuck 状态（用于检测 N->Y 转换）
	bool bWasStuckLastFrame = false;

	// 原始航点（用于 Global Replan）
	TArray<FVector> OriginalWaypoints;

	// 全局规划路径（A* 输出，用于可视化）
	TArray<FVector> GlobalPlannedPath;

	// Local Planner 实例
	UPROPERTY()
	TObjectPtr<UNMPCAvoidance> NMPCAvoidanceInstance;

	// ---- Global Planner 方法 ----

	// 检查是否需要重规划
	bool ShouldReplan(const FVector& CurrentTarget) const;

	// 执行路径规划（单目标点，使用算法规划）
	void PerformPathPlanning(class AUAVPawn* UAVPawn, const FVector& TargetLocation);

	// 处理预定义航点：逐段 A* 避障 + 全局路径精简 + 轨迹优化
	void ProcessPresetWaypoints(class AUAVPawn* UAVPawn);

	// 多航段路径规划：逐段 A* + 拼接 + line-of-sight 精简
	bool PlanMultiSegmentPath(class AUAVPawn* UAVPawn, const TArray<FVector>& Waypoints, TArray<FVector>& OutPath);

	// 创建路径规划器实例（消除重复代码）
	class UPathPlanner* CreatePathPlanner(class AUAVPawn* UAVPawn, const TArray<FObstacleInfo>& Obstacles);

	// 全局路径精简：line-of-sight 去除冗余中间点
	void SimplifyGlobalPath(TArray<FVector>& Path, const class UPathPlanner* Planner, float CollisionRadius, const TSet<int32>& WaypointIndices) const;

	// ---- Local Planner 方法 ----

	// 使用 NMPC 进行局部避障检测和修正
	void CheckCollisionAndAvoid(class AUAVPawn* UAVPawn, float DeltaSeconds);

	// 触发全局重规划（Local Planner 失败时调用）
	void TriggerGlobalReplan(class AUAVPawn* UAVPawn);

	// 更新连续卡死计数并判断是否应触发全局重规划。
	bool UpdateStuckStateAndCheckReplan(bool bIsStuck);

	// 获取或创建 NMPCAvoidance 实例
	UNMPCAvoidance* GetNMPCAvoidance();
};
