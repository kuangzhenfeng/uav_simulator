// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTService.h"
#include "uav_simulator/Core/UAVTypes.h"
#include "BTService_UAVPathPlanning.generated.h"

/**
 * 行为树服务：后台持续进行路径规划和避障检测
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
	// 是否使用预定义航点模式（从UAVPawn获取航点，跳过路径规划）
	UPROPERTY(EditAnywhere, Category = "Path Planning")
	bool bUsePresetWaypoints = true;

	// 目标位置的黑板键（仅在非预定义航点模式下使用）
	UPROPERTY(EditAnywhere, Category = "Blackboard", meta = (EditCondition = "!bUsePresetWaypoints"))
	FBlackboardKeySelector TargetLocationKey;

	// 路径规划算法（仅在非预定义航点模式下使用）
	UPROPERTY(EditAnywhere, Category = "Path Planning", meta = (EditCondition = "!bUsePresetWaypoints"))
	EPathPlanningAlgorithm PathPlanningAlgorithm = EPathPlanningAlgorithm::AStar;

	// 重规划距离阈值 (cm) - 目标移动超过此距离时触发重规划
	UPROPERTY(EditAnywhere, Category = "Path Planning")
	float ReplanningThreshold = 200.0f;

	// 碰撞检测距离 (cm) - 检测前方此距离内的障碍物
	UPROPERTY(EditAnywhere, Category = "Collision Avoidance")
	float CollisionCheckDistance = 300.0f;

	// 碰撞警告距离 (cm) - 触发紧急避障的距离
	UPROPERTY(EditAnywhere, Category = "Collision Avoidance")
	float CollisionWarningDistance = 150.0f;

	// 是否启用动态避障
	UPROPERTY(EditAnywhere, Category = "Collision Avoidance")
	bool bEnableDynamicAvoidance = true;

	// 安全边距 (cm)
	UPROPERTY(EditAnywhere, Category = "Path Planning")
	float SafetyMargin = 50.0f;

	// 最大速度 (cm/s)
	UPROPERTY(EditAnywhere, Category = "Path Planning")
	float MaxVelocity = 500.0f;

	// 最大加速度 (cm/s²)
	UPROPERTY(EditAnywhere, Category = "Path Planning")
	float MaxAcceleration = 200.0f;

private:
	// 上次目标位置（仅在非预定义航点模式下使用）
	FVector LastTargetLocation;

	// 上次规划时间
	float LastPlanningTime;

	// 是否已处理过预定义航点
	bool bWaypointsProcessed = false;

	// 检查是否需要重规划
	bool ShouldReplan(const FVector& CurrentTarget) const;

	// 执行路径规划（使用算法规划）
	void PerformPathPlanning(class AUAVPawn* UAVPawn, const FVector& TargetLocation);

	// 执行预定义航点的轨迹生成
	void ProcessPresetWaypoints(class AUAVPawn* UAVPawn);

	// 检查碰撞并执行避障
	void CheckCollisionAndAvoid(class AUAVPawn* UAVPawn, float DeltaSeconds);
};
