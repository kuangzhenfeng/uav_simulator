// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "../Scenario/ScenarioTypes.h"
#include "DynamicObstacleActor.generated.h"

class UBoxComponent;

/**
 * 动态障碍驱动 Actor。
 *
 * 由 ScenarioLoader 在装配阶段 Spawn，作为 ObstacleManager 中 FObstacleInfo.LinkedActor 的载体。
 * 按 MovementType 自驱动运动，ObstacleManager::UpdateDynamicObstacles 会每帧从本 Actor
 * 反算位置/速度，从而把声明为动态的障碍纳入规划层。
 *
 * - Static：不由此 Actor 处理（ScenarioLoader 仍 Spawn 裸 AActor）。
 * - LinearVelocity：沿 LinearVelocity 匀速直线。
 * - PatrolLoop：沿 PatrolPoints 循环推进。
 * - PatrolPingPong：沿 PatrolPoints 到末端后反向，往返运动。
 *
 * 可视化壳由蓝图负责（与裸 AActor 方案一致），本 Actor 仅持有 USceneComponent 根，
 * 保证 SetActorLocation 生效并具备可销毁性。
 */
UCLASS(ClassGroup = (Planning), meta = (BlueprintSpawnableComponent))
class UAV_SIMULATOR_API ADynamicObstacleActor : public AActor
{
	GENERATED_BODY()

public:
	ADynamicObstacleActor();

	/**
	 * 由场景声明装配运行参数。
	 * 将 Actor 初始位姿落到 PatrolPoints[0]（若非空）或 Entry.Center，
	 * 并填充运动模型字段。
	 */
	void Configure(const FScenarioObstacleEntry& Entry);

	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

private:
	/** 巡逻推进：沿段推进并处理跨段/反向。 */
	void AdvanceAlongPoints(float DeltaTime);

	/** 重新计算当前段（起点/终点）的方向与长度缓存。 */
	void RecomputeCurrentSegment();

	// ===== 运动模型参数（由 Configure 写入） =====

	/** 运动模型 */
	UPROPERTY(VisibleAnywhere, Category = "DynamicObstacle")
	EObstacleMovementType MovementType = EObstacleMovementType::Static;

	/** LinearVelocity 模式的匀速速度（cm/s） */
	UPROPERTY(VisibleAnywhere, Category = "DynamicObstacle")
	FVector LinearVelocity = FVector::ZeroVector;

	/** 巡逻航点（世界坐标 cm） */
	UPROPERTY(VisibleAnywhere, Category = "DynamicObstacle")
	TArray<FVector> PatrolPoints;

	/** 巡逻速度（cm/s） */
	UPROPERTY(VisibleAnywhere, Category = "DynamicObstacle")
	float PatrolSpeed = 300.0f;

	// ===== 运行时状态 =====

	/** 当前段起点在 PatrolPoints 中的索引（PatrolPingPong 反向时跟随方向变化） */
	int32 CurrentSegmentIndex = 0;

	/** 当前段沿方向的已走距离（cm） */
	float SegmentDistance = 0.0f;

	/** PingPong 当前推进方向（true=正向递增，false=反向递减） */
	bool bForward = true;

	/** 当前段方向向量（归一化） */
	FVector SegmentDirection = FVector::ForwardVector;

	/** 当前段长度（cm） */
	float SegmentLength = 0.0f;

	/** 段缓存是否已对当前段索引有效 */
	bool bSegmentValid = false;
};
