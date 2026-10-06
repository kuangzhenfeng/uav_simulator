// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "../Scenario/ScenarioTypes.h"
#include "DynamicObstacleActor.generated.h"

class UStaticMeshComponent;

/** 场景障碍 Actor：声明几何同时驱动网格、碰撞体和规划快照，支持静态及动态运动。 */
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
	virtual FVector GetVelocity() const override;
	FObstacleInfo GetObstacleSnapshot() const;

protected:
	virtual void BeginPlay() override;

private:
	UPROPERTY(VisibleAnywhere, Category = "Obstacle")
	TObjectPtr<UStaticMeshComponent> ObstacleMesh;

	UPROPERTY(VisibleAnywhere, Category = "Obstacle")
	FScenarioObstacleEntry Definition;

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
