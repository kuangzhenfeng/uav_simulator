// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "../Core/UAVTypes.h"
#include "LocalAvoidance.generated.h"

/**
 * APF (Artificial Potential Field) 局部避障结果
 */
USTRUCT(BlueprintType)
struct FLocalAvoidanceResult
{
	GENERATED_BODY()

	// 修正后的期望速度方向 (归一化)
	UPROPERTY(BlueprintReadOnly, Category = "Local Avoidance")
	FVector CorrectedDirection;

	// 合力向量 (未归一化，用于调试可视化)
	UPROPERTY(BlueprintReadOnly, Category = "Local Avoidance")
	FVector TotalForce;

	// 引力分量
	UPROPERTY(BlueprintReadOnly, Category = "Local Avoidance")
	FVector AttractiveForce;

	// 斥力分量
	UPROPERTY(BlueprintReadOnly, Category = "Local Avoidance")
	FVector RepulsiveForce;

	// 是否需要修正 (斥力不为零)
	UPROPERTY(BlueprintReadOnly, Category = "Local Avoidance")
	bool bNeedsCorrection;

	// 是否陷入局部极小值 (合力接近零但未到达目标)
	UPROPERTY(BlueprintReadOnly, Category = "Local Avoidance")
	bool bStuck;

	FLocalAvoidanceResult()
		: CorrectedDirection(FVector::ForwardVector)
		, TotalForce(FVector::ZeroVector)
		, AttractiveForce(FVector::ZeroVector)
		, RepulsiveForce(FVector::ZeroVector)
		, bNeedsCorrection(false)
		, bStuck(false)
	{}
};

/**
 * APF 局部避障器
 * 使用人工势场法实现实时局部避障
 * - 目标产生引力，拉向目标方向
 * - 障碍物产生斥力，推离障碍物
 * - 合力修正速度方向
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API ULocalAvoidance : public UObject
{
	GENERATED_BODY()

public:
	ULocalAvoidance();

	/**
	 * 计算 APF 避障修正
	 * @param CurrentPosition 当前位置
	 * @param TargetPosition 目标位置 (下一个航点)
	 * @param Obstacles 附近障碍物列表
	 * @param CurrentVelocity 当前速度向量
	 * @return 避障结果
	 */
	UFUNCTION(BlueprintCallable, Category = "Local Avoidance")
	FLocalAvoidanceResult ComputeAvoidance(
		const FVector& CurrentPosition,
		const FVector& TargetPosition,
		const TArray<FObstacleInfo>& Obstacles,
		const FVector& CurrentVelocity) const;

	/**
	 * 计算引力 (目标吸引力)
	 * @param CurrentPosition 当前位置
	 * @param TargetPosition 目标位置
	 * @return 引力向量
	 */
	UFUNCTION(BlueprintCallable, Category = "Local Avoidance")
	FVector ComputeAttractiveForce(const FVector& CurrentPosition, const FVector& TargetPosition) const;

	/**
	 * 计算单个障碍物的斥力
	 * @param CurrentPosition 当前位置
	 * @param Obstacle 障碍物信息
	 * @return 斥力向量
	 */
	UFUNCTION(BlueprintCallable, Category = "Local Avoidance")
	FVector ComputeRepulsiveForce(const FVector& CurrentPosition, const FObstacleInfo& Obstacle) const;

	// ---- 参数设置 ----

	// 引力增益
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "APF Parameters")
	float AttractiveGain = 1.0f;

	// 斥力增益
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "APF Parameters")
	float RepulsiveGain = 5000.0f;

	// 障碍物影响距离 (cm) - 超过此距离的障碍物不产生斥力
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "APF Parameters")
	float InfluenceDistance = 500.0f;

	// 最大斥力限制 (cm) - 防止距离过近时斥力爆炸
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "APF Parameters")
	float MaxRepulsiveForce = 2000.0f;

	// 局部极小值检测阈值 - 合力小于此值且未到达目标时判定为 stuck
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "APF Parameters")
	float StuckForceThreshold = 10.0f;

	// 到达目标距离阈值 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "APF Parameters")
	float GoalReachedThreshold = 100.0f;

private:
	// 计算点到障碍物表面的距离
	float CalculateDistanceToObstacle(const FVector& Point, const FObstacleInfo& Obstacle) const;

	// 计算从障碍物表面指向点的方向
	FVector CalculateRepulsionDirection(const FVector& Point, const FObstacleInfo& Obstacle) const;
};
