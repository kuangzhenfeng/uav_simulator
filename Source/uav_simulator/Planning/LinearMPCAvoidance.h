// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "NMPCAvoidance.h"
#include "LinearMPCAvoidance.generated.h"

/**
 * 线性化MPC避障组件
 * 使用状态空间线性化模型和QP求解器
 * 计算量比NMPC低50-70%
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API ULinearMPCAvoidance : public UActorComponent
{
	GENERATED_BODY()

public:
	ULinearMPCAvoidance();

	// 配置（复用NMPC配置）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Linear MPC")
	FNMPCConfig Config;

	// 计算避障控制
	FNMPCAvoidanceResult ComputeAvoidance(
		const FVector& CurrentPosition,
		const FVector& CurrentVelocity,
		const TArray<FVector>& ReferencePoints,
		const TArray<FObstacleInfo>& Obstacles);

	// 计算到障碍物的距离（复用NMPC方法）
	float CalculateDistanceToObstacle(const FVector& Position, const FObstacleInfo& Obstacle) const;

protected:
	virtual void BeginPlay() override;

private:
	// 线性化模型
	void ComputeLinearization(
		const FVector& Position,
		const FVector& Velocity,
		float Dt,
		TArray<float>& OutA,
		TArray<float>& OutB);

	// QP求解器
	bool SolveQP(
		const FVector& X0,
		const TArray<FVector>& Xref,
		const TArray<FObstacleInfo>& Obstacles,
		TArray<FVector>& OutU);

	// 解析梯度计算
	void ComputeAnalyticGradient(
		const TArray<FVector>& X,
		const TArray<FVector>& U,
		const TArray<FVector>& Xref,
		const TArray<FObstacleInfo>& Obstacles,
		TArray<FVector>& OutGradU);

	// 线性前向仿真
	void ForwardSimulateLinear(
		const FVector& X0,
		const TArray<FVector>& U,
		TArray<FVector>& OutX);

	// 投影到约束集
	void ProjectToConstraints(TArray<FVector>& U);

	// 计算代价函数
	float ComputeCost(
		const TArray<FVector>& X,
		const TArray<FVector>& U,
		const TArray<FVector>& Xref,
		const TArray<FObstacleInfo>& Obstacles);

	// 线性化矩阵（缓存）
	TArray<float> A_matrix;
	TArray<float> B_matrix;
};
