// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "../Planning/NMPCAvoidance.h"
#include "StabilityScorer.generated.h"

/**
 * 稳定性评分结果 (各项 0-100)
 */
USTRUCT(BlueprintType)
struct FStabilityScore
{
	GENERATED_BODY()

	// 综合评分
	UPROPERTY(BlueprintReadOnly, Category = "Score")
	float CompositeScore = 100.f;

	// 飞行稳定性评分 (姿态+角速度+速度平滑度)
	UPROPERTY(BlueprintReadOnly, Category = "Score")
	float FlightScore = 100.f;

	// 避障稳定性评分 (障碍距离+修正平滑度)
	UPROPERTY(BlueprintReadOnly, Category = "Score")
	float AvoidanceScore = 100.f;

	// 子项
	UPROPERTY(BlueprintReadOnly, Category = "Score|Detail")
	float AttitudeScore = 100.f;

	UPROPERTY(BlueprintReadOnly, Category = "Score|Detail")
	float AngularVelocityScore = 100.f;

	UPROPERTY(BlueprintReadOnly, Category = "Score|Detail")
	float VelocitySmoothnessScore = 100.f;

	UPROPERTY(BlueprintReadOnly, Category = "Score|Detail")
	float ObstacleDistanceScore = 100.f;

	UPROPERTY(BlueprintReadOnly, Category = "Score|Detail")
	float AvoidanceSmoothnessScore = 100.f;
};

class UObstacleManager;

/**
 * 飞行与避障稳定性评分组件
 *
 * 使用方式:
 *   1. 在 UAVPawn::Tick 中调用 UpdateFlightScore
 *   2. 在 NMPC 避障结果产生后调用 UpdateAvoidanceScore
 *   3. 调用 DrawScoreHUD 在场景中显示评分
 */
UCLASS(ClassGroup=(Debug), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UStabilityScorer : public UActorComponent
{
	GENERATED_BODY()

public:
	UStabilityScorer();

	/** 更新飞行稳定性评分 (每帧调用) */
	UFUNCTION(BlueprintCallable, Category = "Stability")
	void UpdateFlightScore(const FUAVState& State, UObstacleManager* ObstacleManager);

	/** 更新避障稳定性评分 (NMPC 求解后调用) */
	UFUNCTION(BlueprintCallable, Category = "Stability")
	void UpdateAvoidanceScore(const FNMPCAvoidanceResult& AvoidanceResult);

	/** 在世界坐标处绘制评分 HUD */
	UFUNCTION(BlueprintCallable, Category = "Stability")
	void DrawScoreHUD(const FVector& WorldLocation) const;

	UFUNCTION(BlueprintCallable, Category = "Stability")
	FStabilityScore GetCurrentScore() const { return Score; }

	// ---- 可调参数 ----

	// EMA 平滑系数 (越小越平滑, 建议 0.05~0.15)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings", meta = (ClampMin = "0.01", ClampMax = "1.0"))
	float SmoothAlpha = 0.08f;

	// 障碍物安全距离阈值 (cm), 低于此距离开始扣分
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
	float SafeDistance = 300.f;

	// 最大允许倾斜角之和 |Roll|+|Pitch| (度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
	float MaxTiltDeg = 45.f;

	// 最大允许角速度幅值 (rad/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
	float MaxAngVelMag = 3.14f;

	// 最大允许单帧速度变化量 (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
	float MaxVelChange = 300.f;

	// 最大斥力幅值, 用于避障平滑评分 (cm/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
	float MaxRepulsiveMag = 500.f;

private:
	FStabilityScore Score;
	FVector PrevVelocity = FVector::ZeroVector;
	bool bHasPrevVelocity = false;

	FORCEINLINE float EMA(float Prev, float Cur) const { return Prev + SmoothAlpha * (Cur - Prev); }
	FORCEINLINE FColor ToColor(float S) const
	{
		return S >= 75.f ? FColor::Green : (S >= 40.f ? FColor::Yellow : FColor::Red);
	}
};
