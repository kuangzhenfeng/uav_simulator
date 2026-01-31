// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "TrajectoryOptimizer.generated.h"

/**
 * 多项式轨迹段
 * 使用7阶多项式表示单段轨迹
 */
struct FPolynomialSegment
{
	// 多项式系数 (8个系数用于7阶多项式)
	// p(t) = c[0] + c[1]*t + c[2]*t^2 + ... + c[7]*t^7
	TArray<float> CoeffX;
	TArray<float> CoeffY;
	TArray<float> CoeffZ;

	// 段的时间范围
	float StartTime;
	float Duration;

	FPolynomialSegment()
		: StartTime(0.0f)
		, Duration(1.0f)
	{
		CoeffX.SetNum(8);
		CoeffY.SetNum(8);
		CoeffZ.SetNum(8);
		for (int32 i = 0; i < 8; ++i)
		{
			CoeffX[i] = 0.0f;
			CoeffY[i] = 0.0f;
			CoeffZ[i] = 0.0f;
		}
	}
};

/**
 * 轨迹优化器
 * 实现最小Snap (四阶导数) 轨迹优化
 * 生成平滑的多项式轨迹
 */
UCLASS(ClassGroup=(Planning), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UTrajectoryOptimizer : public UActorComponent
{
	GENERATED_BODY()

public:
	UTrajectoryOptimizer();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/**
	 * 从航点生成优化轨迹
	 * @param Waypoints 航点序列
	 * @param MaxVelocity 最大速度 (cm/s)
	 * @param MaxAcceleration 最大加速度 (cm/s²)
	 * @return 优化后的轨迹
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Optimization")
	FTrajectory OptimizeTrajectory(const TArray<FVector>& Waypoints, float MaxVelocity, float MaxAcceleration);

	/**
	 * 从航点和时间分配生成轨迹
	 * @param Waypoints 航点序列
	 * @param SegmentTimes 每段的时间分配
	 * @return 优化后的轨迹
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Optimization")
	FTrajectory OptimizeTrajectoryWithTiming(const TArray<FVector>& Waypoints, const TArray<float>& SegmentTimes);

	/**
	 * 采样轨迹获取指定时间的状态
	 * @param Traj 轨迹
	 * @param Time 时间 (秒)
	 * @return 轨迹点状态
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Optimization")
	FTrajectoryPoint SampleTrajectory(const FTrajectory& Traj, float Time) const;

	/**
	 * 获取轨迹的密集采样点
	 * @param Traj 轨迹
	 * @param SampleInterval 采样间隔 (秒)
	 * @return 采样点数组
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Optimization")
	TArray<FTrajectoryPoint> GetDenseSamples(const FTrajectory& Traj, float SampleInterval = 0.05f) const;

protected:
	// 轨迹采样间隔 (秒)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization Settings")
	float DefaultSampleInterval = 0.05f;

	// 起始速度
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization Settings")
	FVector StartVelocity = FVector::ZeroVector;

	// 终止速度
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization Settings")
	FVector EndVelocity = FVector::ZeroVector;

	// 起始加速度
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization Settings")
	FVector StartAcceleration = FVector::ZeroVector;

	// 终止加速度
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization Settings")
	FVector EndAcceleration = FVector::ZeroVector;

private:
	// 多项式轨迹段
	TArray<FPolynomialSegment> Segments;

	// 计算时间分配
	TArray<float> ComputeTimeAllocation(const TArray<FVector>& Waypoints, float MaxVelocity, float MaxAcceleration) const;

	// 求解最小Snap多项式系数
	void SolveMinimumSnap(const TArray<FVector>& Waypoints, const TArray<float>& SegmentTimes);

	// 求解单轴的多项式系数
	TArray<float> SolveSingleAxis(const TArray<float>& Positions, const TArray<float>& SegmentTimes,
								  float StartVel, float EndVel, float StartAcc, float EndAcc) const;

	// 评估多项式及其导数
	void EvaluatePolynomial(const TArray<float>& Coeffs, float t,
							float& OutPos, float& OutVel, float& OutAcc) const;

	// 查找时间对应的轨迹段
	int32 FindSegmentIndex(float Time) const;

	// 计算段内的局部时间
	float GetLocalTime(float GlobalTime, int32 SegmentIndex) const;
};
