// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "TrajectoryTracker.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnTrajectoryCompleted);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnTrajectoryProgress, float, Progress);

/**
 * 轨迹跟踪控制器
 * 实现时间参数化的轨迹跟踪，提供前馈+反馈控制
 */
UCLASS(ClassGroup=(Planning), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UTrajectoryTracker : public UActorComponent
{
	GENERATED_BODY()

public:
	UTrajectoryTracker();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/**
	 * 设置要跟踪的轨迹
	 * @param InTrajectory 轨迹数据
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	void SetTrajectory(const FTrajectory& InTrajectory);

	/**
	 * 开始跟踪轨迹
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	void StartTracking();

	/**
	 * 停止跟踪轨迹
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	void StopTracking();

	/**
	 * 暂停跟踪
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	void PauseTracking();

	/**
	 * 恢复跟踪
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	void ResumeTracking();

	/**
	 * 重置跟踪器
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	void Reset();

	/**
	 * 获取当前期望状态
	 * @param CurrentTime 当前时间（如果<0则使用内部计时）
	 * @return 期望的轨迹点状态
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	FTrajectoryPoint GetDesiredState(float CurrentTime = -1.0f) const;

	/**
	 * 获取跟踪进度 (0.0 - 1.0)
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	float GetProgress() const;

	/**
	 * 检查是否完成跟踪
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	bool IsComplete() const;

	/**
	 * 检查是否正在跟踪
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	bool IsTracking() const { return bIsTracking && !bIsPaused; }

	/**
	 * 获取当前轨迹
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	const FTrajectory& GetTrajectory() const { return CurrentTrajectory; }

	/**
	 * 获取当前跟踪时间
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	float GetCurrentTime() const { return TrackingTime; }

	/**
	 * 设置速度缩放因子 (用于主动减速)
	 * @param InSpeedScale 速度缩放因子 (0.0-1.0)
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	void SetSpeedScale(float InSpeedScale) { SpeedScale = FMath::Clamp(InSpeedScale, 0.0f, 1.0f); }

	/**
	 * 计算跟踪误差
	 * @param CurrentPosition 当前位置
	 * @return 位置误差向量
	 */
	UFUNCTION(BlueprintCallable, Category = "Trajectory Tracking")
	FVector GetTrackingError(const FVector& CurrentPosition) const;

	// 轨迹完成事件
	UPROPERTY(BlueprintAssignable, Category = "Trajectory Tracking")
	FOnTrajectoryCompleted OnTrajectoryCompleted;

	// 轨迹进度更新事件
	UPROPERTY(BlueprintAssignable, Category = "Trajectory Tracking")
	FOnTrajectoryProgress OnTrajectoryProgress;

protected:
	// 当前轨迹
	UPROPERTY(BlueprintReadOnly, Category = "Trajectory Tracking")
	FTrajectory CurrentTrajectory;

	// 跟踪时间
	UPROPERTY(BlueprintReadOnly, Category = "Trajectory Tracking")
	float TrackingTime;

	// 是否正在跟踪
	UPROPERTY(BlueprintReadOnly, Category = "Trajectory Tracking")
	bool bIsTracking;

	// 是否暂停
	UPROPERTY(BlueprintReadOnly, Category = "Trajectory Tracking")
	bool bIsPaused;

	// 是否已完成
	UPROPERTY(BlueprintReadOnly, Category = "Trajectory Tracking")
	bool bIsComplete;

	// 时间缩放因子 (用于加速/减速播放)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory Tracking")
	float TimeScale = 1.0f;

	// 速度缩放因子 (用于主动减速避障)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory Tracking")
	float SpeedScale = 1.0f;

	// 自适应时间缩放：误差大时自动减速/暂停推进
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Adaptive Tracking")
	bool bEnableAdaptiveTimeScale = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Adaptive Tracking")
	float ErrorSlowdownStart = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Adaptive Tracking")
	float ErrorPauseThreshold = 300.0f;

	// 自适应时间缩放的最小比例（相对 TimeScale），避免 TrackingTime 被完全冻结
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Adaptive Tracking", meta = (ClampMin = "0.01", ClampMax = "1.0"))
	float MinAdaptiveTimeScale = 0.1f;

	// 完成判定半径 (cm)：时间耗尽后，UAV 须在此距离内才标记完成
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Adaptive Tracking")
	float CompletionRadius = 500.0f;

	// 完成判定最大速度 (cm/s)：速度超过此值时不标记完成，等待减速
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Adaptive Tracking")
	float CompletionMaxSpeed = 300.0f;

	// 是否在完成时保持最终状态
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory Tracking")
	bool bHoldFinalState = true;

	// 进度更新间隔 (秒)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory Tracking")
	float ProgressUpdateInterval = 0.1f;

private:
	// 上次进度更新时间
	float LastProgressUpdateTime;

	// 上次进度值
	float LastProgress;

	// 从轨迹采样点进行插值
	FTrajectoryPoint InterpolateTrajectory(float Time) const;
};
