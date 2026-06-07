// Copyright Epic Games, Inc. All Rights Reserved.

#include "TrajectoryTracker.h"
#include "../uav_simulator.h"
#include "uav_simulator/Debug/UAVLogConfig.h"
#include "uav_simulator/Core/UAVPawn.h"
#include "uav_simulator/Utility/Filter.h"

static float GetUAVSpeed(AActor* Owner)
{
	if (AUAVPawn* Pawn = Cast<AUAVPawn>(Owner))
		return Pawn->GetUAVState().Velocity.Size();
	return Owner ? Owner->GetVelocity().Size() : 0.f;
}

UTrajectoryTracker::UTrajectoryTracker()
{
	PrimaryComponentTick.bCanEverTick = true;

	TrackingTime = 0.0f;
	bIsTracking = false;
	bIsPaused = false;
	bIsComplete = false;
	bIsTimedOut = false;
	TimeScale = 1.0f;
	bHoldFinalState = true;
	ProgressUpdateInterval = 0.1f;
	LastProgressUpdateTime = 0.0f;
	LastProgress = 0.0f;
}

void UTrajectoryTracker::BeginPlay()
{
	Super::BeginPlay();
}

// ========== 超时完成检查辅助方法 ==========

bool UTrajectoryTracker::ShouldCheckOvertimeCompletion() const
{
	// 超时后不再重入：防止 HandleOvertimeCompletion 被高频调用
	return !bIsTracking && !bIsComplete && !bIsTimedOut && CurrentTrajectory.bIsValid
		&& TrackingTime >= CurrentTrajectory.TotalDuration
		&& CurrentTrajectory.Points.Num() > 0 && GetOwner();
}

bool UTrajectoryTracker::IsWithinCompletionCriteria(const FVector& CurrentPos, float CurrentSpeed) const
{
	FVector FinalPos = CurrentTrajectory.Points.Last().Position;
	float Dist = FVector::Dist(CurrentPos, FinalPos);
	return Dist <= CompletionRadius && CurrentSpeed <= CompletionMaxSpeed;
}

void UTrajectoryTracker::HandleOvertimeCompletion(float DeltaTime)
{
	FVector CurrentPos = GetOwner()->GetActorLocation();
	float Speed = GetUAVSpeed(GetOwner());

	if (IsWithinCompletionCriteria(CurrentPos, Speed))
	{
		FVector FinalPos = CurrentTrajectory.Points.Last().Position;
		float Dist = FVector::Dist(CurrentPos, FinalPos);
		UE_LOG(LogUAVPlanning, Log, TEXT("[Tracker] Completed (overtime): Dist=%.0f Speed=%.0f"), Dist, Speed);
		bIsComplete = true;
		OvertimeElapsed = 0.0f;
		OnTrajectoryCompleted.Broadcast();
		return;
	}

	OvertimeElapsed += DeltaTime;

	// 主动设置 UAVPawn 目标为轨迹终点，防止被行为树覆盖
	if (AUAVPawn* Pawn = Cast<AUAVPawn>(GetOwner()))
	{
		FVector FinalPos = CurrentTrajectory.Points.Last().Position;
		Pawn->SetTargetPosition(FinalPos);
	}

	if (OvertimeElapsed >= OvertimeTimeout)
	{
		FVector FinalPos = CurrentTrajectory.Points.Last().Position;
		float Dist = FVector::Dist(CurrentPos, FinalPos);
		UE_LOG(LogUAVPlanning, Warning, TEXT("[Tracker] Overtime timeout (%.1fs): Dist=%.0f Speed=%.0f, marking as timed out"),
			OvertimeElapsed, Dist, Speed);
		bIsTimedOut = true;
			// 超时是失败终态，不设 bIsComplete，不广播 OnTrajectoryCompleted
			// 行为树通过 IsTimedOut() 识别失败并返回 Failed
			OvertimeElapsed = 0.0f;
			OnTrajectoryTimedOut.Broadcast();
	}
	else
	{
		FVector FinalPos = CurrentTrajectory.Points.Last().Position;
		float Dist = FVector::Dist(CurrentPos, FinalPos);
		UE_LOG_THROTTLE(1.0, LogUAVPlanning, Log,
			TEXT("[Tracker] Waiting (overtime): Dist=%.0f/%.0f Speed=%.0f/%.0f Elapsed=%.1f/%.1f"),
			Dist, CompletionRadius, Speed, CompletionMaxSpeed, OvertimeElapsed, OvertimeTimeout);
	}
}

void UTrajectoryTracker::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	SCOPE_CYCLE_COUNTER(STAT_TrajectoryTracker);

	// 与 UAVPawn::Tick 保持一致的步长上限，防止首帧大步长导致轨迹时间跳变
	DeltaTime = FMath::Min(DeltaTime, 0.02f);

	// 轨迹时间耗尽后切换为位置控制兜底：持续检测是否到达终点
	if (ShouldCheckOvertimeCompletion())
	{
		HandleOvertimeCompletion(DeltaTime);
		return;
	}

	if (!bIsTracking || bIsPaused || bIsComplete)
	{
		return;
	}

	// 位置兜底完成检查：防止自适应时间缩放导致 TrackingTime 无法到达 TotalDuration
	// 条件：已进入轨迹后段（>80%）+ UAV 到达终点位置 + 速度足够低
	if (CurrentTrajectory.bIsValid && CurrentTrajectory.TotalDuration > 0.0f
		&& TrackingTime >= CurrentTrajectory.TotalDuration * 0.8f
		&& CurrentTrajectory.Points.Num() > 0 && GetOwner())
	{
		FVector CurrentPos = GetOwner()->GetActorLocation();
		FVector FinalPos = CurrentTrajectory.Points.Last().Position;
		float Dist = FVector::Dist(CurrentPos, FinalPos);
		float Speed = GetUAVSpeed(GetOwner());
		if (Dist <= CompletionRadius && Speed <= CompletionMaxSpeed)
		{
			UE_LOG(LogUAVPlanning, Log, TEXT("[Tracker] Completed (position fallback): Dist=%.0f Speed=%.0f TrackTime=%.1f/%.1f"),
				Dist, Speed, TrackingTime, CurrentTrajectory.TotalDuration);
			TrackingTime = CurrentTrajectory.TotalDuration;
			bIsTracking = false;
			bIsComplete = true;
			OnTrajectoryCompleted.Broadcast();
			return;
		}
	}

	// 自适应时间缩放：UAV落后时减速/暂停TrackingTime推进
	// 用前向投影误差（沿轨迹方向的落后量），忽略避障造成的横向偏移
	float EffectiveTimeScale = TimeScale;
	if (bEnableAdaptiveTimeScale)
	{
		const float ClampedMinScale = FMath::Clamp(MinAdaptiveTimeScale, 0.01f, 1.0f);
		FVector CurrentPos = GetOwner()->GetActorLocation();
		FTrajectoryPoint DesiredState = InterpolateTrajectory(TrackingTime);
		FVector ErrorVec = DesiredState.Position - CurrentPos;

		FVector Forward = DesiredState.Velocity.GetSafeNormal();
		float ForwardError = Forward.IsNearlyZero()
			? ErrorVec.Size()
			: FVector::DotProduct(ErrorVec, Forward);

		if (ForwardError > ErrorPauseThreshold)
		{
			EffectiveTimeScale = TimeScale * ClampedMinScale;
		}
		else if (ForwardError > ErrorSlowdownStart)
		{
			float Alpha = (ForwardError - ErrorSlowdownStart) / (ErrorPauseThreshold - ErrorSlowdownStart);
			float AdaptiveScale = FMath::Lerp(1.0f, ClampedMinScale, Alpha);
			EffectiveTimeScale = TimeScale * AdaptiveScale;
		}
		else if (ForwardError < -ErrorSlowdownStart)
		{
			// UAV 超前：加速轨迹时间追上 UAV，减少位置误差和大俯仰角修正
			// 上限 4.0 允许最高 5x 加速，防止障碍物区耗时后轨迹时间提前耗尽
			float Alpha = FMath::Min(4.0f, (-ForwardError - ErrorSlowdownStart) / (ErrorPauseThreshold - ErrorSlowdownStart));
			EffectiveTimeScale = TimeScale * (1.0f + Alpha);
		}

		EffectiveTimeScale = FMath::Max(EffectiveTimeScale, TimeScale * ClampedMinScale);

		// 应用速度缩放（基于障碍物距离的主动减速）
		EffectiveTimeScale *= SpeedScale;

		// 轨迹级卡死检测：UAV 长时间低速且轨迹进度停滞，强制推进轨迹时间
		// 防止自适应时间缩放崩溃导致 UAV 永远无法到达终点
		float Speed = GetUAVSpeed(GetOwner());
		if (Speed < 50.0f && TrackingTime > CurrentTrajectory.TotalDuration * 0.1f)
		{
			LowSpeedStuckAccumulator += DeltaTime;
			if (LowSpeedStuckAccumulator > 3.0f)
			{
				// 强制推进轨迹时间 2 秒，让参考点前进，驱动 UAV 继续飞行
				float BoostAmount = FMath::Min(2.0f, CurrentTrajectory.TotalDuration - TrackingTime);
				if (BoostAmount > 0.1f)
				{
					TrackingTime += BoostAmount;
					LowSpeedStuckAccumulator = 0.0f;
					UE_LOG(LogUAVPlanning, Warning,
						TEXT("[Tracker] Low-speed stuck! Speed=%.0f Progress=%.2f, forced advance +%.1fs"),
						Speed, TrackingTime / FMath::Max(CurrentTrajectory.TotalDuration, 0.001f), BoostAmount);
				}
			}
		}
		else
		{
			LowSpeedStuckAccumulator = 0.0f;
		}
	}

	// 更新跟踪时间
	TrackingTime += DeltaTime * EffectiveTimeScale;

	// 检查是否完成
	if (TrackingTime >= CurrentTrajectory.TotalDuration)
	{
		TrackingTime = CurrentTrajectory.TotalDuration;
		bIsTracking = false; // 停止轨迹跟踪，UAVPawn 回退到位置控制模式驱动 UAV 到达终点

		if (CurrentTrajectory.Points.Num() > 0 && GetOwner())
		{
			FVector CurrentPos = GetOwner()->GetActorLocation();
			FVector FinalPos = CurrentTrajectory.Points.Last().Position;

			float Dist2 = FVector::Dist(CurrentPos, FinalPos);
			float Speed2 = GetUAVSpeed(GetOwner());
			if (Dist2 <= CompletionRadius && Speed2 <= CompletionMaxSpeed)
			{
				UE_LOG(LogUAVPlanning, Log, TEXT("[Tracker] Completed: Dist=%.0f Speed=%.0f"), Dist2, Speed2);
				bIsComplete = true;
				OnTrajectoryCompleted.Broadcast();
			}
		}
	}

	// 更新进度
	float CurrentProgress = GetProgress();
	if (TrackingTime - LastProgressUpdateTime >= ProgressUpdateInterval ||
		FMath::Abs(CurrentProgress - LastProgress) >= 0.05f)
	{
		LastProgressUpdateTime = TrackingTime;
		LastProgress = CurrentProgress;
		OnTrajectoryProgress.Broadcast(CurrentProgress);
	}
}

void UTrajectoryTracker::SetTrajectory(const FTrajectory& InTrajectory)
{
	CurrentTrajectory = InTrajectory;
	Reset();
}

void UTrajectoryTracker::StartTracking()
{
	if (!CurrentTrajectory.bIsValid || CurrentTrajectory.Points.Num() < 2)
	{
		UE_LOG(LogTemp, Warning, TEXT("TrajectoryTracker: Cannot start tracking - invalid trajectory"));
		return;
	}

	bIsTracking = true;
	bIsPaused = false;
	bIsComplete = false;
	bIsTimedOut = false;
	TrackingTime = 0.0f;
	LastProgressUpdateTime = 0.0f;
	LastProgress = 0.0f;
}

void UTrajectoryTracker::StopTracking()
{
	bIsTracking = false;
	bIsPaused = false;
}

void UTrajectoryTracker::PauseTracking()
{
	if (bIsTracking)
	{
		bIsPaused = true;
	}
}

void UTrajectoryTracker::ResumeTracking()
{
	if (bIsTracking)
	{
		bIsPaused = false;
	}
}

void UTrajectoryTracker::Reset()
{
	TrackingTime = 0.0f;
	bIsTracking = false;
	bIsPaused = false;
	bIsComplete = false;
	bIsTimedOut = false;
	LastProgressUpdateTime = 0.0f;
	LastProgress = 0.0f;
	OvertimeElapsed = 0.0f;
}

FTrajectoryPoint UTrajectoryTracker::GetDesiredState(float CurrentTime) const
{
	if (CurrentTime < 0.0f)
		CurrentTime = TrackingTime;
	return InterpolateTrajectory(CurrentTime);
}

float UTrajectoryTracker::GetProgress() const
{
	if (!CurrentTrajectory.bIsValid || CurrentTrajectory.TotalDuration <= 0.0f)
	{
		return 0.0f;
	}

	return FMath::Clamp(TrackingTime / CurrentTrajectory.TotalDuration, 0.0f, 1.0f);
}

bool UTrajectoryTracker::IsComplete() const
{
	return bIsComplete;
}

FVector UTrajectoryTracker::GetTrackingError(const FVector& CurrentPosition) const
{
	FTrajectoryPoint DesiredState = GetDesiredState();
	return DesiredState.Position - CurrentPosition;
}

FTrajectoryPoint UTrajectoryTracker::InterpolateTrajectory(float Time) const
{
	FTrajectoryPoint Result;

	if (!CurrentTrajectory.bIsValid || CurrentTrajectory.Points.Num() == 0)
	{
		return Result;
	}

	// 限制时间范围
	Time = FMath::Clamp(Time, 0.0f, CurrentTrajectory.TotalDuration);

	// 找到包含该时间的两个相邻采样点
	int32 LowIdx = 0;
	int32 HighIdx = 0;

	for (int32 i = 0; i < CurrentTrajectory.Points.Num() - 1; ++i)
	{
		if (CurrentTrajectory.Points[i].TimeStamp <= Time &&
			CurrentTrajectory.Points[i + 1].TimeStamp >= Time)
		{
			LowIdx = i;
			HighIdx = i + 1;
			break;
		}
		LowIdx = i;
		HighIdx = i;
	}

	// 处理边界情况
	if (Time <= CurrentTrajectory.Points[0].TimeStamp)
	{
		return CurrentTrajectory.Points[0];
	}

	if (Time >= CurrentTrajectory.Points.Last().TimeStamp)
	{
		return CurrentTrajectory.Points.Last();
	}

	// 线性插值
	const FTrajectoryPoint& P0 = CurrentTrajectory.Points[LowIdx];
	const FTrajectoryPoint& P1 = CurrentTrajectory.Points[HighIdx];

	float TimeDiff = P1.TimeStamp - P0.TimeStamp;
	if (TimeDiff < KINDA_SMALL_NUMBER)
	{
		return P0;
	}

	float Alpha = (Time - P0.TimeStamp) / TimeDiff;

	Result.TimeStamp = Time;
	Result.Position = FMath::Lerp(P0.Position, P1.Position, Alpha);
	Result.Velocity = FMath::Lerp(P0.Velocity, P1.Velocity, Alpha);
	Result.Acceleration = FMath::Lerp(P0.Acceleration, P1.Acceleration, Alpha);
	Result.Yaw = FMath::Lerp(P0.Yaw, P1.Yaw, Alpha);

	return Result;
}
