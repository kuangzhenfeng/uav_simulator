// Copyright Epic Games, Inc. All Rights Reserved.

#include "TrajectoryTracker.h"

UTrajectoryTracker::UTrajectoryTracker()
{
	PrimaryComponentTick.bCanEverTick = true;

	TrackingTime = 0.0f;
	bIsTracking = false;
	bIsPaused = false;
	bIsComplete = false;
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

void UTrajectoryTracker::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// 与 UAVPawn::Tick 保持一致的步长上限，防止首帧大步长导致轨迹时间跳变
	DeltaTime = FMath::Min(DeltaTime, 0.02f);

	if (!bIsTracking || bIsPaused || bIsComplete)
	{
		return;
	}

	// 自适应时间缩放：UAV落后时减速/暂停TrackingTime推进
	// 用前向投影误差（沿轨迹方向的落后量），忽略避障造成的横向偏移
	float EffectiveTimeScale = TimeScale;
	if (bEnableAdaptiveTimeScale && !bHasDesiredStateOverride)
	{
		FVector CurrentPos = GetOwner()->GetActorLocation();
		FTrajectoryPoint DesiredState = InterpolateTrajectory(TrackingTime);
		FVector ErrorVec = DesiredState.Position - CurrentPos;

		FVector Forward = DesiredState.Velocity.GetSafeNormal();
		float Error = Forward.IsNearlyZero()
			? ErrorVec.Size()
			: FMath::Max(0.0f, FVector::DotProduct(ErrorVec, Forward));

		if (Error > ErrorPauseThreshold)
		{
			EffectiveTimeScale = 0.0f;
		}
		else if (Error > ErrorSlowdownStart)
		{
			float Alpha = (Error - ErrorSlowdownStart) / (ErrorPauseThreshold - ErrorSlowdownStart);
			EffectiveTimeScale = TimeScale * (1.0f - Alpha);
		}
	}

	// 更新跟踪时间（bTimeIsFrozen 时暂停推进，防止 NMPC stuck 期间轨迹提前到期）
	if (!bTimeIsFrozen)
	{
		TrackingTime += DeltaTime * EffectiveTimeScale;
	}

	// 检查是否完成
	if (TrackingTime >= CurrentTrajectory.TotalDuration)
	{
		TrackingTime = CurrentTrajectory.TotalDuration;

		if (CurrentTrajectory.Points.Num() > 0 && GetOwner())
		{
			FVector CurrentPos = GetOwner()->GetActorLocation();
			FVector FinalPos = CurrentTrajectory.Points.Last().Position;

			if (FVector::Dist(CurrentPos, FinalPos) <= CompletionRadius)
			{
				bIsComplete = true;
				bIsTracking = false;
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
	bTimeIsFrozen = false;
	LastProgressUpdateTime = 0.0f;
	LastProgress = 0.0f;
}

FTrajectoryPoint UTrajectoryTracker::GetDesiredState(float CurrentTime) const
{
	// Override 仅在使用内部计时时生效（UAVPawn::UpdateController 的调用路径）
	// 显式传入时间时返回原始轨迹插值（NMPC 参考点采样路径）
	if (CurrentTime < 0.0f)
	{
		if (bHasDesiredStateOverride)
		{
			return OverrideDesiredState;
		}
		CurrentTime = TrackingTime;
	}

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

void UTrajectoryTracker::SetDesiredStateOverride(const FTrajectoryPoint& InOverrideState)
{
	OverrideDesiredState = InOverrideState;
	bHasDesiredStateOverride = true;
}

void UTrajectoryTracker::ClearDesiredStateOverride()
{
	bHasDesiredStateOverride = false;
}
