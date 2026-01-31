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

	if (!bIsTracking || bIsPaused || bIsComplete)
	{
		return;
	}

	// 更新跟踪时间
	TrackingTime += DeltaTime * TimeScale;

	// 检查是否完成
	if (TrackingTime >= CurrentTrajectory.TotalDuration)
	{
		if (bHoldFinalState)
		{
			TrackingTime = CurrentTrajectory.TotalDuration;
		}

		bIsComplete = true;
		bIsTracking = false;
		OnTrajectoryCompleted.Broadcast();
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
	LastProgressUpdateTime = 0.0f;
	LastProgress = 0.0f;
}

FTrajectoryPoint UTrajectoryTracker::GetDesiredState(float CurrentTime) const
{
	if (CurrentTime < 0.0f)
	{
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
