// Copyright Epic Games, Inc. All Rights Reserved.

#include "TrajectoryOptimizer.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UTrajectoryOptimizer::UTrajectoryOptimizer()
{
	PrimaryComponentTick.bCanEverTick = true;
	DefaultSampleInterval = 0.05f;
}

void UTrajectoryOptimizer::BeginPlay()
{
	Super::BeginPlay();
}

void UTrajectoryOptimizer::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

FTrajectory UTrajectoryOptimizer::OptimizeTrajectory(const TArray<FVector>& Waypoints, float MaxVelocity, float MaxAcceleration)
{
	FTrajectory Result;

	if (Waypoints.Num() < 2)
	{
		UE_LOG(LogUAVPlanning, Warning, TEXT("TrajectoryOptimizer: Need at least 2 waypoints"));
		return Result;
	}

	// 计算时间分配
	TArray<float> SegmentTimes = ComputeTimeAllocation(Waypoints, MaxVelocity, MaxAcceleration);

	return OptimizeTrajectoryWithTiming(Waypoints, SegmentTimes);
}

FTrajectory UTrajectoryOptimizer::OptimizeTrajectoryWithTiming(const TArray<FVector>& Waypoints, const TArray<float>& SegmentTimes)
{
	FTrajectory Result;

	if (Waypoints.Num() < 2 || SegmentTimes.Num() != Waypoints.Num() - 1)
	{
		UE_LOG(LogUAVPlanning, Warning, TEXT("TrajectoryOptimizer: Invalid input"));
		return Result;
	}

	// 求解最小Snap多项式
	SolveMinimumSnap(Waypoints, SegmentTimes);

	// 计算总时长
	float TotalDuration = 0.0f;
	for (float T : SegmentTimes)
	{
		TotalDuration += T;
	}

	if (!FMath::IsFinite(TotalDuration) || TotalDuration <= 0.0f)
	{
		UE_LOG(LogUAVPlanning, Error, TEXT("TrajectoryOptimizer: Invalid total duration: %.3f"), TotalDuration);
		return Result;
	}

	// 生成轨迹点
	TArray<FTrajectoryPoint> DenseSamples = GetDenseSamples(Result, DefaultSampleInterval);

	// 直接从段采样生成轨迹
	float CurrentTime = 0.0f;
	while (CurrentTime <= TotalDuration)
	{
		FTrajectoryPoint Point;
		Point.TimeStamp = CurrentTime;

		int32 SegIdx = FindSegmentIndex(CurrentTime);
		if (SegIdx >= 0 && SegIdx < Segments.Num())
		{
			float LocalTime = GetLocalTime(CurrentTime, SegIdx);

			float PosX, VelX, AccX;
			float PosY, VelY, AccY;
			float PosZ, VelZ, AccZ;

			EvaluatePolynomial(Segments[SegIdx].CoeffX, LocalTime, PosX, VelX, AccX);
			EvaluatePolynomial(Segments[SegIdx].CoeffY, LocalTime, PosY, VelY, AccY);
			EvaluatePolynomial(Segments[SegIdx].CoeffZ, LocalTime, PosZ, VelZ, AccZ);

			Point.Position = FVector(PosX, PosY, PosZ);
			Point.Velocity = FVector(VelX, VelY, VelZ);
			Point.Acceleration = FVector(AccX, AccY, AccZ);
		}

		Result.AddPoint(Point);
		CurrentTime += DefaultSampleInterval;
	}

	Result.TotalDuration = TotalDuration;
	Result.bIsValid = true;

	return Result;
}

FTrajectoryPoint UTrajectoryOptimizer::SampleTrajectory(const FTrajectory& Traj, float Time) const
{
	FTrajectoryPoint Result;

	if (!Traj.bIsValid || Traj.Points.Num() == 0)
	{
		return Result;
	}

	// 限制时间范围
	Time = FMath::Clamp(Time, 0.0f, Traj.TotalDuration);

	// 如果没有多项式段，使用线性插值
	if (Segments.Num() == 0)
	{
		// 找到最近的两个采样点
		int32 LowIdx = 0;
		int32 HighIdx = 0;

		for (int32 i = 0; i < Traj.Points.Num() - 1; ++i)
		{
			if (Traj.Points[i].TimeStamp <= Time && Traj.Points[i + 1].TimeStamp >= Time)
			{
				LowIdx = i;
				HighIdx = i + 1;
				break;
			}
		}

		if (LowIdx == HighIdx || Traj.Points[HighIdx].TimeStamp == Traj.Points[LowIdx].TimeStamp)
		{
			return Traj.Points[LowIdx];
		}

		float Alpha = (Time - Traj.Points[LowIdx].TimeStamp) /
					  (Traj.Points[HighIdx].TimeStamp - Traj.Points[LowIdx].TimeStamp);

		Result.TimeStamp = Time;
		Result.Position = FMath::Lerp(Traj.Points[LowIdx].Position, Traj.Points[HighIdx].Position, Alpha);
		Result.Velocity = FMath::Lerp(Traj.Points[LowIdx].Velocity, Traj.Points[HighIdx].Velocity, Alpha);
		Result.Acceleration = FMath::Lerp(Traj.Points[LowIdx].Acceleration, Traj.Points[HighIdx].Acceleration, Alpha);

		return Result;
	}

	// 使用多项式段采样
	int32 SegIdx = FindSegmentIndex(Time);
	if (SegIdx >= 0 && SegIdx < Segments.Num())
	{
		float LocalTime = GetLocalTime(Time, SegIdx);

		float PosX, VelX, AccX;
		float PosY, VelY, AccY;
		float PosZ, VelZ, AccZ;

		EvaluatePolynomial(Segments[SegIdx].CoeffX, LocalTime, PosX, VelX, AccX);
		EvaluatePolynomial(Segments[SegIdx].CoeffY, LocalTime, PosY, VelY, AccY);
		EvaluatePolynomial(Segments[SegIdx].CoeffZ, LocalTime, PosZ, VelZ, AccZ);

		Result.TimeStamp = Time;
		Result.Position = FVector(PosX, PosY, PosZ);
		Result.Velocity = FVector(VelX, VelY, VelZ);
		Result.Acceleration = FVector(AccX, AccY, AccZ);
	}

	return Result;
}

TArray<FTrajectoryPoint> UTrajectoryOptimizer::GetDenseSamples(const FTrajectory& Traj, float SampleInterval) const
{
	TArray<FTrajectoryPoint> Samples;

	if (!Traj.bIsValid)
	{
		return Samples;
	}

	float CurrentTime = 0.0f;
	while (CurrentTime <= Traj.TotalDuration)
	{
		Samples.Add(SampleTrajectory(Traj, CurrentTime));
		CurrentTime += SampleInterval;
	}

	// 确保包含终点
	if (Samples.Num() > 0 && Samples.Last().TimeStamp < Traj.TotalDuration)
	{
		Samples.Add(SampleTrajectory(Traj, Traj.TotalDuration));
	}

	return Samples;
}

TArray<float> UTrajectoryOptimizer::ComputeTimeAllocation(const TArray<FVector>& Waypoints, float MaxVelocity, float MaxAcceleration) const
{
	TArray<float> SegmentTimes;

	if (Waypoints.Num() < 2)
	{
		return SegmentTimes;
	}

	for (int32 i = 0; i < Waypoints.Num() - 1; ++i)
	{
		float Distance = FVector::Dist(Waypoints[i], Waypoints[i + 1]);
		UE_LOG(LogUAVPlanning, Verbose, TEXT("TrajectoryOptimizer: Segment %d distance: %.3f"), i, Distance);

		// 使用梯形速度规划计算时间
		// 加速阶段时间
		float AccelTime = MaxVelocity / MaxAcceleration;
		// 加速阶段距离
		float AccelDist = 0.5f * MaxAcceleration * AccelTime * AccelTime;

		float SegmentTime;
		if (2.0f * AccelDist >= Distance)
		{
			// 无法达到最大速度，使用三角形速度规划
			SegmentTime = 2.0f * FMath::Sqrt(Distance / MaxAcceleration);
		}
		else
		{
			// 梯形速度规划
			float CruiseDist = Distance - 2.0f * AccelDist;
			float CruiseTime = CruiseDist / MaxVelocity;
			SegmentTime = 2.0f * AccelTime + CruiseTime;
		}
		UE_LOG(LogUAVPlanning, Verbose, TEXT("TrajectoryOptimizer: Segment %d time: %.3f"), i, SegmentTime);

		// 添加安全余量
		SegmentTime *= 1.05f;
		SegmentTime = FMath::Max(SegmentTime, 0.5f); // 最小0.5秒

		SegmentTimes.Add(SegmentTime);
	}

	return SegmentTimes;
}

void UTrajectoryOptimizer::SolveMinimumSnap(const TArray<FVector>& Waypoints, const TArray<float>& SegmentTimes)
{
	Segments.Empty();

	int32 NumSegments = Waypoints.Num() - 1;

	// 提取各轴位置
	TArray<float> PosX, PosY, PosZ;
	for (const FVector& WP : Waypoints)
	{
		PosX.Add(WP.X);
		PosY.Add(WP.Y);
		PosZ.Add(WP.Z);
	}

	// 求解各轴多项式系数
	TArray<float> CoeffsX = SolveSingleAxis(PosX, SegmentTimes, StartVelocity.X, EndVelocity.X, StartAcceleration.X, EndAcceleration.X);
	TArray<float> CoeffsY = SolveSingleAxis(PosY, SegmentTimes, StartVelocity.Y, EndVelocity.Y, StartAcceleration.Y, EndAcceleration.Y);
	TArray<float> CoeffsZ = SolveSingleAxis(PosZ, SegmentTimes, StartVelocity.Z, EndVelocity.Z, StartAcceleration.Z, EndAcceleration.Z);

	// 构建轨迹段
	float CurrentTime = 0.0f;
	for (int32 i = 0; i < NumSegments; ++i)
	{
		FPolynomialSegment Seg;
		Seg.StartTime = CurrentTime;
		Seg.Duration = SegmentTimes[i];

		for (int32 j = 0; j < 8; ++j)
		{
			int32 Idx = i * 8 + j;
			Seg.CoeffX[j] = (Idx < CoeffsX.Num()) ? CoeffsX[Idx] : 0.0f;
			Seg.CoeffY[j] = (Idx < CoeffsY.Num()) ? CoeffsY[Idx] : 0.0f;
			Seg.CoeffZ[j] = (Idx < CoeffsZ.Num()) ? CoeffsZ[Idx] : 0.0f;
		}

		Segments.Add(Seg);
		CurrentTime += SegmentTimes[i];
	}
}

TArray<float> UTrajectoryOptimizer::SolveSingleAxis(const TArray<float>& Positions, const TArray<float>& SegmentTimes,
												   float StartVel, float EndVel, float StartAcc, float EndAcc) const
{
	TArray<float> Coeffs;

	int32 NumSegments = Positions.Num() - 1;
	if (NumSegments <= 0)
	{
		return Coeffs;
	}

	// 简化实现：对每个段使用5阶多项式插值
	// p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
	// 边界条件：起点/终点位置、速度、加速度

	for (int32 i = 0; i < NumSegments; ++i)
	{
		float T = SegmentTimes[i];
		float T2 = T * T;
		float T3 = T2 * T;
		float T4 = T3 * T;
		float T5 = T4 * T;

		float P0 = Positions[i];
		float P1 = Positions[i + 1];

		float V0 = (i == 0) ? StartVel : 0.0f;
		float V1 = (i == NumSegments - 1) ? EndVel : 0.0f;

		float A0 = (i == 0) ? StartAcc : 0.0f;
		float A1 = (i == NumSegments - 1) ? EndAcc : 0.0f;

		// 5阶多项式系数计算
		// 使用边界条件求解
		float c0 = P0;
		float c1 = V0;
		float c2 = A0 / 2.0f;

		// 求解 c3, c4, c5
		// P(T) = P1, V(T) = V1, A(T) = A1
		float DeltaP = P1 - P0 - V0 * T - 0.5f * A0 * T2;
		float DeltaV = V1 - V0 - A0 * T;
		float DeltaA = A1 - A0;

		// 简化求解（假设中间点速度/加速度为0）
		float c3 = (20.0f * DeltaP - (8.0f * V1 + 12.0f * V0) * T - (3.0f * A0 - A1) * T2) / (2.0f * T3);
		float c4 = (-30.0f * DeltaP + (14.0f * V1 + 16.0f * V0) * T + (3.0f * A0 - 2.0f * A1) * T2) / (2.0f * T4);
		float c5 = (12.0f * DeltaP - 6.0f * (V1 + V0) * T + (A1 - A0) * T2) / (2.0f * T5);

		// 添加系数（8个系数，高阶为0）
		Coeffs.Add(c0);
		Coeffs.Add(c1);
		Coeffs.Add(c2);
		Coeffs.Add(c3);
		Coeffs.Add(c4);
		Coeffs.Add(c5);
		Coeffs.Add(0.0f);  // c6
		Coeffs.Add(0.0f);  // c7
	}

	return Coeffs;
}

void UTrajectoryOptimizer::EvaluatePolynomial(const TArray<float>& Coeffs, float t,
											  float& OutPos, float& OutVel, float& OutAcc) const
{
	if (Coeffs.Num() < 6)
	{
		OutPos = 0.0f;
		OutVel = 0.0f;
		OutAcc = 0.0f;
		return;
	}

	// 位置: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
	float t2 = t * t;
	float t3 = t2 * t;
	float t4 = t3 * t;
	float t5 = t4 * t;

	OutPos = Coeffs[0] + Coeffs[1] * t + Coeffs[2] * t2 + Coeffs[3] * t3 + Coeffs[4] * t4 + Coeffs[5] * t5;

	// 速度: v(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4
	OutVel = Coeffs[1] + 2.0f * Coeffs[2] * t + 3.0f * Coeffs[3] * t2 + 4.0f * Coeffs[4] * t3 + 5.0f * Coeffs[5] * t4;

	// 加速度: a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
	OutAcc = 2.0f * Coeffs[2] + 6.0f * Coeffs[3] * t + 12.0f * Coeffs[4] * t2 + 20.0f * Coeffs[5] * t3;
}

int32 UTrajectoryOptimizer::FindSegmentIndex(float Time) const
{
	for (int32 i = 0; i < Segments.Num(); ++i)
	{
		if (Time >= Segments[i].StartTime && Time < Segments[i].StartTime + Segments[i].Duration)
		{
			return i;
		}
	}

	// 如果时间超出范围，返回最后一段
	return Segments.Num() - 1;
}

float UTrajectoryOptimizer::GetLocalTime(float GlobalTime, int32 SegmentIndex) const
{
	if (SegmentIndex < 0 || SegmentIndex >= Segments.Num())
	{
		return 0.0f;
	}

	return GlobalTime - Segments[SegmentIndex].StartTime;
}
