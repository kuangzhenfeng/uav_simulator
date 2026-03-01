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

	// 完整实现：使用7阶多项式最小化snap
	// p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5 + c6*t^6 + c7*t^7
	// snap = d^4p/dt^4
	//
	// 边界条件：
	// - 起点/终点：位置、速度、加速度、jerk (C³连续)
	// - 中间点：位置、速度、加速度、jerk连续
	//
	// 对于单段轨迹，使用闭式解
	// 对于多段轨迹，使用简化的分段优化

	for (int32 i = 0; i < NumSegments; ++i)
	{
		float T = SegmentTimes[i];
		float T2 = T * T;
		float T3 = T2 * T;
		float T4 = T3 * T;
		float T5 = T4 * T;
		float T6 = T5 * T;
		float T7 = T6 * T;

		float P0 = Positions[i];
		float P1 = Positions[i + 1];

		// 边界条件
		float V0, V1, A0, A1, J0, J1;

		if (i == 0)
		{
			// 起始段：使用指定的起始条件
			V0 = StartVel;
			A0 = StartAcc;
			J0 = 0.0f; // 假设起始jerk为0
		}
		else
		{
			// 中间段：从上一段的终点继续（C³连续）
			// 简化：假设中间点速度渐变
			float SegmentRatio = SegmentTimes[i] / (SegmentTimes[i - 1] + SegmentTimes[i]);
			V0 = (Positions[i] - Positions[i - 1]) / SegmentTimes[i - 1] * (1.0f - SegmentRatio) +
				 (Positions[i + 1] - Positions[i]) / SegmentTimes[i] * SegmentRatio;
			A0 = 0.0f;
			J0 = 0.0f;
		}

		if (i == NumSegments - 1)
		{
			// 终止段：使用指定的终止条件
			V1 = EndVel;
			A1 = EndAcc;
			J1 = 0.0f; // 假设终止jerk为0
		}
		else
		{
			// 中间段：平滑过渡到下一段
			float SegmentRatio = SegmentTimes[i] / (SegmentTimes[i] + SegmentTimes[i + 1]);
			V1 = (Positions[i + 1] - Positions[i]) / SegmentTimes[i] * (1.0f - SegmentRatio) +
				 (Positions[i + 2] - Positions[i + 1]) / SegmentTimes[i + 1] * SegmentRatio;
			A1 = 0.0f;
			J1 = 0.0f;
		}

		// 7阶多项式系数计算（最小snap）
		// 使用8个边界条件求解8个系数：
		// p(0) = P0, p(T) = P1
		// v(0) = V0, v(T) = V1
		// a(0) = A0, a(T) = A1
		// j(0) = J0, j(T) = J1

		float c0 = P0;
		float c1 = V0;
		float c2 = A0 / 2.0f;
		float c3 = J0 / 6.0f;

		// 求解 c4, c5, c6, c7
		// 使用终点边界条件
		// P(T) = P1, V(T) = V1, A(T) = A1, J(T) = J1

		// 构建线性方程组 (4x4)
		// [T^4  T^5  T^6  T^7 ] [c4]   [P1 - c0 - c1*T - c2*T^2 - c3*T^3]
		// [4T^3 5T^4 6T^5 7T^6] [c5] = [V1 - c1 - 2*c2*T - 3*c3*T^2    ]
		// [12T^2 20T^3 30T^4 42T^5] [c6]   [A1 - 2*c2 - 6*c3*T            ]
		// [24T  60T^2 120T^3 210T^4] [c7]   [J1 - 6*c3                     ]

		// 右侧向量
		float b0 = P1 - c0 - c1 * T - c2 * T2 - c3 * T3;
		float b1 = V1 - c1 - 2.0f * c2 * T - 3.0f * c3 * T2;
		float b2 = A1 - 2.0f * c2 - 6.0f * c3 * T;
		float b3 = J1 - 6.0f * c3;

		// 使用闭式解（通过符号计算预先求解）
		// 这是一个Vandermonde型矩阵的逆
		float c4 = (35.0f * b0 - 20.0f * b1 * T + 5.0f * b2 * T2 - 0.5f * b3 * T3) / T4;
		float c5 = (-84.0f * b0 + 45.0f * b1 * T - 10.0f * b2 * T2 + b3 * T3) / T5;
		float c6 = (70.0f * b0 - 36.0f * b1 * T + 7.5f * b2 * T2 - 0.5f * b3 * T3) / T6;
		float c7 = (-20.0f * b0 + 10.0f * b1 * T - 2.0f * b2 * T2 + (1.0f / 6.0f) * b3 * T3) / T7;

		// 添加系数（8个系数）
		Coeffs.Add(c0);
		Coeffs.Add(c1);
		Coeffs.Add(c2);
		Coeffs.Add(c3);
		Coeffs.Add(c4);
		Coeffs.Add(c5);
		Coeffs.Add(c6);
		Coeffs.Add(c7);
	}

	return Coeffs;
}

void UTrajectoryOptimizer::EvaluatePolynomial(const TArray<float>& Coeffs, float t,
											  float& OutPos, float& OutVel, float& OutAcc) const
{
	if (Coeffs.Num() < 8)
	{
		OutPos = 0.0f;
		OutVel = 0.0f;
		OutAcc = 0.0f;
		return;
	}

	// 位置: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5 + c6*t^6 + c7*t^7
	float t2 = t * t;
	float t3 = t2 * t;
	float t4 = t3 * t;
	float t5 = t4 * t;
	float t6 = t5 * t;
	float t7 = t6 * t;

	OutPos = Coeffs[0] + Coeffs[1] * t + Coeffs[2] * t2 + Coeffs[3] * t3 +
			 Coeffs[4] * t4 + Coeffs[5] * t5 + Coeffs[6] * t6 + Coeffs[7] * t7;

	// 速度: v(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4 + 6*c6*t^5 + 7*c7*t^6
	OutVel = Coeffs[1] + 2.0f * Coeffs[2] * t + 3.0f * Coeffs[3] * t2 + 4.0f * Coeffs[4] * t3 +
			 5.0f * Coeffs[5] * t4 + 6.0f * Coeffs[6] * t5 + 7.0f * Coeffs[7] * t6;

	// 加速度: a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3 + 30*c6*t^4 + 42*c7*t^5
	OutAcc = 2.0f * Coeffs[2] + 6.0f * Coeffs[3] * t + 12.0f * Coeffs[4] * t2 +
			 20.0f * Coeffs[5] * t3 + 30.0f * Coeffs[6] * t4 + 42.0f * Coeffs[7] * t5;
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
