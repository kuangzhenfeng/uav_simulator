// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Planning/TrajectoryOptimizer.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 时间分配测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryOptimizerTimeAllocationTest,
	"UAVSimulator.Planning.TrajectoryOptimizer.TimeAllocation",
	UAV_TEST_FLAGS)

bool FTrajectoryOptimizerTimeAllocationTest::RunTest(const FString& Parameters)
{
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>();

	// 创建简单的航点序列
	TArray<FVector> Waypoints = UAVTestHelpers::CreateWaypoints({
		FVector(0.0f, 0.0f, 0.0f),
		FVector(500.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f)
	});

	// 生成轨迹
	float MaxVelocity = 500.0f;
	float MaxAcceleration = 200.0f;
	FTrajectory Trajectory = Optimizer->OptimizeTrajectory(Waypoints, MaxVelocity, MaxAcceleration);

	// 验证轨迹有效
	TestTrue(TEXT("Trajectory should be valid"), Trajectory.bIsValid);
	TestTrue(TEXT("Trajectory should have points"), Trajectory.Points.Num() > 0);

	// 验证总时长合理
	float DirectDistance = 1000.0f; // 从 (0,0,0) 到 (1000,0,0)
	float MinTime = DirectDistance / MaxVelocity;
	TestTrue(TEXT("Total duration should be at least minimum time"), Trajectory.TotalDuration >= MinTime * 0.8f);

	return true;
}

// ==================== 多项式评估测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryOptimizerPolynomialTest,
	"UAVSimulator.Planning.TrajectoryOptimizer.PolynomialEvaluation",
	UAV_TEST_FLAGS)

bool FTrajectoryOptimizerPolynomialTest::RunTest(const FString& Parameters)
{
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>();

	// 创建航点
	TArray<FVector> Waypoints = UAVTestHelpers::CreateWaypoints({
		FVector(0.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f)
	});

	FTrajectory Trajectory = Optimizer->OptimizeTrajectory(Waypoints, 500.0f, 200.0f);
	TestTrue(TEXT("Trajectory should be valid"), Trajectory.bIsValid);

	// 测试轨迹采样
	if (Trajectory.bIsValid && Trajectory.TotalDuration > 0.0f)
	{
		// 采样起点
		FTrajectoryPoint StartPoint = Optimizer->SampleTrajectory(Trajectory, 0.0f);
		UAV_TEST_VECTOR_EQUAL(StartPoint.Position, FVector(0.0f, 0.0f, 0.0f), 50.0f);

		// 采样终点
		FTrajectoryPoint EndPoint = Optimizer->SampleTrajectory(Trajectory, Trajectory.TotalDuration);
		UAV_TEST_VECTOR_EQUAL(EndPoint.Position, FVector(1000.0f, 0.0f, 0.0f), 50.0f);

		// 采样中点
		FTrajectoryPoint MidPoint = Optimizer->SampleTrajectory(Trajectory, Trajectory.TotalDuration * 0.5f);
		// 中点应该在起点和终点之间
		TestTrue(TEXT("Mid point X should be between start and end"),
			MidPoint.Position.X > 0.0f && MidPoint.Position.X < 1000.0f);
	}

	return true;
}

// ==================== 轨迹采样测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryOptimizerSampleTest,
	"UAVSimulator.Planning.TrajectoryOptimizer.SampleTrajectory",
	UAV_TEST_FLAGS)

bool FTrajectoryOptimizerSampleTest::RunTest(const FString& Parameters)
{
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>();

	// 创建 3D 航点
	TArray<FVector> Waypoints = UAVTestHelpers::CreateWaypoints({
		FVector(0.0f, 0.0f, 0.0f),
		FVector(500.0f, 500.0f, 0.0f),
		FVector(1000.0f, 0.0f, 500.0f)
	});

	FTrajectory Trajectory = Optimizer->OptimizeTrajectory(Waypoints, 500.0f, 200.0f);
	TestTrue(TEXT("3D trajectory should be valid"), Trajectory.bIsValid);

	if (Trajectory.bIsValid)
	{
		// 获取密集采样点
		TArray<FTrajectoryPoint> DenseSamples = Optimizer->GetDenseSamples(Trajectory, 0.1f);
		TestTrue(TEXT("Should have dense samples"), DenseSamples.Num() > 0);

		// 验证采样点时间戳递增
		for (int32 i = 1; i < DenseSamples.Num(); ++i)
		{
			TestTrue(FString::Printf(TEXT("Sample %d timestamp should be greater than previous"), i),
				DenseSamples[i].TimeStamp >= DenseSamples[i - 1].TimeStamp);
		}

		// 验证采样点位置连续
		for (int32 i = 1; i < DenseSamples.Num(); ++i)
		{
			float Distance = FVector::Dist(DenseSamples[i].Position, DenseSamples[i - 1].Position);
			// 采样间隔 0.1s，最大速度 500cm/s，所以最大距离约 50cm + 容差
			TestTrue(FString::Printf(TEXT("Sample %d should be continuous"), i), Distance < 100.0f);
		}
	}

	return true;
}

// ==================== 完整轨迹优化测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryOptimizerFullOptimizationTest,
	"UAVSimulator.Planning.TrajectoryOptimizer.FullOptimization",
	UAV_TEST_FLAGS)

bool FTrajectoryOptimizerFullOptimizationTest::RunTest(const FString& Parameters)
{
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>();

	// 创建复杂航点序列
	TArray<FVector> Waypoints = UAVTestHelpers::CreateWaypoints({
		FVector(0.0f, 0.0f, 0.0f),
		FVector(300.0f, 200.0f, 100.0f),
		FVector(600.0f, 0.0f, 200.0f),
		FVector(900.0f, 200.0f, 100.0f),
		FVector(1200.0f, 0.0f, 0.0f)
	});

	FTrajectory Trajectory = Optimizer->OptimizeTrajectory(Waypoints, 500.0f, 200.0f);

	// 验证轨迹有效性
	TestTrue(TEXT("Complex trajectory should be valid"), Trajectory.bIsValid);
	TestTrue(TEXT("Trajectory should have points"), Trajectory.Points.Num() > 0);
	TestTrue(TEXT("Total duration should be positive"), Trajectory.TotalDuration > 0.0f);

	// 验证轨迹经过所有航点（近似）
	if (Trajectory.bIsValid)
	{
		// 检查起点
		FTrajectoryPoint StartPoint = Optimizer->SampleTrajectory(Trajectory, 0.0f);
		UAV_TEST_VECTOR_EQUAL(StartPoint.Position, Waypoints[0], 100.0f);

		// 检查终点
		FTrajectoryPoint EndPoint = Optimizer->SampleTrajectory(Trajectory, Trajectory.TotalDuration);
		UAV_TEST_VECTOR_EQUAL(EndPoint.Position, Waypoints.Last(), 100.0f);
	}

	return true;
}

// ==================== 速度约束测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryOptimizerVelocityConstraintTest,
	"UAVSimulator.Planning.TrajectoryOptimizer.VelocityConstraint",
	UAV_TEST_FLAGS)

bool FTrajectoryOptimizerVelocityConstraintTest::RunTest(const FString& Parameters)
{
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>();

	TArray<FVector> Waypoints = UAVTestHelpers::CreateWaypoints({
		FVector(0.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f)
	});

	float MaxVelocity = 300.0f;
	FTrajectory Trajectory = Optimizer->OptimizeTrajectory(Waypoints, MaxVelocity, 200.0f);

	TestTrue(TEXT("Trajectory should be valid"), Trajectory.bIsValid);

	if (Trajectory.bIsValid)
	{
		// 采样并检查速度
		TArray<FTrajectoryPoint> Samples = Optimizer->GetDenseSamples(Trajectory, 0.05f);

		for (int32 i = 0; i < Samples.Num(); ++i)
		{
			float Speed = Samples[i].Velocity.Size();
			// 速度应该不超过最大速度（加一些容差）
			TestTrue(FString::Printf(TEXT("Sample %d velocity should be within limit"), i),
				Speed <= MaxVelocity * 1.2f);
		}
	}

	return true;
}

// ==================== 加速度约束测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryOptimizerAccelerationConstraintTest,
	"UAVSimulator.Planning.TrajectoryOptimizer.AccelerationConstraint",
	UAV_TEST_FLAGS)

bool FTrajectoryOptimizerAccelerationConstraintTest::RunTest(const FString& Parameters)
{
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>();

	TArray<FVector> Waypoints = UAVTestHelpers::CreateWaypoints({
		FVector(0.0f, 0.0f, 0.0f),
		FVector(500.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f)
	});

	float MaxAcceleration = 150.0f;
	FTrajectory Trajectory = Optimizer->OptimizeTrajectory(Waypoints, 500.0f, MaxAcceleration);

	TestTrue(TEXT("Trajectory should be valid"), Trajectory.bIsValid);

	if (Trajectory.bIsValid)
	{
		// 采样并检查加速度
		TArray<FTrajectoryPoint> Samples = Optimizer->GetDenseSamples(Trajectory, 0.05f);

		for (int32 i = 0; i < Samples.Num(); ++i)
		{
			float AccelMagnitude = Samples[i].Acceleration.Size();
			// 加速度应该不超过最大加速度（加一些容差）
			TestTrue(FString::Printf(TEXT("Sample %d acceleration should be within limit"), i),
				AccelMagnitude <= MaxAcceleration * 2.0f);
		}
	}

	return true;
}

// ==================== 带时间分配的轨迹优化测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryOptimizerWithTimingTest,
	"UAVSimulator.Planning.TrajectoryOptimizer.WithTiming",
	UAV_TEST_FLAGS)

bool FTrajectoryOptimizerWithTimingTest::RunTest(const FString& Parameters)
{
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>();

	TArray<FVector> Waypoints = UAVTestHelpers::CreateWaypoints({
		FVector(0.0f, 0.0f, 0.0f),
		FVector(500.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f)
	});

	// 指定每段的时间
	TArray<float> SegmentTimes;
	SegmentTimes.Add(2.0f); // 第一段 2 秒
	SegmentTimes.Add(3.0f); // 第二段 3 秒

	FTrajectory Trajectory = Optimizer->OptimizeTrajectoryWithTiming(Waypoints, SegmentTimes);

	TestTrue(TEXT("Trajectory with timing should be valid"), Trajectory.bIsValid);

	if (Trajectory.bIsValid)
	{
		// 验证总时长
		UAV_TEST_FLOAT_EQUAL(Trajectory.TotalDuration, 5.0f, 0.5f);

		// 验证起点和终点
		FTrajectoryPoint StartPoint = Optimizer->SampleTrajectory(Trajectory, 0.0f);
		UAV_TEST_VECTOR_EQUAL(StartPoint.Position, Waypoints[0], 50.0f);

		FTrajectoryPoint EndPoint = Optimizer->SampleTrajectory(Trajectory, Trajectory.TotalDuration);
		UAV_TEST_VECTOR_EQUAL(EndPoint.Position, Waypoints.Last(), 50.0f);
	}

	return true;
}

// ==================== 空航点测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryOptimizerEmptyWaypointsTest,
	"UAVSimulator.Planning.TrajectoryOptimizer.EmptyWaypoints",
	UAV_TEST_FLAGS)

bool FTrajectoryOptimizerEmptyWaypointsTest::RunTest(const FString& Parameters)
{
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>();

	// 测试空航点
	TArray<FVector> EmptyWaypoints;
	FTrajectory Trajectory = Optimizer->OptimizeTrajectory(EmptyWaypoints, 500.0f, 200.0f);
	TestFalse(TEXT("Empty waypoints should produce invalid trajectory"), Trajectory.bIsValid);

	// 测试单个航点
	TArray<FVector> SingleWaypoint;
	SingleWaypoint.Add(FVector(100.0f, 0.0f, 0.0f));
	Trajectory = Optimizer->OptimizeTrajectory(SingleWaypoint, 500.0f, 200.0f);
	// 单个航点可能产生有效或无效轨迹，取决于实现

	return true;
}

// ==================== 轨迹平滑性测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryOptimizerSmoothnessTest,
	"UAVSimulator.Planning.TrajectoryOptimizer.Smoothness",
	UAV_TEST_FLAGS)

bool FTrajectoryOptimizerSmoothnessTest::RunTest(const FString& Parameters)
{
	UTrajectoryOptimizer* Optimizer = NewObject<UTrajectoryOptimizer>();

	// 创建带急转弯的航点
	TArray<FVector> Waypoints = UAVTestHelpers::CreateWaypoints({
		FVector(0.0f, 0.0f, 0.0f),
		FVector(500.0f, 0.0f, 0.0f),
		FVector(500.0f, 500.0f, 0.0f),
		FVector(0.0f, 500.0f, 0.0f)
	});

	FTrajectory Trajectory = Optimizer->OptimizeTrajectory(Waypoints, 500.0f, 200.0f);
	TestTrue(TEXT("Trajectory should be valid"), Trajectory.bIsValid);

	if (Trajectory.bIsValid)
	{
		// 获取密集采样
		TArray<FTrajectoryPoint> Samples = Optimizer->GetDenseSamples(Trajectory, 0.02f);

		// 检查速度变化的平滑性（加速度应该连续）
		for (int32 i = 2; i < Samples.Num(); ++i)
		{
			FVector AccelChange = Samples[i].Acceleration - Samples[i - 1].Acceleration;
			float JerkMagnitude = AccelChange.Size() / 0.02f; // 近似 jerk

			// Jerk 应该在合理范围内（最小 snap 轨迹应该有平滑的 jerk）
			// 这是一个宽松的检查
			TestTrue(FString::Printf(TEXT("Sample %d jerk should be reasonable"), i),
				JerkMagnitude < 10000.0f);
		}
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
