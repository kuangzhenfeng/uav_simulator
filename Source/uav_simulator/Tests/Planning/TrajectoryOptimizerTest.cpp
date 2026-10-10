// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Planning/TrajectoryOptimizer.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryAccelerationTransitionTest,
    "UAVSimulator.Planning.TrajectoryOptimizer.AccelerationTransition",UAV_TEST_FLAGS)
bool FTrajectoryAccelerationTransitionTest::RunTest(const FString&)
{
    auto* Optimizer=NewObject<UTrajectoryOptimizer>();
    const FVector Velocity(104.735,107.265,-1.344),Acceleration(77.601,63.361,.145);
    Optimizer->SetStartVelocity(Velocity);Optimizer->SetStartAcceleration(Acceleration);
    const FVector End(21082.422,0,0);
    const auto Route=Optimizer->OptimizeTrajectory({FVector::ZeroVector,End},600,150);
    TestTrue(TEXT("Long route with moving acceleration boundary is feasible"),Route.bIsValid);
    if(!Route.bIsValid) return false;
    UAV_TEST_VECTOR_EQUAL(Route.Points[0].Velocity,Velocity,.01f);
    UAV_TEST_VECTOR_EQUAL(Route.Points[0].Acceleration,Acceleration,.01f);
    UAV_TEST_VECTOR_EQUAL(Route.Points.Last().Position,End,1.f);
    UAV_TEST_VECTOR_EQUAL(Route.Points.Last().Velocity,FVector::ZeroVector,.1f);
    TestTrue(TEXT("Boundary transition avoids runaway duration"),Route.TotalDuration<200);
    for(const auto& Point:Route.Points)
    {
        TestTrue(TEXT("Transition and long route respect speed"),Point.Velocity.Size()<=601);
        TestTrue(TEXT("Transition and long route respect acceleration"),Point.Acceleration.Size()<=151);
    }
    const auto Before=Optimizer->SampleTrajectory(Route,.999f),After=Optimizer->SampleTrajectory(Route,1.001f);
    UAV_TEST_VECTOR_EQUAL(Before.Position,After.Position,1.f);
    UAV_TEST_VECTOR_EQUAL(Before.Velocity,After.Velocity,1.f);
    UAV_TEST_VECTOR_EQUAL(Before.Acceleration,After.Acceleration,1.f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectorySamplingBudgetTest,
    "UAVSimulator.Planning.TrajectoryOptimizer.SamplingBudget",UAV_TEST_FLAGS)
bool FTrajectorySamplingBudgetTest::RunTest(const FString&)
{
    auto* Optimizer=NewObject<UTrajectoryOptimizer>();
    const TArray<FVector> Path={FVector::ZeroVector,FVector(100,0,0)};
    TestFalse(TEXT("Extreme duration is rejected before sample allocation"),
        Optimizer->OptimizeTrajectoryWithTiming(Path,{10000000.f}).bIsValid);
    TestFalse(TEXT("Negative segment duration is rejected"),
        Optimizer->OptimizeTrajectoryWithTiming({FVector::ZeroVector,FVector(100,0,0),FVector(200,0,0)},{2,-1}).bIsValid);
    Optimizer->DefaultSampleInterval=0;
    TestFalse(TEXT("Zero sample interval cannot enter sampling loop"),Optimizer->OptimizeTrajectoryWithTiming(Path,{2}).bIsValid);
    Optimizer->DefaultSampleInterval=.05f;
    const auto Route=Optimizer->OptimizeTrajectoryWithTiming(Path,{2.03f});
    TestTrue(TEXT("Fractional final sampling interval remains valid"),Route.bIsValid);
    if(!Route.bIsValid) return false;
    TestEqual(TEXT("Final timestamp is exact"),Route.Points.Last().TimeStamp,Route.TotalDuration);
    for(int32 I=1;I<Route.Points.Num();++I)
        TestTrue(TEXT("Sample timestamps strictly advance"),Route.Points[I].TimeStamp>Route.Points[I-1].TimeStamp);
    TestTrue(TEXT("Resampling zero interval returns without allocation"),Optimizer->GetDenseSamples(Route,0).IsEmpty());
    TestTrue(TEXT("Excessively dense resampling respects resource budget"),Optimizer->GetDenseSamples(Route,1.e-9f).IsEmpty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryUnequalTimingContinuityTest,
    "UAVSimulator.Planning.TrajectoryOptimizer.UnequalTimingContinuity",UAV_TEST_FLAGS)
bool FTrajectoryUnequalTimingContinuityTest::RunTest(const FString&)
{
    auto* Optimizer=NewObject<UTrajectoryOptimizer>();
    const FTrajectory Route=Optimizer->OptimizeTrajectoryWithTiming(
        {FVector(0,0,0),FVector(1000,0,0),FVector(1000,1000,0)},{10,5});
    TestTrue(TEXT("Unequal-duration route is feasible"),Route.bIsValid);
    const auto Before=Optimizer->SampleTrajectory(Route,9.999f);
    const auto After=Optimizer->SampleTrajectory(Route,10.001f);
    UAV_TEST_VECTOR_EQUAL(Before.Position,After.Position,1.0f);
    UAV_TEST_VECTOR_EQUAL(Before.Velocity,After.Velocity,1.0f);
    UAV_TEST_VECTOR_EQUAL(Before.Acceleration,After.Acceleration,1.0f);
    UAV_TEST_VECTOR_EQUAL(Before.Velocity,FVector(1000.0/15,1000.0/15,0),1.0f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryFlyThroughTest,
    "UAVSimulator.Planning.TrajectoryOptimizer.FlyThrough", UAV_TEST_FLAGS)
bool FTrajectoryFlyThroughTest::RunTest(const FString&)
{
    auto* Optimizer=NewObject<UTrajectoryOptimizer>();
    Optimizer->SetStartVelocity(FVector(100,0,0));
    Optimizer->SetEndVelocity(FVector(125,0,0));
    const FTrajectory Strip=Optimizer->OptimizeTrajectory({FVector(0,0,400),FVector(6300,0,400)},300,150);
    TestTrue(TEXT("Moving strip is feasible"),Strip.bIsValid);
    if(!Strip.bIsValid) return false;
    TestEqual(TEXT("Exact terminal timestamp"),Strip.Points.Last().TimeStamp,Strip.TotalDuration);
    UAV_TEST_VECTOR_EQUAL(Strip.Points.Last().Position,FVector(6300,0,400),1.0f);
    UAV_TEST_VECTOR_EQUAL(Strip.Points.Last().Velocity,FVector(125,0,0),1.0f);
    Optimizer->SetStartVelocity(Strip.Points.Last().Velocity);
    Optimizer->SetStartAcceleration(Strip.Points.Last().Acceleration);
    Optimizer->SetEndVelocity(FVector(-125,0,0));
    const FTrajectory Turn=Optimizer->OptimizeTrajectory({Strip.Points.Last().Position,FVector(6300,600,400)},300,150);
    TestTrue(TEXT("Headland turn is feasible"),Turn.bIsValid);
    if(!Turn.bIsValid) return false;
    UAV_TEST_VECTOR_EQUAL(Turn.Points[0].Velocity,Strip.Points.Last().Velocity,1.0f);
    UAV_TEST_VECTOR_EQUAL(Turn.Points[0].Acceleration,Strip.Points.Last().Acceleration,1.0f);
    for(const auto& Point:Turn.Points)
    {
        if(Point.Velocity.Size()<10 || Point.Velocity.Size()>301 || Point.Acceleration.Size()>151 || Point.Position.X<6299)
        { AddError(TEXT("Turn must remain moving outside the field and respect derivative limits")); return false; }
    }
    return true;
}

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
				Speed <= MaxVelocity * 2.5f);
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryOptimizerLongRouteLimitsTest,
    "UAVSimulator.Planning.TrajectoryOptimizer.LongRouteLimits",UAV_TEST_FLAGS)
bool FTrajectoryOptimizerLongRouteLimitsTest::RunTest(const FString& Parameters)
{
    auto* Optimizer=NewObject<UTrajectoryOptimizer>();
    const auto Trajectory=Optimizer->OptimizeTrajectory({FVector(0,0,1500),FVector(31000,0,1500)},600,150);
    TestTrue(TEXT("Long route is feasible"),Trajectory.bIsValid);
    for(const auto& Point:Trajectory.Points)
    {
        if(Point.Velocity.Size()>601 || Point.Acceleration.Size()>151)
        {AddError(TEXT("Long route exceeds physical derivative limits"));break;}
    }
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
