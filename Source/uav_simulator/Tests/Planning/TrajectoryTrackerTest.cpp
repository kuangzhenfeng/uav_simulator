// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Planning/TrajectoryTracker.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 状态机测试 - 启动/停止 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTrackerStartStopTest,
	"UAVSimulator.Planning.TrajectoryTracker.StartStop",
	UAV_TEST_FLAGS)

bool FTrajectoryTrackerStartStopTest::RunTest(const FString& Parameters)
{
	UTrajectoryTracker* Tracker = NewObject<UTrajectoryTracker>();

	// 创建测试轨迹
	FTrajectory Trajectory = UAVTestHelpers::CreateLinearTrajectory(
		FVector(0.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f),
		5.0f,
		10
	);

	// 初始状态
	TestFalse(TEXT("Should not be tracking initially"), Tracker->IsTracking());
	TestFalse(TEXT("Should not be complete initially"), Tracker->IsComplete());

	// 设置轨迹
	Tracker->SetTrajectory(Trajectory);

	// 启动跟踪
	Tracker->StartTracking();
	TestTrue(TEXT("Should be tracking after start"), Tracker->IsTracking());
	TestFalse(TEXT("Should not be complete after start"), Tracker->IsComplete());

	// 停止跟踪
	Tracker->StopTracking();
	TestFalse(TEXT("Should not be tracking after stop"), Tracker->IsTracking());

	return true;
}

// ==================== 状态机测试 - 暂停/恢复 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTrackerPauseResumeTest,
	"UAVSimulator.Planning.TrajectoryTracker.PauseResume",
	UAV_TEST_FLAGS)

bool FTrajectoryTrackerPauseResumeTest::RunTest(const FString& Parameters)
{
	UTrajectoryTracker* Tracker = NewObject<UTrajectoryTracker>();

	FTrajectory Trajectory = UAVTestHelpers::CreateLinearTrajectory(
		FVector(0.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f),
		5.0f,
		10
	);

	Tracker->SetTrajectory(Trajectory);
	Tracker->StartTracking();

	// 暂停
	Tracker->PauseTracking();
	TestFalse(TEXT("Should not be tracking when paused"), Tracker->IsTracking());

	// 恢复
	Tracker->ResumeTracking();
	TestTrue(TEXT("Should be tracking after resume"), Tracker->IsTracking());

	return true;
}

// ==================== 进度计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTrackerProgressTest,
	"UAVSimulator.Planning.TrajectoryTracker.Progress",
	UAV_TEST_FLAGS)

bool FTrajectoryTrackerProgressTest::RunTest(const FString& Parameters)
{
	UTrajectoryTracker* Tracker = NewObject<UTrajectoryTracker>();

	FTrajectory Trajectory = UAVTestHelpers::CreateLinearTrajectory(
		FVector(0.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f),
		10.0f,
		20
	);

	Tracker->SetTrajectory(Trajectory);
	Tracker->StartTracking();

	// 初始进度应该是 0
	float InitialProgress = Tracker->GetProgress();
	UAV_TEST_FLOAT_EQUAL(InitialProgress, 0.0f, 0.1f);

	// 获取不同时间点的期望状态来测试进度
	FTrajectoryPoint MidState = Tracker->GetDesiredState(5.0f);
	// 中点位置应该在起点和终点之间
	TestTrue(TEXT("Mid state X should be around 500"), MidState.Position.X > 400.0f && MidState.Position.X < 600.0f);

	FTrajectoryPoint EndState = Tracker->GetDesiredState(10.0f);
	// 终点位置应该接近目标
	UAV_TEST_VECTOR_EQUAL(EndState.Position, FVector(1000.0f, 0.0f, 0.0f), 100.0f);

	return true;
}

// ==================== 轨迹插值测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTrackerInterpolationTest,
	"UAVSimulator.Planning.TrajectoryTracker.Interpolation",
	UAV_TEST_FLAGS)

bool FTrajectoryTrackerInterpolationTest::RunTest(const FString& Parameters)
{
	UTrajectoryTracker* Tracker = NewObject<UTrajectoryTracker>();

	// 创建圆形轨迹
	FTrajectory Trajectory = UAVTestHelpers::CreateCircularTrajectory(
		FVector(0.0f, 0.0f, 0.0f),
		500.0f,
		10.0f,
		36
	);

	Tracker->SetTrajectory(Trajectory);
	Tracker->StartTracking();

	// 测试不同时间点的插值
	for (float t = 0.0f; t <= 10.0f; t += 1.0f)
	{
		FTrajectoryPoint State = Tracker->GetDesiredState(t);

		// 验证位置在圆上（距离圆心约 500）
		float DistanceFromCenter = FVector::Dist(State.Position, FVector(0.0f, 0.0f, 0.0f));
		UAV_TEST_FLOAT_EQUAL(DistanceFromCenter, 500.0f, 50.0f);

		// 验证速度方向与位置垂直（圆周运动）
		if (State.Velocity.Size() > 1.0f)
		{
			FVector PosNorm = State.Position.GetSafeNormal();
			FVector VelNorm = State.Velocity.GetSafeNormal();
			float DotProduct = FMath::Abs(FVector::DotProduct(PosNorm, VelNorm));
			// 圆周运动中，速度应该与位置向量垂直
			TestTrue(FString::Printf(TEXT("Velocity at t=%f should be perpendicular to position"), t),
				DotProduct < 0.3f);
		}
	}

	return true;
}

// ==================== 跟踪误差计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTrackerErrorTest,
	"UAVSimulator.Planning.TrajectoryTracker.TrackingError",
	UAV_TEST_FLAGS)

bool FTrajectoryTrackerErrorTest::RunTest(const FString& Parameters)
{
	UTrajectoryTracker* Tracker = NewObject<UTrajectoryTracker>();

	FTrajectory Trajectory = UAVTestHelpers::CreateLinearTrajectory(
		FVector(0.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f),
		5.0f,
		10
	);

	Tracker->SetTrajectory(Trajectory);
	Tracker->StartTracking();

	// 注意：GetTrackingError 使用内部的 TrackingTime（初始为 0）
	// 所以期望位置是轨迹起点 (0, 0, 0)
	FTrajectoryPoint DesiredState = Tracker->GetDesiredState(0.0f);

	// 测试跟踪误差：当前位置等于期望位置时，误差应该为零
	FVector CurrentPosition = DesiredState.Position; // 完美跟踪
	FVector Error = Tracker->GetTrackingError(CurrentPosition);
	UAV_TEST_VECTOR_EQUAL(Error, FVector::ZeroVector, 10.0f);

	// 测试有偏差的跟踪
	// GetTrackingError 返回 DesiredPosition - CurrentPosition
	FVector OffsetPosition = DesiredState.Position + FVector(100.0f, 50.0f, 0.0f);
	Error = Tracker->GetTrackingError(OffsetPosition);
	// 误差 = 期望位置 - 当前位置 = (0,0,0) - (100,50,0) = (-100,-50,0)
	UAV_TEST_VECTOR_EQUAL(Error, FVector(-100.0f, -50.0f, 0.0f), 10.0f);

	return true;
}

// ==================== 重置测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTrackerResetTest,
	"UAVSimulator.Planning.TrajectoryTracker.Reset",
	UAV_TEST_FLAGS)

bool FTrajectoryTrackerResetTest::RunTest(const FString& Parameters)
{
	UTrajectoryTracker* Tracker = NewObject<UTrajectoryTracker>();

	FTrajectory Trajectory = UAVTestHelpers::CreateLinearTrajectory(
		FVector(0.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f),
		5.0f,
		10
	);

	Tracker->SetTrajectory(Trajectory);
	Tracker->StartTracking();

	// 模拟一些进度
	FTrajectoryPoint State = Tracker->GetDesiredState(2.5f);

	// 重置
	Tracker->Reset();

	// 验证重置后的状态
	TestFalse(TEXT("Should not be tracking after reset"), Tracker->IsTracking());
	TestFalse(TEXT("Should not be complete after reset"), Tracker->IsComplete());
	UAV_TEST_FLOAT_EQUAL(Tracker->GetCurrentTime(), 0.0f, 0.01f);

	return true;
}

// ==================== 轨迹完成测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTrackerCompletionTest,
	"UAVSimulator.Planning.TrajectoryTracker.Completion",
	UAV_TEST_FLAGS)

bool FTrajectoryTrackerCompletionTest::RunTest(const FString& Parameters)
{
	UTrajectoryTracker* Tracker = NewObject<UTrajectoryTracker>();

	FTrajectory Trajectory = UAVTestHelpers::CreateLinearTrajectory(
		FVector(0.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f),
		2.0f,
		10
	);

	Tracker->SetTrajectory(Trajectory);
	Tracker->StartTracking();

	// 获取超过轨迹时长的状态
	FTrajectoryPoint EndState = Tracker->GetDesiredState(3.0f);

	// 终点位置应该保持在轨迹终点
	UAV_TEST_VECTOR_EQUAL(EndState.Position, FVector(1000.0f, 0.0f, 0.0f), 100.0f);

	return true;
}

// ==================== 空轨迹测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTrackerEmptyTrajectoryTest,
	"UAVSimulator.Planning.TrajectoryTracker.EmptyTrajectory",
	UAV_TEST_FLAGS)

bool FTrajectoryTrackerEmptyTrajectoryTest::RunTest(const FString& Parameters)
{
	UTrajectoryTracker* Tracker = NewObject<UTrajectoryTracker>();

	// 设置空轨迹
	FTrajectory EmptyTrajectory;
	Tracker->SetTrajectory(EmptyTrajectory);

	// 尝试启动跟踪
	Tracker->StartTracking();

	// 空轨迹应该立即完成或不启动
	// 具体行为取决于实现

	// 获取状态不应该崩溃
	FTrajectoryPoint State = Tracker->GetDesiredState(0.0f);

	return true;
}

// ==================== 获取当前轨迹测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTrackerGetTrajectoryTest,
	"UAVSimulator.Planning.TrajectoryTracker.GetTrajectory",
	UAV_TEST_FLAGS)

bool FTrajectoryTrackerGetTrajectoryTest::RunTest(const FString& Parameters)
{
	UTrajectoryTracker* Tracker = NewObject<UTrajectoryTracker>();

	FTrajectory Trajectory = UAVTestHelpers::CreateLinearTrajectory(
		FVector(100.0f, 200.0f, 300.0f),
		FVector(1000.0f, 0.0f, 0.0f),
		5.0f,
		10
	);

	Tracker->SetTrajectory(Trajectory);

	// 获取轨迹
	const FTrajectory& RetrievedTrajectory = Tracker->GetTrajectory();

	// 验证轨迹属性
	TestTrue(TEXT("Retrieved trajectory should be valid"), RetrievedTrajectory.bIsValid);
	TestEqual(TEXT("Trajectory point count should match"), RetrievedTrajectory.Points.Num(), 10);
	UAV_TEST_FLOAT_EQUAL(RetrievedTrajectory.TotalDuration, 5.0f, 0.1f);

	// 验证起点
	if (RetrievedTrajectory.Points.Num() > 0)
	{
		UAV_TEST_VECTOR_EQUAL(RetrievedTrajectory.Points[0].Position, FVector(100.0f, 200.0f, 300.0f), 1.0f);
	}

	return true;
}

// ==================== 时间缩放测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTrackerTimeScaleTest,
	"UAVSimulator.Planning.TrajectoryTracker.TimeScale",
	UAV_TEST_FLAGS)

bool FTrajectoryTrackerTimeScaleTest::RunTest(const FString& Parameters)
{
	UTrajectoryTracker* Tracker = NewObject<UTrajectoryTracker>();

	FTrajectory Trajectory = UAVTestHelpers::CreateLinearTrajectory(
		FVector(0.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f),
		10.0f,
		20
	);

	Tracker->SetTrajectory(Trajectory);
	Tracker->StartTracking();

	// 测试正常时间的状态
	FTrajectoryPoint NormalState = Tracker->GetDesiredState(5.0f);

	// 状态应该在中点附近
	TestTrue(TEXT("Normal state should be around midpoint"),
		NormalState.Position.X > 400.0f && NormalState.Position.X < 600.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
