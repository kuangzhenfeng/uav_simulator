// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Core/UAVTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== FUAVState 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVStateTest,
	"UAVSimulator.Core.UAVTypes.UAVState",
	UAV_TEST_FLAGS)

bool FUAVStateTest::RunTest(const FString& Parameters)
{
	// 测试默认构造
	FUAVState DefaultState;
	UAV_TEST_VECTOR_EQUAL(DefaultState.Position, FVector::ZeroVector, 0.1f);
	UAV_TEST_VECTOR_EQUAL(DefaultState.Velocity, FVector::ZeroVector, 0.1f);
	UAV_TEST_ROTATOR_EQUAL(DefaultState.Rotation, FRotator::ZeroRotator, 0.1f);
	UAV_TEST_VECTOR_EQUAL(DefaultState.AngularVelocity, FVector::ZeroVector, 0.1f);

	// 测试赋值
	FUAVState State;
	State.Position = FVector(100.0f, 200.0f, 300.0f);
	State.Velocity = FVector(10.0f, 20.0f, 30.0f);
	State.Rotation = FRotator(15.0f, 30.0f, 45.0f);
	State.AngularVelocity = FVector(1.0f, 2.0f, 3.0f);

	UAV_TEST_VECTOR_EQUAL(State.Position, FVector(100.0f, 200.0f, 300.0f), 0.1f);
	UAV_TEST_VECTOR_EQUAL(State.Velocity, FVector(10.0f, 20.0f, 30.0f), 0.1f);
	UAV_TEST_ROTATOR_EQUAL(State.Rotation, FRotator(15.0f, 30.0f, 45.0f), 0.1f);
	UAV_TEST_VECTOR_EQUAL(State.AngularVelocity, FVector(1.0f, 2.0f, 3.0f), 0.1f);

	// 测试辅助函数创建
	FUAVState HelperState = UAVTestHelpers::CreateUAVState(
		FVector(500.0f, 0.0f, 100.0f),
		FVector(50.0f, 0.0f, 0.0f),
		FRotator(0.0f, 90.0f, 0.0f),
		FVector(0.0f, 0.0f, 5.0f)
	);
	UAV_TEST_VECTOR_EQUAL(HelperState.Position, FVector(500.0f, 0.0f, 100.0f), 0.1f);

	return true;
}

// ==================== FMotorOutput 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMotorOutputTest,
	"UAVSimulator.Core.UAVTypes.MotorOutput",
	UAV_TEST_FLAGS)

bool FMotorOutputTest::RunTest(const FString& Parameters)
{
	// 测试默认构造
	FMotorOutput DefaultOutput;
	TestEqual(TEXT("Should have 4 motors"), DefaultOutput.Thrusts.Num(), 4);

	for (int32 i = 0; i < 4; ++i)
	{
		UAV_TEST_FLOAT_EQUAL(DefaultOutput.Thrusts[i], 0.0f, 0.01f);
	}

	// 测试赋值
	FMotorOutput Output;
	Output.Thrusts[0] = 0.5f;
	Output.Thrusts[1] = 0.6f;
	Output.Thrusts[2] = 0.7f;
	Output.Thrusts[3] = 0.8f;

	UAV_TEST_FLOAT_EQUAL(Output.Thrusts[0], 0.5f, 0.01f);
	UAV_TEST_FLOAT_EQUAL(Output.Thrusts[1], 0.6f, 0.01f);
	UAV_TEST_FLOAT_EQUAL(Output.Thrusts[2], 0.7f, 0.01f);
	UAV_TEST_FLOAT_EQUAL(Output.Thrusts[3], 0.8f, 0.01f);

	return true;
}

// ==================== FTrajectoryPoint 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryPointTest,
	"UAVSimulator.Core.UAVTypes.TrajectoryPoint",
	UAV_TEST_FLAGS)

bool FTrajectoryPointTest::RunTest(const FString& Parameters)
{
	// 测试默认构造
	FTrajectoryPoint DefaultPoint;
	UAV_TEST_VECTOR_EQUAL(DefaultPoint.Position, FVector::ZeroVector, 0.1f);
	UAV_TEST_VECTOR_EQUAL(DefaultPoint.Velocity, FVector::ZeroVector, 0.1f);
	UAV_TEST_VECTOR_EQUAL(DefaultPoint.Acceleration, FVector::ZeroVector, 0.1f);
	UAV_TEST_FLOAT_EQUAL(DefaultPoint.TimeStamp, 0.0f, 0.01f);
	UAV_TEST_FLOAT_EQUAL(DefaultPoint.Yaw, 0.0f, 0.01f);

	// 测试位置+时间构造
	FTrajectoryPoint PosTimePoint(FVector(100.0f, 200.0f, 300.0f), 5.0f);
	UAV_TEST_VECTOR_EQUAL(PosTimePoint.Position, FVector(100.0f, 200.0f, 300.0f), 0.1f);
	UAV_TEST_FLOAT_EQUAL(PosTimePoint.TimeStamp, 5.0f, 0.01f);

	// 测试完整构造
	FTrajectoryPoint FullPoint(
		FVector(500.0f, 0.0f, 100.0f),
		FVector(50.0f, 0.0f, 0.0f),
		FVector(5.0f, 0.0f, 0.0f),
		10.0f,
		45.0f
	);
	UAV_TEST_VECTOR_EQUAL(FullPoint.Position, FVector(500.0f, 0.0f, 100.0f), 0.1f);
	UAV_TEST_VECTOR_EQUAL(FullPoint.Velocity, FVector(50.0f, 0.0f, 0.0f), 0.1f);
	UAV_TEST_VECTOR_EQUAL(FullPoint.Acceleration, FVector(5.0f, 0.0f, 0.0f), 0.1f);
	UAV_TEST_FLOAT_EQUAL(FullPoint.TimeStamp, 10.0f, 0.01f);
	UAV_TEST_FLOAT_EQUAL(FullPoint.Yaw, 45.0f, 0.01f);

	// 测试辅助函数
	FTrajectoryPoint HelperPoint = UAVTestHelpers::CreateTrajectoryPoint(
		FVector(1000.0f, 0.0f, 0.0f),
		FVector(100.0f, 0.0f, 0.0f),
		FVector::ZeroVector,
		15.0f,
		90.0f
	);
	UAV_TEST_FLOAT_EQUAL(HelperPoint.Yaw, 90.0f, 0.01f);

	return true;
}

// ==================== FTrajectory 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTrajectoryTest,
	"UAVSimulator.Core.UAVTypes.Trajectory",
	UAV_TEST_FLAGS)

bool FTrajectoryTest::RunTest(const FString& Parameters)
{
	// 测试默认构造
	FTrajectory DefaultTrajectory;
	TestTrue(TEXT("Default trajectory should be empty"), DefaultTrajectory.IsEmpty());
	TestEqual(TEXT("Default trajectory should have 0 points"), DefaultTrajectory.Num(), 0);
	TestFalse(TEXT("Default trajectory should be invalid"), DefaultTrajectory.bIsValid);
	UAV_TEST_FLOAT_EQUAL(DefaultTrajectory.TotalDuration, 0.0f, 0.01f);

	// 测试添加点
	FTrajectory Trajectory;
	Trajectory.bIsValid = true;

	FTrajectoryPoint Point1(FVector(0.0f, 0.0f, 0.0f), 0.0f);
	FTrajectoryPoint Point2(FVector(500.0f, 0.0f, 0.0f), 2.5f);
	FTrajectoryPoint Point3(FVector(1000.0f, 0.0f, 0.0f), 5.0f);

	Trajectory.AddPoint(Point1);
	TestEqual(TEXT("Should have 1 point"), Trajectory.Num(), 1);
	TestFalse(TEXT("Should not be empty"), Trajectory.IsEmpty());

	Trajectory.AddPoint(Point2);
	Trajectory.AddPoint(Point3);
	TestEqual(TEXT("Should have 3 points"), Trajectory.Num(), 3);
	UAV_TEST_FLOAT_EQUAL(Trajectory.TotalDuration, 5.0f, 0.01f);

	// 测试清空
	Trajectory.Clear();
	TestTrue(TEXT("Should be empty after clear"), Trajectory.IsEmpty());
	TestFalse(TEXT("Should be invalid after clear"), Trajectory.bIsValid);
	UAV_TEST_FLOAT_EQUAL(Trajectory.TotalDuration, 0.0f, 0.01f);

	// 测试辅助函数创建线性轨迹
	FTrajectory LinearTrajectory = UAVTestHelpers::CreateLinearTrajectory(
		FVector(0.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f),
		10.0f,
		20
	);
	TestTrue(TEXT("Linear trajectory should be valid"), LinearTrajectory.bIsValid);
	TestEqual(TEXT("Linear trajectory should have 20 points"), LinearTrajectory.Num(), 20);
	UAV_TEST_FLOAT_EQUAL(LinearTrajectory.TotalDuration, 10.0f, 0.01f);

	// 测试辅助函数创建圆形轨迹
	FTrajectory CircularTrajectory = UAVTestHelpers::CreateCircularTrajectory(
		FVector(0.0f, 0.0f, 0.0f),
		500.0f,
		10.0f,
		36
	);
	TestTrue(TEXT("Circular trajectory should be valid"), CircularTrajectory.bIsValid);
	TestEqual(TEXT("Circular trajectory should have 36 points"), CircularTrajectory.Num(), 36);

	return true;
}

// ==================== FObstacleInfo 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleInfoTest,
	"UAVSimulator.Core.UAVTypes.ObstacleInfo",
	UAV_TEST_FLAGS)

bool FObstacleInfoTest::RunTest(const FString& Parameters)
{
	// 测试默认构造
	FObstacleInfo DefaultObstacle;
	TestEqual(TEXT("Default ID should be -1"), DefaultObstacle.ObstacleID, -1);
	TestEqual(TEXT("Default type should be Sphere"),
		static_cast<int32>(DefaultObstacle.Type),
		static_cast<int32>(EObstacleType::Sphere));
	UAV_TEST_VECTOR_EQUAL(DefaultObstacle.Center, FVector::ZeroVector, 0.1f);
	UAV_TEST_FLOAT_EQUAL(DefaultObstacle.SafetyMargin, 50.0f, 0.1f);
	TestFalse(TEXT("Default should not be dynamic"), DefaultObstacle.bIsDynamic);

	// 测试参数构造
	FObstacleInfo ParamObstacle(1, EObstacleType::Box, FVector(100.0f, 200.0f, 300.0f), FVector(50.0f, 75.0f, 100.0f));
	TestEqual(TEXT("ID should be 1"), ParamObstacle.ObstacleID, 1);
	TestEqual(TEXT("Type should be Box"),
		static_cast<int32>(ParamObstacle.Type),
		static_cast<int32>(EObstacleType::Box));
	UAV_TEST_VECTOR_EQUAL(ParamObstacle.Center, FVector(100.0f, 200.0f, 300.0f), 0.1f);
	UAV_TEST_VECTOR_EQUAL(ParamObstacle.Extents, FVector(50.0f, 75.0f, 100.0f), 0.1f);

	// 测试辅助函数创建球体
	FObstacleInfo SphereObstacle = UAVTestHelpers::CreateSphereObstacle(
		10, FVector(500.0f, 0.0f, 0.0f), 150.0f, 25.0f
	);
	TestEqual(TEXT("Sphere ID should be 10"), SphereObstacle.ObstacleID, 10);
	TestEqual(TEXT("Type should be Sphere"),
		static_cast<int32>(SphereObstacle.Type),
		static_cast<int32>(EObstacleType::Sphere));
	UAV_TEST_FLOAT_EQUAL(SphereObstacle.Extents.X, 150.0f, 0.1f);
	UAV_TEST_FLOAT_EQUAL(SphereObstacle.SafetyMargin, 25.0f, 0.1f);

	// 测试辅助函数创建盒体
	FObstacleInfo BoxObstacle = UAVTestHelpers::CreateBoxObstacle(
		20, FVector(0.0f, 500.0f, 0.0f), FVector(100.0f, 200.0f, 50.0f),
		FRotator(0.0f, 45.0f, 0.0f), 30.0f
	);
	TestEqual(TEXT("Box ID should be 20"), BoxObstacle.ObstacleID, 20);
	TestEqual(TEXT("Type should be Box"),
		static_cast<int32>(BoxObstacle.Type),
		static_cast<int32>(EObstacleType::Box));
	UAV_TEST_ROTATOR_EQUAL(BoxObstacle.Rotation, FRotator(0.0f, 45.0f, 0.0f), 0.1f);

	// 测试辅助函数创建圆柱体
	FObstacleInfo CylinderObstacle = UAVTestHelpers::CreateCylinderObstacle(
		30, FVector(0.0f, 0.0f, 500.0f), 100.0f, 200.0f, 40.0f
	);
	TestEqual(TEXT("Cylinder ID should be 30"), CylinderObstacle.ObstacleID, 30);
	TestEqual(TEXT("Type should be Cylinder"),
		static_cast<int32>(CylinderObstacle.Type),
		static_cast<int32>(EObstacleType::Cylinder));
	UAV_TEST_FLOAT_EQUAL(CylinderObstacle.Extents.X, 100.0f, 0.1f); // 半径
	UAV_TEST_FLOAT_EQUAL(CylinderObstacle.Extents.Z, 200.0f, 0.1f); // 半高度

	return true;
}

// ==================== EObstacleType 枚举测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleTypeEnumTest,
	"UAVSimulator.Core.UAVTypes.ObstacleTypeEnum",
	UAV_TEST_FLAGS)

bool FObstacleTypeEnumTest::RunTest(const FString& Parameters)
{
	// 验证枚举值
	TestEqual(TEXT("Sphere should be 0"), static_cast<int32>(EObstacleType::Sphere), 0);
	TestEqual(TEXT("Box should be 1"), static_cast<int32>(EObstacleType::Box), 1);
	TestEqual(TEXT("Cylinder should be 2"), static_cast<int32>(EObstacleType::Cylinder), 2);
	TestEqual(TEXT("Custom should be 3"), static_cast<int32>(EObstacleType::Custom), 3);

	// 测试枚举转换
	EObstacleType Type = EObstacleType::Box;
	TestEqual(TEXT("Type should be Box"), static_cast<int32>(Type), 1);

	return true;
}

// ==================== EPathPlanningAlgorithm 枚举测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPathPlanningAlgorithmEnumTest,
	"UAVSimulator.Core.UAVTypes.PathPlanningAlgorithmEnum",
	UAV_TEST_FLAGS)

bool FPathPlanningAlgorithmEnumTest::RunTest(const FString& Parameters)
{
	// 验证枚举值
	TestEqual(TEXT("AStar should be 0"), static_cast<int32>(EPathPlanningAlgorithm::AStar), 0);
	TestEqual(TEXT("RRT should be 1"), static_cast<int32>(EPathPlanningAlgorithm::RRT), 1);
	TestEqual(TEXT("RRTStar should be 2"), static_cast<int32>(EPathPlanningAlgorithm::RRTStar), 2);

	return true;
}

// ==================== EUAVControlMode 枚举测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVControlModeEnumTest,
	"UAVSimulator.Core.UAVTypes.UAVControlModeEnum",
	UAV_TEST_FLAGS)

bool FUAVControlModeEnumTest::RunTest(const FString& Parameters)
{
	// 验证枚举值
	TestEqual(TEXT("Attitude should be 0"), static_cast<int32>(EUAVControlMode::Attitude), 0);
	TestEqual(TEXT("Position should be 1"), static_cast<int32>(EUAVControlMode::Position), 1);
	TestEqual(TEXT("Trajectory should be 2"), static_cast<int32>(EUAVControlMode::Trajectory), 2);

	return true;
}

// ==================== FPlanningConfig 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanningConfigTest,
	"UAVSimulator.Core.UAVTypes.PlanningConfig",
	UAV_TEST_FLAGS)

bool FPlanningConfigTest::RunTest(const FString& Parameters)
{
	// 测试默认值
	FPlanningConfig DefaultConfig;
	TestEqual(TEXT("Default algorithm should be AStar"),
		static_cast<int32>(DefaultConfig.Algorithm),
		static_cast<int32>(EPathPlanningAlgorithm::AStar));
	TestTrue(TEXT("Default should enable trajectory optimization"), DefaultConfig.bEnableTrajectoryOptimization);
	UAV_TEST_FLOAT_EQUAL(DefaultConfig.MaxVelocity, 500.0f, 0.1f);
	UAV_TEST_FLOAT_EQUAL(DefaultConfig.MaxAcceleration, 200.0f, 0.1f);
	UAV_TEST_FLOAT_EQUAL(DefaultConfig.SafetyMargin, 50.0f, 0.1f);
	UAV_TEST_FLOAT_EQUAL(DefaultConfig.GridResolution, 50.0f, 0.1f);
	UAV_TEST_FLOAT_EQUAL(DefaultConfig.RRTStepSize, 100.0f, 0.1f);
	TestEqual(TEXT("Default RRT max iterations should be 5000"), DefaultConfig.RRTMaxIterations, 5000);
	UAV_TEST_FLOAT_EQUAL(DefaultConfig.RRTGoalBias, 0.1f, 0.01f);
	TestTrue(TEXT("Default should enable dynamic avoidance"), DefaultConfig.bEnableDynamicAvoidance);
	UAV_TEST_FLOAT_EQUAL(DefaultConfig.ReplanningThreshold, 200.0f, 0.1f);

	// 测试修改配置
	FPlanningConfig Config;
	Config.Algorithm = EPathPlanningAlgorithm::RRTStar;
	Config.MaxVelocity = 800.0f;
	Config.bEnableTrajectoryOptimization = false;

	TestEqual(TEXT("Algorithm should be RRTStar"),
		static_cast<int32>(Config.Algorithm),
		static_cast<int32>(EPathPlanningAlgorithm::RRTStar));
	UAV_TEST_FLOAT_EQUAL(Config.MaxVelocity, 800.0f, 0.1f);
	TestFalse(TEXT("Trajectory optimization should be disabled"), Config.bEnableTrajectoryOptimization);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
