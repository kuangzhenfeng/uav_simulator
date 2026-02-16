// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Planning/LocalAvoidance.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 引力计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLocalAvoidanceAttractiveForceTest,
	"UAVSimulator.Planning.LocalAvoidance.AttractiveForce",
	UAV_TEST_FLAGS)

bool FLocalAvoidanceAttractiveForceTest::RunTest(const FString& Parameters)
{
	ULocalAvoidance* APF = NewObject<ULocalAvoidance>();
	APF->AttractiveGain = 1.0f;

	// 目标在正前方
	FVector Force = APF->ComputeAttractiveForce(FVector::ZeroVector, FVector(500.0f, 0.0f, 0.0f));
	TestTrue(TEXT("Attractive force should point toward target"), Force.X > 0.0f);
	UAV_TEST_FLOAT_EQUAL(Force.Y, 0.0f, 1.0f);
	UAV_TEST_FLOAT_EQUAL(Force.Z, 0.0f, 1.0f);

	// 目标在正后方
	Force = APF->ComputeAttractiveForce(FVector::ZeroVector, FVector(-300.0f, 0.0f, 0.0f));
	TestTrue(TEXT("Attractive force should point backward"), Force.X < 0.0f);

	// 当前位置与目标重合
	Force = APF->ComputeAttractiveForce(FVector(100.0f, 0.0f, 0.0f), FVector(100.0f, 0.0f, 0.0f));
	TestTrue(TEXT("Force should be zero when at target"), Force.IsNearlyZero());

	// 引力大小应随距离增大（线性，但有上限 1000）
	FVector ForceNear = APF->ComputeAttractiveForce(FVector::ZeroVector, FVector(100.0f, 0.0f, 0.0f));
	FVector ForceFar = APF->ComputeAttractiveForce(FVector::ZeroVector, FVector(500.0f, 0.0f, 0.0f));
	TestTrue(TEXT("Farther target should produce larger force"), ForceFar.Size() > ForceNear.Size());

	return true;
}

// ==================== 斥力计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLocalAvoidanceRepulsiveForceTest,
	"UAVSimulator.Planning.LocalAvoidance.RepulsiveForce",
	UAV_TEST_FLAGS)

bool FLocalAvoidanceRepulsiveForceTest::RunTest(const FString& Parameters)
{
	ULocalAvoidance* APF = NewObject<ULocalAvoidance>();
	APF->RepulsiveGain = 5000.0f;
	APF->InfluenceDistance = 500.0f;
	APF->MaxRepulsiveForce = 2000.0f;

	// 球体障碍物在原点，半径 100
	FObstacleInfo Obstacle = UAVTestHelpers::CreateSphereObstacle(1, FVector::ZeroVector, 100.0f, 0.0f);

	// 在影响距离内，应产生斥力（远离障碍物方向）
	FVector Force = APF->ComputeRepulsiveForce(FVector(300.0f, 0.0f, 0.0f), Obstacle);
	TestTrue(TEXT("Repulsive force should push away from obstacle"), Force.X > 0.0f);

	// 在影响距离外，不应产生斥力
	Force = APF->ComputeRepulsiveForce(FVector(700.0f, 0.0f, 0.0f), Obstacle);
	TestTrue(TEXT("No repulsive force outside influence distance"), Force.IsNearlyZero());

	// 距离越近，斥力越大
	FVector ForceNear = APF->ComputeRepulsiveForce(FVector(150.0f, 0.0f, 0.0f), Obstacle);
	FVector ForceFar = APF->ComputeRepulsiveForce(FVector(400.0f, 0.0f, 0.0f), Obstacle);
	TestTrue(TEXT("Closer point should have larger repulsive force"), ForceNear.Size() > ForceFar.Size());

	// 斥力不应超过最大值
	FVector ForceVeryClose = APF->ComputeRepulsiveForce(FVector(101.0f, 0.0f, 0.0f), Obstacle);
	TestTrue(TEXT("Repulsive force should not exceed max"), ForceVeryClose.Size() <= APF->MaxRepulsiveForce + 1.0f);

	return true;
}

// ==================== 合力与避障修正测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLocalAvoidanceComputeAvoidanceTest,
	"UAVSimulator.Planning.LocalAvoidance.ComputeAvoidance",
	UAV_TEST_FLAGS)

bool FLocalAvoidanceComputeAvoidanceTest::RunTest(const FString& Parameters)
{
	ULocalAvoidance* APF = NewObject<ULocalAvoidance>();
	APF->AttractiveGain = 1.0f;
	APF->RepulsiveGain = 5000.0f;
	APF->InfluenceDistance = 500.0f;

	// 场景：目标在 (1000,0,0)，障碍物在 (300,0,0)，当前位置在原点
	FVector CurrentPos = FVector::ZeroVector;
	FVector TargetPos = FVector(1000.0f, 0.0f, 0.0f);
	FVector CurrentVel = FVector(100.0f, 0.0f, 0.0f);

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(300.0f, 0.0f, 0.0f), 100.0f, 0.0f));

	FLocalAvoidanceResult Result = APF->ComputeAvoidance(CurrentPos, TargetPos, Obstacles, CurrentVel);

	// 应该需要修正
	TestTrue(TEXT("Should need correction with obstacle in path"), Result.bNeedsCorrection);

	// 修正方向应该是归一化的
	UAV_TEST_FLOAT_EQUAL(Result.CorrectedDirection.Size(), 1.0f, 0.01f);

	// 引力应该指向目标
	TestTrue(TEXT("Attractive force should point toward target"), Result.AttractiveForce.X > 0.0f);

	// 斥力应该远离障碍物
	TestTrue(TEXT("Repulsive force should push away from obstacle"), Result.RepulsiveForce.X < 0.0f);

	return true;
}

// ==================== 无障碍物场景测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLocalAvoidanceNoObstacleTest,
	"UAVSimulator.Planning.LocalAvoidance.NoObstacle",
	UAV_TEST_FLAGS)

bool FLocalAvoidanceNoObstacleTest::RunTest(const FString& Parameters)
{
	ULocalAvoidance* APF = NewObject<ULocalAvoidance>();

	FVector CurrentPos = FVector::ZeroVector;
	FVector TargetPos = FVector(1000.0f, 0.0f, 0.0f);
	FVector CurrentVel = FVector(100.0f, 0.0f, 0.0f);
	TArray<FObstacleInfo> NoObstacles;

	FLocalAvoidanceResult Result = APF->ComputeAvoidance(CurrentPos, TargetPos, NoObstacles, CurrentVel);

	// 无障碍物时不需要修正
	TestFalse(TEXT("Should not need correction without obstacles"), Result.bNeedsCorrection);
	TestFalse(TEXT("Should not be stuck without obstacles"), Result.bStuck);

	// 斥力应为零
	TestTrue(TEXT("Repulsive force should be zero"), Result.RepulsiveForce.IsNearlyZero());

	return true;
}

// ==================== 到达目标测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLocalAvoidanceGoalReachedTest,
	"UAVSimulator.Planning.LocalAvoidance.GoalReached",
	UAV_TEST_FLAGS)

bool FLocalAvoidanceGoalReachedTest::RunTest(const FString& Parameters)
{
	ULocalAvoidance* APF = NewObject<ULocalAvoidance>();
	APF->GoalReachedThreshold = 100.0f;

	FVector CurrentPos = FVector(50.0f, 0.0f, 0.0f);
	FVector TargetPos = FVector::ZeroVector;
	FVector CurrentVel = FVector(10.0f, 0.0f, 0.0f);

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(200.0f, 0.0f, 0.0f), 50.0f, 0.0f));

	FLocalAvoidanceResult Result = APF->ComputeAvoidance(CurrentPos, TargetPos, Obstacles, CurrentVel);

	// 已到达目标，修正方向应为零
	TestTrue(TEXT("Corrected direction should be zero at goal"), Result.CorrectedDirection.IsNearlyZero());
	TestFalse(TEXT("Should not be stuck at goal"), Result.bStuck);

	return true;
}

// ==================== 多障碍物斥力叠加测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLocalAvoidanceMultiObstacleTest,
	"UAVSimulator.Planning.LocalAvoidance.MultipleObstacles",
	UAV_TEST_FLAGS)

bool FLocalAvoidanceMultiObstacleTest::RunTest(const FString& Parameters)
{
	ULocalAvoidance* APF = NewObject<ULocalAvoidance>();
	APF->InfluenceDistance = 500.0f;

	FVector CurrentPos = FVector::ZeroVector;
	FVector TargetPos = FVector(1000.0f, 0.0f, 0.0f);
	FVector CurrentVel = FVector(100.0f, 0.0f, 0.0f);

	// 两个障碍物分别在左右两侧
	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(200.0f, 200.0f, 0.0f), 100.0f, 0.0f));
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(2, FVector(200.0f, -200.0f, 0.0f), 100.0f, 0.0f));

	FLocalAvoidanceResult Result = APF->ComputeAvoidance(CurrentPos, TargetPos, Obstacles, CurrentVel);

	// 两侧对称障碍物，Y 方向斥力应大致抵消
	TestTrue(TEXT("Should need correction with obstacles"), Result.bNeedsCorrection);
	TestTrue(TEXT("Y component of repulsive force should be small due to symmetry"),
		FMath::Abs(Result.RepulsiveForce.Y) < Result.RepulsiveForce.Size() * 0.3f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
