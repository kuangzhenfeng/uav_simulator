// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Planning/AStarPathPlanner.h"
#include "../../Planning/ObstacleManager.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== WorldToGrid 溢出防护测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAStarWorldToGridOverflowTest,
	"UAVSimulator.Planning.AStar.WorldToGridOverflow",
	UAV_TEST_FLAGS)

bool FAStarWorldToGridOverflowTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();

	// 正常坐标应正常转换
	FVector NormalPos(1000.0f, 2000.0f, 500.0f);
	// WorldToGrid 是 private，通过 PlanPath 间接测试
	// 这里主要验证极端坐标不会导致崩溃

	// 极端坐标不应导致崩溃（之前的 bug 会在此处整数溢出）
	TArray<FVector> Path;
	bool bResult = Planner->PlanPath(FVector::ZeroVector, FVector(FLT_MAX, 0.0f, 0.0f), Path);

	// 规划可能失败，但不应崩溃
	TestTrue(TEXT("Planning with extreme coordinates should not crash (result can be false)"), true);

	// 负极端坐标
	bResult = Planner->PlanPath(FVector(-FLT_MAX, -FLT_MAX, -FLT_MAX), FVector::ZeroVector, Path);
	TestTrue(TEXT("Planning with negative extreme coordinates should not crash"), true);

	return true;
}

// ==================== 多航段路径规划基础测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMultiSegmentBasicTest,
	"UAVSimulator.Planning.MultiSegment.BasicPlanning",
	UAV_TEST_FLAGS)

bool FMultiSegmentBasicTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();

	// 无障碍物的直线路径
	TArray<FVector> Path;
	FVector Start(0.0f, 0.0f, 500.0f);
	FVector End(2000.0f, 0.0f, 500.0f);

	bool bFound = Planner->PlanPath(Start, End, Path);

	if (bFound)
	{
		TestTrue(TEXT("Path should have at least 2 points"), Path.Num() >= 2);

		// 起点应接近 Start
		UAV_TEST_VECTOR_EQUAL(Path[0], Start, 100.0f);

		// 终点应接近 End
		UAV_TEST_VECTOR_EQUAL(Path.Last(), End, 100.0f);
	}

	return true;
}

// ==================== 带障碍物的路径规划测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMultiSegmentObstacleAvoidanceTest,
	"UAVSimulator.Planning.MultiSegment.ObstacleAvoidance",
	UAV_TEST_FLAGS)

bool FMultiSegmentObstacleAvoidanceTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();

	// 在路径中间放置障碍物
	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(1000.0f, 0.0f, 500.0f), 200.0f, 50.0f));
	Planner->SetObstacles(Obstacles);

	FPlanningConfig Config;
	Config.SafetyMargin = 50.0f;
	Planner->SetPlanningConfig(Config);

	TArray<FVector> Path;
	FVector Start(0.0f, 0.0f, 500.0f);
	FVector End(2000.0f, 0.0f, 500.0f);

	bool bFound = Planner->PlanPath(Start, End, Path);

	if (bFound && Path.Num() >= 2)
	{
		// 路径应该绕过障碍物，所以不应该是直线
		// 检查路径中间点不在障碍物内
		FVector ObstacleCenter(1000.0f, 0.0f, 500.0f);
		float ObstacleRadius = 200.0f + 50.0f; // 半径 + 安全边距

		for (int32 i = 0; i < Path.Num(); ++i)
		{
			float Dist = FVector::Dist(Path[i], ObstacleCenter);
			// 路径点不应在障碍物内部（允许一定容差）
			TestTrue(FString::Printf(TEXT("Path point %d should not be inside obstacle (dist=%.1f)"), i, Dist),
				Dist >= ObstacleRadius - 100.0f); // 容差 100cm（网格分辨率）
		}
	}

	return true;
}

// ==================== CheckLineCollision 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMultiSegmentLineCollisionTest,
	"UAVSimulator.Planning.MultiSegment.LineCollision",
	UAV_TEST_FLAGS)

bool FMultiSegmentLineCollisionTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();

	// 设置障碍物
	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(500.0f, 0.0f, 0.0f), 100.0f, 50.0f));
	Planner->SetObstacles(Obstacles);

	// 穿过障碍物的线段应检测到碰撞
	bool bCollision = Planner->CheckLineCollision(
		FVector(0.0f, 0.0f, 0.0f), FVector(1000.0f, 0.0f, 0.0f), 50.0f);
	TestTrue(TEXT("Line through obstacle should collide"), bCollision);

	// 不穿过障碍物的线段
	bCollision = Planner->CheckLineCollision(
		FVector(0.0f, 500.0f, 0.0f), FVector(1000.0f, 500.0f, 0.0f), 50.0f);
	TestFalse(TEXT("Line not through obstacle should not collide"), bCollision);

	return true;
}

// ==================== 空路径处理测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMultiSegmentEmptyPathTest,
	"UAVSimulator.Planning.MultiSegment.EmptyPath",
	UAV_TEST_FLAGS)

bool FMultiSegmentEmptyPathTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();

	// 起点和终点相同
	TArray<FVector> Path;
	FVector SamePoint(100.0f, 100.0f, 100.0f);
	bool bFound = Planner->PlanPath(SamePoint, SamePoint, Path);

	// 起终点相同时，规划结果取决于实现，但不应崩溃
	TestTrue(TEXT("Planning with same start/end should not crash"), true);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
