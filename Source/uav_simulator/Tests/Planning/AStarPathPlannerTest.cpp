// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Planning/AStarPathPlanner.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 坐标转换测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAStarWorldToGridTest,
	"UAVSimulator.Planning.AStar.WorldToGrid",
	UAV_TEST_FLAGS)

bool FAStarWorldToGridTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();

	// 设置搜索边界
	Planner->SetSearchBounds(FVector(-1000.0f, -1000.0f, -1000.0f), FVector(1000.0f, 1000.0f, 1000.0f));

	// 测试路径规划（间接测试坐标转换）
	// 由于 WorldToGrid 和 GridToWorld 是私有方法，我们通过路径规划来验证其正确性
	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(500.0f, 500.0f, 0.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);

	// 验证路径规划成功
	TestTrue(TEXT("Path planning should succeed"), bSuccess);

	// 验证路径起点和终点
	if (OutPath.Num() > 0)
	{
		UAV_TEST_VECTOR_EQUAL(OutPath[0], Start, 100.0f);
		UAV_TEST_VECTOR_EQUAL(OutPath.Last(), Goal, 100.0f);
	}

	return true;
}

// ==================== 启发式计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAStarHeuristicTest,
	"UAVSimulator.Planning.AStar.Heuristic",
	UAV_TEST_FLAGS)

bool FAStarHeuristicTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();
	Planner->SetSearchBounds(FVector(-1000.0f), FVector(1000.0f));

	// 测试直线路径（启发式应该引导直线搜索）
	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(500.0f, 0.0f, 0.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("Straight line path should succeed"), bSuccess);

	// 验证路径大致沿直线
	if (OutPath.Num() > 2)
	{
		for (int32 i = 1; i < OutPath.Num() - 1; ++i)
		{
			// Y 和 Z 坐标应该接近 0
			UAV_TEST_FLOAT_IN_RANGE(OutPath[i].Y, -100.0f, 100.0f);
			UAV_TEST_FLOAT_IN_RANGE(OutPath[i].Z, -100.0f, 100.0f);
		}
	}

	return true;
}

// ==================== 邻居生成测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAStarNeighborsTest,
	"UAVSimulator.Planning.AStar.Neighbors",
	UAV_TEST_FLAGS)

bool FAStarNeighborsTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();
	Planner->SetSearchBounds(FVector(-1000.0f), FVector(1000.0f));

	// 测试对角线移动
	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(500.0f, 500.0f, 500.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("Diagonal path should succeed"), bSuccess);

	// 验证路径存在
	TestTrue(TEXT("Path should have points"), OutPath.Num() > 0);

	// 验证路径起点和终点正确
	if (OutPath.Num() >= 2)
	{
		UAV_TEST_VECTOR_EQUAL(OutPath[0], Start, 100.0f);
		UAV_TEST_VECTOR_EQUAL(OutPath.Last(), Goal, 100.0f);
	}

	// 注意：路径可能被简化，所以不检查相邻点距离
	// 只验证路径总长度合理
	float TotalLength = 0.0f;
	for (int32 i = 1; i < OutPath.Num(); ++i)
	{
		TotalLength += FVector::Dist(OutPath[i], OutPath[i - 1]);
	}
	float DirectDistance = FVector::Dist(Start, Goal);
	// 路径长度应该至少等于直线距离
	TestTrue(TEXT("Path length should be at least direct distance"), TotalLength >= DirectDistance * 0.99f);

	return true;
}

// ==================== 边界验证测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAStarBoundsTest,
	"UAVSimulator.Planning.AStar.Bounds",
	UAV_TEST_FLAGS)

bool FAStarBoundsTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();
	Planner->SetSearchBounds(FVector(-500.0f), FVector(500.0f));

	// 测试边界内的路径
	TArray<FVector> OutPath;
	bool bSuccess = Planner->PlanPath(FVector(-400.0f, 0.0f, 0.0f), FVector(400.0f, 0.0f, 0.0f), OutPath);
	TestTrue(TEXT("Path within bounds should succeed"), bSuccess);

	// 测试起点在边界外
	OutPath.Empty();
	bSuccess = Planner->PlanPath(FVector(-1000.0f, 0.0f, 0.0f), FVector(0.0f, 0.0f, 0.0f), OutPath);
	// 起点在边界外，规划可能失败或被裁剪
	// 具体行为取决于实现

	// 测试自动计算边界
	Planner->AutoComputeSearchBounds(FVector(-2000.0f, 0.0f, 0.0f), FVector(2000.0f, 0.0f, 0.0f), 500.0f);
	OutPath.Empty();
	bSuccess = Planner->PlanPath(FVector(-2000.0f, 0.0f, 0.0f), FVector(2000.0f, 0.0f, 0.0f), OutPath);
	TestTrue(TEXT("Path with auto bounds should succeed"), bSuccess);

	return true;
}

// ==================== 完整路径规划测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAStarPlanPathTest,
	"UAVSimulator.Planning.AStar.PlanPath",
	UAV_TEST_FLAGS)

bool FAStarPlanPathTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();
	Planner->SetSearchBounds(FVector(-2000.0f), FVector(2000.0f));

	// 测试简单路径
	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(1000.0f, 0.0f, 0.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("Simple path should succeed"), bSuccess);
	TestTrue(TEXT("Path should have at least 2 points"), OutPath.Num() >= 2);

	// 验证起点和终点
	if (OutPath.Num() >= 2)
	{
		UAV_TEST_VECTOR_EQUAL(OutPath[0], Start, 100.0f);
		UAV_TEST_VECTOR_EQUAL(OutPath.Last(), Goal, 100.0f);
	}

	// 测试起点等于终点
	OutPath.Empty();
	bSuccess = Planner->PlanPath(Start, Start, OutPath);
	TestTrue(TEXT("Same start and goal should succeed"), bSuccess);

	return true;
}

// ==================== 障碍物避障测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAStarObstacleAvoidanceTest,
	"UAVSimulator.Planning.AStar.ObstacleAvoidance",
	UAV_TEST_FLAGS)

bool FAStarObstacleAvoidanceTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();

	Planner->SetSearchBounds(FVector(-2000.0f), FVector(2000.0f));

	// 在路径中间添加障碍物
	FObstacleInfo Obstacle = UAVTestHelpers::CreateSphereObstacle(1, FVector(500.0f, 0.0f, 0.0f), 200.0f, 50.0f);
	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(Obstacle);
	Planner->SetObstacles(Obstacles);

	// 规划路径
	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(1000.0f, 0.0f, 0.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("Path around obstacle should succeed"), bSuccess);

	// 验证路径不穿过障碍物
	if (bSuccess && OutPath.Num() > 0)
	{
		for (const FVector& Point : OutPath)
		{
			bool bCollides = Planner->CheckCollision(Point, 0.0f);
			TestFalse(TEXT("Path point should not collide with obstacle"), bCollides);
		}
	}

	return true;
}

// ==================== 路径平滑性测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAStarPathSmoothnessTest,
	"UAVSimulator.Planning.AStar.PathSmoothness",
	UAV_TEST_FLAGS)

bool FAStarPathSmoothnessTest::RunTest(const FString& Parameters)
{
	UAStarPathPlanner* Planner = NewObject<UAStarPathPlanner>();
	Planner->SetSearchBounds(FVector(-2000.0f), FVector(2000.0f));

	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(1000.0f, 500.0f, 200.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("3D path should succeed"), bSuccess);

	// 计算路径总长度
	float TotalLength = 0.0f;
	for (int32 i = 1; i < OutPath.Num(); ++i)
	{
		TotalLength += FVector::Dist(OutPath[i], OutPath[i - 1]);
	}

	// 计算直线距离
	float DirectDistance = FVector::Dist(Start, Goal);

	// 路径长度不应该比直线距离长太多（考虑网格化的影响）
	float LengthRatio = TotalLength / DirectDistance;
	TestTrue(TEXT("Path length should be reasonable"), LengthRatio < 2.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
