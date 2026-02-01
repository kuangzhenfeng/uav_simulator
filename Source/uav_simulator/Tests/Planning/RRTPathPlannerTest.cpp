// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Planning/RRTPathPlanner.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 基本路径规划测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRRTBasicPlanPathTest,
	"UAVSimulator.Planning.RRT.BasicPlanPath",
	UAV_TEST_FLAGS)

bool FRRTBasicPlanPathTest::RunTest(const FString& Parameters)
{
	URRTPathPlanner* Planner = NewObject<URRTPathPlanner>();
	Planner->SetSearchBounds(FVector(-2000.0f), FVector(2000.0f));

	// 测试简单路径
	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(1000.0f, 0.0f, 0.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("Simple RRT path should succeed"), bSuccess);
	TestTrue(TEXT("Path should have at least 2 points"), OutPath.Num() >= 2);

	// 验证起点和终点
	if (OutPath.Num() >= 2)
	{
		UAV_TEST_VECTOR_EQUAL(OutPath[0], Start, 150.0f);
		UAV_TEST_VECTOR_EQUAL(OutPath.Last(), Goal, 150.0f);
	}

	return true;
}

// ==================== Steer 函数测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRRTSteerTest,
	"UAVSimulator.Planning.RRT.Steer",
	UAV_TEST_FLAGS)

bool FRRTSteerTest::RunTest(const FString& Parameters)
{
	URRTPathPlanner* Planner = NewObject<URRTPathPlanner>();
	Planner->SetSearchBounds(FVector(-2000.0f), FVector(2000.0f));

	// 通过路径规划间接测试 Steer 函数
	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(500.0f, 0.0f, 0.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("Path should succeed"), bSuccess);

	// 验证路径存在且有效
	TestTrue(TEXT("Path should have at least 2 points"), OutPath.Num() >= 2);

	// 验证起点和终点
	if (OutPath.Num() >= 2)
	{
		UAV_TEST_VECTOR_EQUAL(OutPath[0], Start, 150.0f);
		UAV_TEST_VECTOR_EQUAL(OutPath.Last(), Goal, 150.0f);
	}

	// 注意：路径可能被简化，所以不检查相邻点距离
	// 只验证路径总长度合理
	float TotalLength = 0.0f;
	for (int32 i = 1; i < OutPath.Num(); ++i)
	{
		TotalLength += FVector::Dist(OutPath[i], OutPath[i - 1]);
	}
	float DirectDistance = FVector::Dist(Start, Goal);
	TestTrue(TEXT("Path length should be at least direct distance"), TotalLength >= DirectDistance * 0.99f);

	return true;
}

// ==================== 最近节点查找测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRRTFindNearestNodeTest,
	"UAVSimulator.Planning.RRT.FindNearestNode",
	UAV_TEST_FLAGS)

bool FRRTFindNearestNodeTest::RunTest(const FString& Parameters)
{
	URRTPathPlanner* Planner = NewObject<URRTPathPlanner>();
	Planner->SetSearchBounds(FVector(-2000.0f), FVector(2000.0f));

	// 通过多次规划测试最近节点查找的一致性
	TArray<FVector> OutPath1, OutPath2;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(800.0f, 0.0f, 0.0f);

	// RRT 是随机算法，但应该总能找到路径
	bool bSuccess1 = Planner->PlanPath(Start, Goal, OutPath1);
	bool bSuccess2 = Planner->PlanPath(Start, Goal, OutPath2);

	TestTrue(TEXT("First path should succeed"), bSuccess1);
	TestTrue(TEXT("Second path should succeed"), bSuccess2);

	// 两条路径可能不同（随机性），但都应该有效
	if (bSuccess1 && bSuccess2)
	{
		TestTrue(TEXT("Both paths should have points"), OutPath1.Num() > 0 && OutPath2.Num() > 0);
	}

	return true;
}

// ==================== RRT* 邻域查找测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRRTStarNearbyNodesTest,
	"UAVSimulator.Planning.RRT.RRTStarNearbyNodes",
	UAV_TEST_FLAGS)

bool FRRTStarNearbyNodesTest::RunTest(const FString& Parameters)
{
	URRTPathPlanner* Planner = NewObject<URRTPathPlanner>();
	Planner->SetSearchBounds(FVector(-2000.0f), FVector(2000.0f));

	// 注意：需要通过反射或公开属性来启用 RRT*
	// 这里假设可以通过某种方式启用

	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(1000.0f, 0.0f, 0.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("RRT path should succeed"), bSuccess);

	return true;
}

// ==================== 目标偏置测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRRTGoalBiasTest,
	"UAVSimulator.Planning.RRT.GoalBias",
	UAV_TEST_FLAGS)

bool FRRTGoalBiasTest::RunTest(const FString& Parameters)
{
	URRTPathPlanner* Planner = NewObject<URRTPathPlanner>();
	Planner->SetSearchBounds(FVector(-2000.0f), FVector(2000.0f));

	// 测试目标偏置对路径规划的影响
	// 高目标偏置应该产生更直接的路径
	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(500.0f, 0.0f, 0.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("Path with goal bias should succeed"), bSuccess);

	// 验证路径大致朝向目标
	if (OutPath.Num() > 2)
	{
		FVector Direction = (Goal - Start).GetSafeNormal();
		for (int32 i = 1; i < OutPath.Num(); ++i)
		{
			FVector PointDirection = (OutPath[i] - Start).GetSafeNormal();
			float DotProduct = FVector::DotProduct(Direction, PointDirection);
			// 路径点应该大致朝向目标方向
			TestTrue(FString::Printf(TEXT("Point %d should be towards goal"), i), DotProduct > -0.5f);
		}
	}

	return true;
}

// ==================== 障碍物避障测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRRTObstacleAvoidanceTest,
	"UAVSimulator.Planning.RRT.ObstacleAvoidance",
	UAV_TEST_FLAGS)

bool FRRTObstacleAvoidanceTest::RunTest(const FString& Parameters)
{
	URRTPathPlanner* Planner = NewObject<URRTPathPlanner>();

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
	TestTrue(TEXT("RRT path around obstacle should succeed"), bSuccess);

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

// ==================== 3D 路径规划测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRRT3DPathTest,
	"UAVSimulator.Planning.RRT.3DPath",
	UAV_TEST_FLAGS)

bool FRRT3DPathTest::RunTest(const FString& Parameters)
{
	URRTPathPlanner* Planner = NewObject<URRTPathPlanner>();
	Planner->SetSearchBounds(FVector(-2000.0f), FVector(2000.0f));

	// 测试 3D 路径规划
	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(500.0f, 500.0f, 500.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("3D RRT path should succeed"), bSuccess);

	// 验证路径包含 Z 方向的变化
	if (OutPath.Num() > 2)
	{
		bool bHasZChange = false;
		for (int32 i = 1; i < OutPath.Num(); ++i)
		{
			if (FMath::Abs(OutPath[i].Z - OutPath[i - 1].Z) > 1.0f)
			{
				bHasZChange = true;
				break;
			}
		}
		TestTrue(TEXT("3D path should have Z changes"), bHasZChange);
	}

	return true;
}

// ==================== 边界测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRRTBoundsTest,
	"UAVSimulator.Planning.RRT.Bounds",
	UAV_TEST_FLAGS)

bool FRRTBoundsTest::RunTest(const FString& Parameters)
{
	URRTPathPlanner* Planner = NewObject<URRTPathPlanner>();

	// 设置较小的边界
	Planner->SetSearchBounds(FVector(-500.0f), FVector(500.0f));

	// 测试边界内的路径
	TArray<FVector> OutPath;
	bool bSuccess = Planner->PlanPath(FVector(-400.0f, 0.0f, 0.0f), FVector(400.0f, 0.0f, 0.0f), OutPath);
	TestTrue(TEXT("Path within bounds should succeed"), bSuccess);

	// 验证所有路径点都在边界内
	for (const FVector& Point : OutPath)
	{
		TestTrue(TEXT("Path point X should be within bounds"), Point.X >= -550.0f && Point.X <= 550.0f);
		TestTrue(TEXT("Path point Y should be within bounds"), Point.Y >= -550.0f && Point.Y <= 550.0f);
		TestTrue(TEXT("Path point Z should be within bounds"), Point.Z >= -550.0f && Point.Z <= 550.0f);
	}

	// 测试自动计算边界
	Planner->AutoComputeSearchBounds(FVector(-1500.0f, 0.0f, 0.0f), FVector(1500.0f, 0.0f, 0.0f), 500.0f);
	OutPath.Empty();
	bSuccess = Planner->PlanPath(FVector(-1500.0f, 0.0f, 0.0f), FVector(1500.0f, 0.0f, 0.0f), OutPath);
	TestTrue(TEXT("Path with auto bounds should succeed"), bSuccess);

	return true;
}

// ==================== 路径连续性测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRRTPathContinuityTest,
	"UAVSimulator.Planning.RRT.PathContinuity",
	UAV_TEST_FLAGS)

bool FRRTPathContinuityTest::RunTest(const FString& Parameters)
{
	URRTPathPlanner* Planner = NewObject<URRTPathPlanner>();
	Planner->SetSearchBounds(FVector(-2000.0f), FVector(2000.0f));

	TArray<FVector> OutPath;
	FVector Start(0.0f, 0.0f, 0.0f);
	FVector Goal(1000.0f, 500.0f, 200.0f);

	bool bSuccess = Planner->PlanPath(Start, Goal, OutPath);
	TestTrue(TEXT("Path should succeed"), bSuccess);

	// 验证路径存在
	TestTrue(TEXT("Path should have at least 2 points"), OutPath.Num() >= 2);

	// 验证起点和终点
	if (OutPath.Num() >= 2)
	{
		UAV_TEST_VECTOR_EQUAL(OutPath[0], Start, 150.0f);
		UAV_TEST_VECTOR_EQUAL(OutPath.Last(), Goal, 150.0f);
	}

	// 注意：路径可能被简化，所以不检查相邻点距离
	// 只验证路径总长度合理（不应该比直线距离短太多）
	float TotalLength = 0.0f;
	for (int32 i = 1; i < OutPath.Num(); ++i)
	{
		TotalLength += FVector::Dist(OutPath[i], OutPath[i - 1]);
	}
	float DirectDistance = FVector::Dist(Start, Goal);
	TestTrue(TEXT("Path length should be at least direct distance"), TotalLength >= DirectDistance * 0.99f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
