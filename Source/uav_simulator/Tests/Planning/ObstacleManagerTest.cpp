// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Planning/ObstacleManager.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 球体碰撞检测测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerSphereCollisionTest,
	"UAVSimulator.Planning.ObstacleManager.SphereCollision",
	UAV_TEST_FLAGS)

bool FObstacleManagerSphereCollisionTest::RunTest(const FString& Parameters)
{
	// 创建障碍物管理器
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册球体障碍物：中心 (0,0,0)，半径 100，安全边距 50
	FObstacleInfo SphereObstacle = UAVTestHelpers::CreateSphereObstacle(1, FVector::ZeroVector, 100.0f, 50.0f);
	Manager->RegisterObstacle(SphereObstacle);

	// 测试点在障碍物内部（应该碰撞）
	TestTrue(TEXT("Point inside sphere should collide"), Manager->CheckCollision(FVector::ZeroVector, 0.0f));
	TestTrue(TEXT("Point at sphere surface should collide"), Manager->CheckCollision(FVector(100.0f, 0.0f, 0.0f), 0.0f));

	// 测试点在安全边距内（应该碰撞）
	TestTrue(TEXT("Point in safety margin should collide"), Manager->CheckCollision(FVector(120.0f, 0.0f, 0.0f), 0.0f));

	// 测试点在安全边距外（不应该碰撞）
	TestFalse(TEXT("Point outside safety margin should not collide"), Manager->CheckCollision(FVector(200.0f, 0.0f, 0.0f), 0.0f));

	// 测试带半径的碰撞检测
	TestTrue(TEXT("Point with radius should collide"), Manager->CheckCollision(FVector(180.0f, 0.0f, 0.0f), 50.0f));
	TestFalse(TEXT("Point with small radius should not collide"), Manager->CheckCollision(FVector(200.0f, 0.0f, 0.0f), 10.0f));

	return true;
}

// ==================== 盒体碰撞检测测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerBoxCollisionTest,
	"UAVSimulator.Planning.ObstacleManager.BoxCollision",
	UAV_TEST_FLAGS)

bool FObstacleManagerBoxCollisionTest::RunTest(const FString& Parameters)
{
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册盒体障碍物：中心 (0,0,0)，半尺寸 (100,100,100)，安全边距 50
	FObstacleInfo BoxObstacle = UAVTestHelpers::CreateBoxObstacle(1, FVector::ZeroVector, FVector(100.0f), FRotator::ZeroRotator, 50.0f);
	Manager->RegisterObstacle(BoxObstacle);

	// 测试点在盒体内部
	TestTrue(TEXT("Point inside box should collide"), Manager->CheckCollision(FVector::ZeroVector, 0.0f));
	TestTrue(TEXT("Point at box corner should collide"), Manager->CheckCollision(FVector(100.0f, 100.0f, 100.0f), 0.0f));

	// 测试点在安全边距内
	TestTrue(TEXT("Point in safety margin should collide"), Manager->CheckCollision(FVector(120.0f, 0.0f, 0.0f), 0.0f));

	// 测试点在安全边距外
	TestFalse(TEXT("Point outside safety margin should not collide"), Manager->CheckCollision(FVector(200.0f, 0.0f, 0.0f), 0.0f));

	return true;
}

// ==================== 圆柱体碰撞检测测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerCylinderCollisionTest,
	"UAVSimulator.Planning.ObstacleManager.CylinderCollision",
	UAV_TEST_FLAGS)

bool FObstacleManagerCylinderCollisionTest::RunTest(const FString& Parameters)
{
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册圆柱体障碍物：中心 (0,0,0)，半径 100，半高度 200，安全边距 50
	FObstacleInfo CylinderObstacle = UAVTestHelpers::CreateCylinderObstacle(1, FVector::ZeroVector, 100.0f, 200.0f, 50.0f);
	Manager->RegisterObstacle(CylinderObstacle);

	// 测试点在圆柱体内部
	TestTrue(TEXT("Point inside cylinder should collide"), Manager->CheckCollision(FVector::ZeroVector, 0.0f));
	TestTrue(TEXT("Point at cylinder surface should collide"), Manager->CheckCollision(FVector(100.0f, 0.0f, 0.0f), 0.0f));

	// 测试点在圆柱体顶部/底部
	TestTrue(TEXT("Point at cylinder top should collide"), Manager->CheckCollision(FVector(0.0f, 0.0f, 200.0f), 0.0f));

	// 测试点在安全边距外
	TestFalse(TEXT("Point outside cylinder should not collide"), Manager->CheckCollision(FVector(200.0f, 0.0f, 0.0f), 0.0f));

	return true;
}

// ==================== 距离计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerDistanceTest,
	"UAVSimulator.Planning.ObstacleManager.DistanceCalculation",
	UAV_TEST_FLAGS)

bool FObstacleManagerDistanceTest::RunTest(const FString& Parameters)
{
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册球体障碍物：中心 (0,0,0)，半径 100，安全边距 0
	FObstacleInfo SphereObstacle = UAVTestHelpers::CreateSphereObstacle(1, FVector::ZeroVector, 100.0f, 0.0f);
	Manager->RegisterObstacle(SphereObstacle);

	FObstacleInfo NearestObstacle;

	// 测试点在障碍物外部
	float Distance = Manager->GetDistanceToNearestObstacle(FVector(200.0f, 0.0f, 0.0f), NearestObstacle);
	UAV_TEST_FLOAT_EQUAL(Distance, 100.0f, 1.0f);
	TestEqual(TEXT("Nearest obstacle ID should match"), NearestObstacle.ObstacleID, 1);

	// 测试点在障碍物内部（负距离）
	Distance = Manager->GetDistanceToNearestObstacle(FVector(50.0f, 0.0f, 0.0f), NearestObstacle);
	TestTrue(TEXT("Distance inside obstacle should be negative"), Distance < 0.0f);

	// 测试点在障碍物表面
	Distance = Manager->GetDistanceToNearestObstacle(FVector(100.0f, 0.0f, 0.0f), NearestObstacle);
	UAV_TEST_FLOAT_EQUAL(Distance, 0.0f, 1.0f);

	return true;
}

// ==================== 线段碰撞检测测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerLineCollisionTest,
	"UAVSimulator.Planning.ObstacleManager.LineCollision",
	UAV_TEST_FLAGS)

bool FObstacleManagerLineCollisionTest::RunTest(const FString& Parameters)
{
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册球体障碍物：中心 (0,0,0)，半径 100，安全边距 50
	FObstacleInfo SphereObstacle = UAVTestHelpers::CreateSphereObstacle(1, FVector::ZeroVector, 100.0f, 50.0f);
	Manager->RegisterObstacle(SphereObstacle);

	// 测试穿过障碍物的线段
	TestTrue(TEXT("Line through obstacle should collide"),
		Manager->CheckLineCollision(FVector(-200.0f, 0.0f, 0.0f), FVector(200.0f, 0.0f, 0.0f), 0.0f));

	// 测试不穿过障碍物的线段
	TestFalse(TEXT("Line not through obstacle should not collide"),
		Manager->CheckLineCollision(FVector(-200.0f, 200.0f, 0.0f), FVector(200.0f, 200.0f, 0.0f), 0.0f));

	// 测试起点在障碍物内的线段
	TestTrue(TEXT("Line starting inside obstacle should collide"),
		Manager->CheckLineCollision(FVector::ZeroVector, FVector(200.0f, 0.0f, 0.0f), 0.0f));

	// 测试带半径的线段碰撞
	TestTrue(TEXT("Line with radius should collide"),
		Manager->CheckLineCollision(FVector(-200.0f, 180.0f, 0.0f), FVector(200.0f, 180.0f, 0.0f), 50.0f));

	return true;
}

// ==================== 范围查询测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerRangeQueryTest,
	"UAVSimulator.Planning.ObstacleManager.RangeQuery",
	UAV_TEST_FLAGS)

bool FObstacleManagerRangeQueryTest::RunTest(const FString& Parameters)
{
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册多个障碍物
	Manager->RegisterObstacle(UAVTestHelpers::CreateSphereObstacle(1, FVector(0.0f, 0.0f, 0.0f), 50.0f, 0.0f));
	Manager->RegisterObstacle(UAVTestHelpers::CreateSphereObstacle(2, FVector(200.0f, 0.0f, 0.0f), 50.0f, 0.0f));
	Manager->RegisterObstacle(UAVTestHelpers::CreateSphereObstacle(3, FVector(500.0f, 0.0f, 0.0f), 50.0f, 0.0f));
	Manager->RegisterObstacle(UAVTestHelpers::CreateSphereObstacle(4, FVector(0.0f, 500.0f, 0.0f), 50.0f, 0.0f));

	// 查询中心点 (0,0,0) 半径 300 内的障碍物
	TArray<FObstacleInfo> NearbyObstacles = Manager->GetObstaclesInRange(FVector::ZeroVector, 300.0f);
	TestEqual(TEXT("Should find 2 obstacles in range"), NearbyObstacles.Num(), 2);

	// 查询中心点 (0,0,0) 半径 600 内的障碍物
	NearbyObstacles = Manager->GetObstaclesInRange(FVector::ZeroVector, 600.0f);
	TestEqual(TEXT("Should find all 4 obstacles in larger range"), NearbyObstacles.Num(), 4);

	// 查询远离所有障碍物的点
	NearbyObstacles = Manager->GetObstaclesInRange(FVector(1000.0f, 1000.0f, 0.0f), 100.0f);
	TestEqual(TEXT("Should find no obstacles far away"), NearbyObstacles.Num(), 0);

	return true;
}

// ==================== 障碍物注册和移除测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerRegistrationTest,
	"UAVSimulator.Planning.ObstacleManager.Registration",
	UAV_TEST_FLAGS)

bool FObstacleManagerRegistrationTest::RunTest(const FString& Parameters)
{
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册障碍物
	int32 ID1 = Manager->RegisterObstacle(UAVTestHelpers::CreateSphereObstacle(0, FVector::ZeroVector, 100.0f));
	int32 ID2 = Manager->RegisterObstacle(UAVTestHelpers::CreateSphereObstacle(0, FVector(500.0f, 0.0f, 0.0f), 100.0f));

	TestTrue(TEXT("First obstacle ID should be valid"), ID1 >= 0);
	TestTrue(TEXT("Second obstacle ID should be valid"), ID2 >= 0);
	TestNotEqual(TEXT("Obstacle IDs should be different"), ID1, ID2);

	// 验证障碍物数量
	TestEqual(TEXT("Should have 2 obstacles"), Manager->GetAllObstacles().Num(), 2);

	// 移除障碍物
	TestTrue(TEXT("Should successfully remove obstacle"), Manager->RemoveObstacle(ID1));
	TestEqual(TEXT("Should have 1 obstacle after removal"), Manager->GetAllObstacles().Num(), 1);

	// 尝试移除不存在的障碍物
	TestFalse(TEXT("Should fail to remove non-existent obstacle"), Manager->RemoveObstacle(ID1));

	// 清除所有障碍物
	Manager->ClearAllObstacles();
	TestEqual(TEXT("Should have 0 obstacles after clear"), Manager->GetAllObstacles().Num(), 0);

	return true;
}

// ==================== 获取障碍物测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerGetObstacleTest,
	"UAVSimulator.Planning.ObstacleManager.GetObstacle",
	UAV_TEST_FLAGS)

bool FObstacleManagerGetObstacleTest::RunTest(const FString& Parameters)
{
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册障碍物
	FObstacleInfo OriginalObstacle = UAVTestHelpers::CreateSphereObstacle(0, FVector(100.0f, 200.0f, 300.0f), 150.0f, 25.0f);
	int32 ID = Manager->RegisterObstacle(OriginalObstacle);

	// 获取障碍物
	FObstacleInfo RetrievedObstacle;
	TestTrue(TEXT("Should successfully get obstacle"), Manager->GetObstacle(ID, RetrievedObstacle));
	TestEqual(TEXT("Obstacle ID should match"), RetrievedObstacle.ObstacleID, ID);
	UAV_TEST_VECTOR_EQUAL(RetrievedObstacle.Center, FVector(100.0f, 200.0f, 300.0f), 0.1f);
	UAV_TEST_FLOAT_EQUAL(RetrievedObstacle.Extents.X, 150.0f, 0.1f);

	// 尝试获取不存在的障碍物
	TestFalse(TEXT("Should fail to get non-existent obstacle"), Manager->GetObstacle(999, RetrievedObstacle));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
