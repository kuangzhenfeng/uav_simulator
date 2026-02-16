// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Sensors/ObstacleDetector.h"
#include "../../Planning/ObstacleManager.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== ObstacleDetector 初始化测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleDetectorInitTest,
	"UAVSimulator.Sensors.ObstacleDetector.Initialization",
	UAV_TEST_FLAGS)

bool FObstacleDetectorInitTest::RunTest(const FString& Parameters)
{
	UObstacleDetector* Detector = NewObject<UObstacleDetector>();

	// 验证默认参数
	TestTrue(TEXT("Detector should be enabled by default"), Detector->IsEnabled());

	// 检测到的障碍物列表应为空
	TestEqual(TEXT("Should have no detected obstacles initially"), Detector->GetDetectedObstacles().Num(), 0);

	return true;
}

// ==================== ObstacleManager 感知障碍物注册测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerPerceivedRegistrationTest,
	"UAVSimulator.Planning.ObstacleManager.PerceivedRegistration",
	UAV_TEST_FLAGS)

bool FObstacleManagerPerceivedRegistrationTest::RunTest(const FString& Parameters)
{
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册感知障碍物
	FObstacleInfo PerceivedObs = UAVTestHelpers::CreateSphereObstacle(0, FVector(500.0f, 0.0f, 0.0f), 100.0f, 50.0f);
	int32 ID = Manager->RegisterPerceivedObstacle(PerceivedObs);

	TestTrue(TEXT("Perceived obstacle ID should be valid"), ID > 0);

	// 验证标记为感知障碍物
	FObstacleInfo Retrieved;
	TestTrue(TEXT("Should find perceived obstacle"), Manager->GetObstacle(ID, Retrieved));
	TestTrue(TEXT("Should be marked as perceived"), Retrieved.bIsPerceived);

	// 注册普通障碍物
	FObstacleInfo NormalObs = UAVTestHelpers::CreateSphereObstacle(0, FVector(-500.0f, 0.0f, 0.0f), 100.0f, 50.0f);
	int32 NormalID = Manager->RegisterObstacle(NormalObs);

	// 验证分类查询
	TArray<FObstacleInfo> Perceived = Manager->GetPerceivedObstacles();
	TArray<FObstacleInfo> Preregistered = Manager->GetPreregisteredObstacles();

	TestEqual(TEXT("Should have 1 perceived obstacle"), Perceived.Num(), 1);
	TestEqual(TEXT("Should have 1 preregistered obstacle"), Preregistered.Num(), 1);

	return true;
}

// ==================== 感知障碍物刷新测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerPerceivedRefreshTest,
	"UAVSimulator.Planning.ObstacleManager.PerceivedRefresh",
	UAV_TEST_FLAGS)

bool FObstacleManagerPerceivedRefreshTest::RunTest(const FString& Parameters)
{
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册感知障碍物
	FObstacleInfo PerceivedObs = UAVTestHelpers::CreateSphereObstacle(0, FVector(500.0f, 0.0f, 0.0f), 100.0f, 50.0f);
	int32 ID = Manager->RegisterPerceivedObstacle(PerceivedObs);

	// 刷新不应崩溃
	Manager->RefreshPerceivedObstacle(ID);

	// 刷新不存在的 ID 也不应崩溃
	Manager->RefreshPerceivedObstacle(999);

	// 障碍物应仍然存在
	FObstacleInfo Retrieved;
	TestTrue(TEXT("Obstacle should still exist after refresh"), Manager->GetObstacle(ID, Retrieved));

	return true;
}

// ==================== 过期感知障碍物清理测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerStaleRemovalTest,
	"UAVSimulator.Planning.ObstacleManager.StaleRemoval",
	UAV_TEST_FLAGS)

bool FObstacleManagerStaleRemovalTest::RunTest(const FString& Parameters)
{
	UObstacleManager* Manager = NewObject<UObstacleManager>();

	// 注册感知障碍物（LastPerceivedTime 为 0）
	FObstacleInfo PerceivedObs = UAVTestHelpers::CreateSphereObstacle(0, FVector(500.0f, 0.0f, 0.0f), 100.0f, 50.0f);
	PerceivedObs.bIsPerceived = true;
	PerceivedObs.LastPerceivedTime = 0.0f;
	Manager->RegisterObstacle(PerceivedObs);

	// 注册普通障碍物
	FObstacleInfo NormalObs = UAVTestHelpers::CreateSphereObstacle(0, FVector(-500.0f, 0.0f, 0.0f), 100.0f, 50.0f);
	Manager->RegisterObstacle(NormalObs);

	TestEqual(TEXT("Should have 2 obstacles before cleanup"), Manager->GetAllObstacles().Num(), 2);

	// 清理过期障碍物（MaxAge=0 表示立即过期）
	// 注意：没有 World 时 GetTimeSeconds 返回 0，所以 age = 0 - 0 = 0，不会超过 MaxAge=5
	// 使用 MaxAge=-1 强制清理
	int32 Removed = Manager->RemoveStalePerceivedObstacles(-1.0f);

	// 感知障碍物应被移除，普通障碍物保留
	TestEqual(TEXT("Should have removed 1 perceived obstacle"), Removed, 1);
	TestEqual(TEXT("Should have 1 obstacle after cleanup"), Manager->GetAllObstacles().Num(), 1);

	// 剩余的应该是普通障碍物
	TArray<FObstacleInfo> Remaining = Manager->GetPreregisteredObstacles();
	TestEqual(TEXT("Remaining should be preregistered"), Remaining.Num(), 1);

	return true;
}

// ==================== FDetectedObstacle 结构体测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDetectedObstacleStructTest,
	"UAVSimulator.Sensors.ObstacleDetector.DetectedObstacleStruct",
	UAV_TEST_FLAGS)

bool FDetectedObstacleStructTest::RunTest(const FString& Parameters)
{
	FDetectedObstacle Obstacle;

	// 验证默认值
	TestTrue(TEXT("Default center should be zero"), Obstacle.Center.IsNearlyZero());
	TestEqual(TEXT("Default type should be Sphere"), Obstacle.EstimatedType, EObstacleType::Sphere);
	UAV_TEST_FLOAT_EQUAL(Obstacle.Distance, 0.0f, 0.01f);
	TestEqual(TEXT("Default registered ID should be -1"), Obstacle.RegisteredObstacleID, -1);
	TestFalse(TEXT("Default actor should be invalid"), Obstacle.DetectedActor.IsValid());

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
