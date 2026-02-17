// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Planning/ObstacleManager.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	UWorld* CreateObstacleManagerTestWorld(const TCHAR* WorldName)
	{
		if (!GEngine)
		{
			return nullptr;
		}

		UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, FName(WorldName));
		if (!World)
		{
			return nullptr;
		}

		FWorldContext& WorldContext = GEngine->CreateNewWorldContext(EWorldType::Game);
		WorldContext.SetCurrentWorld(World);

		World->InitializeNewWorld(UWorld::InitializationValues());
		World->BeginPlay();

		return World;
	}

	void DestroyObstacleManagerTestWorld(UWorld* World)
	{
		if (!World || !GEngine)
		{
			return;
		}

		World->DestroyWorld(false);
		GEngine->DestroyWorldContext(World);
	}

	AActor* SpawnBoxActor(UWorld* World, const FVector& Location, const FRotator& Rotation, const FVector& BoxExtent)
	{
		if (!World)
		{
			return nullptr;
		}

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

		AActor* Actor = World->SpawnActor<AActor>(AActor::StaticClass(), Location, Rotation, SpawnParams);
		if (!Actor)
		{
			return nullptr;
		}

		UBoxComponent* BoxComponent = NewObject<UBoxComponent>(Actor);
		BoxComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		BoxComponent->InitBoxExtent(BoxExtent);
		Actor->SetRootComponent(BoxComponent);
		BoxComponent->RegisterComponent();
		Actor->SetActorLocationAndRotation(Location, Rotation, false, nullptr, ETeleportType::TeleportPhysics);

		return Actor;
	}
}

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

// ==================== 感知障碍物重复注册刷新测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerPerceivedObstacleReRegisterUpdatesPoseAndExtentsTest,
	"UAVSimulator.Planning.ObstacleManager.PerceivedObstacleReRegisterUpdatesPoseAndExtents",
	UAV_TEST_FLAGS)

bool FObstacleManagerPerceivedObstacleReRegisterUpdatesPoseAndExtentsTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateObstacleManagerTestWorld(TEXT("ObstacleManager_PerceivedReRegister"));
	TestNotNull(TEXT("Test world should be created"), World);
	if (!World)
	{
		return false;
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AActor* OwnerActor = World->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
	TestNotNull(TEXT("Owner actor should be created"), OwnerActor);
	if (!OwnerActor)
	{
		DestroyObstacleManagerTestWorld(World);
		return false;
	}

	UObstacleManager* Manager = NewObject<UObstacleManager>(OwnerActor);
	Manager->RegisterComponent();

	const FVector InitialLocation(100.0f, -50.0f, 300.0f);
	const FRotator InitialRotation(0.0f, 10.0f, 0.0f);
	const FVector InitialExtent(80.0f, 60.0f, 25.0f);
	AActor* TargetActor = SpawnBoxActor(World, InitialLocation, InitialRotation, InitialExtent);
	TestNotNull(TEXT("Target actor should be created"), TargetActor);
	if (!TargetActor)
	{
		Manager->UnregisterComponent();
		DestroyObstacleManagerTestWorld(World);
		return false;
	}

	const int32 FirstID = Manager->RegisterPerceivedObstacleFromActor(TargetActor, EObstacleType::Box, 20.0f);
	TestTrue(TEXT("First perceived registration should succeed"), FirstID > 0);

	const FVector UpdatedLocation(450.0f, 120.0f, 520.0f);
	const FRotator UpdatedRotation(0.0f, 55.0f, 0.0f);
	const FVector UpdatedExtent(140.0f, 90.0f, 40.0f);
	if (UBoxComponent* BoxComponent = Cast<UBoxComponent>(TargetActor->GetRootComponent()))
	{
		BoxComponent->SetBoxExtent(UpdatedExtent);
	}
	TargetActor->SetActorLocationAndRotation(UpdatedLocation, UpdatedRotation, false, nullptr, ETeleportType::TeleportPhysics);

	const int32 SecondID = Manager->RegisterPerceivedObstacleFromActor(TargetActor, EObstacleType::Box, 75.0f);
	TestEqual(TEXT("Re-registering same actor should reuse obstacle ID"), SecondID, FirstID);

	FObstacleInfo UpdatedObstacle;
	TestTrue(TEXT("Updated obstacle should be retrievable"), Manager->GetObstacle(FirstID, UpdatedObstacle));
	UAV_TEST_VECTOR_EQUAL(UpdatedObstacle.Center, UpdatedLocation, 1.0f);
	UAV_TEST_VECTOR_EQUAL(UpdatedObstacle.Extents, UpdatedExtent, 1.0f);
	UAV_TEST_ROTATOR_EQUAL(UpdatedObstacle.Rotation, UpdatedRotation, 1.0f);
	UAV_TEST_FLOAT_EQUAL(UpdatedObstacle.SafetyMargin, 75.0f, 0.1f);
	TestTrue(TEXT("Perceived obstacle should remain dynamic"), UpdatedObstacle.bIsDynamic);

	Manager->UnregisterComponent();
	DestroyObstacleManagerTestWorld(World);
	return true;
}

// ==================== 感知障碍物默认动态更新测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerPerceivedObstacleDefaultDynamicUpdatesWithActorTest,
	"UAVSimulator.Planning.ObstacleManager.PerceivedObstacleDefaultDynamicUpdatesWithActor",
	UAV_TEST_FLAGS)

bool FObstacleManagerPerceivedObstacleDefaultDynamicUpdatesWithActorTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateObstacleManagerTestWorld(TEXT("ObstacleManager_PerceivedDynamicUpdate"));
	TestNotNull(TEXT("Test world should be created"), World);
	if (!World)
	{
		return false;
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AActor* OwnerActor = World->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
	TestNotNull(TEXT("Owner actor should be created"), OwnerActor);
	if (!OwnerActor)
	{
		DestroyObstacleManagerTestWorld(World);
		return false;
	}

	UObstacleManager* Manager = NewObject<UObstacleManager>(OwnerActor);
	Manager->RegisterComponent();

	AActor* TargetActor = SpawnBoxActor(
		World,
		FVector(0.0f, 0.0f, 300.0f),
		FRotator::ZeroRotator,
		FVector(90.0f, 70.0f, 30.0f));
	TestNotNull(TEXT("Target actor should be created"), TargetActor);
	if (!TargetActor)
	{
		Manager->UnregisterComponent();
		DestroyObstacleManagerTestWorld(World);
		return false;
	}

	const int32 ObstacleID = Manager->RegisterPerceivedObstacleFromActor(TargetActor, EObstacleType::Box, 30.0f);
	TestTrue(TEXT("Perceived obstacle registration should succeed"), ObstacleID > 0);

	const FVector MovedLocation(220.0f, -180.0f, 420.0f);
	TargetActor->SetActorLocation(MovedLocation);
	Manager->TickComponent(0.2f, LEVELTICK_All, nullptr);

	FObstacleInfo UpdatedObstacle;
	TestTrue(TEXT("Updated obstacle should be retrievable"), Manager->GetObstacle(ObstacleID, UpdatedObstacle));
	TestTrue(TEXT("Perceived obstacle should be dynamic by default"), UpdatedObstacle.bIsDynamic);
	UAV_TEST_VECTOR_EQUAL(UpdatedObstacle.Center, MovedLocation, 1.0f);
	TestTrue(TEXT("Velocity should be updated after movement"), UpdatedObstacle.Velocity.Size() > KINDA_SMALL_NUMBER);

	Manager->UnregisterComponent();
	DestroyObstacleManagerTestWorld(World);
	return true;
}

// ==================== 感知障碍物忽略 Owner 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObstacleManagerPerceivedObstacleIgnoresOwnerTest,
	"UAVSimulator.Planning.ObstacleManager.PerceivedObstacleIgnoresOwner",
	UAV_TEST_FLAGS)

bool FObstacleManagerPerceivedObstacleIgnoresOwnerTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateObstacleManagerTestWorld(TEXT("ObstacleManager_PerceivedIgnoreOwner"));
	TestNotNull(TEXT("Test world should be created"), World);
	if (!World)
	{
		return false;
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AActor* OwnerActor = World->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
	TestNotNull(TEXT("Owner actor should be created"), OwnerActor);
	if (!OwnerActor)
	{
		DestroyObstacleManagerTestWorld(World);
		return false;
	}

	UObstacleManager* Manager = NewObject<UObstacleManager>(OwnerActor);
	Manager->RegisterComponent();

	const int32 ObstacleID = Manager->RegisterPerceivedObstacleFromActor(OwnerActor, EObstacleType::Box, 10.0f);
	TestEqual(TEXT("Owner actor should not be registered as perceived obstacle"), ObstacleID, -1);
	TestEqual(TEXT("No obstacle should be registered"), Manager->GetAllObstacles().Num(), 0);

	Manager->UnregisterComponent();
	DestroyObstacleManagerTestWorld(World);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
