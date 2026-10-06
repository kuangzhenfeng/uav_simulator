// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Scenario/ScenarioLoader.h"
#include "../../Scenario/ScenarioTypes.h"
#include "../../Planning/ObstacleManager.h"
#include "../../Core/UAVTypes.h"
#include "../../Planning/DynamicObstacleActor.h"
#include "Components/StaticMeshComponent.h"
#include "../../Planning/ObstacleGeometry.h"

#include "Logging/LogMacros.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	// 合成 World helper 已集中到 UAVTestCommon.h，避免跨 TU 重定义。

	// 构造一个内存 UScenario，挂上指定障碍条目。
	UScenario* MakeScenarioWithObstacles(UObject* Outer, std::initializer_list<FScenarioObstacleEntry> Entries)
	{
		UScenario* Scenario = NewObject<UScenario>(Outer);
		UObstacleLayout* Layout = NewObject<UObstacleLayout>(Scenario);
		for (const FScenarioObstacleEntry& E : Entries)
		{
			Layout->Obstacles.Add(E);
		}
		Scenario->ObstacleLayout = Layout;
		Scenario->Name = TEXT("TestScenario");
		return Scenario;
	}
}

// ==================== 装配障碍：注册数量与几何一致 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioLoadObstaclesRegistersAllTest,
	"UAVSimulator.Scenario.LoadObstacles.RegistersAll",
	UAV_TEST_FLAGS)

bool FScenarioLoadObstaclesRegistersAllTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("ScenarioLoadObsAll"));
	if (!TestNotNull(TEXT("合成 World 创建成功"), World))
	{
		return false;
	}

	UObstacleManager* Manager = NewObject<UObstacleManager>();
	UScenarioLoader* Loader = NewObject<UScenarioLoader>();

	FScenarioObstacleEntry A;
	A.Type = EObstacleType::Sphere;
	A.Center = FVector(1000.0f, 0.0f, 0.0f);
	A.Extents = FVector(150.0f);
	A.SafetyMargin = 50.0f;

	FScenarioObstacleEntry B;
	B.Type = EObstacleType::Box;
	B.Center = FVector(2000.0f, 500.0f, 0.0f);
	B.Extents = FVector(200.0f, 200.0f, 300.0f);
	B.SafetyMargin = 100.0f;

	UScenario* Scenario = MakeScenarioWithObstacles(GetTransientPackage(), { A, B });

	const int32 Registered = Loader->AssembleObstacles(Scenario, Manager, World);

	TestEqual(TEXT("注册障碍数 = 声明数"), Registered, 2);
	TestEqual(TEXT("ObstacleManager 障碍数 = 2"), Manager->GetAllObstacles().Num(), 2);

	DestroyScenarioTestWorld(World);
	return true;
}

// ==================== 装配障碍：几何被忠实复制 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioLoadObstaclesGeometryTest,
	"UAVSimulator.Scenario.LoadObstacles.Geometry",
	UAV_TEST_FLAGS)

bool FScenarioLoadObstaclesGeometryTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("ScenarioLoadObsGeo"));
	if (!TestNotNull(TEXT("合成 World 创建成功"), World))
	{
		return false;
	}

	UObstacleManager* Manager = NewObject<UObstacleManager>();
	UScenarioLoader* Loader = NewObject<UScenarioLoader>();

	FScenarioObstacleEntry Entry;
	Entry.Type = EObstacleType::Sphere;
	Entry.Center = FVector(3000.0f, -500.0f, 200.0f);
	Entry.Extents = FVector(250.0f);
	Entry.SafetyMargin = 75.0f;

	UScenario* Scenario = MakeScenarioWithObstacles(GetTransientPackage(), { Entry });
	Loader->AssembleObstacles(Scenario, Manager, World);

	const TArray<FObstacleInfo>& All = Manager->GetAllObstacles();
	TestEqual(TEXT("注册了 1 个障碍"), All.Num(), 1);

	const FObstacleInfo& Registered = All[0];
	TestEqual(TEXT("类型一致"), (int32)Registered.Type, (int32)EObstacleType::Sphere);
	UAV_TEST_VECTOR_EQUAL(Registered.Center, Entry.Center, 1.0f);
	UAV_TEST_VECTOR_EQUAL(Registered.Extents, Entry.Extents, 1.0f);
	UAV_TEST_FLOAT_EQUAL(Registered.SafetyMargin, Entry.SafetyMargin, 1e-3f);

	DestroyScenarioTestWorld(World);
	return true;
}

// ==================== 装配障碍：声明为空时注册 0 个 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioLoadObstaclesEmptyLayoutTest,
	"UAVSimulator.Scenario.LoadObstacles.EmptyLayout",
	UAV_TEST_FLAGS)

bool FScenarioLoadObstaclesEmptyLayoutTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("ScenarioLoadObsEmpty"));
	if (!TestNotNull(TEXT("合成 World 创建成功"), World))
	{
		return false;
	}

	UObstacleManager* Manager = NewObject<UObstacleManager>();
	UScenarioLoader* Loader = NewObject<UScenarioLoader>();

	// 空布局的场景
	UScenario* Scenario = NewObject<UScenario>(GetTransientPackage());
	UObstacleLayout* EmptyLayout = NewObject<UObstacleLayout>(Scenario);
	Scenario->ObstacleLayout = EmptyLayout;

	const int32 Registered = Loader->AssembleObstacles(Scenario, Manager, World);
	TestEqual(TEXT("空布局注册 0 个"), Registered, 0);
	TestEqual(TEXT("ObstacleManager 仍为空"), Manager->GetAllObstacles().Num(), 0);

	DestroyScenarioTestWorld(World);
	return true;
}

// ==================== 装配障碍：可视化 Actor 被 Spawn ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioLoadObstaclesSpawnsActorTest,
	"UAVSimulator.Scenario.LoadObstacles.SpawnsActor",
	UAV_TEST_FLAGS)

bool FScenarioLoadObstaclesSpawnsActorTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("ScenarioLoadObsSpawn"));
	if (!TestNotNull(TEXT("合成 World 创建成功"), World))
	{
		return false;
	}

	UObstacleManager* Manager = NewObject<UObstacleManager>();
	UScenarioLoader* Loader = NewObject<UScenarioLoader>();

	FScenarioObstacleEntry Entry;
	Entry.Type = EObstacleType::Sphere;
	Entry.Center = FVector(5000.0f, 0.0f, 0.0f);
	Entry.Extents = FVector(100.0f);

	UScenario* Scenario = MakeScenarioWithObstacles(GetTransientPackage(), { Entry });
	Loader->AssembleObstacles(Scenario, Manager, World);

	const TArray<FObstacleInfo>& All = Manager->GetAllObstacles();
	TestTrue(TEXT("注册了障碍"), All.Num() == 1);
	if (All.Num() == 1)
	{
		// 逻辑碰撞与可视化表现同源。
		TestTrue(TEXT("注册障碍关联了可视化 Actor"), All[0].LinkedActor.IsValid());
	}

	DestroyScenarioTestWorld(World);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioDynamicGeometryRegressionTest,
	"UAVSimulator.Scenario.LoadObstacles.DynamicGeometryAndVelocity", UAV_TEST_FLAGS)

bool FScenarioDynamicGeometryRegressionTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("DynamicGeometryRegression"));
	if (!World) return false;
	UObstacleManager* Manager = NewObject<UObstacleManager>(World->SpawnActor<AActor>());
	Manager->RegisterComponent();
	UScenarioLoader* Loader = NewObject<UScenarioLoader>();
	FScenarioObstacleEntry Entry;
	Entry.Type = EObstacleType::Box;
	Entry.Center = FVector(1000, 0, 1000);
	Entry.Extents = FVector(200, 300, 400);
	Entry.Rotation = FRotator(0, 30, 0);
	Entry.SafetyMargin = 100;
	Entry.MovementType = EObstacleMovementType::PatrolPingPong;
	Entry.PatrolPoints = { FVector(1000, -100, 1000), FVector(1000, 100, 1000) };
	Entry.PatrolSpeed = 100;
	UScenario* Scenario = MakeScenarioWithObstacles(GetTransientPackage(), { Entry });
	Loader->AssembleObstacles(Scenario, Manager, World);
	ADynamicObstacleActor* Actor = Cast<ADynamicObstacleActor>(Manager->GetAllObstacles()[0].LinkedActor.Get());
	if (TestNotNull(TEXT("声明生成障碍 Actor"), Actor))
	{
		UStaticMeshComponent* Mesh = Actor->FindComponentByClass<UStaticMeshComponent>();
		TestTrue(TEXT("网格与碰撞可查询"), Mesh && Mesh->GetStaticMesh() && Mesh->IsQueryCollisionEnabled());
		UAV_TEST_VECTOR_EQUAL(Actor->GetActorLocation(), Entry.PatrolPoints[0], 0.01f);
		Actor->Tick(3.0f);
		Manager->TickComponent(3.0f, LEVELTICK_All, nullptr);
		const FObstacleInfo& Snapshot = Manager->GetAllObstacles()[0];
		UAV_TEST_VECTOR_EQUAL(Snapshot.Center, FVector(1000, 0, 1000), 0.01f);
		UAV_TEST_VECTOR_EQUAL(Snapshot.Extents, Entry.Extents, 0.01f);
		UAV_TEST_VECTOR_EQUAL(Snapshot.Velocity, FVector(0, -100, 0), 0.01f);
		TestEqual(TEXT("安全边距未被感知覆盖"), Snapshot.SafetyMargin, Entry.SafetyMargin);
		TestTrue(TEXT("逻辑旋转未被世界 AABB 覆盖"), Snapshot.Rotation.Equals(Entry.Rotation, 0.01f));
		TestEqual(TEXT("重复感知不新增障碍"), Manager->RegisterPerceivedObstacleFromActor(Actor), Snapshot.ObstacleID);
		TestEqual(TEXT("仅一个逻辑障碍"), Manager->GetAllObstacles().Num(), 1);
	}
	DestroyScenarioTestWorld(World);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioPatrolPredictionTest,
	"UAVSimulator.Scenario.LoadObstacles.PatrolPrediction", UAV_TEST_FLAGS)
bool FScenarioPatrolPredictionTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateScenarioTestWorld(TEXT("PatrolPrediction"));
	if (!World) return false;
	ADynamicObstacleActor* Actor = World->SpawnActor<ADynamicObstacleActor>();
	for (EObstacleMovementType Type : {EObstacleMovementType::PatrolLoop, EObstacleMovementType::PatrolPingPong})
	{
		FScenarioObstacleEntry Entry;
		Entry.MovementType = Type;
		Entry.PatrolSpeed = 100;
		Entry.PatrolPoints = {FVector(0,0,1000), FVector(0,0,1000), FVector(200,0,1000), FVector(200,200,1000)};
		for (float Time : {0.5f, 2.0f, 3.0f, 5.0f, 31.5f})
		{
			Actor->Configure(Entry);
			UAV_TEST_VECTOR_EQUAL(Actor->GetVelocity(), FVector(100,0,0), 0.001f);
			Actor->Tick(0.25f);
			FObstacleInfo Initial = Actor->GetObstacleSnapshot();
			Initial.Extents = FVector(123,234,345);
			Initial.SafetyMargin = 87;
			const FObstacleInfo Predicted = ObstacleGeometry::Predict(Initial, Time);
			UAV_TEST_VECTOR_EQUAL(Actor->GetActorLocation(), Initial.Center, 0.001f);
			// 小步实际推进独立验证预测的跨段、转向和多周期行为。
			for (int32 i = 0; i < FMath::RoundToInt(Time * 100); ++i) Actor->Tick(0.01f);
			UAV_TEST_VECTOR_EQUAL(Predicted.Center, Actor->GetActorLocation(), 0.1f);
			UAV_TEST_VECTOR_EQUAL(Predicted.Velocity, Actor->GetVelocity(), 0.1f);
			UAV_TEST_VECTOR_EQUAL(Predicted.Extents, Initial.Extents, 0.001f);
			TestEqual(TEXT("Prediction preserves geometry margin"), Predicted.SafetyMargin, Initial.SafetyMargin);
		}
	}
	DestroyScenarioTestWorld(World);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
