// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Scenario/ScenarioLoader.h"
#include "../../Scenario/ScenarioTypes.h"
#include "../../Planning/ObstacleManager.h"
#include "../../Core/UAVTypes.h"

#include "Engine/Engine.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"
#include "Logging/LogMacros.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	// 复用项目已有的合成 World 模式（参照 UAVPawnCrashPhysicsTest）。
	UWorld* CreateScenarioTestWorld(const TCHAR* WorldName)
	{
		if (!GEngine)
		{
			return nullptr;
		}

		static int32 WorldCounter = 0;
		const FName UniqueWorldName(*FString::Printf(TEXT("%s_%d"), WorldName, ++WorldCounter));
		UWorld::InitializationValues InitValues;
		InitValues.AllowAudioPlayback(false)
			.CreatePhysicsScene(true)
			.RequiresHitProxies(false)
			.CreateNavigation(false)
			.CreateAISystem(false)
			.ShouldSimulatePhysics(false)
			.SetTransactional(false);
		UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, UniqueWorldName, GetTransientPackage(), false, ERHIFeatureLevel::Num, &InitValues);
		if (!World)
		{
			return nullptr;
		}

		FWorldContext& WorldContext = GEngine->CreateNewWorldContext(EWorldType::Game);
		UGameInstance* GameInstance = NewObject<UGameInstance>(GEngine);
		World->AddToRoot();
		World->SetGameInstance(GameInstance);
		WorldContext.OwningGameInstance = GameInstance;
		WorldContext.SetCurrentWorld(World);
		GameInstance->Init();

		const FURL URL;
		World->SetGameMode(URL);
		World->InitializeActorsForPlay(URL);
		World->BeginPlay();
		return World;
	}

	void DestroyScenarioTestWorld(UWorld* World)
	{
		if (!World || !GEngine)
		{
			return;
		}
		if (World->HasBegunPlay())
		{
			World->BeginTearingDown();
			World->EndPlay(EEndPlayReason::Quit);
		}
		GEngine->DestroyWorldContext(World);
		if (World->GetGameInstance())
		{
			World->GetGameInstance()->Shutdown();
		}
		World->DestroyWorld(false);
		World->RemoveFromRoot();
	}

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
		// 逻辑碰撞与可视化表现同源：注册的障碍关联了 Spawn 出的可视化 Actor。
		// 注意：此处 Spawn 的是裸 AActor（测试无 BP 资产），仅验证"关联已建立"这一外部行为；
		// 具体 Actor 位姿由真实 BP_Obstacle_Default 的 Mesh 决定，不在此断言。
		TestTrue(TEXT("注册障碍关联了可视化 Actor"), All[0].LinkedActor.IsValid());
	}

	DestroyScenarioTestWorld(World);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
