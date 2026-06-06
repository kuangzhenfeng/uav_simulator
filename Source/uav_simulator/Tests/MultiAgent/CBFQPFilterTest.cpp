// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../MultiAgent/CBFQPFilter.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== ComputeHValue 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPComputeHValuePositiveTest,
	"UAVSimulator.MultiAgent.CBFQPFilter.ComputeHValue_Positive",
	UAV_TEST_FLAGS)

bool FCBFQPComputeHValuePositiveTest::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	// Pi=(0,0,0), Pj=(1000,0,0), DSafe=500
	// h = ||Pi-Pj||² - DSafe² = 1000000 - 250000 = 750000
	float H = Filter->ComputeHValue(FVector(0, 0, 0), FVector(1000, 0, 0), 500.0f);
	UAV_TEST_FLOAT_EQUAL(H, 750000.0f, 1.0f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPComputeHValueNegativeTest,
	"UAVSimulator.MultiAgent.CBFQPFilter.ComputeHValue_Negative",
	UAV_TEST_FLAGS)

bool FCBFQPComputeHValueNegativeTest::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	// Pi=(0,0,0), Pj=(200,0,0), DSafe=500
	// h = 40000 - 250000 = -210000
	float H = Filter->ComputeHValue(FVector(0, 0, 0), FVector(200, 0, 0), 500.0f);
	UAV_TEST_FLOAT_EQUAL(H, -210000.0f, 1.0f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPComputeHValueZeroTest,
	"UAVSimulator.MultiAgent.CBFQPFilter.ComputeHValue_Zero",
	UAV_TEST_FLAGS)

bool FCBFQPComputeHValueZeroTest::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	// Pi=(0,0,0), Pj=(500,0,0), DSafe=500
	// h = 250000 - 250000 = 0
	float H = Filter->ComputeHValue(FVector(0, 0, 0), FVector(500, 0, 0), 500.0f);
	UAV_TEST_FLOAT_EQUAL(H, 0.0f, 1.0f);

	return true;
}

// ==================== ComputeHDot 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPComputeHDotTest,
	"UAVSimulator.MultiAgent.CBFQPFilter.ComputeHDot_Calculation",
	UAV_TEST_FLAGS)

bool FCBFQPComputeHDotTest::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	// Pi=(100,0,0), Pj=(0,0,0), Vi=(0,0,0), Vj=(100,0,0)
	// h_dot = 2 * (Pi-Pj) · (Vi-Vj) = 2 * (100,0,0) · (-100,0,0) = 2 * (-10000) = -20000
	float HDot = Filter->ComputeHDot(
		FVector(100, 0, 0), FVector(0, 0, 0),
		FVector(0, 0, 0), FVector(100, 0, 0));
	UAV_TEST_FLOAT_EQUAL(HDot, -20000.0f, 1.0f);

	return true;
}

// ==================== BuildCBFConstraints 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBQPBuildConstraintsTest,
	"UAVSimulator.MultiAgent.CBFQPFilter.BuildConstraints_SingleNeighbor",
	UAV_TEST_FLAGS)

bool FCBQPBuildConstraintsTest::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	// 本机在原点，邻居在 (300, 0, 0) — 在安全距离内
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(0, 0, 0));

	TArray<FAgentStateSnapshot> Neighbors;
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		1, FVector(300, 0, 0), FVector(0, 0, 0)));

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;

	TArray<FVector> Normals;
	TArray<float> Bounds;
	Filter->BuildCBFConstraints(MyState, Neighbors, Config, Normals, Bounds);

	// 应该有 1 个约束
	TestEqual(TEXT("Should have 1 constraint"), Normals.Num(), 1);
	TestEqual(TEXT("Should have 1 bound"), Bounds.Num(), 1);

		// 约束法线方向：A_k = -2*DeltaP = -2*(Pi-Pj) = -2*(-300,0,0) = (600,0,0)
		// 归一化后方向应大致为 X 正方向（ui 的约束方向）
		if (Normals.Num() > 0)
		{
			FVector NormalDir = Normals[0].GetSafeNormal();
			TestTrue(TEXT("Constraint normal should point roughly +X"),
				NormalDir.X > 0.9f);
	}

	return true;
}

// ==================== QP 求解器测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPSolverProjectionTest,
	"UAVSimulator.MultiAgent.CBFQPFilter.QPSolver_Projection",
	UAV_TEST_FLAGS)

bool FCBFQPSolverProjectionTest::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	// 标称加速度指向邻居（不安全方向）
	FVector NominalAccel(-500.0f, 0.0f, 0.0f);

	// 约束：A·u ≤ b，阻止向邻居方向加速
	TArray<FVector> Normals;
	Normals.Add(FVector(-1, 0, 0)); // 法线指向 -X

	TArray<float> Bounds;
	Bounds.Add(-100.0f); // A·u ≤ -100 → -ux ≤ -100 → ux ≥ 100

	FCBFQPConfig Config;
	Config.MaxIterations = 50;

	FVector SafeAccel = Filter->SolveProjectedGradientQP(
		NominalAccel, Normals, Bounds, Config);

	// 滤波后 X 分量应满足 ux >= 100
	TestTrue(TEXT("Filtered X should be >= 100"),
		SafeAccel.X >= 99.0f);

	return true;
}

// ==================== Filter 全流程测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilterFullPipelineTest,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_FullPipeline",
	UAV_TEST_FLAGS)

bool FCBFQPFilterFullPipelineTest::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;
	Config.bEnabled = true;

		// ---- 测试 1：有逼近邻居，应触发滤波 ----
		{
			FUAVState MyState = UAVTestHelpers::CreateUAVState(
				FVector(0, 0, 0), FVector(100, 0, 0));

			TArray<FAgentStateSnapshot> Neighbors;
			Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
				1, FVector(300, 0, 0), FVector(-100, 0, 0)));

			// 加速度指向邻居（+X 方向），CBF 应将其推离
			FVector NominalAccel(200, 0, 0);
			FCBFQPResult Result = Filter->Filter(NominalAccel, MyState, Neighbors, Config);

			// 安全加速度的 X 分量应比标称值小（远离邻居方向）
			TestTrue(TEXT("SafeAccel.X should be less than nominal when approaching neighbor"),
				Result.SafeAcceleration.X < NominalAccel.X);
		}

	// ---- 测试 2：无邻居，不应滤波 ----
	{
		FUAVState MyState = UAVTestHelpers::CreateUAVState(
			FVector(0, 0, 0), FVector(100, 0, 0));

		TArray<FAgentStateSnapshot> NoNeighbors;
		FVector NominalAccel(100, 0, 0);
		FCBFQPResult Result = Filter->Filter(NominalAccel, MyState, NoNeighbors, Config);

		TestFalse(TEXT("Should not filter with no neighbors"), Result.bWasFiltered);
	}

	return true;
}

// ==================== 方向性测试：对向逼近 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPDirectionalApproachingTest,
	"UAVSimulator.MultiAgent.CBFQPFilter.Directional_Approaching",
	UAV_TEST_FLAGS)

bool FCBFQPDirectionalApproachingTest::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;

	// 两机对向飞行：本机在原点向+X飞，邻居在(600,0,0)向-X飞
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(200, 0, 0));

	TArray<FAgentStateSnapshot> Neighbors;
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		1, FVector(600, 0, 0), FVector(-200, 0, 0)));

	// NMPC 想向+X加速（朝向邻居）
	FVector NominalAccel(300, 0, 0);
	FCBFQPResult Result = Filter->Filter(NominalAccel, MyState, Neighbors, Config);

	// CBF 应将加速度推离邻居：SafeAccel.X 应小于 NominalAccel.X
	TestTrue(TEXT("Approaching: SafeAccel.X should be less than nominal"),
		Result.SafeAcceleration.X < NominalAccel.X);

	return true;
}

// ==================== 方向性测试：远离运动 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPDirectionalSeparatingTest,
	"UAVSimulator.MultiAgent.CBFQPFilter.Directional_Separating",
	UAV_TEST_FLAGS)

bool FCBFQPDirectionalSeparatingTest::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;

	// 两机远离：本机在原点向-X飞，邻居在(1000,0,0)向+X飞（已远离）
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(-100, 0, 0));

	TArray<FAgentStateSnapshot> Neighbors;
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		1, FVector(1000, 0, 0), FVector(100, 0, 0)));

	// 向-X加速（远离邻居）
	FVector NominalAccel(-200, 0, 0);
	FCBFQPResult Result = Filter->Filter(NominalAccel, MyState, Neighbors, Config);

	// 远离时 h 增大，CBF 不应限制
	TestFalse(TEXT("Separating: should not filter"), Result.bWasFiltered);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
