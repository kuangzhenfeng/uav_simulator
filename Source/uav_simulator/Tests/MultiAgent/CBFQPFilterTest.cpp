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

// ==================== FilterV2: 静态障碍 CBF 方向性测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilterV2_StaticApproachingSphere,
	"UAVSimulator.MultiAgent.CBFQPFilter.FilterV2_StaticApproachingSphere",
	UAV_TEST_FLAGS)

bool FCBFQPFilterV2_StaticApproachingSphere::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 200.0f;
	Config.StaticInfluenceDistance = 2000.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 本机在 (0,0,0)，以 300cm/s 速度朝向球体（位于 (600,0,0)，半径 200）
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(300, 0, 0));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(600, 0, 0), 200.0f));

	// NMPC 输出向 +X 加速（朝向球体）
	FVector NominalAccel(300, 0, 0);

	FCBFQPResult Result = Filter->FilterV2(
		NominalAccel, MyState,
		TArray<FAgentStateSnapshot>(), Obstacles, Config);

	// CBF 应减小 +X 方向加速度（减速或偏转）
	TestTrue(TEXT("Approaching sphere: SafeAccel.X should be less than nominal"),
		Result.SafeAcceleration.X < NominalAccel.X);
	TestTrue(TEXT("Should have static constraint"),
		Result.StaticConstraintCount > 0);

	return true;
}

// ==================== FilterV2: 静态障碍 CBF 远离不激活 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilterV2_StaticSeparatingNoFilter,
	"UAVSimulator.MultiAgent.CBFQPFilter.FilterV2_StaticSeparatingNoFilter",
	UAV_TEST_FLAGS)

bool FCBFQPFilterV2_StaticSeparatingNoFilter::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 200.0f;
	Config.StaticInfluenceDistance = 2000.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;

	// 本机在 (0,0,0)，以 -X 速度远离球体（位于 (600,0,0)，半径 200）
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(-300, 0, 0));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(600, 0, 0), 200.0f));

	// 向 -X 加速（远离球体）
	FVector NominalAccel(-200, 0, 0);

	FCBFQPResult Result = Filter->FilterV2(
		NominalAccel, MyState,
		TArray<FAgentStateSnapshot>(), Obstacles, Config);

	// 远离时加速度方向不变（可能幅值略有差异但方向一致）
	TestTrue(TEXT("Separating: SafeAccel.X should remain negative"),
		Result.SafeAcceleration.X < 0.0f);

	return true;
}

// ==================== FilterV2: 多约束（静态 + 机间）同时激活 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilterV2_MultiConstraint,
	"UAVSimulator.MultiAgent.CBFQPFilter.FilterV2_MultiConstraint",
	UAV_TEST_FLAGS)

bool FCBFQPFilterV2_MultiConstraint::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;
	Config.DSafeStatic = 200.0f;
	Config.StaticInfluenceDistance = 2000.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 本机在 (0,0,0)，球体在 (400,0,0)，邻居在 (0,400,0)
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(200, 200, 0));

	TArray<FAgentStateSnapshot> Neighbors;
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		1, FVector(0, 400, 0), FVector(-50, 0, 0)));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(400, 0, 0), 100.0f));

	// NMPC 输出同时朝向球体和邻居的加速度
	FVector NominalAccel(300, 200, 0);

	FCBFQPResult Result = Filter->FilterV2(
		NominalAccel, MyState, Neighbors, Obstacles, Config);

	// 应同时有静态和机间约束
	TestTrue(TEXT("Should have static constraints"),
		Result.StaticConstraintCount > 0);
	TestTrue(TEXT("Should have agent constraints"),
		Result.AgentConstraintCount > 0);
	// X 和 Y 分量应至少有一个被减小
	bool bXReduced = Result.SafeAcceleration.X < NominalAccel.X;
	bool bYReduced = Result.SafeAcceleration.Y < NominalAccel.Y;
	TestTrue(TEXT("At least one component should be reduced"),
		bXReduced || bYReduced);

	return true;
}

// ==================== FilterV2: Slack 激活测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilterV2_SlackActivation,
	"UAVSimulator.MultiAgent.CBFQPFilter.FilterV2_SlackActivation",
	UAV_TEST_FLAGS)

bool FCBFQPFilterV2_SlackActivation::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 300.0f;
	Config.StaticInfluenceDistance = 5000.0f;
	Config.MaxAccelerationQP = 200.0f;  // 低加速度上限，制造不可行
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;
	Config.RhoStatic = 50.0f;
	Config.RhoAgent = 50.0f;

	// 本机在球体内部（h < 0，高度危险）
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(500, 0, 0));

	TArray<FObstacleInfo> Obstacles;
	// 球心在原点，半径 100，本机在内部
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(0, 0, 0), 100.0f, 0.0f));

	FVector NominalAccel(400, 0, 0);

	FCBFQPResult Result = Filter->FilterV2(
		NominalAccel, MyState,
		TArray<FAgentStateSnapshot>(), Obstacles, Config);

	// 内部场景: slack 应 > 0（执行器约束下无法完全满足 CBF）
	TestTrue(TEXT("StaticSlack should be >= 0"),
		Result.StaticSlack >= 0.0f);
	// 求解应为 Solved 或 SolvedWithSlack
		TestTrue(TEXT("Solve status should not be NumericalFailure"),
			Result.SolveStatus != ECBFQPStatus::NumericalFailure);

	return true;
}

// ==================== FilterV2: 降级模式测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilterV2_DegradedMode,
	"UAVSimulator.MultiAgent.CBFQPFilter.FilterV2_DegradedMode",
	UAV_TEST_FLAGS)

bool FCBFQPFilterV2_DegradedMode::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 无约束场景: QP 应直接返回 Solved
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(0, 0, 0));

	FVector NominalAccel(100, 50, 0);

	FCBFQPResult Result = Filter->FilterV2(
		NominalAccel, MyState,
		TArray<FAgentStateSnapshot>(), TArray<FObstacleInfo>(), Config);

	// 无约束时解应等于标称加速度
	TestTrue(TEXT("No-constraint status should be Solved"),
		Result.SolveStatus == ECBFQPStatus::Solved);
	UAV_TEST_FLOAT_EQUAL(Result.SafeAcceleration.X, NominalAccel.X, 5.0f);
	UAV_TEST_FLOAT_EQUAL(Result.SafeAcceleration.Y, NominalAccel.Y, 5.0f);

	return true;
}

// ==================== Active-Set QP: 无约束直接求解 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPASQP_NoConstraints,
	"UAVSimulator.MultiAgent.CBFQPFilter.ASQP_NoConstraints",
	UAV_TEST_FLAGS)

bool FCBFQPASQP_NoConstraints::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.QPMaxIterations = 50;
	Config.QPKKTTolerance = 1e-4f;

	const int32 N = 5;
	TArray<float> H, G;
	H.SetNumZeroed(N * N);
	G.SetNumZeroed(N);

	// H = diag(1, 1, 1, 200, 200), g = [-100, -50, 0, 0, 0]
	H[0] = 1.0f; H[6] = 1.0f; H[12] = 1.0f; H[18] = 200.0f; H[24] = 200.0f;
	G[0] = -100.0f; G[1] = -50.0f;

	TArray<float> A, B; // 无约束
	TArray<float> Z;
	ECBFQPStatus Status;
	float KKT, MaxViol;
	int32 Iters;

	Filter->SolveActiveSetQP(N, 0, H, G, A, B, Config, Z, Status, KKT, MaxViol, Iters);

	TestTrue(TEXT("Status should be Solved"), Status == ECBFQPStatus::Solved);
	TestEqual(TEXT("Z should have N elements"), Z.Num(), N);
	// z = H^{-1}(-g) = [100, 50, 0, 0, 0]
	UAV_TEST_FLOAT_EQUAL(Z[0], 100.0f, 1.0f);
	UAV_TEST_FLOAT_EQUAL(Z[1], 50.0f, 1.0f);
	UAV_TEST_FLOAT_EQUAL(Z[2], 0.0f, 1.0f);

	return true;
}

// ==================== Active-Set QP: 单约束投影 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPASQP_SingleConstraint,
	"UAVSimulator.MultiAgent.CBFQPFilter.ASQP_SingleConstraint",
	UAV_TEST_FLAGS)

bool FCBFQPASQP_SingleConstraint::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.QPMaxIterations = 50;
	Config.QPKKTTolerance = 1e-4f;

	const int32 N = 5;
	TArray<float> H, G;
	H.SetNumZeroed(N * N);
	G.SetNumZeroed(N);
	H[0] = 1.0f; H[6] = 1.0f; H[12] = 1.0f; H[18] = 200.0f; H[24] = 200.0f;
	G[0] = -100.0f; G[1] = -50.0f;

	// 约束: u_x ≤ 50 → [1,0,0,0,0]·z ≤ 50
	TArray<float> A, B;
	A.SetNumZeroed(N);
	A[0] = 1.0f;
	B.Add(50.0f);

	TArray<float> Z;
	ECBFQPStatus Status;
	float KKT, MaxViol;
	int32 Iters;

	Filter->SolveActiveSetQP(N, 1, H, G, A, B, Config, Z, Status, KKT, MaxViol, Iters);

	TestTrue(TEXT("Status should be Solved"), Status == ECBFQPStatus::Solved);
	// 无约束最优 z0=[100,50,0,0,0]，约束 ux ≤ 50 → 投影后 ux ≈ 50
	UAV_TEST_FLOAT_EQUAL(Z[0], 50.0f, 2.0f);
	UAV_TEST_FLOAT_EQUAL(Z[1], 50.0f, 2.0f);

	return true;
}

// ==================== 静态障碍 HOCBF 约束构建测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPBuildStatic_SphereConstraints,
	"UAVSimulator.MultiAgent.CBFQPFilter.BuildStatic_SphereConstraints",
	UAV_TEST_FLAGS)

bool FCBFQPBuildStatic_SphereConstraints::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 200.0f;
	Config.StaticInfluenceDistance = 2000.0f;
	Config.StaticAlpha0 = 1.0f;
	Config.StaticAlpha1 = 2.0f;

	// 本机在 (0,0,0)，球体在 (500,0,0)，半径 100
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(100, 0, 0));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(500, 0, 0), 100.0f));

	TArray<float> AFlat, Bounds;
	Filter->BuildStaticObstacleConstraints(MyState, Obstacles, Config, AFlat, Bounds);

	TestEqual(TEXT("Should have 1 constraint"), Bounds.Num(), 1);
	TestEqual(TEXT("A should have 5 columns"), AFlat.Num(), 5);

	// 梯度应指向 -X 方向（从障碍物指向查询点）
	// A 行 = [-GradD.x, -GradD.y, -GradD.z, -1, 0]
	// GradD 从障碍物表面指向查询点 = (-1,0,0) → A[0] = 1, A[1] = 0
	TestTrue(TEXT("A[0] should be > 0 (gradient points away from obstacle)"),
		AFlat[0] > 0.5f);
	TestTrue(TEXT("A[3] should be -1 (slack coefficient)"),
		AFlat[3] < -0.5f);

	return true;
}

// ==================== FilterV2: 对向逼近零碰撞 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilterV2_HeadOnZeroCollision,
	"UAVSimulator.MultiAgent.CBFQPFilter.FilterV2_HeadOnZeroCollision",
	UAV_TEST_FLAGS)

bool FCBFQPFilterV2_HeadOnZeroCollision::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 两机对向飞行：本机在 (0,0,0) 向 +X 飞，邻居在 (600,0,0) 向 -X 飞
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(300, 0, 0));

	TArray<FAgentStateSnapshot> Neighbors;
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		1, FVector(600, 0, 0), FVector(-300, 0, 0)));

	// NMPC 想继续向 +X 加速（朝向邻居）
	FVector NominalAccel(400, 0, 0);

	FCBFQPResult Result = Filter->FilterV2(
		NominalAccel, MyState, Neighbors, TArray<FObstacleInfo>(), Config);

	// CBF 应显著减小朝向邻居的加速度
	TestTrue(TEXT("Head-on: SafeAccel.X should be much less than nominal"),
		Result.SafeAcceleration.X < NominalAccel.X * 0.5f);
	TestTrue(TEXT("Should have agent constraint"),
		Result.AgentConstraintCount > 0);

	return true;
}

// ==================== FilterV2: 三机汇聚多约束 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilterV2_TripleMerge,
	"UAVSimulator.MultiAgent.CBFQPFilter.FilterV2_TripleMerge",
	UAV_TEST_FLAGS)

bool FCBFQPFilterV2_TripleMerge::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 三机汇聚：本机在原点，邻居从三个方向逼近
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(0, 0, 0));

	TArray<FAgentStateSnapshot> Neighbors;
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		1, FVector(500, 0, 0), FVector(-100, 0, 0)));
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		2, FVector(0, 500, 0), FVector(0, -100, 0)));

	// 向原点加速（朝向两个邻居）
	FVector NominalAccel(200, 200, 0);

	FCBFQPResult Result = Filter->FilterV2(
		NominalAccel, MyState, Neighbors, TArray<FObstacleInfo>(), Config);

	// 三机汇聚: 应有 2 个机间约束
	TestEqual(TEXT("Should have 2 agent constraints"),
		Result.AgentConstraintCount, 2);
	// 安全加速度应被偏转（不能同时朝两个邻居加速）
	float NomMag = NominalAccel.Size();
	float SafeMag = Result.SafeAcceleration.Size();
	TestTrue(TEXT("SafeAccel should be <= nominal magnitude"),
		SafeMag <= NomMag + 10.0f);
	// 求解应成功
	TestTrue(TEXT("Status should be Solved or SolvedWithSlack"),
		Result.SolveStatus == ECBFQPStatus::Solved ||
		Result.SolveStatus == ECBFQPStatus::SolvedWithSlack);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
