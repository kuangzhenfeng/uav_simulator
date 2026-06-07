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

// ==================== Filter: 静态障碍 CBF 方向性测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_StaticApproachingSphere,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_StaticApproachingSphere",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_StaticApproachingSphere::RunTest(const FString& Parameters)
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

	FCBFQPResult Result = Filter->Filter(
		NominalAccel, MyState,
		TArray<FAgentStateSnapshot>(), Obstacles, Config);

	// CBF 应减小 +X 方向加速度（减速或偏转）
	TestTrue(TEXT("Approaching sphere: SafeAccel.X should be less than nominal"),
		Result.SafeAcceleration.X < NominalAccel.X);
	TestTrue(TEXT("Should have static constraint"),
		Result.StaticConstraintCount > 0);

	return true;
}

// ==================== Filter: 静态障碍 CBF 远离不激活 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_StaticSeparatingNoFilter,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_StaticSeparatingNoFilter",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_StaticSeparatingNoFilter::RunTest(const FString& Parameters)
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

	FCBFQPResult Result = Filter->Filter(
		NominalAccel, MyState,
		TArray<FAgentStateSnapshot>(), Obstacles, Config);

	// 远离时加速度方向不变（可能幅值略有差异但方向一致）
	TestTrue(TEXT("Separating: SafeAccel.X should remain negative"),
		Result.SafeAcceleration.X < 0.0f);

	return true;
}

// ==================== Filter: 多约束（静态 + 机间）同时激活 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_MultiConstraint,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_MultiConstraint",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_MultiConstraint::RunTest(const FString& Parameters)
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

	FCBFQPResult Result = Filter->Filter(
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

// ==================== Filter: Slack 激活测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_SlackActivation,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_SlackActivation",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_SlackActivation::RunTest(const FString& Parameters)
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

	FCBFQPResult Result = Filter->Filter(
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

// ==================== Filter: 降级模式测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_DegradedMode,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_DegradedMode",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_DegradedMode::RunTest(const FString& Parameters)
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

	FCBFQPResult Result = Filter->Filter(
		NominalAccel, MyState,
		TArray<FAgentStateSnapshot>(), TArray<FObstacleInfo>(), Config);

	// 无约束时解应等于标称加速度
	TestTrue(TEXT("No-constraint status should be Solved"),
		Result.SolveStatus == ECBFQPStatus::Solved);
	UAV_TEST_FLOAT_EQUAL(Result.SafeAcceleration.X, NominalAccel.X, 5.0f);
	UAV_TEST_FLOAT_EQUAL(Result.SafeAcceleration.Y, NominalAccel.Y, 5.0f);

	return true;
}

// ==================== Filter: 倾角可执行加速度锥 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_RespectsTiltAccelerationCone,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_RespectsTiltAccelerationCone",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_RespectsTiltAccelerationCone::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 1000), FVector::ZeroVector);

	// 该标称加速度满足旧的逐轴盒约束，但在 uz=-800 时水平加速度超过 30 度倾角可执行锥。
	FVector NominalAccel(800, 800, -800);

	FCBFQPResult Result = Filter->Filter(
		NominalAccel, MyState,
		TArray<FAgentStateSnapshot>(), TArray<FObstacleInfo>(), Config);

	const float Gravity = 980.0f;
	const float MaxTiltRad = FMath::DegreesToRadians(30.0f);
	const float TiltLimitedHorizontal = FMath::Tan(MaxTiltRad) *
		FMath::Max(0.0f, Gravity + Result.SafeAcceleration.Z);

	TestTrue(TEXT("Safe acceleration should stay inside the 30-degree executable tilt cone"),
		Result.SafeAcceleration.Size2D() <= TiltLimitedHorizontal + 5.0f);
	TestTrue(TEXT("Solve status should remain valid"),
		Result.SolveStatus == ECBFQPStatus::Solved ||
		Result.SolveStatus == ECBFQPStatus::SolvedWithSlack);

	return true;
}

// ==================== Filter: 速度包络主动制动 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_BrakesAboveSpeedEnvelope,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_BrakesAboveSpeedEnvelope",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_BrakesAboveSpeedEnvelope::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.MaxVelocity = 800.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 1000), FVector(1200, 0, 0));

	FVector NominalAccel(500, 0, 0);
	FCBFQPResult Result = Filter->Filter(
		NominalAccel, MyState,
		TArray<FAgentStateSnapshot>(), TArray<FObstacleInfo>(), Config);

	const FVector VelDir = MyState.Velocity.GetSafeNormal();
	const float AlongVelocityAccel = FVector::DotProduct(Result.SafeAcceleration, VelDir);

	TestTrue(TEXT("Above speed envelope should command braking along velocity"),
		AlongVelocityAccel < 0.0f);
	TestTrue(TEXT("Speed envelope should still respect acceleration bounds"),
		Result.SafeAcceleration.Size() <= Config.MaxAccelerationQP + 5.0f);
	TestTrue(TEXT("Solve status should remain valid"),
		Result.SolveStatus == ECBFQPStatus::Solved ||
		Result.SolveStatus == ECBFQPStatus::SolvedWithSlack);

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
	Config.MaxAccelerationQP = 50.0f;

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

// ==================== Active-Set QP: 高密度混合尺度约束 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPASQP_DenseMixedScale,
	"UAVSimulator.MultiAgent.CBFQPFilter.ASQP_DenseMixedScale",
	UAV_TEST_FLAGS)

bool FCBFQPASQP_DenseMixedScale::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.QPMaxIterations = 50;
	Config.QPKKTTolerance = 1e-3f;
	Config.MaxAccelerationQP = 800.0f;

	const int32 N = 5;
	TArray<float> H, G;
	H.SetNumZeroed(N * N);
	G.SetNumZeroed(N);
	H[0] = 1.0f; H[6] = 1.0f; H[12] = 1.0f;
	H[18] = 200.0f; H[24] = 200.0f;
	G[0] = -600.0f;
	G[1] = -450.0f;

	TArray<float> A, B;
	constexpr int32 DirectionCount = 48;
	for (int32 Index = 0; Index < DirectionCount; ++Index)
	{
		const float Angle = 2.0f * PI * static_cast<float>(Index) / DirectionCount;
		const float X = FMath::Cos(Angle);
		const float Y = FMath::Sin(Angle);

		// 静态约束: n·u - xi_static <= 100
		A.Append({X, Y, 0.0f, -1.0f, 0.0f});
		B.Add(100.0f);

		// 机间约束使用厘米尺度法线，验证混合量纲下的数值稳定性。
		A.Append({1000.0f * X, 1000.0f * Y, 0.0f, 0.0f, -1.0f});
		B.Add(100000.0f);
	}

	A.Append({0.0f, 0.0f, 0.0f, -1.0f, 0.0f});
	B.Add(0.0f);
	A.Append({0.0f, 0.0f, 0.0f, 0.0f, -1.0f});
	B.Add(0.0f);

	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		TArray<float> Positive;
		Positive.SetNumZeroed(N);
		Positive[Axis] = 1.0f;
		A.Append(Positive);
		B.Add(Config.MaxAccelerationQP);

		TArray<float> Negative;
		Negative.SetNumZeroed(N);
		Negative[Axis] = -1.0f;
		A.Append(Negative);
		B.Add(Config.MaxAccelerationQP);
	}

	TArray<float> Z;
	ECBFQPStatus Status;
	float KKT, MaxViol;
	int32 Iters;
	Filter->SolveActiveSetQP(
		N, B.Num(), H, G, A, B, Config, Z, Status, KKT, MaxViol, Iters);

	TestTrue(TEXT("Dense mixed-scale QP should converge"),
		Status == ECBFQPStatus::Solved ||
		Status == ECBFQPStatus::SolvedWithSlack);
	TestTrue(TEXT("Dense mixed-scale QP should remain feasible"), MaxViol <= 2.0f);
	TestTrue(TEXT("Dense mixed-scale QP should respect acceleration bounds"),
		FMath::Abs(Z[0]) <= Config.MaxAccelerationQP + 1.0f &&
		FMath::Abs(Z[1]) <= Config.MaxAccelerationQP + 1.0f &&
		FMath::Abs(Z[2]) <= Config.MaxAccelerationQP + 1.0f);
	TestTrue(TEXT("Dense mixed-scale QP should use acceleration instead of large agent slack when feasible"),
		Z.Num() >= 5 && Z[4] <= 1000.0f);

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

// ==================== Filter: 对向逼近零碰撞 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_HeadOnZeroCollision,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_HeadOnZeroCollision",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_HeadOnZeroCollision::RunTest(const FString& Parameters)
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

	FCBFQPResult Result = Filter->Filter(
		NominalAccel, MyState, Neighbors, TArray<FObstacleInfo>(), Config);

	// CBF 应显著减小朝向邻居的加速度
	TestTrue(TEXT("Head-on: SafeAccel.X should be much less than nominal"),
		Result.SafeAcceleration.X < NominalAccel.X * 0.5f);
	TestTrue(TEXT("Should have agent constraint"),
		Result.AgentConstraintCount > 0);
	TestTrue(TEXT("Head-on feasible QP should converge without degraded status"),
		Result.SolveStatus == ECBFQPStatus::Solved ||
		Result.SolveStatus == ECBFQPStatus::SolvedWithSlack);
	TestTrue(TEXT("Head-on feasible QP should not rely on large agent slack"),
		Result.AgentSlack <= 1000.0f);

	return true;
}

// ==================== Filter: 机间安全距离违反触发降级 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_AgentSafetyViolationTriggersDegradedStatus,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_AgentSafetyViolationTriggersDegradedStatus",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_AgentSafetyViolationTriggersDegradedStatus::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 复现仿真日志中的 MinH < 0：邻机已经进入机间安全距离。
	// 这种状态不能作为普通求解结果继续执行，否则上层不会进入保守逃逸。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(0, 800, 0));

	TArray<FAgentStateSnapshot> Neighbors;
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		1, FVector(250, 0, 0), FVector(0, -800, 0)));

	FCBFQPResult Result = Filter->Filter(
		FVector::ZeroVector, MyState, Neighbors, TArray<FObstacleInfo>(), Config);

	TestTrue(TEXT("Agent safety violation should enter degraded status"),
		Result.SolveStatus == ECBFQPStatus::MaxIterations ||
		Result.SolveStatus == ECBFQPStatus::NumericalFailure);
	TestTrue(TEXT("Regression setup should have negative inter-agent h"),
		Result.MinHValue < 0.0f);

	return true;
}

// ==================== Filter: 邻机障碍去重 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_DeduplicatesNeighborObstacle,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_DeduplicatesNeighborObstacle",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_DeduplicatesNeighborObstacle::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;
	Config.DSafeStatic = 200.0f;
	Config.StaticInfluenceDistance = 2000.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 复现日志中的三机初始布局：邻机同时作为通信邻居和障碍物进入安全层。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, -300, 1000), FVector(300, 0, 0));

	TArray<FAgentStateSnapshot> Neighbors;
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		1, FVector(600, 0, 1000), FVector(-100, 0, 0),
		FRotator::ZeroRotator, FVector::ZeroVector, FVector(-200, 0, 0)));

	TArray<FObstacleInfo> Obstacles;
	FObstacleInfo DuplicateUAVObstacle = UAVTestHelpers::CreateSphereObstacle(
		5, FVector(600, 0, 1000), 112.5f, 50.0f);
	DuplicateUAVObstacle.bIsDynamic = true;
	DuplicateUAVObstacle.Velocity = FVector(-100, 0, 0);
	Obstacles.Add(DuplicateUAVObstacle);

	FVector NominalAccel(420, -7, 9);
	FCBFQPResult Result = Filter->Filter(
		NominalAccel, MyState, Neighbors, Obstacles, Config);

	TestEqual(TEXT("Duplicate neighbor obstacle should be handled only by agent CBF"),
		Result.StaticConstraintCount, 0);
	TestEqual(TEXT("Duplicate neighbor should still have one agent constraint"),
		Result.AgentConstraintCount, 1);
	TestTrue(TEXT("Deduplicated mixed input should converge"),
		Result.SolveStatus == ECBFQPStatus::Solved ||
		Result.SolveStatus == ECBFQPStatus::SolvedWithSlack);
	TestTrue(TEXT("Deduplicated mixed input should not rely on large agent slack"),
		Result.AgentSlack <= 1000.0f);

	return true;
}

// ==================== Filter: 三机汇聚多约束 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_TripleMerge,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_TripleMerge",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_TripleMerge::RunTest(const FString& Parameters)
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

	FCBFQPResult Result = Filter->Filter(
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

// ==================== Filter: 静态障碍大 slack 触发降级 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_StaticLargeSlackTriggersDegradedStatus,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_StaticLargeSlackTriggersDegradedStatus",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_StaticLargeSlackTriggersDegradedStatus::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 200.0f;
	Config.StaticInfluenceDistance = 2000.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 已非常接近静态障碍并高速逼近，HOCBF 约束超出执行器可行域。
	// 这种状态不能靠大 slack 被标记为正常求解，否则上层不会进入保守逃逸。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(600, 0, 0));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(260, 0, 0), 50.0f, 50.0f));

	FCBFQPResult Result = Filter->Filter(
		FVector(-400, 0, 0), MyState,
		TArray<FAgentStateSnapshot>(), Obstacles, Config);

	TestTrue(TEXT("Large static slack should be reported as degraded status"),
		Result.SolveStatus == ECBFQPStatus::MaxIterations ||
		Result.SolveStatus == ECBFQPStatus::NumericalFailure);
	TestTrue(TEXT("Regression setup should require large static slack"),
		Result.StaticSlack > Config.MaxAccelerationQP * 0.25f);

	return true;
}

// ==================== Filter: 静态障碍离散时间即将穿透触发降级 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_StaticImminentDiscretePenetrationTriggersDegradedStatus,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_StaticImminentDiscretePenetrationTriggersDegradedStatus",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_StaticImminentDiscretePenetrationTriggersDegradedStatus::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 200.0f;
	Config.StaticInfluenceDistance = 2000.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 球体真实碰撞距离只剩 1cm，且当前速度会在一个 50Hz 控制周期内穿透安全裕度。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(100, 0, 0));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(101, 0, 0), 50.0f, 50.0f));

	FCBFQPResult Result = Filter->Filter(
		FVector::ZeroVector, MyState,
		TArray<FAgentStateSnapshot>(), Obstacles, Config);

	TestTrue(TEXT("Imminent one-step static penetration should enter degraded status"),
		Result.SolveStatus == ECBFQPStatus::MaxIterations ||
		Result.SolveStatus == ECBFQPStatus::NumericalFailure);

	return true;
}

// ==================== Filter: 静态障碍近碰撞滤波触发降级 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_StaticEmergencyFilteredNearCollisionTriggersDegradedStatus,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_StaticEmergencyFilteredNearCollisionTriggersDegradedStatus",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_StaticEmergencyFilteredNearCollisionTriggersDegradedStatus::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 200.0f;
	Config.StaticInfluenceDistance = 2000.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 真实碰撞净空只剩 30cm，虽然 HOCBF 仍可给出一个数学可行的修正，
	// 但安全层已经主动覆盖标称加速度，应进入降级制动给姿态执行留出余量。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector::ZeroVector);

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(130, 0, 0), 50.0f, 50.0f));

	FCBFQPResult Result = Filter->Filter(
		FVector(300, 0, 0), MyState,
		TArray<FAgentStateSnapshot>(), Obstacles, Config);

	TestTrue(TEXT("Emergency near-collision filtering should enter degraded status"),
		Result.SolveStatus == ECBFQPStatus::MaxIterations ||
		Result.SolveStatus == ECBFQPStatus::NumericalFailure);
	TestTrue(TEXT("Regression setup should require safety filtering"), Result.bWasFiltered);

	return true;
}

// ==================== Filter: 静态障碍高速近距离掠过触发降级 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_StaticHighSpeedNearCollisionTriggersDegradedStatus,
	"UAVSimulator.MultiAgent.CBFQPFilter.Filter_StaticHighSpeedNearCollisionTriggersDegradedStatus",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_StaticHighSpeedNearCollisionTriggersDegradedStatus::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 200.0f;
	Config.StaticInfluenceDistance = 2000.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.QPMaxIterations = 100;
	Config.QPKKTTolerance = 1e-3f;

	// 真实净空还有 300cm，但切向速度已经需要约 4m 的加速度预算才能重定向，
	// 继续执行普通 QP 修正会把降级留到姿态执行已经来不及的阶段。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(0, 800, 0));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(400, 0, 0), 50.0f, 50.0f));

	FCBFQPResult Result = Filter->Filter(
		FVector(300, 0, 0), MyState,
		TArray<FAgentStateSnapshot>(), Obstacles, Config);

	TestTrue(TEXT("High-speed near-collision filtering should enter degraded status"),
		Result.SolveStatus == ECBFQPStatus::MaxIterations ||
		Result.SolveStatus == ECBFQPStatus::NumericalFailure);
	TestTrue(TEXT("Regression setup should require safety filtering"), Result.bWasFiltered);

	return true;
}

// ==================== Degraded: 近障碍逃逸加速度 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_DegradedSafeAccelerationUsesFullStaticEscape,
	"UAVSimulator.MultiAgent.CBFQPFilter.DegradedSafeAccelerationUsesFullStaticEscape",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_DegradedSafeAccelerationUsesFullStaticEscape::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 200.0f;
	Config.MaxAccelerationQP = 800.0f;

	// 近障碍时，速度主要沿切向运动。单纯沿速度反向制动不能增加真实净空，
	// 降级安全加速度必须直接沿最近障碍法线逃逸。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(0, 600, 0));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(120, 0, 0), 50.0f, 50.0f));

	const FVector EscapeAccel = Filter->ComputeDegradedSafeAcceleration(MyState, Obstacles, Config);
	const FVector AwayFromObstacle = FVector(-1, 0, 0);

	constexpr float GravityAcceleration = 980.0f;
	constexpr int32 TiltFacetCount = 16;
	const float FacetScale = FMath::Cos(PI / static_cast<float>(TiltFacetCount));
	const float TiltSlope = FMath::Tan(FMath::DegreesToRadians(Config.MaxTiltAngleDeg));
	const float ExecutableHorizontalAccel = TiltSlope * FacetScale * (GravityAcceleration + EscapeAccel.Z);

	TestTrue(TEXT("Degraded acceleration should point away from the nearest static obstacle"),
		FVector::DotProduct(EscapeAccel, AwayFromObstacle) > ExecutableHorizontalAccel * 0.9f);
	TestTrue(TEXT("Degraded acceleration should stay inside QP acceleration budget"),
		EscapeAccel.Size() <= Config.MaxAccelerationQP + 1.0f);

	return true;
}

// ==================== Degraded: 静态障碍近于邻机时优先静态逃逸 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_DegradedSafeAccelerationPrefersNearestStaticRisk,
	"UAVSimulator.MultiAgent.CBFQPFilter.DegradedSafeAccelerationPrefersNearestStaticRisk",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_DegradedSafeAccelerationPrefersNearestStaticRisk::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;
	Config.DSafeStatic = 200.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.MaxTiltAngleDeg = 30.0f;

	// 邻机在安全距离之外，静态障碍已经极近时，降级逃逸必须选择静态障碍法线。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(0, 600, 0));

	TArray<FAgentStateSnapshot> Neighbors;
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		1, FVector(-2000, 0, 0), FVector::ZeroVector));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(120, 0, 0), 50.0f, 50.0f));

	const FVector EscapeAccel = Filter->ComputeDegradedSafeAcceleration(
		MyState, Neighbors, Obstacles, Config);

	TestTrue(TEXT("Nearest static risk should win over a far safe neighbor"),
		FVector::DotProduct(EscapeAccel, FVector(-1, 0, 0)) > 500.0f);

	return true;
}

// ==================== Degraded: 机间安全违反沿邻机法线逃逸 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_DegradedSafeAccelerationUsesAgentEscape,
	"UAVSimulator.MultiAgent.CBFQPFilter.DegradedSafeAccelerationUsesAgentEscape",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_DegradedSafeAccelerationUsesAgentEscape::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafe = 500.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.MaxTiltAngleDeg = 30.0f;

	// 邻机已经进入 DSafe，且没有静态障碍时，降级逃逸必须沿机间法线远离邻机。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(0, 600, 0));

	TArray<FAgentStateSnapshot> Neighbors;
	Neighbors.Add(UAVTestHelpers::CreateAgentSnapshot(
		1, FVector(250, 0, 0), FVector::ZeroVector));

	const FVector EscapeAccel = Filter->ComputeDegradedSafeAcceleration(
		MyState, Neighbors, TArray<FObstacleInfo>(), Config);
	const float HorizontalAccel = FVector2D(EscapeAccel.X, EscapeAccel.Y).Size();

	constexpr float GravityAcceleration = 980.0f;
	constexpr int32 TiltFacetCount = 16;
	const float FacetScale = FMath::Cos(PI / static_cast<float>(TiltFacetCount));
	const float TiltSlope = FMath::Tan(FMath::DegreesToRadians(Config.MaxTiltAngleDeg));
	const float ExecutableHorizontalAccel = TiltSlope * FacetScale * (GravityAcceleration + EscapeAccel.Z);

	TestTrue(TEXT("Degraded acceleration should point away from the unsafe neighbor"),
		FVector::DotProduct(EscapeAccel, FVector(-1, 0, 0)) > ExecutableHorizontalAccel * 0.9f);
	TestTrue(TEXT("Agent degraded acceleration should stay inside the executable tilt cone"),
		HorizontalAccel <= ExecutableHorizontalAccel + 1.0f);

	return true;
}

// ==================== Degraded: 逃逸加速度满足倾角可执行域 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_DegradedSafeAccelerationMaximizesExecutableHorizontalEscape,
	"UAVSimulator.MultiAgent.CBFQPFilter.DegradedSafeAccelerationMaximizesExecutableHorizontalEscape",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_DegradedSafeAccelerationMaximizesExecutableHorizontalEscape::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 200.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.MaxTiltAngleDeg = 30.0f;

	// 障碍物略高于本机时，简单沿三维法线逃逸会给出向下加速度，降低水平可执行预算。
	// 降级模式应在倾角锥内最大化逃逸法线投影，用上推换取更强水平逃逸。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(0, 600, 0));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(120, 0, 30), 50.0f, 50.0f));

	const FVector EscapeAccel = Filter->ComputeDegradedSafeAcceleration(MyState, Obstacles, Config);
	const float HorizontalAccel = FVector2D(EscapeAccel.X, EscapeAccel.Y).Size();

	constexpr float GravityAcceleration = 980.0f;
	constexpr int32 TiltFacetCount = 16;
	const float FacetScale = FMath::Cos(PI / static_cast<float>(TiltFacetCount));
	const float TiltSlope = FMath::Tan(FMath::DegreesToRadians(Config.MaxTiltAngleDeg));
	const float ExecutableHorizontalAccel = TiltSlope * FacetScale * (GravityAcceleration + EscapeAccel.Z);

	TestTrue(TEXT("Degraded acceleration should use upward thrust to increase horizontal escape authority"),
		EscapeAccel.Z > 0.0f);
	TestTrue(TEXT("Degraded acceleration should produce stronger executable horizontal escape"),
		HorizontalAccel > 600.0f);
	TestTrue(TEXT("Optimized degraded acceleration should stay inside the executable tilt cone"),
		HorizontalAccel <= ExecutableHorizontalAccel + 1.0f);
	TestTrue(TEXT("Optimized degraded acceleration should still point away from the obstacle"),
		FVector::DotProduct(EscapeAccel, FVector(-1, 0, 0)) > 600.0f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCBFQPFilter_DegradedSafeAccelerationRespectsTiltCone,
	"UAVSimulator.MultiAgent.CBFQPFilter.DegradedSafeAccelerationRespectsTiltCone",
	UAV_TEST_FLAGS)

bool FCBFQPFilter_DegradedSafeAccelerationRespectsTiltCone::RunTest(const FString& Parameters)
{
	UCBFQPFilter* Filter = NewObject<UCBFQPFilter>();

	FCBFQPConfig Config;
	Config.DSafeStatic = 200.0f;
	Config.MaxAccelerationQP = 800.0f;
	Config.MaxTiltAngleDeg = 30.0f;

	// 纯水平逃逸若直接给满 800cm/s²，会超过 30° 倾角对应的可执行水平加速度。
	// 降级输出必须和正常 QP 共用同一执行器可行域，否则姿态环会兑现不了安全层意图。
	FUAVState MyState = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector(0, 600, 0));

	TArray<FObstacleInfo> Obstacles;
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(120, 0, 0), 50.0f, 50.0f));

	const FVector EscapeAccel = Filter->ComputeDegradedSafeAcceleration(MyState, Obstacles, Config);
	const float HorizontalAccel = FVector2D(EscapeAccel.X, EscapeAccel.Y).Size();

	constexpr float GravityAcceleration = 980.0f;
	constexpr int32 TiltFacetCount = 16;
	const float FacetScale = FMath::Cos(PI / static_cast<float>(TiltFacetCount));
	const float TiltSlope = FMath::Tan(FMath::DegreesToRadians(Config.MaxTiltAngleDeg));
	const float ExecutableHorizontalAccel = TiltSlope * FacetScale * (GravityAcceleration + EscapeAccel.Z);

	TestTrue(TEXT("Degraded acceleration should stay inside the executable tilt cone"),
		HorizontalAccel <= ExecutableHorizontalAccel + 1.0f);
	TestTrue(TEXT("Degraded acceleration should still point away from the nearest static obstacle"),
		FVector::DotProduct(EscapeAccel, FVector(-1, 0, 0)) > ExecutableHorizontalAccel * 0.9f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
