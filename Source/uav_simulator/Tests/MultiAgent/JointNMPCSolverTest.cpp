// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../MultiAgent/JointNMPCSolver.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== ForwardSimulateAgent 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointNMPCForwardSimZeroControlTest,
	"UAVSimulator.MultiAgent.JointNMPCSolver.ForwardSim_ZeroControl",
	UAV_TEST_FLAGS)

bool FJointNMPCForwardSimZeroControlTest::RunTest(const FString& Parameters)
{
	UJointNMPCSolver* Solver = NewObject<UJointNMPCSolver>();

	FVector InitPos(0, 0, 100);
	FVector InitVel(100, 0, 0);
	float Dt = 0.1f;
	float MaxVel = 2000.0f;

	// 3 步零控制
	TArray<FVector> Controls;
	Controls.Add(FVector::ZeroVector);
	Controls.Add(FVector::ZeroVector);
	Controls.Add(FVector::ZeroVector);

	TArray<FVector> Positions, Velocities;
	Solver->ForwardSimulateAgent(InitPos, InitVel, Controls, Dt, MaxVel, Positions, Velocities);

	// 应有 N+1 = 4 个点
	TestEqual(TEXT("Should have 4 positions"), Positions.Num(), 4);
	TestEqual(TEXT("Should have 4 velocities"), Velocities.Num(), 4);

	// 速度不变（零加速度）
	for (int32 i = 0; i < Velocities.Num(); ++i)
	{
		UAV_TEST_VECTOR_EQUAL(Velocities[i], InitVel, 1.0f);
	}

	// 位置线性增长
	for (int32 i = 1; i < Positions.Num(); ++i)
	{
		FVector Expected = Positions[i - 1] + InitVel * Dt;
		UAV_TEST_VECTOR_EQUAL(Positions[i], Expected, 1.0f);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointNMPCForwardSimConstantAccelTest,
	"UAVSimulator.MultiAgent.JointNMPCSolver.ForwardSim_ConstantAccel",
	UAV_TEST_FLAGS)

bool FJointNMPCForwardSimConstantAccelTest::RunTest(const FString& Parameters)
{
	UJointNMPCSolver* Solver = NewObject<UJointNMPCSolver>();

	FVector InitPos(0, 0, 0);
	FVector InitVel(0, 0, 0);
	float Dt = 0.1f;
	float MaxVel = 2000.0f;
	FVector Accel(100, 0, 0); // 恒加速度 100 cm/s²

	TArray<FVector> Controls;
	Controls.Add(Accel);
	Controls.Add(Accel);
	Controls.Add(Accel);

	TArray<FVector> Positions, Velocities;
	Solver->ForwardSimulateAgent(InitPos, InitVel, Controls, Dt, MaxVel, Positions, Velocities);

	// 速度应线性增长: v = v0 + a*dt
	for (int32 i = 1; i < Velocities.Num(); ++i)
	{
		FVector Expected = Velocities[i - 1] + Accel * Dt;
		UAV_TEST_VECTOR_EQUAL(Velocities[i], Expected, 1.0f);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointNMPCForwardSimVelocityClampTest,
	"UAVSimulator.MultiAgent.JointNMPCSolver.ForwardSim_VelocityClamp",
	UAV_TEST_FLAGS)

bool FJointNMPCForwardSimVelocityClampTest::RunTest(const FString& Parameters)
{
	UJointNMPCSolver* Solver = NewObject<UJointNMPCSolver>();

	FVector InitPos(0, 0, 0);
	FVector InitVel(1900, 0, 0);
	float Dt = 0.1f;
	float MaxVel = 2000.0f;
	FVector LargeAccel(5000, 0, 0); // 超大加速度

	TArray<FVector> Controls;
	Controls.Add(LargeAccel);
	Controls.Add(LargeAccel);

	TArray<FVector> Positions, Velocities;
	Solver->ForwardSimulateAgent(InitPos, InitVel, Controls, Dt, MaxVel, Positions, Velocities);

	// 所有速度分量不应超过 MaxVel
	for (int32 i = 0; i < Velocities.Num(); ++i)
	{
		TestTrue(TEXT("Velocity should be clamped to MaxVel"),
			Velocities[i].Size() <= MaxVel + 1.0f);
	}

	return true;
}

// ==================== InterAgentCollisionCost 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointNMPCInterAgentCostFarTest,
	"UAVSimulator.MultiAgent.JointNMPCSolver.InterAgentCost_FarAgents",
	UAV_TEST_FLAGS)

bool FJointNMPCInterAgentCostFarTest::RunTest(const FString& Parameters)
{
	UJointNMPCSolver* Solver = NewObject<UJointNMPCSolver>();

	// 距离远大于影响距离 → 代价应为 0
	FVector PosA(0, 0, 0);
	FVector PosB(10000, 0, 0); // 距离 10000cm

	float Cost = Solver->ComputeInterAgentCollisionCost(
		PosA, PosB, 500.0f, 3000.0f, 1.0f);

	TestTrue(TEXT("Cost should be 0 for far agents"), Cost < 0.001f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointNMPCInterAgentCostCloseTest,
	"UAVSimulator.MultiAgent.JointNMPCSolver.InterAgentCost_CloseAgents",
	UAV_TEST_FLAGS)

bool FJointNMPCInterAgentCostCloseTest::RunTest(const FString& Parameters)
{
	UJointNMPCSolver* Solver = NewObject<UJointNMPCSolver>();

	// 距离小于安全距离 → 正代价
	FVector PosA(0, 0, 0);
	FVector PosB(200, 0, 0); // 距离 200cm < SafeDist=500

	float Cost = Solver->ComputeInterAgentCollisionCost(
		PosA, PosB, 500.0f, 3000.0f, 1.0f);

	TestTrue(TEXT("Cost should be positive for close agents"), Cost > 0.0f);

	// 同位置 → 更大惩罚
	float SamePosCost = Solver->ComputeInterAgentCollisionCost(
		FVector(0, 0, 0), FVector(0, 0, 0), 500.0f, 3000.0f, 1.0f);
	TestTrue(TEXT("Same position should have larger cost"), SamePosCost > Cost);

	return true;
}

// ==================== FormationCost 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointNMPCFormationCostTest,
	"UAVSimulator.MultiAgent.JointNMPCSolver.FormationCost",
	UAV_TEST_FLAGS)

bool FJointNMPCFormationCostTest::RunTest(const FString& Parameters)
{
	UJointNMPCSolver* Solver = NewObject<UJointNMPCSolver>();

	// 完全匹配 → 代价 = 0
	float ZeroCost = Solver->ComputeFormationCost(
		FVector(100, 200, 300), FVector(100, 200, 300));
	UAV_TEST_FLOAT_EQUAL(ZeroCost, 0.0f, 0.01f);

	// 有偏差 → 代价 = dist²
	FVector Actual(100, 200, 300);
	FVector Desired(200, 200, 300); // 偏差 100cm
	float Cost = Solver->ComputeFormationCost(Actual, Desired);
	float ExpectedDistSq = 100.0f * 100.0f;
	UAV_TEST_FLOAT_EQUAL(Cost, ExpectedDistSq, 1.0f);

	return true;
}

// ==================== ProjectAgentControls 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointNMPCProjectControlsTest,
	"UAVSimulator.MultiAgent.JointNMPCSolver.ProjectAgentControls",
	UAV_TEST_FLAGS)

bool FJointNMPCProjectControlsTest::RunTest(const FString& Parameters)
{
	UJointNMPCSolver* Solver = NewObject<UJointNMPCSolver>();

	float MaxAccel = 500.0f;
	float MaxVel = 2000.0f;
	FVector InitVel(0, 0, 0);

	// 超限控制
	TArray<FVector> OverControls;
	OverControls.Add(FVector(1000, 0, 0)); // 超过 MaxAccel
	OverControls.Add(FVector(-2000, 0, 0));

	TArray<FVector> Projected = OverControls;
	Solver->ProjectAgentControls(Projected, InitVel, MaxAccel, MaxVel);

	for (int32 i = 0; i < Projected.Num(); ++i)
	{
		TestTrue(FString::Printf(TEXT("Control[%d] magnitude should be clamped"), i),
			Projected[i].Size() <= MaxAccel + 1.0f);
	}

	// 正常控制应不变
	TArray<FVector> NormalControls;
	NormalControls.Add(FVector(100, 100, 100)); // 远小于 MaxAccel

	TArray<FVector> NormalProjected = NormalControls;
	Solver->ProjectAgentControls(NormalProjected, InitVel, MaxAccel, MaxVel);

	for (int32 i = 0; i < NormalProjected.Num(); ++i)
	{
		UAV_TEST_VECTOR_EQUAL(NormalProjected[i], NormalControls[i], 1.0f);
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
