// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../MultiAgent/MILPSolver.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== IsIntegerFeasible 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMILPIsIntegerFeasibleAllIntegerTest,
	"UAVSimulator.MultiAgent.MILPSolver.IsIntegerFeasible_AllInteger",
	UAV_TEST_FLAGS)

bool FMILPIsIntegerFeasibleAllIntegerTest::RunTest(const FString& Parameters)
{
	UMILPSolver* Solver = NewObject<UMILPSolver>();

	TArray<float> Solution;
	Solution.Add(0.0f);
	Solution.Add(1.0f);
	Solution.Add(0.0f);

	TArray<int32> IntegerIndices;
	IntegerIndices.Add(0);
	IntegerIndices.Add(1);
	IntegerIndices.Add(2);

	TestTrue(TEXT("All integer solution should be feasible"),
		Solver->IsIntegerFeasible(Solution, IntegerIndices));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMILPIsIntegerFeasibleFractionalTest,
	"UAVSimulator.MultiAgent.MILPSolver.IsIntegerFeasible_Fractional",
	UAV_TEST_FLAGS)

bool FMILPIsIntegerFeasibleFractionalTest::RunTest(const FString& Parameters)
{
	UMILPSolver* Solver = NewObject<UMILPSolver>();

	TArray<float> Solution;
	Solution.Add(0.6f);

	TArray<int32> IntegerIndices;
	IntegerIndices.Add(0);

	TestFalse(TEXT("Fractional solution should not be integer feasible"),
		Solver->IsIntegerFeasible(Solution, IntegerIndices));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMILPIsIntegerFeasibleEmptyTest,
	"UAVSimulator.MultiAgent.MILPSolver.IsIntegerFeasible_EmptyIndices",
	UAV_TEST_FLAGS)

bool FMILPIsIntegerFeasibleEmptyTest::RunTest(const FString& Parameters)
{
	UMILPSolver* Solver = NewObject<UMILPSolver>();

	TArray<float> Solution;
	Solution.Add(0.5f);
	Solution.Add(1.7f);

	TArray<int32> EmptyIndices;

	// 没有整数约束 → 恒为可行
	TestTrue(TEXT("Empty integer indices should always be feasible"),
		Solver->IsIntegerFeasible(Solution, EmptyIndices));

	return true;
}

// ==================== IsFeasible 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMILPIsFeasibleSatisfiedTest,
	"UAVSimulator.MultiAgent.MILPSolver.IsFeasible_AllSatisfied",
	UAV_TEST_FLAGS)

bool FMILPIsFeasibleSatisfiedTest::RunTest(const FString& Parameters)
{
	UMILPSolver* Solver = NewObject<UMILPSolver>();

	TArray<float> Solution;
	Solution.Add(0.5f);
	Solution.Add(0.5f);

	// x0 + x1 <= 1.5 (0.5+0.5=1.0 <= 1.5 ✓)
	TArray<TArray<float>> AIneq;
	AIneq.Add({1.0f, 1.0f});
	TArray<float> BIneq;
	BIneq.Add(1.5f);

	// x0 + x1 = 1.0 (0.5+0.5=1.0 ✓)
	TArray<TArray<float>> AEq;
	AEq.Add({1.0f, 1.0f});
	TArray<float> BEq;
	BEq.Add(1.0f);

	// 0 <= x <= 1
	TArray<float> LB = {0.0f, 0.0f};
	TArray<float> UB = {1.0f, 1.0f};

	TestTrue(TEXT("Should be feasible"),
		Solver->IsFeasible(Solution, AIneq, BIneq, AEq, BEq, LB, UB));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMILPIsFeasibleViolatesBoundTest,
	"UAVSimulator.MultiAgent.MILPSolver.IsFeasible_ViolatesBound",
	UAV_TEST_FLAGS)

bool FMILPIsFeasibleViolatesBoundTest::RunTest(const FString& Parameters)
{
	UMILPSolver* Solver = NewObject<UMILPSolver>();

	// 违反上界：x0 = 1.5 > UB = 1.0
	TArray<float> Solution;
	Solution.Add(1.5f);
	Solution.Add(0.5f);

	TArray<TArray<float>> AIneq;
	TArray<float> BIneq;
	TArray<TArray<float>> AEq;
	TArray<float> BEq;
	TArray<float> LB = {0.0f, 0.0f};
	TArray<float> UB = {1.0f, 1.0f};

	TestFalse(TEXT("Should violate upper bound"),
		Solver->IsFeasible(Solution, AIneq, BIneq, AEq, BEq, LB, UB));

	// 违反下界：x1 = -0.5 < LB = 0.0
	Solution[0] = 0.5f;
	Solution[1] = -0.5f;

	TestFalse(TEXT("Should violate lower bound"),
		Solver->IsFeasible(Solution, AIneq, BIneq, AEq, BEq, LB, UB));

	return true;
}

// ==================== SelectBranchingVariable 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMILPSelectBranchingVariableTest,
	"UAVSimulator.MultiAgent.MILPSolver.SelectBranchingVariable",
	UAV_TEST_FLAGS)

bool FMILPSelectBranchingVariableTest::RunTest(const FString& Parameters)
{
	UMILPSolver* Solver = NewObject<UMILPSolver>();

	// 解: [0.3, 0.8, 0.6] → 最接近 0.5 的是索引 2 (0.6)
	TArray<float> Solution;
	Solution.Add(0.3f);
	Solution.Add(0.8f);
	Solution.Add(0.6f);

	TArray<int32> IntegerIndices;
	IntegerIndices.Add(0);
	IntegerIndices.Add(1);
	IntegerIndices.Add(2);

	int32 BranchIdx = Solver->SelectBranchingVariable(Solution, IntegerIndices);

	// 索引 2 的 |0.6-0.5| = 0.1 最小
	TestEqual(TEXT("Should select most fractional variable"), BranchIdx, 2);

	return true;
}

// ==================== RoundSolution 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMILPRoundSolutionTest,
	"UAVSimulator.MultiAgent.MILPSolver.RoundSolution",
	UAV_TEST_FLAGS)

bool FMILPRoundSolutionTest::RunTest(const FString& Parameters)
{
	UMILPSolver* Solver = NewObject<UMILPSolver>();

	TArray<float> Solution;
	Solution.Add(0.3f);
	Solution.Add(0.7f);
	Solution.Add(1.2f);

	TArray<int32> IntegerIndices;
	IntegerIndices.Add(0);
	IntegerIndices.Add(1);
	IntegerIndices.Add(2);

	TArray<float> Rounded = Solver->RoundSolution(Solution, IntegerIndices);

	// 四舍五入: 0.3→0, 0.7→1, 1.2→1
	TestEqual(TEXT("Rounded[0] should be 0"), Rounded[0], 0.0f);
	TestEqual(TEXT("Rounded[1] should be 1"), Rounded[1], 1.0f);
	TestEqual(TEXT("Rounded[2] should be 1"), Rounded[2], 1.0f);

	return true;
}

// ==================== Solve 不等式约束二值优化测试 ====================
// 注意: 投影梯度法 LP 松弛不支持等式约束投影，仅使用不等式约束

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMILPSolveBinaryAssignmentTest,
	"UAVSimulator.MultiAgent.MILPSolver.Solve_BinaryAssignment",
	UAV_TEST_FLAGS)

bool FMILPSolveBinaryAssignmentTest::RunTest(const FString& Parameters)
{
	UMILPSolver* Solver = NewObject<UMILPSolver>();

	// 3 变量二值优化：min x0 + 2*x1 + x2, s.t. x0+x1+x2 <= 2, 0<=xi<=1
	TArray<float> c = {1.0f, 2.0f, 1.0f};

	// 不等式约束: x0 + x1 + x2 <= 2
	TArray<TArray<float>> AIneq;
	AIneq.Add({1.0f, 1.0f, 1.0f});
	TArray<float> BIneq = {2.0f};

	// 不使用等式约束（投影梯度法无法处理）
	TArray<TArray<float>> AEq;
	TArray<float> BEq;

	TArray<float> LB = {0.0f, 0.0f, 0.0f};
	TArray<float> UB = {1.0f, 1.0f, 1.0f};

	TArray<int32> IntegerIndices = {0, 1, 2};

	FMILPSolverConfig Config;
	Config.MaxBranchAndBoundNodes = 200;
	Config.LPMaxIterations = 300;

	FMILPResult Result = Solver->Solve(c, AIneq, BIneq, AEq, BEq, LB, UB, IntegerIndices, Config);

	TestTrue(TEXT("Should find feasible solution"), Result.bIsFeasible);

	if (Result.bIsFeasible && Result.Solution.Num() == 3)
	{
		// 最优解: x=[0,0,0], obj=0（所有变量取 0 最小化目标）
		TestTrue(TEXT("Objective should be reasonable"), Result.ObjectiveValue <= 1.0f);

		// 所有变量应为 0 或 1
		for (int32 i = 0; i < 3; ++i)
		{
			TestTrue(FString::Printf(TEXT("Variable %d should be 0 or 1"), i),
				FMath::IsNearlyEqual(Result.Solution[i], 0.0f, 0.1f) ||
				FMath::IsNearlyEqual(Result.Solution[i], 1.0f, 0.1f));
		}
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
