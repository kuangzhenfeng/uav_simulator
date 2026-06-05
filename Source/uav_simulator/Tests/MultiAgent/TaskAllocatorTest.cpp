// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../MultiAgent/TaskAllocator.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== EstimateTravelCost 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskAllocTravelCostBasicTest,
	"UAVSimulator.MultiAgent.TaskAllocator.EstimateTravelCost_Basic",
	UAV_TEST_FLAGS)

bool FTaskAllocTravelCostBasicTest::RunTest(const FString& Parameters)
{
	UTaskAllocator* Allocator = NewObject<UTaskAllocator>();

	// 距离 2000cm / 速度 1000cm/s = 2.0s
	float Cost = Allocator->EstimateTravelCost(
		FVector(0, 0, 0), FVector(2000, 0, 0), 1000.0f);
	UAV_TEST_FLOAT_EQUAL(Cost, 2.0f, 0.01f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskAllocTravelCostZeroSpeedTest,
	"UAVSimulator.MultiAgent.TaskAllocator.EstimateTravelCost_ZeroSpeed",
	UAV_TEST_FLAGS)

bool FTaskAllocTravelCostZeroSpeedTest::RunTest(const FString& Parameters)
{
	UTaskAllocator* Allocator = NewObject<UTaskAllocator>();

	// 速度为 0 → 返回 MAX_FLT（不可达）
	float Cost = Allocator->EstimateTravelCost(
		FVector(0, 0, 0), FVector(2000, 0, 0), 0.0f);
	TestEqual(TEXT("Zero speed should return MAX_FLT"), Cost, MAX_FLT);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskAllocTravelCostSamePosTest,
	"UAVSimulator.MultiAgent.TaskAllocator.EstimateTravelCost_SamePosition",
	UAV_TEST_FLAGS)

bool FTaskAllocTravelCostSamePosTest::RunTest(const FString& Parameters)
{
	UTaskAllocator* Allocator = NewObject<UTaskAllocator>();

	// 相同位置 → 代价 0
	float Cost = Allocator->EstimateTravelCost(
		FVector(100, 200, 300), FVector(100, 200, 300), 1000.0f);
	UAV_TEST_FLOAT_EQUAL(Cost, 0.0f, 0.01f);

	return true;
}

// ==================== DeriveCapabilities 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskAllocDeriveCapabilitiesTest,
	"UAVSimulator.MultiAgent.TaskAllocator.DeriveCapabilities",
	UAV_TEST_FLAGS)

bool FTaskAllocDeriveCapabilitiesTest::RunTest(const FString& Parameters)
{
	TArray<FAgentStateSnapshot> AgentStates;
	AgentStates.Add(UAVTestHelpers::CreateAgentSnapshot(0, FVector(0, 0, 0)));
	AgentStates.Add(UAVTestHelpers::CreateAgentSnapshot(1, FVector(1000, 0, 0)));
	AgentStates.Add(UAVTestHelpers::CreateAgentSnapshot(2, FVector(2000, 0, 0)));

	TArray<FUAVCapability> Caps = UTaskAllocator::DeriveCapabilities(AgentStates);

	TestEqual(TEXT("Should have 3 capabilities"), Caps.Num(), 3);

	if (Caps.Num() >= 3)
	{
		TestEqual(TEXT("Cap[0] AgentID"), Caps[0].AgentID, 0);
		TestEqual(TEXT("Cap[1] AgentID"), Caps[1].AgentID, 1);
		TestEqual(TEXT("Cap[2] AgentID"), Caps[2].AgentID, 2);

		UAV_TEST_VECTOR_EQUAL(Caps[0].CurrentPosition, FVector(0, 0, 0), 1.0f);
		UAV_TEST_VECTOR_EQUAL(Caps[1].CurrentPosition, FVector(1000, 0, 0), 1.0f);
	}

	return true;
}

// ==================== BuildMILPModel 维度测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskAllocBuildModelDimensionsTest,
	"UAVSimulator.MultiAgent.TaskAllocator.BuildMILPModel_Dimensions",
	UAV_TEST_FLAGS)

bool FTaskAllocBuildModelDimensionsTest::RunTest(const FString& Parameters)
{
	UTaskAllocator* Allocator = NewObject<UTaskAllocator>();

	// 3 Agent × 4 Task → 12 决策变量
	TArray<FTaskDescriptor> Tasks;
	for (int32 i = 0; i < 4; ++i)
	{
		Tasks.Add(UAVTestHelpers::CreateTaskDescriptor(i,
			FVector(i * 1000.0f, 0, 0)));
	}

	TArray<FUAVCapability> Caps;
	for (int32 i = 0; i < 3; ++i)
	{
		Caps.Add(UAVTestHelpers::CreateUAVCapability(i,
			FVector(i * 500.0f, 0, 0)));
	}

	TArray<float> Objective;
	TArray<TArray<float>> AIneq, AEq;
	TArray<float> BIneq, BEq, LB, UB;
	TArray<int32> IntegerIndices;

	Allocator->BuildMILPModel(Tasks, Caps, Objective, AIneq, BIneq, AEq, BEq, LB, UB, IntegerIndices);

	int32 N = 3 * 4; // 12
	TestEqual(TEXT("Objective should have 12 variables"), Objective.Num(), N);
	TestEqual(TEXT("LB should have 12 variables"), LB.Num(), N);
	TestEqual(TEXT("UB should have 12 variables"), UB.Num(), N);
	TestEqual(TEXT("IntegerIndices should have 12"), IntegerIndices.Num(), N);

	// 所有 LB 应为 0，UB 应为 1
	for (int32 i = 0; i < N; ++i)
	{
		TestEqual(FString::Printf(TEXT("LB[%d] should be 0"), i), LB[i], 0.0f);
		TestEqual(FString::Printf(TEXT("UB[%d] should be 1"), i), UB[i], 1.0f);
	}

	return true;
}

// ==================== BuildMILPModel 能力过滤测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskAllocBuildModelCapabilityFilterTest,
	"UAVSimulator.MultiAgent.TaskAllocator.BuildMILPModel_CapabilityFilter",
	UAV_TEST_FLAGS)

bool FTaskAllocBuildModelCapabilityFilterTest::RunTest(const FString& Parameters)
{
	UTaskAllocator* Allocator = NewObject<UTaskAllocator>();

	// 1 个需要特殊能力的任务
	TArray<FTaskDescriptor> Tasks;
	Tasks.Add(UAVTestHelpers::CreateTaskDescriptor(0,
		FVector(1000, 0, 0), ETaskPriority::Normal, 10.0f, 1.0f, 0.0f, 0x02));

	// Agent 0 有能力 0x01（不匹配），Agent 1 有能力 0x03（匹配）
	TArray<FUAVCapability> Caps;
	Caps.Add(UAVTestHelpers::CreateUAVCapability(0, FVector::ZeroVector, 2000.0f, 600.0f, 10.0f, 0x01));
	Caps.Add(UAVTestHelpers::CreateUAVCapability(1, FVector::ZeroVector, 2000.0f, 600.0f, 10.0f, 0x03));

	TArray<float> Objective;
	TArray<TArray<float>> AIneq, AEq;
	TArray<float> BIneq, BEq, LB, UB;
	TArray<int32> IntegerIndices;

	Allocator->BuildMILPModel(Tasks, Caps, Objective, AIneq, BIneq, AEq, BEq, LB, UB, IntegerIndices);

	// Agent 0 × Task 0 = index 0，Agent 1 × Task 0 = index 1
	// Agent 0 不匹配 → UB[0] = 0；Agent 1 匹配 → UB[1] = 1
	TestEqual(TEXT("UB[0] should be 0 (capability mismatch)"), UB[0], 0.0f);
	TestEqual(TEXT("UB[1] should be 1 (capability match)"), UB[1], 1.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
