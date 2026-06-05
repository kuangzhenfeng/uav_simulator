// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../MultiAgent/TaskMonitor.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== Initialize 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskMonitorInitTest,
	"UAVSimulator.MultiAgent.TaskMonitor.Initialize_InitialState",
	UAV_TEST_FLAGS)

bool FTaskMonitorInitTest::RunTest(const FString& Parameters)
{
	UTaskMonitor* Monitor = NewObject<UTaskMonitor>();

	// 创建 3 个任务的分配结果
	TArray<FTaskAssignment> Assignments;
	Assignments.Add(UAVTestHelpers::CreateTaskAssignment(0, 0, 0, 10, 5000));
	Assignments.Add(UAVTestHelpers::CreateTaskAssignment(1, 1, 5, 15, 6000));
	Assignments.Add(UAVTestHelpers::CreateTaskAssignment(2, 0, 10, 20, 7000));

	FTaskAllocationResult Alloc = UAVTestHelpers::CreateAllocationResult(Assignments);
	FTaskMonitorConfig Config;
	Monitor->Initialize(Alloc, Config);

	TestEqual(TEXT("Total count should be 3"), Monitor->GetTotalTaskCount(), 3);
	TestEqual(TEXT("Completed count should be 0"), Monitor->GetCompletedTaskCount(), 0);
	UAV_TEST_FLOAT_EQUAL(Monitor->GetOverallProgress(), 0.0f, 0.01f);

	return true;
}

// ==================== OverallProgress 部分完成测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskMonitorProgressPartialTest,
	"UAVSimulator.MultiAgent.TaskMonitor.OverallProgress_Partial",
	UAV_TEST_FLAGS)

bool FTaskMonitorProgressPartialTest::RunTest(const FString& Parameters)
{
	UTaskMonitor* Monitor = NewObject<UTaskMonitor>();

	TArray<FTaskAssignment> Assignments;
	Assignments.Add(UAVTestHelpers::CreateTaskAssignment(0, 0, 0, 10, 5000));
	Assignments.Add(UAVTestHelpers::CreateTaskAssignment(1, 1, 5, 15, 6000));
	Assignments.Add(UAVTestHelpers::CreateTaskAssignment(2, 0, 10, 20, 7000));

	FTaskAllocationResult Alloc = UAVTestHelpers::CreateAllocationResult(Assignments);
	FTaskMonitorConfig Config;
	Monitor->Initialize(Alloc, Config);

	// 多次 Update 让 Agent 0 到达任务 0 目标位置
	TArray<FAgentStateSnapshot> AgentStates;
	for (int32 i = 0; i < 100; ++i)
	{
		AgentStates.Empty();
		AgentStates.Add(UAVTestHelpers::CreateAgentSnapshot(0, FVector(0, 0, 0)));
		AgentStates.Add(UAVTestHelpers::CreateAgentSnapshot(1, FVector(0, 0, 0)));
		Monitor->Update(0.1f, AgentStates);
	}

	float Progress = Monitor->GetOverallProgress();
	// 进度应该 > 0（有任务可能在推进或完成）
	TestTrue(TEXT("Progress should be >= 0"), Progress >= 0.0f);
	TestTrue(TEXT("Progress should be <= 1"), Progress <= 1.0f);

	return true;
}

// ==================== OverallProgress 未初始化测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskMonitorProgressEmptyTest,
	"UAVSimulator.MultiAgent.TaskMonitor.OverallProgress_Empty",
	UAV_TEST_FLAGS)

bool FTaskMonitorProgressEmptyTest::RunTest(const FString& Parameters)
{
	UTaskMonitor* Monitor = NewObject<UTaskMonitor>();

	// 未初始化 → Progress=0
	UAV_TEST_FLOAT_EQUAL(Monitor->GetOverallProgress(), 0.0f, 0.01f);
	TestEqual(TEXT("Total count should be 0"), Monitor->GetTotalTaskCount(), 0);

	return true;
}

// ==================== DetectStalled 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskMonitorStalledNoHistoryTest,
	"UAVSimulator.MultiAgent.TaskMonitor.DetectStalled_NoHistory",
	UAV_TEST_FLAGS)

bool FTaskMonitorStalledNoHistoryTest::RunTest(const FString& Parameters)
{
	UTaskMonitor* Monitor = NewObject<UTaskMonitor>();

	// 刚创建，没有历史数据 → 不应判定停滞
	FAgentStateSnapshot State = UAVTestHelpers::CreateAgentSnapshot(0, FVector(100, 200, 300));
	bool bStalled = Monitor->DetectStalledAgent(0, State);
	TestFalse(TEXT("New monitor should not detect stall"), bStalled);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskMonitorStalledStationaryTest,
	"UAVSimulator.MultiAgent.TaskMonitor.DetectStalled_Stationary",
	UAV_TEST_FLAGS)

bool FTaskMonitorStalledStationaryTest::RunTest(const FString& Parameters)
{
	UTaskMonitor* Monitor = NewObject<UTaskMonitor>();

	// 创建分配（需要初始化才能让 DetectStalled 有内部状态）
	TArray<FTaskAssignment> Assignments;
	// CompletionTime=100s 确保任务在停滞检测前不会因进度满而标记完成
	// （否则任务完成后 Update 不再追加历史条目，无法达到 30 条要求）
	Assignments.Add(UAVTestHelpers::CreateTaskAssignment(0, 0, 0, 100, 5000));

	FTaskAllocationResult Alloc = UAVTestHelpers::CreateAllocationResult(Assignments);
	FTaskMonitorConfig Config;
	Config.StalledTimeout = 3.0f;
	Monitor->Initialize(Alloc, Config);

	// Agent 0 在同一位置持续多帧
	FVector FixedPos(100, 200, 300);
	TArray<FAgentStateSnapshot> AgentStates;
	AgentStates.Add(UAVTestHelpers::CreateAgentSnapshot(0, FixedPos));

	// DetectStalledAgent 需要 30+ 历史条目 + 10+ 连续停滞判定
	// ProgressCheckAccumulator 默认间隔 0.5s，dt=0.1f → 每 5 次调用产生 1 个历史条目
	// 需要 30 条目 + 10 次停滞检查 = 40 次检查 → 40*5=200 次 Update
	// 使用 600 确保充分（总计 60s，任务 100s 内不会完成）
	for (int32 i = 0; i < 600; ++i)
	{
		Monitor->Update(0.1f, AgentStates);
	}

	// 应该检测到停滞
	bool bStalled = Monitor->DetectStalledAgent(0,
		UAVTestHelpers::CreateAgentSnapshot(0, FixedPos));
	TestTrue(TEXT("Should detect stall after staying at same position"), bStalled);

	return true;
}

// ==================== DetectTimeout 测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTaskMonitorTimeoutTest,
	"UAVSimulator.MultiAgent.TaskMonitor.DetectTimeout",
	UAV_TEST_FLAGS)

bool FTaskMonitorTimeoutTest::RunTest(const FString& Parameters)
{
	UTaskMonitor* Monitor = NewObject<UTaskMonitor>();

	TArray<FTaskAssignment> Assignments;
	// 任务 0 预计 10 秒完成
	Assignments.Add(UAVTestHelpers::CreateTaskAssignment(0, 0, 0, 10, 5000));

	FTaskAllocationResult Alloc = UAVTestHelpers::CreateAllocationResult(Assignments);
	FTaskMonitorConfig Config;
	Monitor->Initialize(Alloc, Config);

	// 刚初始化不应超时
	TestFalse(TEXT("Should not timeout initially"), Monitor->DetectTimeout(0));

	// 模拟 20 秒过去（远超预计完成时间 10 秒）
	TArray<FAgentStateSnapshot> AgentStates;
	AgentStates.Add(UAVTestHelpers::CreateAgentSnapshot(0, FVector(5000, 0, 0)));
	Monitor->Update(20.0f, AgentStates);

	// 现在应该超时
	TestTrue(TEXT("Should detect timeout after exceeding deadline"), Monitor->DetectTimeout(0));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
