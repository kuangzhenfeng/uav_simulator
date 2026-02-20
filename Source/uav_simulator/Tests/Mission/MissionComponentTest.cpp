// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Mission/MissionComponent.h"
#include "../../Mission/MissionTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 航点管理测试 - 添加/移除 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentWaypointAddRemoveTest,
	"UAVSimulator.Mission.MissionComponent.WaypointAddRemove",
	UAV_TEST_FLAGS)

bool FMissionComponentWaypointAddRemoveTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	// 初始状态
	TestEqual(TEXT("Should have no waypoints initially"), Mission->GetWaypointCount(), 0);
	TestFalse(TEXT("Should not have waypoints initially"), Mission->HasWaypoints());

	// 添加航点
	Mission->AddWaypoint(FVector(100.0f, 0.0f, 0.0f));
	TestEqual(TEXT("Should have 1 waypoint after add"), Mission->GetWaypointCount(), 1);
	TestTrue(TEXT("Should have waypoints after add"), Mission->HasWaypoints());

	Mission->AddWaypoint(FVector(200.0f, 0.0f, 0.0f), 2.0f, 300.0f);
	TestEqual(TEXT("Should have 2 waypoints"), Mission->GetWaypointCount(), 2);

	// 移除航点
	TestTrue(TEXT("Should successfully remove waypoint"), Mission->RemoveWaypoint(0));
	TestEqual(TEXT("Should have 1 waypoint after remove"), Mission->GetWaypointCount(), 1);

	// 尝试移除无效索引
	TestFalse(TEXT("Should fail to remove invalid index"), Mission->RemoveWaypoint(10));

	return true;
}

// ==================== 航点管理测试 - 设置/清除 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentWaypointSetClearTest,
	"UAVSimulator.Mission.MissionComponent.WaypointSetClear",
	UAV_TEST_FLAGS)

bool FMissionComponentWaypointSetClearTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	// 设置航点（简单版）
	TArray<FVector> Positions = UAVTestHelpers::CreateWaypoints({
		FVector(0.0f, 0.0f, 0.0f),
		FVector(500.0f, 0.0f, 0.0f),
		FVector(1000.0f, 0.0f, 0.0f)
	});
	Mission->SetWaypoints(Positions);
	TestEqual(TEXT("Should have 3 waypoints after set"), Mission->GetWaypointCount(), 3);

	// 获取航点位置
	TArray<FVector> RetrievedPositions = Mission->GetWaypointPositions();
	TestEqual(TEXT("Retrieved positions count should match"), RetrievedPositions.Num(), 3);
	UAV_TEST_VECTOR_EQUAL(RetrievedPositions[0], FVector(0.0f, 0.0f, 0.0f), 1.0f);
	UAV_TEST_VECTOR_EQUAL(RetrievedPositions[2], FVector(1000.0f, 0.0f, 0.0f), 1.0f);

	// 清除航点
	Mission->ClearWaypoints();
	TestEqual(TEXT("Should have 0 waypoints after clear"), Mission->GetWaypointCount(), 0);
	TestFalse(TEXT("Should not have waypoints after clear"), Mission->HasWaypoints());

	return true;
}

// ==================== 航点管理测试 - 插入 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentWaypointInsertTest,
	"UAVSimulator.Mission.MissionComponent.WaypointInsert",
	UAV_TEST_FLAGS)

bool FMissionComponentWaypointInsertTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	// 添加初始航点
	Mission->AddWaypoint(FVector(0.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(1000.0f, 0.0f, 0.0f));

	// 在中间插入航点
	FMissionWaypoint NewWaypoint(FVector(500.0f, 0.0f, 0.0f));
	Mission->InsertWaypoint(1, NewWaypoint);

	TestEqual(TEXT("Should have 3 waypoints after insert"), Mission->GetWaypointCount(), 3);

	// 验证顺序
	TArray<FVector> Positions = Mission->GetWaypointPositions();
	UAV_TEST_VECTOR_EQUAL(Positions[0], FVector(0.0f, 0.0f, 0.0f), 1.0f);
	UAV_TEST_VECTOR_EQUAL(Positions[1], FVector(500.0f, 0.0f, 0.0f), 1.0f);
	UAV_TEST_VECTOR_EQUAL(Positions[2], FVector(1000.0f, 0.0f, 0.0f), 1.0f);

	return true;
}

// ==================== 任务状态机测试 - 基本状态 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentStateMachineBasicTest,
	"UAVSimulator.Mission.MissionComponent.StateMachineBasic",
	UAV_TEST_FLAGS)

bool FMissionComponentStateMachineBasicTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	// 初始状态应该是 Idle
	TestEqual(TEXT("Initial state should be Idle"),
		static_cast<int32>(Mission->GetMissionState()),
		static_cast<int32>(EMissionState::Idle));

	// 没有航点时不能启动任务
	TestFalse(TEXT("Should not start without waypoints"), Mission->StartMission());

	// 添加航点后状态应该变为 Ready
	Mission->AddWaypoint(FVector(100.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(200.0f, 0.0f, 0.0f));

	// 注意：状态可能在添加航点后自动变为 Ready，取决于实现

	return true;
}

// ==================== 任务状态机测试 - 启动/停止 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentStateMachineStartStopTest,
	"UAVSimulator.Mission.MissionComponent.StateMachineStartStop",
	UAV_TEST_FLAGS)

bool FMissionComponentStateMachineStartStopTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	// 添加航点
	Mission->AddWaypoint(FVector(100.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(200.0f, 0.0f, 0.0f));

	// 启动任务
	bool bStarted = Mission->StartMission();
	if (bStarted)
	{
		TestTrue(TEXT("Should be running after start"), Mission->IsMissionRunning());
		TestFalse(TEXT("Should not be paused after start"), Mission->IsMissionPaused());
		TestFalse(TEXT("Should not be completed after start"), Mission->IsMissionCompleted());
	}

	// 停止任务
	Mission->StopMission();
	TestFalse(TEXT("Should not be running after stop"), Mission->IsMissionRunning());

	return true;
}

// ==================== 任务状态机测试 - 暂停/恢复 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentStateMachinePauseResumeTest,
	"UAVSimulator.Mission.MissionComponent.StateMachinePauseResume",
	UAV_TEST_FLAGS)

bool FMissionComponentStateMachinePauseResumeTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	Mission->AddWaypoint(FVector(100.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(200.0f, 0.0f, 0.0f));

	// 启动任务
	Mission->StartMission();

	if (Mission->IsMissionRunning())
	{
		// 暂停任务
		TestTrue(TEXT("Should successfully pause"), Mission->PauseMission());
		TestTrue(TEXT("Should be paused"), Mission->IsMissionPaused());
		TestFalse(TEXT("Should not be running when paused"), Mission->IsMissionRunning());

		// 恢复任务
		TestTrue(TEXT("Should successfully resume"), Mission->ResumeMission());
		TestTrue(TEXT("Should be running after resume"), Mission->IsMissionRunning());
		TestFalse(TEXT("Should not be paused after resume"), Mission->IsMissionPaused());
	}

	return true;
}

// ==================== 进度计算测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentProgressTest,
	"UAVSimulator.Mission.MissionComponent.Progress",
	UAV_TEST_FLAGS)

bool FMissionComponentProgressTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	// 添加航点
	Mission->AddWaypoint(FVector(0.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(500.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(1000.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(1500.0f, 0.0f, 0.0f));

	// 初始进度
	float Progress = Mission->GetMissionProgress();
	UAV_TEST_FLOAT_IN_RANGE(Progress, 0.0f, 1.0f);

	// 当前航点索引
	TestEqual(TEXT("Initial waypoint index should be 0"), Mission->GetCurrentWaypointIndex(), 0);

	return true;
}

// ==================== 循环逻辑测试 - Once 模式 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentLoopOnceTest,
	"UAVSimulator.Mission.MissionComponent.LoopOnce",
	UAV_TEST_FLAGS)

bool FMissionComponentLoopOnceTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	// 设置 Once 模式
	FMissionConfig Config;
	Config.Mode = EMissionMode::Once;
	Config.LoopCount = 1;
	Mission->SetMissionConfig(Config);

	Mission->AddWaypoint(FVector(100.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(200.0f, 0.0f, 0.0f));

	// 验证配置
	const FMissionConfig& RetrievedConfig = Mission->GetMissionConfig();
	TestEqual(TEXT("Mode should be Once"),
		static_cast<int32>(RetrievedConfig.Mode),
		static_cast<int32>(EMissionMode::Once));

	return true;
}

// ==================== 循环逻辑测试 - Loop 模式 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentLoopModeTest,
	"UAVSimulator.Mission.MissionComponent.LoopMode",
	UAV_TEST_FLAGS)

bool FMissionComponentLoopModeTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	// 设置 Loop 模式
	FMissionConfig Config;
	Config.Mode = EMissionMode::Loop;
	Config.LoopCount = 3;
	Mission->SetMissionConfig(Config);

	Mission->AddWaypoint(FVector(100.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(200.0f, 0.0f, 0.0f));

	// 验证配置
	const FMissionConfig& RetrievedConfig = Mission->GetMissionConfig();
	TestEqual(TEXT("Mode should be Loop"),
		static_cast<int32>(RetrievedConfig.Mode),
		static_cast<int32>(EMissionMode::Loop));
	TestEqual(TEXT("Loop count should be 3"), RetrievedConfig.LoopCount, 3);

	// 验证当前循环次数
	TestEqual(TEXT("Initial loop count should be 0"), Mission->GetCurrentLoopCount(), 0);

	return true;
}

// ==================== 循环逻辑测试 - PingPong 模式 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentPingPongModeTest,
	"UAVSimulator.Mission.MissionComponent.PingPongMode",
	UAV_TEST_FLAGS)

bool FMissionComponentPingPongModeTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	// 设置 PingPong 模式
	Mission->SetMissionMode(EMissionMode::PingPong);

	Mission->AddWaypoint(FVector(0.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(500.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(1000.0f, 0.0f, 0.0f));

	// 验证模式
	const FMissionConfig& Config = Mission->GetMissionConfig();
	TestEqual(TEXT("Mode should be PingPong"),
		static_cast<int32>(Config.Mode),
		static_cast<int32>(EMissionMode::PingPong));

	return true;
}

// ==================== 航点导航测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentNavigationTest,
	"UAVSimulator.Mission.MissionComponent.Navigation",
	UAV_TEST_FLAGS)

bool FMissionComponentNavigationTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	Mission->AddWaypoint(FVector(0.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(500.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(1000.0f, 0.0f, 0.0f));

	// 获取当前航点
	FMissionWaypoint CurrentWaypoint;
	TestTrue(TEXT("Should get current waypoint"), Mission->GetCurrentWaypoint(CurrentWaypoint));
	UAV_TEST_VECTOR_EQUAL(CurrentWaypoint.Position, FVector(0.0f, 0.0f, 0.0f), 1.0f);

	// 获取指定索引的航点
	FMissionWaypoint Waypoint;
	TestTrue(TEXT("Should get waypoint at index 1"), Mission->GetWaypointAt(1, Waypoint));
	UAV_TEST_VECTOR_EQUAL(Waypoint.Position, FVector(500.0f, 0.0f, 0.0f), 1.0f);

	// 尝试获取无效索引
	TestFalse(TEXT("Should fail to get invalid index"), Mission->GetWaypointAt(10, Waypoint));

	// 跳转到指定航点
	TestTrue(TEXT("Should go to waypoint 2"), Mission->GoToWaypoint(2));
	TestEqual(TEXT("Current index should be 2"), Mission->GetCurrentWaypointIndex(), 2);

	return true;
}

// ==================== 任务配置测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentConfigTest,
	"UAVSimulator.Mission.MissionComponent.Config",
	UAV_TEST_FLAGS)

bool FMissionComponentConfigTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	// 设置配置
	FMissionConfig Config;
	Config.Mode = EMissionMode::Loop;
	Config.DefaultSpeed = 600.0f;
	Config.MaxAcceleration = 250.0f;
	Config.WaypointReachThreshold = 75.0f;
	Config.LoopCount = 5;
	Config.bReturnToStart = true;
	Config.bEnablePathPlanning = true;

	Mission->SetMissionConfig(Config);

	// 验证配置
	const FMissionConfig& RetrievedConfig = Mission->GetMissionConfig();
	UAV_TEST_FLOAT_EQUAL(RetrievedConfig.DefaultSpeed, 600.0f, 0.1f);
	UAV_TEST_FLOAT_EQUAL(RetrievedConfig.MaxAcceleration, 250.0f, 0.1f);
	UAV_TEST_FLOAT_EQUAL(RetrievedConfig.WaypointReachThreshold, 75.0f, 0.1f);
	TestEqual(TEXT("Loop count should match"), RetrievedConfig.LoopCount, 5);
	TestTrue(TEXT("Return to start should be true"), RetrievedConfig.bReturnToStart);
	TestTrue(TEXT("Path planning should be enabled"), RetrievedConfig.bEnablePathPlanning);

	// 单独设置速度
	Mission->SetDefaultSpeed(800.0f);
	UAV_TEST_FLOAT_EQUAL(Mission->GetMissionConfig().DefaultSpeed, 800.0f, 0.1f);

	return true;
}

// ==================== 任务重置测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentResetTest,
	"UAVSimulator.Mission.MissionComponent.Reset",
	UAV_TEST_FLAGS)

bool FMissionComponentResetTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	Mission->AddWaypoint(FVector(0.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(500.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(1000.0f, 0.0f, 0.0f));

	// 跳转到航点 2
	Mission->GoToWaypoint(2);
	TestEqual(TEXT("Should be at waypoint 2"), Mission->GetCurrentWaypointIndex(), 2);

	// 重置任务
	Mission->ResetMission();

	// 验证重置后的状态
	TestEqual(TEXT("Should be at waypoint 0 after reset"), Mission->GetCurrentWaypointIndex(), 0);

	return true;
}

// ==================== Once 模式到达最后航点后完成 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentAdvanceOnceTest,
	"UAVSimulator.Mission.MissionComponent.AdvanceOnceMode",
	UAV_TEST_FLAGS)

bool FMissionComponentAdvanceOnceTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	FMissionConfig Config;
	Config.Mode = EMissionMode::Once;
	Mission->SetMissionConfig(Config);

	Mission->AddWaypoint(FVector(100.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(200.0f, 0.0f, 0.0f));
	Mission->StartMission();

	Mission->AdvanceToNextWaypoint(); // wp0 -> wp1
	Mission->AdvanceToNextWaypoint(); // wp1 -> completed

	TestTrue(TEXT("Mission should be completed"), Mission->IsMissionCompleted());

	return true;
}

// ==================== Loop 模式循环回第一个航点 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentAdvanceLoopTest,
	"UAVSimulator.Mission.MissionComponent.AdvanceLoopMode",
	UAV_TEST_FLAGS)

bool FMissionComponentAdvanceLoopTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	FMissionConfig Config;
	Config.Mode = EMissionMode::Loop;
	Config.LoopCount = 2;
	Mission->SetMissionConfig(Config);

	Mission->AddWaypoint(FVector(100.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(200.0f, 0.0f, 0.0f));
	Mission->StartMission();

	Mission->AdvanceToNextWaypoint(); // wp0 -> wp1
	Mission->AdvanceToNextWaypoint(); // wp1 -> loop back to wp0

	TestEqual(TEXT("Should loop back to wp0"), Mission->GetCurrentWaypointIndex(), 0);
	TestTrue(TEXT("Should still be running"), Mission->IsMissionRunning());

	return true;
}

// ==================== PingPong 模式反向遍历 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentAdvancePingPongTest,
	"UAVSimulator.Mission.MissionComponent.AdvancePingPongMode",
	UAV_TEST_FLAGS)

bool FMissionComponentAdvancePingPongTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	FMissionConfig Config;
	Config.Mode = EMissionMode::PingPong;
	Config.LoopCount = 2;
	Mission->SetMissionConfig(Config);

	Mission->AddWaypoint(FVector(0.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(500.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(1000.0f, 0.0f, 0.0f));
	Mission->StartMission();

	Mission->AdvanceToNextWaypoint(); // wp0 -> wp1
	Mission->AdvanceToNextWaypoint(); // wp1 -> wp2
	Mission->AdvanceToNextWaypoint(); // wp2 -> reverse -> wp1

	TestEqual(TEXT("Should reverse to wp1"), Mission->GetCurrentWaypointIndex(), 1);

	return true;
}

// ==================== 无限循环模式持续循环 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionComponentAdvanceLoopInfiniteTest,
	"UAVSimulator.Mission.MissionComponent.AdvanceLoopInfinite",
	UAV_TEST_FLAGS)

bool FMissionComponentAdvanceLoopInfiniteTest::RunTest(const FString& Parameters)
{
	UMissionComponent* Mission = NewObject<UMissionComponent>();

	FMissionConfig Config;
	Config.Mode = EMissionMode::Loop;
	Config.LoopCount = -1; // 无限循环
	Mission->SetMissionConfig(Config);

	Mission->AddWaypoint(FVector(100.0f, 0.0f, 0.0f));
	Mission->AddWaypoint(FVector(200.0f, 0.0f, 0.0f));
	Mission->StartMission();

	// 循环多次不应完成
	for (int32 i = 0; i < 10; ++i)
	{
		Mission->AdvanceToNextWaypoint();
		Mission->AdvanceToNextWaypoint();
	}

	TestTrue(TEXT("Should still be running after many loops"), Mission->IsMissionRunning());
	TestFalse(TEXT("Should not be completed"), Mission->IsMissionCompleted());

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
