// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Scenario/ScenarioEvaluator.h"
#include "../../Scenario/ScenarioTypes.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 评估：无验收标准 -> fail-closed FAIL ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioEvalNullCriteriaFailTest,
	"UAVSimulator.Scenario.Evaluate.NullCriteriaFailClosed",
	UAV_TEST_FLAGS)

bool FScenarioEvalNullCriteriaFailTest::RunTest(const FString& Parameters)
{
	// 无验收标准时必须 fail-closed：否则任务失败（0 航点、原地震荡）
	// 会被误报为 PASS。
	FScenarioMetrics Metrics;
	Metrics.WaypointsReached = 0;
	Metrics.WaypointsTotal = 1;
	Metrics.bCollided = false;

	const FScenarioVerdict V = UScenarioEvaluator::Evaluate(Metrics, nullptr);
	TestFalse(TEXT("无验收标准 -> fail-closed FAIL"), V.bPassed);
	TestTrue(TEXT("失败项含 NoAcceptanceCriteria"),
		V.Failures.ContainsByPredicate([](const FString& F) { return F.Contains(TEXT("NoAcceptanceCriteria")); }));

	return true;
}

// ==================== AcceptanceCriteria 评估：全 PASS ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioEvalAllPassTest,
	"UAVSimulator.Scenario.Evaluate.AllPass",
	UAV_TEST_FLAGS)

bool FScenarioEvalAllPassTest::RunTest(const FString& Parameters)
{
	UAcceptanceCriteria* Criteria = NewObject<UAcceptanceCriteria>();
	Criteria->bRequireAllWaypoints = true;
	Criteria->MinClearanceCm = 100.0f;
	Criteria->MaxLateralDeviationCm = 300.0f;
	Criteria->TimeoutSeconds = 60.0f;
	Criteria->EnergyBudget = 0.0f; // 不限制

	FScenarioMetrics Metrics;
	Metrics.WaypointsReached = 3;
	Metrics.WaypointsTotal = 3;
	Metrics.MinClearanceCm = 250.0f;    // > 100
	Metrics.MaxLateralDeviationCm = 180.0f; // < 300
	Metrics.ElapsedSec = 12.0f;          // < 60
	Metrics.bCollided = false;

	const FScenarioVerdict V = UScenarioEvaluator::Evaluate(Metrics, Criteria);
	TestTrue(TEXT("全 PASS"), V.bPassed);
	TestEqual(TEXT("无失败项"), V.Failures.Num(), 0);

	return true;
}

// ==================== 评估：航点未全部到达 -> FAIL ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioEvalWaypointsFailTest,
	"UAVSimulator.Scenario.Evaluate.WaypointsFail",
	UAV_TEST_FLAGS)

bool FScenarioEvalWaypointsFailTest::RunTest(const FString& Parameters)
{
	UAcceptanceCriteria* Criteria = NewObject<UAcceptanceCriteria>();
	Criteria->bRequireAllWaypoints = true;

	FScenarioMetrics Metrics;
	Metrics.WaypointsReached = 2;
	Metrics.WaypointsTotal = 3;

	const FScenarioVerdict V = UScenarioEvaluator::Evaluate(Metrics, Criteria);
	TestFalse(TEXT("航点未全部到达 -> FAIL"), V.bPassed);
	TestTrue(TEXT("失败项含 WaypointsNotReached"),
		V.Failures.ContainsByPredicate([](const FString& F) { return F.Contains(TEXT("WaypointsNotReached")); }));

	return true;
}

// ==================== 评估：净空不足 -> FAIL ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioEvalClearanceFailTest,
	"UAVSimulator.Scenario.Evaluate.ClearanceFail",
	UAV_TEST_FLAGS)

bool FScenarioEvalClearanceFailTest::RunTest(const FString& Parameters)
{
	UAcceptanceCriteria* Criteria = NewObject<UAcceptanceCriteria>();
	Criteria->MinClearanceCm = 200.0f;

	FScenarioMetrics Metrics;
	Metrics.MinClearanceCm = 80.0f; // < 200

	const FScenarioVerdict V = UScenarioEvaluator::Evaluate(Metrics, Criteria);
	TestFalse(TEXT("净空不足 -> FAIL"), V.bPassed);
	TestTrue(TEXT("失败项含 Clearance"), V.Failures.ContainsByPredicate(
		[](const FString& F) { return F.Contains(TEXT("Clearance")); }));

	return true;
}

// ==================== 评估：碰撞 -> 硬失败 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioEvalCollisionFailTest,
	"UAVSimulator.Scenario.Evaluate.CollisionFail",
	UAV_TEST_FLAGS)

bool FScenarioEvalCollisionFailTest::RunTest(const FString& Parameters)
{
	UAcceptanceCriteria* Criteria = NewObject<UAcceptanceCriteria>();

	FScenarioMetrics Metrics;
	Metrics.bCollided = true;

	const FScenarioVerdict V = UScenarioEvaluator::Evaluate(Metrics, Criteria);
	TestFalse(TEXT("碰撞 -> FAIL"), V.bPassed);
	TestTrue(TEXT("失败项含 Collision"), V.Failures.Contains(TEXT("Collision")));

	return true;
}

// ==================== 评估：超时 + 偏差双失败 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioEvalMultipleFailTest,
	"UAVSimulator.Scenario.Evaluate.MultipleFail",
	UAV_TEST_FLAGS)

bool FScenarioEvalMultipleFailTest::RunTest(const FString& Parameters)
{
	UAcceptanceCriteria* Criteria = NewObject<UAcceptanceCriteria>();
	Criteria->MaxLateralDeviationCm = 200.0f;
	Criteria->TimeoutSeconds = 30.0f;

	FScenarioMetrics Metrics;
	Metrics.MaxLateralDeviationCm = 450.0f; // 超偏差
	Metrics.ElapsedSec = 55.0f;              // 超时

	const FScenarioVerdict V = UScenarioEvaluator::Evaluate(Metrics, Criteria);
	TestFalse(TEXT("双失败 -> FAIL"), V.bPassed);
	TestEqual(TEXT("两项失败"), V.Failures.Num(), 2);

	return true;
}

// ==================== 评估：能耗预算超限 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioEvalEnergyFailTest,
	"UAVSimulator.Scenario.Evaluate.EnergyFail",
	UAV_TEST_FLAGS)

bool FScenarioEvalEnergyFailTest::RunTest(const FString& Parameters)
{
	UAcceptanceCriteria* Criteria = NewObject<UAcceptanceCriteria>();
	Criteria->EnergyBudget = 0.5f;

	FScenarioMetrics Metrics;
	Metrics.EnergyBudgetUsed = 0.8f; // 超预算

	const FScenarioVerdict V = UScenarioEvaluator::Evaluate(Metrics, Criteria);
	TestFalse(TEXT("能耗超限 -> FAIL"), V.bPassed);
	TestTrue(TEXT("失败项含 Energy"), V.Failures.ContainsByPredicate(
		[](const FString& F) { return F.Contains(TEXT("Energy")); }));

	return true;
}

// ==================== WriteResultJson：写入磁盘并内容正确 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioWriteResultJsonTest,
	"UAVSimulator.Scenario.WriteResultJson",
	UAV_TEST_FLAGS)

bool FScenarioWriteResultJsonTest::RunTest(const FString& Parameters)
{
	const FString TempPath = FPaths::CreateTempFilename(*FPaths::ProjectIntermediateDir(), TEXT("ScenarioResult"), TEXT(".json"));

	FScenarioVerdict Verdict;
	Verdict.bPassed = true;
	Verdict.Metrics.WaypointsReached = 3;
	Verdict.Metrics.WaypointsTotal = 3;
	Verdict.Metrics.MinClearanceCm = 250.0f;
	Verdict.Metrics.MaxLateralDeviationCm = 180.0f;
	Verdict.Metrics.ElapsedSec = 12.4f;
	Verdict.Metrics.EnergyBudgetUsed = 0.62f;
	Verdict.Metrics.bCollided = false;

	const bool bWritten = UScenarioEvaluator::WriteResultJson(TEXT("StraightThrough"), 42, Verdict, TempPath);
	TestTrue(TEXT("JSON 写入成功"), bWritten);

	FString Content;
	TestTrue(TEXT("JSON 可读回"), FFileHelper::LoadFileToString(Content, *TempPath));
	TestTrue(TEXT("含 verdict PASS"), Content.Contains(TEXT("\"verdict\": \"PASS\"")));
	TestTrue(TEXT("含场景名"), Content.Contains(TEXT("\"scenario\": \"StraightThrough\"")));
	TestTrue(TEXT("含 seed"), Content.Contains(TEXT("\"seed\": 42")));

	// 清理临时文件
	IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
	PF.DeleteFile(*TempPath);

	return true;
}

// ==================== WriteResultJson：FAIL 时 failures 非空 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioWriteResultJsonFailTest,
	"UAVSimulator.Scenario.WriteResultJsonFail",
	UAV_TEST_FLAGS)

bool FScenarioWriteResultJsonFailTest::RunTest(const FString& Parameters)
{
	const FString TempPath = FPaths::CreateTempFilename(*FPaths::ProjectIntermediateDir(), TEXT("ScenarioResult"), TEXT(".json"));

	FScenarioVerdict Verdict;
	Verdict.bPassed = false;
	Verdict.Failures.Add(TEXT("Collision"));
	Verdict.Failures.Add(TEXT("Timeout(55.0>30.0)"));

	const bool bWritten = UScenarioEvaluator::WriteResultJson(TEXT("FailingScenario"), 7, Verdict, TempPath);
	TestTrue(TEXT("JSON 写入成功"), bWritten);

	FString Content;
	TestTrue(TEXT("JSON 可读回"), FFileHelper::LoadFileToString(Content, *TempPath));
	TestTrue(TEXT("含 verdict FAIL"), Content.Contains(TEXT("\"verdict\": \"FAIL\"")));
	TestTrue(TEXT("含 Collision 失败项"), Content.Contains(TEXT("Collision")));

	IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
	PF.DeleteFile(*TempPath);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
