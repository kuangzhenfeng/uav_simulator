// Copyright Epic Games, Inc. All Rights Reserved.

#include "ScenarioEvaluator.h"
#include "../uav_simulator.h"
#include "../Core/UAVPawn.h"
#include "../Mission/MissionComponent.h"
#include "../Planning/ObstacleManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"

DEFINE_LOG_CATEGORY_STATIC(LogScenarioEval, Log, All);

UScenarioEvaluator::UScenarioEvaluator()
{
}

FScenarioVerdict UScenarioEvaluator::Evaluate(const FScenarioMetrics& Metrics, const UAcceptanceCriteria* Criteria)
{
	FScenarioVerdict Verdict;
	Verdict.Metrics = Metrics;

	if (!Criteria)
	{
		// fail-closed：没有验收标准即无法证明场景成功，默认判定 FAIL。
		// 否则任务失败（原地震荡、0 航点到达等）会被误报为 PASS。
		Verdict.Failures.Add(TEXT("NoAcceptanceCriteria"));
		if (Metrics.bCollided)
		{
			Verdict.Failures.Add(TEXT("Collision"));
		}
		Verdict.bPassed = false;
		return Verdict;
	}

	// 逐项对照阈值（顺序即失败项列出的优先级）

	// 1. 碰撞（若有碰撞记录，硬失败）
	if (Metrics.bCollided)
	{
		Verdict.Failures.Add(TEXT("Collision"));
	}

	// 2. 航点到达
	if (Criteria->bRequireAllWaypoints && Metrics.WaypointsTotal > 0)
	{
		if (Metrics.WaypointsReached < Metrics.WaypointsTotal)
		{
			Verdict.Failures.Add(FString::Printf(TEXT("WaypointsNotReached(%d/%d)"),
				Metrics.WaypointsReached, Metrics.WaypointsTotal));
		}
	}

	// 3. 最小净空（>0 阈值才检查；FLT_MAX 表示从未接近障碍）
	if (Criteria->MinClearanceCm > 0.0f && Metrics.MinClearanceCm < Criteria->MinClearanceCm)
	{
		Verdict.Failures.Add(FString::Printf(TEXT("Clearance(%.0f<%.0f)"),
			Metrics.MinClearanceCm, Criteria->MinClearanceCm));
	}

	// 4. 横向偏差（>0 阈值才检查）
	if (Criteria->MaxLateralDeviationCm > 0.0f && Metrics.MaxLateralDeviationCm > Criteria->MaxLateralDeviationCm)
	{
		Verdict.Failures.Add(FString::Printf(TEXT("LateralDeviation(%.0f>%.0f)"),
			Metrics.MaxLateralDeviationCm, Criteria->MaxLateralDeviationCm));
	}

	// 5. 超时（>0 阈值才检查）
	if (Criteria->TimeoutSeconds > 0.0f && Metrics.ElapsedSec > Criteria->TimeoutSeconds)
	{
		Verdict.Failures.Add(FString::Printf(TEXT("Timeout(%.1f>%.1f)"),
			Metrics.ElapsedSec, Criteria->TimeoutSeconds));
	}

	// 6. 能耗预算（>0 阈值才检查；0 表示不限制）
	if (Criteria->EnergyBudget > 0.0f && Metrics.EnergyBudgetUsed > Criteria->EnergyBudget)
	{
		Verdict.Failures.Add(FString::Printf(TEXT("Energy(%.2f>%.2f)"),
			Metrics.EnergyBudgetUsed, Criteria->EnergyBudget));
	}

	Verdict.bPassed = (Verdict.Failures.Num() == 0);
	return Verdict;
}

bool UScenarioEvaluator::WriteResultJson(const FString& ScenarioName, int32 Seed,
	const FScenarioVerdict& Verdict, const FString& FilePath)
{	const FString Path = FilePath.IsEmpty()
		? FPaths::Combine(FPaths::ProjectDir(), TEXT("Logs/scenario_result.json"))
		: FilePath;

	const FString VerdictStr = Verdict.bPassed ? TEXT("PASS") : TEXT("FAIL");

	TArray<FString> FailureLines;
	for (const FString& F : Verdict.Failures)
	{
		FailureLines.Add(FString::Printf(TEXT("    \"%s\""), *F.ReplaceQuotesWithEscapedQuotes()));
	}
	const FString FailuresBlock = FailureLines.Num() > 0
		? FString::Join(FailureLines, TEXT(",\n"))
		: TEXT("");

	const FString Json = FString::Printf(TEXT(
		"{\n"
		"  \"scenario\": \"%s\",\n"
		"  \"seed\": %d,\n"
		"  \"verdict\": \"%s\",\n"
		"  \"metrics\": {\n"
		"    \"waypointsReached\": %d,\n"
		"    \"waypointsTotal\": %d,\n"
		"    \"minClearanceCm\": %.2f,\n"
		"    \"maxLateralDevCm\": %.2f,\n"
		"    \"elapsedSec\": %.2f,\n"
		"    \"energyBudgetUsed\": %.4f,\n"
		"    \"collided\": %s\n"
		"  },\n"
		"  \"failures\": [\n%s\n  ]\n"
		"}\n"),
		*ScenarioName.ReplaceQuotesWithEscapedQuotes(),
		Seed,
		*VerdictStr,
		Verdict.Metrics.WaypointsReached,
		Verdict.Metrics.WaypointsTotal,
		Verdict.Metrics.MinClearanceCm,
		Verdict.Metrics.MaxLateralDeviationCm,
		Verdict.Metrics.ElapsedSec,
		Verdict.Metrics.EnergyBudgetUsed,
		Verdict.Metrics.bCollided ? TEXT("true") : TEXT("false"),
		*FailuresBlock);

	// 确保目录存在
	FPlatformFileManager::Get().GetPlatformFile().CreateDirectoryTree(*FPaths::GetPath(Path));

	const bool bOk = FFileHelper::SaveStringToFile(Json, *Path);
	if (bOk)
	{
		UE_LOG(LogScenarioEval, Log, TEXT("[Scenario] RESULT=%s | Reached=%d/%d | Clearance=%.0fcm | LateralDev=%.0fcm | Elapsed=%.1fs | %s"),
			Verdict.bPassed ? TEXT("PASS") : TEXT("FAIL"),
			Verdict.Metrics.WaypointsReached, Verdict.Metrics.WaypointsTotal,
			Verdict.Metrics.MinClearanceCm, Verdict.Metrics.MaxLateralDeviationCm,
			Verdict.Metrics.ElapsedSec,
			Verdict.Failures.Num() > 0 ? *FString::Join(Verdict.Failures, TEXT(", ")) : TEXT("all checks passed"));
	}
	else
	{
		UE_LOG(LogScenarioEval, Error, TEXT("[Scenario] Failed to write result JSON to %s"), *Path);
	}
	return bOk;
}

// ==================== UScenarioEvaluatorComponent ====================

UScenarioEvaluatorComponent::UScenarioEvaluatorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void UScenarioEvaluatorComponent::Initialize(UScenario* InScenario, AUAVPawn* InLeadUAV)
{
	Scenario = InScenario;
	LeadUAV = InLeadUAV;

	if (Scenario)
	{
		Criteria = Scenario->AcceptanceCriteria.LoadSynchronous();
		if (!Criteria)
		{
			UE_LOG(LogScenarioEval, Warning,
				TEXT("[Scenario] No AcceptanceCriteria on scenario '%s' — verdict will fail-closed"),
				*Scenario->Name);
		}
		else
		{
			// 初始化时打印验收配置摘要，便于核对场景资产的实际阈值
			UE_LOG(LogScenarioEval, Log,
				TEXT("[Scenario] AcceptanceCriteria: RequireAllWaypoints=%d ArrivalRadius=%.0fcm MinClearance=%.0fcm MaxLateralDev=%.0fcm Timeout=%.1fs EnergyBudget=%.2f"),
				Criteria->bRequireAllWaypoints ? 1 : 0,
				Criteria->WaypointArrivalRadius,
				Criteria->MinClearanceCm,
				Criteria->MaxLateralDeviationCm,
				Criteria->TimeoutSeconds,
				Criteria->EnergyBudget);
		}
	}

	// 订阅机队任务完成/失败委托
	if (LeadUAV)
	{
		if (UMissionComponent* Mission = LeadUAV->FindComponentByClass<UMissionComponent>())
		{
			Mission->OnMissionCompleted.AddDynamic(this, &UScenarioEvaluatorComponent::HandleMissionCompleted);
			Mission->OnMissionFailed.AddDynamic(this, &UScenarioEvaluatorComponent::HandleMissionFailed);

			if (Criteria)
			{
				Accumulated.WaypointsTotal = Mission->GetWaypointCount();
			}
		}
	}
}

FScenarioMetrics UScenarioEvaluatorComponent::CollectMetrics() const
{
	FScenarioMetrics M = Accumulated;
	M.ElapsedSec = ElapsedTime;

	if (LeadUAV)
	{
		// 炸机检测：lead UAV 进入 Crashed 状态即视为场景级碰撞硬失败
		if (LeadUAV->IsCrashed())
		{
			M.bCollided = true;
		}

		// 航点到达数：从 MissionComponent 状态读取
		if (UMissionComponent* Mission = LeadUAV->FindComponentByClass<UMissionComponent>())
		{
			M.WaypointsReached = Mission->GetCurrentWaypointIndex();
			if (M.WaypointsTotal == 0)
			{
				M.WaypointsTotal = Mission->GetWaypointCount();
			}
		}

		// 净空：到最近障碍的表面距离
		if (UObstacleManager* Obs = LeadUAV->FindComponentByClass<UObstacleManager>())
		{
			FObstacleInfo Nearest;
			const float Dist = Obs->GetDistanceToNearestObstacle(LeadUAV->GetActorLocation(), Nearest);
			if (Dist >= 0.0f && Dist < M.MinClearanceCm)
			{
				M.MinClearanceCm = Dist;
			}
		}
	}

	return M;
}

void UScenarioEvaluatorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!Scenario)
	{
		return;
	}

	ElapsedTime += DeltaTime;

	// 更新累积指标
	FScenarioMetrics Current = CollectMetrics();
	Accumulated.MinClearanceCm = FMath::Min(Accumulated.MinClearanceCm, Current.MinClearanceCm);
	Accumulated.MaxLateralDeviationCm = FMath::Max(Accumulated.MaxLateralDeviationCm, Current.MaxLateralDeviationCm);
	Accumulated.WaypointsReached = Current.WaypointsReached;
	Accumulated.WaypointsTotal = Current.WaypointsTotal;

	// 周期快照写 JSON（pkill 强杀时留最近一次）
	SnapshotAccumulator += DeltaTime;
	if (SnapshotAccumulator >= SnapshotInterval)
	{
		SnapshotAccumulator = 0.0f;

		FScenarioMetrics Snapshot = Accumulated;
		Snapshot.ElapsedSec = ElapsedTime;

		const FScenarioVerdict Verdict = UScenarioEvaluator::Evaluate(Snapshot, Criteria);
		UScenarioEvaluator::WriteResultJson(Scenario->Name, Scenario->RandomSeed, Verdict);
	}
}

void UScenarioEvaluatorComponent::FlushFinalResult()
{
	if (bFinalFlushed || !Scenario)
	{
		return;
	}
	bFinalFlushed = true;

	FScenarioMetrics Final = Accumulated;
	Final.ElapsedSec = ElapsedTime;

	const FScenarioVerdict Verdict = UScenarioEvaluator::Evaluate(Final, Criteria);
	UScenarioEvaluator::WriteResultJson(Scenario->Name, Scenario->RandomSeed, Verdict);
}

void UScenarioEvaluatorComponent::HandleMissionCompleted(bool bSuccess)
{
	// 任务完成：若成功到达全部航点则视为达到终点，写最终结果
	if (bSuccess)
	{
		Accumulated.WaypointsReached = Accumulated.WaypointsTotal;
	}
	FlushFinalResult();
}

void UScenarioEvaluatorComponent::HandleMissionFailed(FString Reason)
{
	// 任务失败：直接写最终结果（多为 FAIL）
	FlushFinalResult();
}
