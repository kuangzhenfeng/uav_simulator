// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "Components/ActorComponent.h"
#include "ScenarioTypes.h"
#include "ScenarioEvaluator.generated.h"

class AUAVPawn;
class UMissionComponent;
class UScenario;
class UAcceptanceCriteria;
class AMultiAgentGameMode;

/**
 * 仿真指标快照（Evaluator 周期采集）。
 * 只含验收所需的可观测数据，不关心内部仿真状态。
 */
USTRUCT(BlueprintType)
struct FScenarioMetrics
{
	GENERATED_BODY()

	/** 已到达航点数 */
	UPROPERTY(BlueprintReadOnly, Category = "Scenario|Metrics")
	int32 WaypointsReached = 0;

	/** 航点总数 */
	UPROPERTY(BlueprintReadOnly, Category = "Scenario|Metrics")
	int32 WaypointsTotal = 0;

	/** 最小净空（障碍表面距离，cm）。FLT_MAX 表示尚未遇到障碍 */
	UPROPERTY(BlueprintReadOnly, Category = "Scenario|Metrics")
	float MinClearanceCm = FLT_MAX;

	/** 最大横向轨迹偏差（cm） */
	UPROPERTY(BlueprintReadOnly, Category = "Scenario|Metrics")
	float MaxLateralDeviationCm = 0.0f;

	/** 已耗时（秒） */
	UPROPERTY(BlueprintReadOnly, Category = "Scenario|Metrics")
	float ElapsedSec = 0.0f;

	/** 能耗预算使用（归一化 0~1）。0 表示未受限/未计算 */
	UPROPERTY(BlueprintReadOnly, Category = "Scenario|Metrics")
	float EnergyBudgetUsed = 0.0f;

	/** 是否发生过碰撞 */
	UPROPERTY(BlueprintReadOnly, Category = "Scenario|Metrics")
	bool bCollided = false;
};

/**
 * 验收结果。
 */
USTRUCT(BlueprintType)
struct FScenarioVerdict
{
	GENERATED_BODY()

	/** 总体判定：PASS / FAIL */
	UPROPERTY(BlueprintReadOnly, Category = "Scenario|Verdict")
	bool bPassed = false;

	/** 触发的失败项（人类可读） */
	UPROPERTY(BlueprintReadOnly, Category = "Scenario|Verdict")
	TArray<FString> Failures;

	/** 判定时的指标快照 */
	UPROPERTY(BlueprintReadOnly, Category = "Scenario|Verdict")
	FScenarioMetrics Metrics;
};

/**
 * 场景验收器（ScenarioEvaluator）。
 *
 * 对照 UScenario.AcceptanceCriteria 判定仿真结果 PASS/FAIL，
 * 周期写 Logs/scenario_result.json（不依赖进程正常退出，pkill 强杀时留最近一次快照）。
 *
 * 判定逻辑是纯函数：Evaluate(Metrics, Criteria) -> Verdict，
 * 可独立单元测试、不依赖 World。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UScenarioEvaluator : public UObject
{
	GENERATED_BODY()

public:
	UScenarioEvaluator();

	/**
	 * 纯函数：按验收标准评估指标快照，返回判决。
	 * 逐项对照阈值：航点到达 / 净空 / 横向偏差 / 超时 / 能耗 / 碰撞。
	 *
	 * @param Metrics  当前指标快照
	 * @param Criteria 验收标准
	 * @return 判决（PASS/FAIL + 失败项 + 指标）
	 */
	UFUNCTION(BlueprintCallable, Category = "Scenario")
	static FScenarioVerdict Evaluate(const FScenarioMetrics& Metrics, const UAcceptanceCriteria* Criteria);

	/**
	 * 把判决序列化为 scenario_result.json 并写入磁盘。
	 * @param ScenarioName 场景名（写入 JSON 的 scenario 字段）
	 * @param Seed         随机种子
	 * @param Verdict      判决
	 * @param FilePath     输出路径（默认 Logs/scenario_result.json）
	 * @return 是否写入成功
	 */
	UFUNCTION(BlueprintCallable, Category = "Scenario")
	static bool WriteResultJson(const FString& ScenarioName, int32 Seed,
		const FScenarioVerdict& Verdict, const FString& FilePath = TEXT(""));
};

/**
 * 场景验收器运行时组件。
 *
 * 挂在 GameMode 上，周期采集机队指标快照，对照 ActiveScenario.AcceptanceCriteria
 * 判定 PASS/FAIL，覆盖式写 Logs/scenario_result.json。
 * 周期快照是关键设计：不依赖进程正常退出，pkill 强杀时磁盘留最近一次快照。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UScenarioEvaluatorComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UScenarioEvaluatorComponent();

	/** 初始化：绑定场景资产与机队，订阅任务完成委托，启动周期快照 */
	void Initialize(UScenario* InScenario, AUAVPawn* InLeadUAV);

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
	/** 采集当前指标快照（从机队与 ObstacleManager 读取） */
	FScenarioMetrics CollectMetrics() const;

	/** 立即评估并写 JSON（任务完成/超时硬触发时调用） */
	void FlushFinalResult();

	UFUNCTION()
	void HandleMissionCompleted(bool bSuccess);

	UFUNCTION()
	void HandleMissionFailed(FString Reason);

	UPROPERTY(Transient)
	TObjectPtr<UScenario> Scenario;

	UPROPERTY(Transient)
	TObjectPtr<AUAVPawn> LeadUAV;

	UPROPERTY(Transient)
	TObjectPtr<UAcceptanceCriteria> Criteria;

	// 指标累积状态（跨周期保留极值）
	FScenarioMetrics Accumulated;

	// 周期快照累加器
	float SnapshotAccumulator = 0.0f;

	// 已耗时
	float ElapsedTime = 0.0f;

	// 是否已输出最终结果（避免重复）
	bool bFinalFlushed = false;

	// 快照周期（秒）
	static constexpr float SnapshotInterval = 1.0f;
};
