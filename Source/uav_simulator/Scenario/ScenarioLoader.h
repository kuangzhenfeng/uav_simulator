// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "ScenarioTypes.h"
#include "ScenarioLoader.generated.h"

class UObstacleManager;
class UWindField;
class UMissionComponent;
class AUAVPawn;
class AMultiAgentGameMode;
class UFleetSetup;
class UMissionProfile;

/**
 * 场景装配器（ScenarioLoader）。
 *
 * 读取 UScenario 资产，按声明内容创建"运行中的世界"。
 * 是 Scenario 从数据变成世界的唯一入口。
 *
 * 装配按子流程组织（风场 → 障碍 → 机队/任务），各子流程可独立调用与测试。
 * 各子流程只验证装配后的世界状态（外部行为），不暴露内部步骤顺序。
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UScenarioLoader : public UObject
{
	GENERATED_BODY()

public:
	UScenarioLoader();
	/**
	 * 装配障碍：把 UScenario.ObstacleLayout 声明的障碍几何
	 * 注册到 ObstacleManager，并在世界中 Spawn 可视化 Actor。
	 *
	 * @param Scenario      场景资产（提供 ObstacleLayout 引用）
	 * @param ObstacleManager 装配目标（障碍注册到此）
	 * @param WorldContext   提供 World 的上下文对象（用于 Spawn 可视化；可为空表示仅注册逻辑）
	 * @return 成功注册的障碍数量
	 */
	UFUNCTION(BlueprintCallable, Category = "Scenario")
	int32 AssembleObstacles(const UScenario* Scenario, UObstacleManager* ObstacleManager, UObject* WorldContext);

	/**
	 * 装配机队与任务：按 UScenario.FleetSetup Spawn UAV 机队，
	 * 把 UScenario.MissionProfile 的航点下发给首个 UAV 的 MissionComponent 并启动任务。
	 *
	 * @param Scenario      场景资产（提供 FleetSetup / MissionProfile 引用）
	 * @param WorldContext   提供 World 的上下文对象
	 * @param OutFleet       输出：Spawn 出的 UAV 列表（调用方可继续装配/验收）
	 * @param DefaultUAVClass Agent 未指定蓝图时的回退 UAV 类
	 * @return 成功 Spawn 的 UAV 数量
	 */
	UFUNCTION(BlueprintCallable, Category = "Scenario")
	int32 AssembleFleetAndMission(
		const UScenario* Scenario,
		UObject* WorldContext,
		TArray<AUAVPawn*>& OutFleet,
		TSubclassOf<AUAVPawn> DefaultUAVClass = nullptr);

	/**
	 * 装配风场：把 UScenario.WindProfile 的 FWindConfig 下发到场景级 WindField。
	 * 应在机队 Spawn 之前调用（机队物理积分前风场已就绪）。
	 *
	 * @param Scenario   场景资产（提供 WindProfile 引用）
	 * @param WindField  场景级 WindField 单例（ADR-0002）
	 * @return 是否成功下发（无 WindProfile 或 WindField 为空时返回 false）
	 */
	UFUNCTION(BlueprintCallable, Category = "Scenario")
	bool AssembleWind(const UScenario* Scenario, UWindField* WindField);
};
