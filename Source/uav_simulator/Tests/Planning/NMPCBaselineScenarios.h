// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "../UAVTestCommon.h"
#include "uav_simulator/Planning/NMPCAvoidance.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * 基准场景参数：固定所有随机因素，确保可复现
 */
struct FNMPCBaselineScenario
{
	// 场景名称
	FString Name;

	// 初始状态
	FVector StartPosition = FVector::ZeroVector;
	FVector StartVelocity = FVector::ZeroVector;

	// 目标位置
	FVector GoalPosition = FVector(5000, 0, 0);

	// 障碍物列表
	TArray<FObstacleInfo> Obstacles;

	// 仿真步数（每步 dt = Config.GetDt()）
	int32 SimSteps = 200;

	// 验收：最小净空（障碍物表面距离，cm）
	float MinClearanceRequired = 0.0f;

	// 验收：最小前进距离（cm）
	float MinProgressRequired = 500.0f;
};

/**
 * 创建基准场景列表
 */
void NMPCBaseline_GetScenarios(TArray<FNMPCBaselineScenario>& OutScenarios);

/**
 * 运行单个基准场景，返回是否通过
 */
bool NMPCBaseline_RunScenario(
	const FNMPCBaselineScenario& Scenario,
	UNMPCAvoidance* NMPC,
	float& OutMinClearance,
	float& OutProgress);

#endif // WITH_DEV_AUTOMATION_TESTS
