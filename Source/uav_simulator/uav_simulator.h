// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

// ========== 性能统计 ==========
DECLARE_STATS_GROUP(TEXT("UAVSimulator"), STATGROUP_UAVSim, STATCAT_Advanced);

// Actor 级别
DECLARE_CYCLE_STAT(TEXT("UAVPawn::Tick"), STAT_UAVPawnTick, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("UAVActor::Tick"), STAT_UAVActorTick, STATGROUP_UAVSim);

// 物理子步进
DECLARE_CYCLE_STAT(TEXT("PhysicsSubStep"), STAT_PhysicsSubStep, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("RK4Integration"), STAT_RK4Integration, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("ComputeForcesAndTorques"), STAT_ComputeForces, STATGROUP_UAVSim);

// 控制器
DECLARE_CYCLE_STAT(TEXT("PositionController"), STAT_PositionController, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("AttitudeController"), STAT_AttitudeController, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("StateEstimator"), STAT_StateEstimator, STATGROUP_UAVSim);

// 规划
DECLARE_CYCLE_STAT(TEXT("TrajectoryTracker"), STAT_TrajectoryTracker, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("NMPCSolver"), STAT_NMPCSolver, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("TrajectoryOptimizer"), STAT_TrajectoryOptimizer, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("ObstacleManager"), STAT_ObstacleManager, STATGROUP_UAVSim);

// 传感器
DECLARE_CYCLE_STAT(TEXT("SensorBase"), STAT_SensorBase, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("ObstacleDetector"), STAT_ObstacleDetector, STATGROUP_UAVSim);

// 调试可视化
DECLARE_CYCLE_STAT(TEXT("DebugVisualizer"), STAT_DebugVisualizer, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("PlanningVisualizer"), STAT_PlanningVisualizer, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("StabilityScorer"), STAT_StabilityScorer, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("DataLogger"), STAT_DataLogger, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("ControlParameterTuner"), STAT_ControlParameterTuner, STATGROUP_UAVSim);

// 其他组件
DECLARE_CYCLE_STAT(TEXT("WindField"), STAT_WindField, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("MissionComponent"), STAT_MissionComponent, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("FormationComponent"), STAT_FormationComponent, STATGROUP_UAVSim);

// 子步骤细分（UAVPawn::Tick 内部）
DECLARE_CYCLE_STAT(TEXT("Pawn::SensorUpdate"), STAT_PawnSensorUpdate, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("Pawn::ControllerUpdate"), STAT_PawnControllerUpdate, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("Pawn::PhysicsUpdate"), STAT_PawnPhysicsUpdate, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("Pawn::DebugDraw"), STAT_PawnDebugDraw, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("Pawn::PlanningDraw"), STAT_PawnPlanningDraw, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("Pawn::HUDUpdate"), STAT_PawnHUDUpdate, STATGROUP_UAVSim);

// 多机 & 安全滤波
DECLARE_CYCLE_STAT(TEXT("AgentManager::Tick"), STAT_AgentManagerTick, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("CBFQPFilter"), STAT_CBFQPFilter, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("UAVWreckActor::Tick"), STAT_WreckActorTick, STATGROUP_UAVSim);
DECLARE_CYCLE_STAT(TEXT("CollisionCheck"), STAT_CollisionCheck, STATGROUP_UAVSim);

