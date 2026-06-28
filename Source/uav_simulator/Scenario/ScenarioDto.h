// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "ScenarioTypes.h"
#include "ScenarioDto.generated.h"

class UScenario;

/**
 * Web 控制面板的统一数据契约（ScenarioDto）。
 *
 * 浏览器 <-> Python 反向代理 <-> UE HTTP 控制端 三方共享同一份 JSON 结构。
 * 经 FJsonObjectConverter（JsonUtilities）自动序列化，避免手写解析。
 *
 * 坐标系约定：全程使用 UE 原生（cm，左手系 X前/Y右/Z上），与 ScenarioLoader 内部一致；
 * 仅在 Web 前端展示层做 m / 右手系变换（见 Tools/vis/server.py 的 to_web）。
 * 这样数据链路不被坐标系变换污染，只有展示边界做转换。
 *
 * 传输格式约定（Web 三方契约）：JSON key 用 camelCase 小写（name/fleet/initPos...），
 * 坐标向量用数组 [x,y,z]。本结构体仅用于 C++ 内部 DTO→Scenario 转换，
 * HTTP 边界（HttpControlComponent::BuildDtoFromWebJson）负责 Web JSON ↔ 本结构体的适配，
 * 故这里的 UPROPERTY 命名（FVector/PascalCase）不影响线上的 JSON 形态。
 */

// ---- 仿真控制参数 ----

USTRUCT(BlueprintType)
struct FScenarioDtoSim
{
	GENERATED_BODY()

	/** 时标倍速（0.1~N），实时调参可改；0 表示不覆盖（保持启动值） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Sim")
	float Slomo = 0.0f;

	/** 仿真墙钟时长上限（秒），<=0 表示不限（由任务完成/外部退出决定） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Sim")
	float DurationSec = 60.0f;

	/** 覆盖控制模式（空字符串表示不覆盖，用 Pawn 默认）：Attitude/Position/Trajectory */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Sim")
	FString ControlMode;

	/** 覆盖 MPC 类型（空字符串表示不覆盖）：Nonlinear/Linear */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Sim")
	FString MPCType;
};

// ---- 风场参数 ----

USTRUCT(BlueprintType)
struct FScenarioDtoWind
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Wind")
	FString Type = TEXT("Constant"); // None/Constant/Gust/Turbulent

	/** 稳态风速（cm/s，世界系） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Wind")
	FVector Steady = FVector(0.0f, 100.0f, 0.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Wind")
	float GustAmplitude = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Wind")
	float GustDuration = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Wind")
	float GustFrequency = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Wind")
	float TurbulenceIntensity = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Wind")
	float TurbulenceLengthScale = 5000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Wind")
	bool bEnabled = true;
};

// ---- 验收阈值 ----

USTRUCT(BlueprintType)
struct FScenarioDtoAcceptance
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Acceptance")
	bool bRequireAllWaypoints = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Acceptance")
	float WaypointRadiusCm = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Acceptance")
	float MinClearanceCm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Acceptance")
	float MaxLateralDeviationCm = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Acceptance")
	float TimeoutSec = 120.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Acceptance")
	float EnergyBudget = 0.0f;
};

// ---- 航点 ----

USTRUCT(BlueprintType)
struct FScenarioDtoWaypoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Waypoint")
	FVector Pos = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Waypoint")
	float Speed = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Waypoint")
	float Hover = 0.0f;
};

// ---- 机队成员 ----

USTRUCT(BlueprintType)
struct FScenarioDtoAgent
{
	GENERATED_BODY()

	/** 机型枚举名（EUAVModelID）：Agri_AG20/Agri_AG60/Agri_AG100/Map_SVPro/Map_SVLiDAR */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Agent")
	FString Model = TEXT("Agri_AG20");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Agent")
	FVector InitPos = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Agent")
	float Yaw = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Agent")
	bool bIsLeader = false;

	/** 该机独立航线（空=回退全局任务/编队跟随） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Agent")
	TArray<FScenarioDtoWaypoint> Waypoints;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto|Agent")
	FString Mode = TEXT("Once"); // Once/Loop/PingPong
};

// ---- 顶层 DTO ----

USTRUCT(BlueprintType)
struct FScenarioDto
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto")
	FString Name = TEXT("WebScenario");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto")
	FScenarioDtoSim Sim;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto")
	FScenarioDtoWind Wind;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto")
	FScenarioDtoAcceptance Acceptance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto")
	TArray<FScenarioDtoAgent> Fleet;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dto")
	TArray<FScenarioObstacleEntry> Obstacles;
};
