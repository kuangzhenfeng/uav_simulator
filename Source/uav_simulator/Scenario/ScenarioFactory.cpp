// Copyright Epic Games, Inc. All Rights Reserved.

#include "ScenarioFactory.h"
#include "ScenarioLoader.h" // 仅为语义关联（Factory 产物喂给 Loader），无直接调用
#include "../uav_simulator.h"

DEFINE_LOG_CATEGORY_STATIC(LogScenarioFactory, Log, All);

// ==================== 字符串 -> 枚举 ====================

// 大小写敏感比对；空串与不匹配返回默认值。
// Web 契约侧枚举名固定（见 ScenarioDto.h 注释），故用 CaseSensitive 避免误匹配。

EWindFieldType UScenarioFactory::ParseWindFieldType(const FString& InName)
{
	if (InName.Equals(TEXT("None"), ESearchCase::CaseSensitive)) return EWindFieldType::None;
	if (InName.Equals(TEXT("Constant"), ESearchCase::CaseSensitive)) return EWindFieldType::Constant;
	if (InName.Equals(TEXT("Gust"), ESearchCase::CaseSensitive)) return EWindFieldType::Gust;
	if (InName.Equals(TEXT("Turbulent"), ESearchCase::CaseSensitive)) return EWindFieldType::Turbulent;
	return EWindFieldType::Constant;
}

EUAVModelID UScenarioFactory::ParseUAVModelID(const FString& InName)
{
	if (InName.Equals(TEXT("Agri_AG20"), ESearchCase::CaseSensitive)) return EUAVModelID::Agri_AG20;
	if (InName.Equals(TEXT("Agri_AG60"), ESearchCase::CaseSensitive)) return EUAVModelID::Agri_AG60;
	if (InName.Equals(TEXT("Agri_AG100"), ESearchCase::CaseSensitive)) return EUAVModelID::Agri_AG100;
	if (InName.Equals(TEXT("Map_SVPro"), ESearchCase::CaseSensitive)) return EUAVModelID::Map_SVPro;
	if (InName.Equals(TEXT("Map_SVLiDAR"), ESearchCase::CaseSensitive)) return EUAVModelID::Map_SVLiDAR;
	return EUAVModelID::Agri_AG20;
}

EMissionMode UScenarioFactory::ParseMissionMode(const FString& InName)
{
	if (InName.Equals(TEXT("Loop"), ESearchCase::CaseSensitive)) return EMissionMode::Loop;
	if (InName.Equals(TEXT("PingPong"), ESearchCase::CaseSensitive)) return EMissionMode::PingPong;
	return EMissionMode::Once;
}

EMPCType UScenarioFactory::ParseMPCType(const FString& InName)
{
	if (InName.Equals(TEXT("Linear"), ESearchCase::CaseSensitive)) return EMPCType::Linear;
	return EMPCType::Nonlinear;
}

EUAVControlMode UScenarioFactory::ParseControlMode(const FString& InName)
{
	if (InName.Equals(TEXT("Attitude"), ESearchCase::CaseSensitive)) return EUAVControlMode::Attitude;
	if (InName.Equals(TEXT("Position"), ESearchCase::CaseSensitive)) return EUAVControlMode::Position;
	return EUAVControlMode::Trajectory;
}

// ==================== BuildFromDto ====================

UScenario* UScenarioFactory::BuildFromDto(const FScenarioDto& Dto, UObject* Outer)
{
	UObject* Owner = Outer ? Outer : GetTransientPackage();

	// ---- UScenario 外壳 ----
	UScenario* Scenario = NewObject<UScenario>(Owner);
	if (!Scenario)
	{
		UE_LOG(LogScenarioFactory, Error, TEXT("[ScenarioFactory] Failed to create UScenario"));
		return nullptr;
	}

	Scenario->Name = Dto.Name;
	// Description 不在 DTO 中（DTO 极简），留空。

	// ---- 障碍布局 ----
	UObstacleLayout* ObstacleLayout = NewObject<UObstacleLayout>(Scenario);
	ObstacleLayout->Obstacles = Dto.Obstacles; // Direct copy — already typed
	Scenario->ObstacleLayout = ObstacleLayout;

	// ---- 风场档案 ----
	UWindProfile* WindProfile = NewObject<UWindProfile>(Scenario);
	FWindConfig& WindCfg = WindProfile->Config;
	WindCfg.WindType = ParseWindFieldType(Dto.Wind.Type);
	WindCfg.SteadyWindVelocity = Dto.Wind.Steady;
	WindCfg.GustAmplitude = Dto.Wind.GustAmplitude;
	WindCfg.GustDuration = Dto.Wind.GustDuration;
	WindCfg.GustFrequency = Dto.Wind.GustFrequency;
	WindCfg.TurbulenceIntensity = Dto.Wind.TurbulenceIntensity;
	WindCfg.TurbulenceLengthScale = Dto.Wind.TurbulenceLengthScale;
	WindCfg.bEnabled = Dto.Wind.bEnabled;
	Scenario->WindProfile = WindProfile;

	// ---- 机队配置 ----
	UFleetSetup* FleetSetup = NewObject<UFleetSetup>(Scenario);
	FleetSetup->Agents.Reserve(Dto.Fleet.Num());
	for (const FScenarioDtoAgent& DtoAgent : Dto.Fleet)
	{
		FScenarioAgentEntry& Agent = FleetSetup->Agents.AddDefaulted_GetRef();
		Agent.UAVClass = nullptr; // Web 契约不指定蓝图，由 Loader 用默认 BP_UAVPawn_Default
		Agent.ModelID = ParseUAVModelID(DtoAgent.Model);
		Agent.InitialPosition = DtoAgent.InitPos;
		Agent.InitialYaw = DtoAgent.Yaw;
		Agent.bIsLeader = DtoAgent.bIsLeader;
		Agent.MissionMode = ParseMissionMode(DtoAgent.Mode);

		Agent.Waypoints.Reserve(DtoAgent.Waypoints.Num());
		for (const FScenarioDtoWaypoint& DtoWP : DtoAgent.Waypoints)
		{
			FMissionWaypoint& WP = Agent.Waypoints.AddDefaulted_GetRef();
			WP.Position = DtoWP.Pos;
			WP.DesiredSpeed = DtoWP.Speed;
			WP.HoverDuration = DtoWP.Hover;
			// DesiredYaw / Tag 保持 FMissionWaypoint 默认（未设置）
		}
	}
	Scenario->FleetSetup = FleetSetup;

	// ---- 任务档案 ----
	// DTO 无全局航线（航线全在每机 Waypoints），MissionProfile 留空。
	// Loader 逐机装配时每机内联航线非空则不会回退全局，故不需要全局 MissionProfile。
	Scenario->MissionProfile = nullptr;

	// ---- 验收标准 ----
	UAcceptanceCriteria* Acceptance = NewObject<UAcceptanceCriteria>(Scenario);
	Acceptance->bRequireAllWaypoints = Dto.Acceptance.bRequireAllWaypoints;
	Acceptance->WaypointArrivalRadius = Dto.Acceptance.WaypointRadiusCm;
	Acceptance->MinClearanceCm = Dto.Acceptance.MinClearanceCm;
	Acceptance->MaxLateralDeviationCm = Dto.Acceptance.MaxLateralDeviationCm;
	Acceptance->TimeoutSeconds = Dto.Acceptance.TimeoutSec;
	Acceptance->EnergyBudget = Dto.Acceptance.EnergyBudget;
	Scenario->AcceptanceCriteria = Acceptance;

	// ---- 算法覆盖（DTO.Sim.ControlMode/MPCType，非空才覆盖）----
	FScenarioAlgorithmOverride& Override = Scenario->AlgorithmOverride;
	if (!Dto.Sim.ControlMode.IsEmpty())
	{
		Override.bOverrideControlMode = true;
		Override.ControlMode = ParseControlMode(Dto.Sim.ControlMode);
	}
	else
	{
		Override.bOverrideControlMode = false;
	}

	if (!Dto.Sim.MPCType.IsEmpty())
	{
		Override.bOverrideMPCType = true;
		Override.MPCType = ParseMPCType(Dto.Sim.MPCType);
	}
	else
	{
		Override.bOverrideMPCType = false;
	}

	UE_LOG(LogScenarioFactory, Log,
		TEXT("[ScenarioFactory] Built scenario '%s': Agents=%d Obstacles=%d Wind=%s"),
		*Scenario->Name, FleetSetup->Agents.Num(), ObstacleLayout->Obstacles.Num(), *Dto.Wind.Type);

	return Scenario;
}
