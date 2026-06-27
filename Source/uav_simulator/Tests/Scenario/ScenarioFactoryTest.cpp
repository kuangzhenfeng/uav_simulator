// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Scenario/ScenarioFactory.h"
#include "../../Scenario/ScenarioDto.h"
#include "../../Scenario/ScenarioTypes.h"
#include "../../Mission/MissionTypes.h"
#include "../../Environment/EnvironmentTypes.h"
#include "../../Core/UAVTypes.h"
#include "../../Core/UAVProductTypes.h"
#include "../../Planning/NMPCAvoidance.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 构造一个完整的合成 DTO ====================

namespace
{
	// 构造一个覆盖所有字段的 DTO，用于往返一致性验证。
	FScenarioDto MakeFullDto()
	{
		FScenarioDto Dto;
		Dto.Name = TEXT("FactoryRoundTrip");
		Dto.RandomSeed = 12345;

		// 仿真控制
		Dto.Sim.ControlMode = TEXT("Position");
		Dto.Sim.MPCType = TEXT("Linear");

		// 风场
		Dto.Wind.Type = TEXT("Turbulent");
		Dto.Wind.Steady = FVector(10.0f, 20.0f, 30.0f);
		Dto.Wind.GustAmplitude = 333.0f;
		Dto.Wind.GustDuration = 7.0f;
		Dto.Wind.GustFrequency = 9.0f;
		Dto.Wind.TurbulenceIntensity = 444.0f;
		Dto.Wind.TurbulenceLengthScale = 5555.0f;
		Dto.Wind.bEnabled = true;

		// 验收
		Dto.Acceptance.bRequireAllWaypoints = false;
		Dto.Acceptance.WaypointRadiusCm = 250.0f;
		Dto.Acceptance.MinClearanceCm = 80.0f;
		Dto.Acceptance.MaxLateralDeviationCm = 270.0f;
		Dto.Acceptance.TimeoutSec = 90.0f;
		Dto.Acceptance.EnergyBudget = 0.5f;

		// 机队：两架，一架长机带航线，一架僚机带不同机型/模式
		{
			FScenarioDtoAgent& Leader = Dto.Fleet.AddDefaulted_GetRef();
			Leader.Model = TEXT("Agri_AG60");
			Leader.InitPos = FVector(1000.0f, 0.0f, 200.0f);
			Leader.Yaw = 45.0f;
			Leader.bIsLeader = true;
			Leader.Mode = TEXT("Loop");
			{
				FScenarioDtoWaypoint& WP0 = Leader.Waypoints.AddDefaulted_GetRef();
				WP0.Pos = FVector(5000.0f, 0.0f, 200.0f);
				WP0.Speed = 800.0f;
				WP0.Hover = 2.0f;
				FScenarioDtoWaypoint& WP1 = Leader.Waypoints.AddDefaulted_GetRef();
				WP1.Pos = FVector(5000.0f, 5000.0f, 200.0f);
				WP1.Speed = 600.0f;
				WP1.Hover = 0.0f;
			}
		}
		{
			FScenarioDtoAgent& Wingman = Dto.Fleet.AddDefaulted_GetRef();
			Wingman.Model = TEXT("Map_SVLiDAR");
			Wingman.InitPos = FVector(1000.0f, 1000.0f, 200.0f);
			Wingman.Yaw = 0.0f;
			Wingman.bIsLeader = false;
			Wingman.Mode = TEXT("PingPong");
			{
				FScenarioDtoWaypoint& WP0 = Wingman.Waypoints.AddDefaulted_GetRef();
				WP0.Pos = FVector(6000.0f, 1000.0f, 200.0f);
				WP0.Speed = 500.0f;
				WP0.Hover = 0.0f;
			}
		}

		// 障碍：覆盖三种几何 + 两种运动模型
		{
			FScenarioDtoObstacle& Obs = Dto.Obstacles.AddDefaulted_GetRef();
			Obs.Type = TEXT("Sphere");
			Obs.Center = FVector(3000.0f, 0.0f, 200.0f);
			Obs.Extents = FVector(150.0f);
			Obs.SafetyMargin = 30.0f;
			Obs.Movement = TEXT("Static");
		}
		{
			FScenarioDtoObstacle& Obs = Dto.Obstacles.AddDefaulted_GetRef();
			Obs.Type = TEXT("Box");
			Obs.Center = FVector(4000.0f, 0.0f, 200.0f);
			Obs.Extents = FVector(200.0f, 100.0f, 50.0f);
			Obs.SafetyMargin = 40.0f;
			Obs.Movement = TEXT("LinearVelocity");
			Obs.Velocity = FVector(100.0f, 0.0f, 0.0f);
		}
		{
			FScenarioDtoObstacle& Obs = Dto.Obstacles.AddDefaulted_GetRef();
			Obs.Type = TEXT("Cylinder");
			Obs.Center = FVector(4500.0f, 1000.0f, 200.0f);
			Obs.Extents = FVector(120.0f, 120.0f, 300.0f);
			Obs.SafetyMargin = 50.0f;
			Obs.Movement = TEXT("PatrolLoop");
			Obs.PatrolPoints = { FVector(4500.0f, 1000.0f, 200.0f), FVector(4500.0f, 2000.0f, 200.0f) };
			Obs.PatrolSpeed = 250.0f;
		}

		return Dto;
	}
}

// ==================== 往返一致性：机队/障碍/风/验收/航点/算法覆盖 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioFactoryRoundTripTest,
	"UAVSimulator.Scenario.Factory.RoundTrip",
	UAV_TEST_FLAGS)

bool FScenarioFactoryRoundTripTest::RunTest(const FString& Parameters)
{
	const FScenarioDto Dto = MakeFullDto();

	UScenario* Scenario = UScenarioFactory::BuildFromDto(Dto, GetTransientPackage());
	if (!TestNotNull(TEXT("BuildFromDto 返回非空"), Scenario))
	{
		return false;
	}

	// 外壳字段
	TestEqual(TEXT("Name 往返"), Scenario->Name, Dto.Name);
	TestEqual(TEXT("RandomSeed 往返"), Scenario->RandomSeed, Dto.RandomSeed);

	// ---- 机队 ----
	const UObstacleLayout* Layout = Scenario->ObstacleLayout.Get();
	const UFleetSetup* Fleet = Scenario->FleetSetup.Get();
	const UWindProfile* Wind = Scenario->WindProfile.Get();
	const UAcceptanceCriteria* Acceptance = Scenario->AcceptanceCriteria.Get();

	if (!TestNotNull(TEXT("ObstacleLayout 已构造"), Layout)) return false;
	if (!TestNotNull(TEXT("FleetSetup 已构造"), Fleet)) return false;
	if (!TestNotNull(TEXT("WindProfile 已构造"), Wind)) return false;
	if (!TestNotNull(TEXT("AcceptanceCriteria 已构造"), Acceptance)) return false;

	TestEqual(TEXT("Fleet 数量 == Dto.Fleet.Num()"), Fleet->Agents.Num(), Dto.Fleet.Num());
	TestEqual(TEXT("障碍数量 == Dto.Obstacles.Num()"), Layout->Obstacles.Num(), Dto.Obstacles.Num());

	// 长机：机型、位姿、长机标志、模式、航点数
	if (TestTrue(TEXT("至少两架机"), Fleet->Agents.Num() >= 2))
	{
		const FScenarioAgentEntry& Leader = Fleet->Agents[0];
		TestEqual(TEXT("长机机型"), (int32)Leader.ModelID, (int32)EUAVModelID::Agri_AG60);
		UAV_TEST_VECTOR_EQUAL(Leader.InitialPosition, Dto.Fleet[0].InitPos, 1.0f);
		TestEqual(TEXT("长机偏航"), Leader.InitialYaw, Dto.Fleet[0].Yaw);
		TestTrue(TEXT("长机标志"), Leader.bIsLeader);
		TestEqual(TEXT("长机模式"), (int32)Leader.MissionMode, (int32)EMissionMode::Loop);
		TestEqual(TEXT("长机航点数"), Leader.Waypoints.Num(), 2);

		// 第一个航点字段逐项核对
		if (TestEqual(TEXT("长机首航点存在"), Leader.Waypoints.Num(), 2))
		{
			UAV_TEST_VECTOR_EQUAL(Leader.Waypoints[0].Position, Dto.Fleet[0].Waypoints[0].Pos, 1.0f);
			TestEqual(TEXT("长机首航点速度"), Leader.Waypoints[0].DesiredSpeed, Dto.Fleet[0].Waypoints[0].Speed);
			TestEqual(TEXT("长机首航点悬停"), Leader.Waypoints[0].HoverDuration, Dto.Fleet[0].Waypoints[0].Hover);
		}

		// 僚机：机型、模式
		const FScenarioAgentEntry& Wingman = Fleet->Agents[1];
		TestEqual(TEXT("僚机机型"), (int32)Wingman.ModelID, (int32)EUAVModelID::Map_SVLiDAR);
		TestEqual(TEXT("僚机模式"), (int32)Wingman.MissionMode, (int32)EMissionMode::PingPong);
		TestEqual(TEXT("僚机航点数"), Wingman.Waypoints.Num(), 1);
	}

	// ---- 障碍：三种几何 + 运动模型 ----
	if (TestEqual(TEXT("障碍数 == 3"), Layout->Obstacles.Num(), 3))
	{
		TestEqual(TEXT("障碍0 Sphere"), (int32)Layout->Obstacles[0].Type, (int32)EObstacleType::Sphere);
		TestEqual(TEXT("障碍0 Static"), (int32)Layout->Obstacles[0].MovementType, (int32)EObstacleMovementType::Static);

		TestEqual(TEXT("障碍1 Box"), (int32)Layout->Obstacles[1].Type, (int32)EObstacleType::Box);
		TestEqual(TEXT("障碍1 LinearVelocity"), (int32)Layout->Obstacles[1].MovementType, (int32)EObstacleMovementType::LinearVelocity);
		UAV_TEST_VECTOR_EQUAL(Layout->Obstacles[1].Velocity, Dto.Obstacles[1].Velocity, 1.0f);

		TestEqual(TEXT("障碍2 Cylinder"), (int32)Layout->Obstacles[2].Type, (int32)EObstacleType::Cylinder);
		TestEqual(TEXT("障碍2 PatrolLoop"), (int32)Layout->Obstacles[2].MovementType, (int32)EObstacleMovementType::PatrolLoop);
		TestEqual(TEXT("障碍2 巡逻点数"), Layout->Obstacles[2].PatrolPoints.Num(), 2);
		TestEqual(TEXT("障碍2 巡逻速度"), Layout->Obstacles[2].PatrolSpeed, Dto.Obstacles[2].PatrolSpeed);
		TestEqual(TEXT("障碍0 安全边距"), Layout->Obstacles[0].SafetyMargin, Dto.Obstacles[0].SafetyMargin);
	}

	// ---- 风场 ----
	TestEqual(TEXT("风类型 Turbulent"), (int32)Wind->Config.WindType, (int32)EWindFieldType::Turbulent);
	UAV_TEST_VECTOR_EQUAL(Wind->Config.SteadyWindVelocity, Dto.Wind.Steady, 1.0f);
	TestEqual(TEXT("阵风幅度"), Wind->Config.GustAmplitude, Dto.Wind.GustAmplitude);
	TestEqual(TEXT("湍流强度"), Wind->Config.TurbulenceIntensity, Dto.Wind.TurbulenceIntensity);
	TestTrue(TEXT("风场启用"), Wind->Config.bEnabled);

	// ---- 验收阈值 ----
	TestEqual(TEXT("要求全部航点"), Acceptance->bRequireAllWaypoints, Dto.Acceptance.bRequireAllWaypoints);
	TestEqual(TEXT("航点半径"), Acceptance->WaypointArrivalRadius, Dto.Acceptance.WaypointRadiusCm);
	TestEqual(TEXT("最小净空"), Acceptance->MinClearanceCm, Dto.Acceptance.MinClearanceCm);
	TestEqual(TEXT("最大横向偏差"), Acceptance->MaxLateralDeviationCm, Dto.Acceptance.MaxLateralDeviationCm);
	TestEqual(TEXT("超时"), Acceptance->TimeoutSeconds, Dto.Acceptance.TimeoutSec);
	TestEqual(TEXT("能耗预算"), Acceptance->EnergyBudget, Dto.Acceptance.EnergyBudget);

	// ---- 算法覆盖 ----
	TestTrue(TEXT("覆盖控制模式"), Scenario->AlgorithmOverride.bOverrideControlMode);
	TestEqual(TEXT("控制模式 Position"), (int32)Scenario->AlgorithmOverride.ControlMode, (int32)EUAVControlMode::Position);
	TestTrue(TEXT("覆盖 MPC 类型"), Scenario->AlgorithmOverride.bOverrideMPCType);
	TestEqual(TEXT("MPC 类型 Linear"), (int32)Scenario->AlgorithmOverride.MPCType, (int32)EMPCType::Linear);

	return true;
}

// ==================== 枚举不匹配回退默认值不崩溃 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioFactoryEnumFallbackTest,
	"UAVSimulator.Scenario.Factory.EnumFallback",
	UAV_TEST_FLAGS)

bool FScenarioFactoryEnumFallbackTest::RunTest(const FString& Parameters)
{
	FScenarioDto Dto;
	Dto.Name = TEXT("EnumFallback");
	// 全部填非法字符串枚举名
	Dto.Wind.Type = TEXT("NotAWindType");
	Dto.Sim.ControlMode = TEXT("BogusMode");
	Dto.Sim.MPCType = TEXT("BogusMPC");

	FScenarioDtoAgent& Agent = Dto.Fleet.AddDefaulted_GetRef();
	Agent.Model = TEXT("UnknownModel");
	Agent.Mode = TEXT("UnknownMode");
	FScenarioDtoWaypoint& WP = Agent.Waypoints.AddDefaulted_GetRef();
	WP.Pos = FVector(1000.0f, 0.0f, 0.0f);

	FScenarioDtoObstacle& Obs = Dto.Obstacles.AddDefaulted_GetRef();
	Obs.Type = TEXT("Dodecahedron");
	Obs.Movement = TEXT("Teleport");

	// 不应崩溃，且应回退到合理默认值
	UScenario* Scenario = UScenarioFactory::BuildFromDto(Dto, GetTransientPackage());
	if (!TestNotNull(TEXT("BuildFromDto 不崩溃且返回非空"), Scenario))
	{
		return false;
	}

	const UFleetSetup* Fleet = Scenario->FleetSetup.Get();
	const UObstacleLayout* Layout = Scenario->ObstacleLayout.Get();
	const UWindProfile* Wind = Scenario->WindProfile.Get();

	if (!TestNotNull(TEXT("Fleet 非空"), Fleet)) return false;
	if (!TestNotNull(TEXT("Layout 非空"), Layout)) return false;
	if (!TestNotNull(TEXT("Wind 非空"), Wind)) return false;

	// 默认值断言（与 Factory.cpp 中的回退一致）
	TestEqual(TEXT("非法机型 -> Agri_AG20"), (int32)Fleet->Agents[0].ModelID, (int32)EUAVModelID::Agri_AG20);
	TestEqual(TEXT("非法任务模式 -> Once"), (int32)Fleet->Agents[0].MissionMode, (int32)EMissionMode::Once);
	TestEqual(TEXT("非法障碍类型 -> Box"), (int32)Layout->Obstacles[0].Type, (int32)EObstacleType::Box);
	TestEqual(TEXT("非法运动 -> Static"), (int32)Layout->Obstacles[0].MovementType, (int32)EObstacleMovementType::Static);
	TestEqual(TEXT("非法风类型 -> Constant"), (int32)Wind->Config.WindType, (int32)EWindFieldType::Constant);

	// 非法 ControlMode/MPCType 仍应被解析（非空字符串触发覆盖），回退默认枚举值
	TestTrue(TEXT("非法 ControlMode 仍触发覆盖"), Scenario->AlgorithmOverride.bOverrideControlMode);
	TestEqual(TEXT("非法 ControlMode -> Trajectory"), (int32)Scenario->AlgorithmOverride.ControlMode, (int32)EUAVControlMode::Trajectory);
	TestTrue(TEXT("非法 MPCType 仍触发覆盖"), Scenario->AlgorithmOverride.bOverrideMPCType);
	TestEqual(TEXT("非法 MPCType -> Nonlinear"), (int32)Scenario->AlgorithmOverride.MPCType, (int32)EMPCType::Nonlinear);

	return true;
}

// ==================== 算法覆盖：空字符串不触发覆盖 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioFactoryNoOverrideTest,
	"UAVSimulator.Scenario.Factory.NoOverride",
	UAV_TEST_FLAGS)

bool FScenarioFactoryNoOverrideTest::RunTest(const FString& Parameters)
{
	FScenarioDto Dto;
	Dto.Name = TEXT("NoOverride");
	// ControlMode/MPCType 默认空字符串
	Dto.Sim.ControlMode = TEXT("");
	Dto.Sim.MPCType = TEXT("");

	UScenario* Scenario = UScenarioFactory::BuildFromDto(Dto, GetTransientPackage());
	if (!TestNotNull(TEXT("BuildFromDto 返回非空"), Scenario))
	{
		return false;
	}

	TestFalse(TEXT("空 ControlMode 不覆盖"), Scenario->AlgorithmOverride.bOverrideControlMode);
	TestFalse(TEXT("空 MPCType 不覆盖"), Scenario->AlgorithmOverride.bOverrideMPCType);

	return true;
}

// ==================== Outer 与子资产归属 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioFactoryOwnershipTest,
	"UAVSimulator.Scenario.Factory.Ownership",
	UAV_TEST_FLAGS)

bool FScenarioFactoryOwnershipTest::RunTest(const FString& Parameters)
{
	const FScenarioDto Dto = MakeFullDto();
	UObject* Outer = GetTransientPackage();

	UScenario* Scenario = UScenarioFactory::BuildFromDto(Dto, Outer);
	if (!TestNotNull(TEXT("BuildFromDto 返回非空"), Scenario))
	{
		return false;
	}

	// 子资产 outer 应为 Scenario（与 NewObject<...>(Scenario) 一致）
	TestEqual(TEXT("ObstacleLayout outer == Scenario"), Cast<UObject>(Scenario->ObstacleLayout.Get()->GetOuter()), Cast<UObject>(Scenario));
	TestEqual(TEXT("FleetSetup outer == Scenario"), Cast<UObject>(Scenario->FleetSetup.Get()->GetOuter()), Cast<UObject>(Scenario));
	TestEqual(TEXT("WindProfile outer == Scenario"), Cast<UObject>(Scenario->WindProfile.Get()->GetOuter()), Cast<UObject>(Scenario));
	TestEqual(TEXT("AcceptanceCriteria outer == Scenario"), Cast<UObject>(Scenario->AcceptanceCriteria.Get()->GetOuter()), Cast<UObject>(Scenario));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
