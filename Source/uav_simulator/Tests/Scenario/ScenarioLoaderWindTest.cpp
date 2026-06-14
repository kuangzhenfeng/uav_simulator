// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Scenario/ScenarioLoader.h"
#include "../../Scenario/ScenarioTypes.h"
#include "../../Environment/WindField.h"
#include "../../Environment/EnvironmentTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 装配风场：Config 一致下发 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioWindConfigAppliedTest,
	"UAVSimulator.Scenario.Wind.ConfigApplied",
	UAV_TEST_FLAGS)

bool FScenarioWindConfigAppliedTest::RunTest(const FString& Parameters)
{
	UScenario* Scenario = NewObject<UScenario>(GetTransientPackage());
	Scenario->Name = TEXT("WindTest");

	UWindProfile* Profile = NewObject<UWindProfile>(Scenario);
	Profile->Config.WindType = EWindFieldType::Constant;
	Profile->Config.SteadyWindVelocity = FVector(800.0f, 200.0f, 0.0f);
	Profile->Config.AirDensity = 1.3f;
	Profile->Config.DragArea = 0.05f;
	Profile->Config.DragCoefficient = 1.2f;
	Scenario->WindProfile = Profile;

	UWindField* WindField = NewObject<UWindField>();
	UScenarioLoader* Loader = NewObject<UScenarioLoader>();

	const bool bApplied = Loader->AssembleWind(Scenario, WindField);
	TestTrue(TEXT("风场配置成功下发"), bApplied);

	FWindConfig ReadBack = WindField->GetWindConfig();
	TestEqual(TEXT("风场类型一致"), (int32)ReadBack.WindType, (int32)EWindFieldType::Constant);
	UAV_TEST_VECTOR_EQUAL(ReadBack.SteadyWindVelocity, Profile->Config.SteadyWindVelocity, 1.0f);
	UAV_TEST_FLOAT_EQUAL(ReadBack.AirDensity, Profile->Config.AirDensity, 1e-4f);

	return true;
}

// ==================== 装配风场：无 Profile 时保持 WindField 现状 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FScenarioWindNoProfileTest,
	"UAVSimulator.Scenario.Wind.NoProfile",
	UAV_TEST_FLAGS)

bool FScenarioWindNoProfileTest::RunTest(const FString& Parameters)
{
	UScenario* Scenario = NewObject<UScenario>(GetTransientPackage());
	// 故意不设置 WindProfile

	UWindField* WindField = NewObject<UWindField>();
	FWindConfig Original = WindField->GetWindConfig();

	UScenarioLoader* Loader = NewObject<UScenarioLoader>();
	const bool bApplied = Loader->AssembleWind(Scenario, WindField);

	TestFalse(TEXT("无 WindProfile 时返回 false"), bApplied);

	// WindField 保持原配置不变（外部行为：装配未改变风场）
	FWindConfig After = WindField->GetWindConfig();
	UAV_TEST_VECTOR_EQUAL(After.SteadyWindVelocity, Original.SteadyWindVelocity, 1.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
