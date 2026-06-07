// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Sensors/AnemometerSensor.h"
#include "../../Environment/WindField.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 有风场时风速测量测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnemometerWithWindFieldTest,
	"UAVSimulator.Sensors.Anemometer.WithWindField",
	UAV_TEST_FLAGS)

bool FAnemometerWithWindFieldTest::RunTest(const FString& Parameters)
{
	UAnemometerSensor* Anemo = NewObject<UAnemometerSensor>();
	Anemo->SetEnabled(true);

	// 创建风场并设置风速
	UWindField* WindField = NewObject<UWindField>();
	FWindConfig Config;
	Config.bEnabled = true;
	Config.SteadyWindVelocity = FVector(0, 1000, 0); // Y 方向 1000 cm/s
	WindField->SetWindConfig(Config);

	Anemo->SetWindField(WindField);

	FUAVState State = UAVTestHelpers::CreateUAVState(FVector(0, 0, 0));
	// DeltaTime 必须大于更新间隔 (1/20Hz = 0.05s) 才能触发传感器更新
	Anemo->UpdateSensor(State, 0.06f);

	FAnemometerData Data = Anemo->GetAnemometerData();

	// 风速应接近 1000 cm/s（容差考虑噪声）
	TestTrue(TEXT("Wind speed should be near 1000"),
		Data.WindSpeed > 800.0f && Data.WindSpeed < 1200.0f);

	// 风向应接近 90°（Y 正方向）
	TestTrue(TEXT("Wind direction should be near 90 degrees"),
		Data.WindDirection > 70.0f && Data.WindDirection < 110.0f);

	return true;
}

// ==================== 无风场时不崩溃测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnemometerNullWindFieldTest,
	"UAVSimulator.Sensors.Anemometer.NullWindField",
	UAV_TEST_FLAGS)

bool FAnemometerNullWindFieldTest::RunTest(const FString& Parameters)
{
	UAnemometerSensor* Anemo = NewObject<UAnemometerSensor>();
	Anemo->SetEnabled(true);
	// 不设置 WindField → WindFieldRef = nullptr

	FUAVState State = UAVTestHelpers::CreateUAVState(FVector(0, 0, 0));

	// 不应崩溃
	Anemo->UpdateSensor(State, 0.01f);

	FAnemometerData Data = Anemo->GetAnemometerData();

	// 无风场时风速应为 0
	TestEqual(TEXT("Wind speed should be 0 without wind field"), Data.WindSpeed, 0.0f);

	return true;
}

// ==================== 统计收敛性测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnemometerConvergenceTest,
	"UAVSimulator.Sensors.Anemometer.StatisticalConvergence",
	UAV_TEST_FLAGS)

bool FAnemometerConvergenceTest::RunTest(const FString& Parameters)
{
	UAnemometerSensor* Anemo = NewObject<UAnemometerSensor>();
	Anemo->SetEnabled(true);

	UWindField* WindField = NewObject<UWindField>();
	FWindConfig Config;
	Config.bEnabled = true;
	Config.SteadyWindVelocity = FVector(1000, 0, 0); // X 方向 1000 cm/s
	WindField->SetWindConfig(Config);
	Anemo->SetWindField(WindField);

	FUAVState State = UAVTestHelpers::CreateUAVState(FVector(0, 0, 0));

	double SpeedSum = 0.0;
	int32 N = 200;

	for (int32 i = 0; i < N; ++i)
	{
		// DeltaTime 必须大于更新间隔 (1/20Hz = 0.05s) 才能触发传感器更新
		Anemo->UpdateSensor(State, 0.06f);
		FAnemometerData Data = Anemo->GetAnemometerData();
		SpeedSum += Data.WindSpeed;
	}

	float AvgSpeed = static_cast<float>(SpeedSum / N);

	// 200 样本均值应收敛到 1000 附近
	TestTrue(TEXT("Average wind speed should converge to ~1000"),
		AvgSpeed > 900.0f && AvgSpeed < 1100.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
