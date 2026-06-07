// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Sensors/BarometerSensor.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 海平面气压测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBarometerSeaLevelTest,
	"UAVSimulator.Sensors.Barometer.AltitudeAtSeaLevel",
	UAV_TEST_FLAGS)

bool FBarometerSeaLevelTest::RunTest(const FString& Parameters)
{
	UBarometerSensor* Baro = NewObject<UBarometerSensor>();
	Baro->SetEnabled(true);

	// 海平面 Z=0
	FUAVState State = UAVTestHelpers::CreateUAVState(FVector(0, 0, 0));
	Baro->UpdateSensor(State, 0.01f);

	FBarometerData Data = Baro->GetBarometerData();

	// 海平面气压应接近 1013.25 hPa
	UAV_TEST_FLOAT_EQUAL(Data.Pressure, 1013.25f, 5.0f);

	// 海平面高度应接近 0
	UAV_TEST_FLOAT_EQUAL(Data.Altitude, 0.0f, 1000.0f); // 容差 1000cm = 10m

	return true;
}

// ==================== 高空气压测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBarometerAltitudeTest,
	"UAVSimulator.Sensors.Barometer.AltitudeAtHeight",
	UAV_TEST_FLAGS)

bool FBarometerAltitudeTest::RunTest(const FString& Parameters)
{
	UBarometerSensor* Baro = NewObject<UBarometerSensor>();
	Baro->SetEnabled(true);

	// Z = 100000cm = 1000m
	FUAVState State = UAVTestHelpers::CreateUAVState(FVector(0, 0, 100000.0f));
	// DeltaTime 必须大于更新间隔 (1/50Hz = 0.02s) 才能触发传感器更新
	Baro->UpdateSensor(State, 0.03f);

	FBarometerData Data = Baro->GetBarometerData();

	// 1000m 高空气压应低于海平面
	TestTrue(TEXT("Pressure at 1000m should be lower than sea level"),
		Data.Pressure < 1013.25f);

	// ISA 模型: P(1000m) ≈ 898.75 hPa
	TestTrue(TEXT("Pressure at 1000m should be around 898 hPa"),
		Data.Pressure > 800.0f && Data.Pressure < 950.0f);

	// 高度测量应接近 100000cm
	UAV_TEST_FLOAT_EQUAL(Data.Altitude, 100000.0f, 5000.0f); // 容差 50m

	return true;
}

// ==================== 统计收敛性测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBarometerConvergenceTest,
	"UAVSimulator.Sensors.Barometer.StatisticalConvergence",
	UAV_TEST_FLAGS)

bool FBarometerConvergenceTest::RunTest(const FString& Parameters)
{
	UBarometerSensor* Baro = NewObject<UBarometerSensor>();
	Baro->SetEnabled(true);

	// 海平面 Z=0
	FUAVState State = UAVTestHelpers::CreateUAVState(FVector(0, 0, 0));

	double PressureSum = 0.0;
	double AltitudeSum = 0.0;
	int32 N = 200;

	for (int32 i = 0; i < N; ++i)
	{
		Baro->UpdateSensor(State, 0.01f);
		FBarometerData Data = Baro->GetBarometerData();
		PressureSum += Data.Pressure;
		AltitudeSum += Data.Altitude;
	}

	float AvgPressure = static_cast<float>(PressureSum / N);
	float AvgAltitude = static_cast<float>(AltitudeSum / N);

	// 200 样本均值应收敛到真值附近
	UAV_TEST_FLOAT_EQUAL(AvgPressure, 1013.25f, 1.0f);
	UAV_TEST_FLOAT_EQUAL(AvgAltitude, 0.0f, 1000.0f);

	return true;
}

// ==================== 禁用不更新测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBarometerDisabledTest,
	"UAVSimulator.Sensors.Barometer.DisabledNoUpdate",
	UAV_TEST_FLAGS)

bool FBarometerDisabledTest::RunTest(const FString& Parameters)
{
	UBarometerSensor* Baro = NewObject<UBarometerSensor>();
	Baro->SetEnabled(false);

	// 先记录初始数据
	FBarometerData InitialData = Baro->GetBarometerData();

	// 在高空更新
	FUAVState State = UAVTestHelpers::CreateUAVState(FVector(0, 0, 100000.0f));
	Baro->UpdateSensor(State, 0.01f);

	FBarometerData AfterData = Baro->GetBarometerData();

	// 禁用状态下数据应保持不变
	TestEqual(TEXT("Pressure should not change when disabled"),
		AfterData.Pressure, InitialData.Pressure);
	TestEqual(TEXT("Altitude should not change when disabled"),
		AfterData.Altitude, InitialData.Altitude);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
