// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Sensors/MagnetometerSensor.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 正北航向测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMagnetometerHeadingNorthTest,
	"UAVSimulator.Sensors.Magnetometer.HeadingNorth",
	UAV_TEST_FLAGS)

bool FMagnetometerHeadingNorthTest::RunTest(const FString& Parameters)
{
	UMagnetometerSensor* Mag = NewObject<UMagnetometerSensor>();
	Mag->SetEnabled(true);

	// Yaw=0 → 朝正北 → Heading≈0°
	FUAVState State = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector::ZeroVector, FRotator(0, 0, 0));
	Mag->UpdateSensor(State, 0.01f);

	FMagnetometerData Data = Mag->GetMagnetometerData();

	// 航向应在 0° 附近（容差 20° 考虑噪声）
	float Heading = FMath::Fmod(Data.Heading, 360.0f);
	if (Heading < 0.0f) Heading += 360.0f;
	TestTrue(TEXT("Heading should be near 0 when facing North"),
		Heading < 20.0f || Heading > 340.0f);

	return true;
}

// ==================== 正东航向测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMagnetometerHeadingEastTest,
	"UAVSimulator.Sensors.Magnetometer.HeadingEast",
	UAV_TEST_FLAGS)

bool FMagnetometerHeadingEastTest::RunTest(const FString& Parameters)
{
	UMagnetometerSensor* Mag = NewObject<UMagnetometerSensor>();
	Mag->SetEnabled(true);

	// Yaw=90 → 朝正东 → Heading≈90°
	FUAVState State = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector::ZeroVector, FRotator(0, 90, 0));
	Mag->UpdateSensor(State, 0.01f);

	FMagnetometerData Data = Mag->GetMagnetometerData();

	float Heading = Data.Heading;
	TestTrue(TEXT("Heading should be near 90 when facing East"),
		Heading > 70.0f && Heading < 110.0f);

	return true;
}

// ==================== 正西航向测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMagnetometerHeadingWestTest,
	"UAVSimulator.Sensors.Magnetometer.HeadingWest",
	UAV_TEST_FLAGS)

bool FMagnetometerHeadingWestTest::RunTest(const FString& Parameters)
{
	UMagnetometerSensor* Mag = NewObject<UMagnetometerSensor>();
	Mag->SetEnabled(true);

	// Yaw=270 → 朝正西 → Heading≈270°
	FUAVState State = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector::ZeroVector, FRotator(0, 270, 0));
	Mag->UpdateSensor(State, 0.01f);

	FMagnetometerData Data = Mag->GetMagnetometerData();

	float Heading = Data.Heading;
	TestTrue(TEXT("Heading should be near 270 when facing West"),
		Heading > 250.0f && Heading < 290.0f);

	return true;
}

// ==================== 统计收敛性测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMagnetometerConvergenceTest,
	"UAVSimulator.Sensors.Magnetometer.StatisticalConvergence",
	UAV_TEST_FLAGS)

bool FMagnetometerConvergenceTest::RunTest(const FString& Parameters)
{
	UMagnetometerSensor* Mag = NewObject<UMagnetometerSensor>();
	Mag->SetEnabled(true);

	// Yaw=45° → 东北方向
	FUAVState State = UAVTestHelpers::CreateUAVState(
		FVector(0, 0, 0), FVector::ZeroVector, FRotator(0, 45, 0));

	double HeadingSum = 0.0;
	int32 N = 200;

	for (int32 i = 0; i < N; ++i)
	{
		Mag->UpdateSensor(State, 0.01f);
		FMagnetometerData Data = Mag->GetMagnetometerData();
		HeadingSum += Data.Heading;
	}

	float AvgHeading = static_cast<float>(HeadingSum / N);

	// 200 样本均值应收敛到 45° 附近
	TestTrue(TEXT("Average heading should converge to ~45"),
		AvgHeading > 35.0f && AvgHeading < 55.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
