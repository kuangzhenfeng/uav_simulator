// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Sensors/GPSSensor.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 输出在真实状态附近 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGPSSensorOutputNearTrueTest,
	"UAVSimulator.Sensors.GPS.OutputNearTrueState",
	UAV_TEST_FLAGS)

bool FGPSSensorOutputNearTrueTest::RunTest(const FString& Parameters)
{
	UGPSSensor* GPS = NewObject<UGPSSensor>();

	FUAVState TrueState = UAVTestHelpers::CreateUAVState(
		FVector(1000.0f, 2000.0f, 500.0f),
		FVector(100.0f, 50.0f, 0.0f));

	// 用足够大的 DeltaTime 触发更新 (10Hz -> 0.1s interval)
	GPS->UpdateSensor(TrueState, 0.2f);

	FGPSData Data = GPS->GetGPSData();
	TestTrue(TEXT("GPS data should be valid"), Data.bIsValid);
	// 位置噪声 StdDev=100cm, 5sigma 范围内
	UAV_TEST_VECTOR_EQUAL(Data.Position, TrueState.Position, 500.0f);
	UAV_TEST_VECTOR_EQUAL(Data.Velocity, TrueState.Velocity, 50.0f);

	return true;
}

// ==================== 统计收敛 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGPSSensorStatisticalConvergenceTest,
	"UAVSimulator.Sensors.GPS.StatisticalConvergence",
	UAV_TEST_FLAGS)

bool FGPSSensorStatisticalConvergenceTest::RunTest(const FString& Parameters)
{
	FVector TruePos(500.0f, 500.0f, 500.0f);
	FUAVState TrueState = UAVTestHelpers::CreateUAVState(TruePos);

	FVector Sum = FVector::ZeroVector;
	const int32 N = 200;

	for (int32 i = 0; i < N; ++i)
	{
		UGPSSensor* GPS = NewObject<UGPSSensor>();
		GPS->UpdateSensor(TrueState, 0.2f);
		Sum += GPS->GetGPSData().Position;
	}

	FVector Mean = Sum / static_cast<float>(N);
	UAV_TEST_VECTOR_EQUAL(Mean, TruePos, 50.0f);

	return true;
}

// ==================== 禁用后不更新 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGPSSensorDisabledTest,
	"UAVSimulator.Sensors.GPS.DisabledSensorNoUpdate",
	UAV_TEST_FLAGS)

bool FGPSSensorDisabledTest::RunTest(const FString& Parameters)
{
	UGPSSensor* GPS = NewObject<UGPSSensor>();
	GPS->SetEnabled(false);

	FUAVState TrueState = UAVTestHelpers::CreateUAVState(FVector(1000.0f, 2000.0f, 3000.0f));
	GPS->UpdateSensor(TrueState, 0.2f);

	FGPSData Data = GPS->GetGPSData();
	TestFalse(TEXT("Disabled GPS should not produce valid data"), Data.bIsValid);

	return true;
}

// ==================== 更新率控制 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGPSSensorRateControlTest,
	"UAVSimulator.Sensors.GPS.RateControl",
	UAV_TEST_FLAGS)

bool FGPSSensorRateControlTest::RunTest(const FString& Parameters)
{
	UGPSSensor* GPS = NewObject<UGPSSensor>();

	FUAVState State1 = UAVTestHelpers::CreateUAVState(FVector(100.0f, 0.0f, 0.0f));
	// DeltaTime 小于更新间隔 (1/10Hz = 0.1s)，不应更新
	GPS->UpdateSensor(State1, 0.05f);
	TestFalse(TEXT("Should not update before interval"), GPS->GetGPSData().bIsValid);

	// 累计时间超过间隔，应该更新
	FUAVState State2 = UAVTestHelpers::CreateUAVState(FVector(200.0f, 0.0f, 0.0f));
	GPS->UpdateSensor(State2, 0.06f);
	TestTrue(TEXT("Should update after interval"), GPS->GetGPSData().bIsValid);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
