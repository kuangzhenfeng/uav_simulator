// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Control/StateEstimator.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 零加速度预测 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStateEstimatorPredictZeroAccelTest,
	"UAVSimulator.Control.StateEstimator.PredictZeroAcceleration",
	UAV_TEST_FLAGS)

bool FStateEstimatorPredictZeroAccelTest::RunTest(const FString& Parameters)
{
	UStateEstimator* Estimator = NewObject<UStateEstimator>();

	FUAVState InitState = UAVTestHelpers::CreateUAVState(
		FVector(100.0f, 200.0f, 300.0f),
		FVector::ZeroVector);
	Estimator->Reset(InitState);

	Estimator->Predict(FVector::ZeroVector, FVector::ZeroVector, 0.1f);

	FUAVState Result = Estimator->GetEstimatedState();
	UAV_TEST_VECTOR_EQUAL(Result.Position, FVector(100.0f, 200.0f, 300.0f), 1.0f);
	UAV_TEST_VECTOR_EQUAL(Result.Velocity, FVector::ZeroVector, 1.0f);

	return true;
}

// ==================== 恒定加速度预测 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStateEstimatorPredictConstAccelTest,
	"UAVSimulator.Control.StateEstimator.PredictConstantAcceleration",
	UAV_TEST_FLAGS)

bool FStateEstimatorPredictConstAccelTest::RunTest(const FString& Parameters)
{
	UStateEstimator* Estimator = NewObject<UStateEstimator>();

	FUAVState InitState = UAVTestHelpers::CreateUAVState(
		FVector::ZeroVector,
		FVector(100.0f, 0.0f, 0.0f));
	Estimator->Reset(InitState);

	FVector Accel(50.0f, 0.0f, 0.0f);
	float dt = 0.1f;
	Estimator->Predict(Accel, FVector::ZeroVector, dt);

	FUAVState Result = Estimator->GetEstimatedState();
	// p = v*dt + 0.5*a*dt^2 = 100*0.1 + 0.5*50*0.01 = 10 + 0.25 = 10.25
	UAV_TEST_FLOAT_EQUAL(Result.Position.X, 10.25f, 0.5f);
	// v = v + a*dt = 100 + 50*0.1 = 105
	UAV_TEST_FLOAT_EQUAL(Result.Velocity.X, 105.0f, 0.5f);

	return true;
}

// ==================== GPS 更新拉近估计值 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStateEstimatorUpdateGPSPullsTest,
	"UAVSimulator.Control.StateEstimator.UpdateGPSPullsEstimate",
	UAV_TEST_FLAGS)

bool FStateEstimatorUpdateGPSPullsTest::RunTest(const FString& Parameters)
{
	UStateEstimator* Estimator = NewObject<UStateEstimator>();

	Estimator->Reset(UAVTestHelpers::CreateUAVState(FVector::ZeroVector));

	FVector GPSPos(1000.0f, 0.0f, 0.0f);
	Estimator->UpdateGPS(GPSPos, FVector::ZeroVector);

	FUAVState Result = Estimator->GetEstimatedState();
	// 更新后位置应该向 GPS 测量值方向移动
	TestTrue(TEXT("Position should move toward GPS"), Result.Position.X > 0.0f);

	return true;
}

// ==================== 融合比纯预测更准确 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStateEstimatorFusionAccuracyTest,
	"UAVSimulator.Control.StateEstimator.FusionMoreAccurateThanPredictOnly",
	UAV_TEST_FLAGS)

bool FStateEstimatorFusionAccuracyTest::RunTest(const FString& Parameters)
{
	FVector TruePos(500.0f, 500.0f, 500.0f);

	// 纯预测
	UStateEstimator* PredictOnly = NewObject<UStateEstimator>();
	PredictOnly->Reset(UAVTestHelpers::CreateUAVState());
	PredictOnly->Predict(FVector(100.0f, 100.0f, 100.0f), FVector::ZeroVector, 0.1f);

	// 预测 + GPS 融合
	UStateEstimator* Fused = NewObject<UStateEstimator>();
	Fused->Reset(UAVTestHelpers::CreateUAVState());
	Fused->Predict(FVector(100.0f, 100.0f, 100.0f), FVector::ZeroVector, 0.1f);
	Fused->UpdateGPS(TruePos, FVector::ZeroVector);

	float PredictError = FVector::Dist(PredictOnly->GetEstimatedState().Position, TruePos);
	float FusedError = FVector::Dist(Fused->GetEstimatedState().Position, TruePos);

	TestTrue(TEXT("Fused estimate should be closer to truth"), FusedError < PredictError);

	return true;
}

// ==================== Reset 恢复状态 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStateEstimatorResetTest,
	"UAVSimulator.Control.StateEstimator.ResetRestoresState",
	UAV_TEST_FLAGS)

bool FStateEstimatorResetTest::RunTest(const FString& Parameters)
{
	UStateEstimator* Estimator = NewObject<UStateEstimator>();

	// 做一些预测改变状态
	Estimator->Reset(UAVTestHelpers::CreateUAVState(FVector(100.0f, 0.0f, 0.0f)));
	Estimator->Predict(FVector(50.0f, 0.0f, 0.0f), FVector::ZeroVector, 1.0f);

	// Reset 到新状态
	FUAVState NewState = UAVTestHelpers::CreateUAVState(FVector(999.0f, 888.0f, 777.0f));
	Estimator->Reset(NewState);

	FUAVState Result = Estimator->GetEstimatedState();
	UAV_TEST_VECTOR_EQUAL(Result.Position, FVector(999.0f, 888.0f, 777.0f), 1.0f);

	return true;
}

// ==================== 协方差在预测中增长 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStateEstimatorCovGrowsTest,
	"UAVSimulator.Control.StateEstimator.CovarianceGrowsDuringPredict",
	UAV_TEST_FLAGS)

bool FStateEstimatorCovGrowsTest::RunTest(const FString& Parameters)
{
	UStateEstimator* Estimator = NewObject<UStateEstimator>();
	Estimator->Reset(UAVTestHelpers::CreateUAVState());

	// 预测前 GPS 更新，记录更新后位置偏移量
	Estimator->UpdateGPS(FVector(100.0f, 0.0f, 0.0f), FVector::ZeroVector);
	float PullBefore = Estimator->GetEstimatedState().Position.X;

	// Reset 并做多次预测增大协方差
	Estimator->Reset(UAVTestHelpers::CreateUAVState());
	for (int i = 0; i < 50; ++i)
	{
		Estimator->Predict(FVector::ZeroVector, FVector::ZeroVector, 0.1f);
	}
	Estimator->UpdateGPS(FVector(100.0f, 0.0f, 0.0f), FVector::ZeroVector);
	float PullAfter = Estimator->GetEstimatedState().Position.X;

	// 协方差增大后，GPS 更新的拉力应该更大（卡尔曼增益更大）
	TestTrue(TEXT("GPS pull should be larger after covariance grows"), PullAfter > PullBefore);

	return true;
}

// ==================== 多次循环收敛 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStateEstimatorConvergenceTest,
	"UAVSimulator.Control.StateEstimator.MultipleCyclesConverge",
	UAV_TEST_FLAGS)

bool FStateEstimatorConvergenceTest::RunTest(const FString& Parameters)
{
	UStateEstimator* Estimator = NewObject<UStateEstimator>();
	Estimator->Reset(UAVTestHelpers::CreateUAVState());

	FVector TruePos(1000.0f, 500.0f, 300.0f);
	FVector TrueVel(10.0f, 5.0f, 0.0f);

	for (int i = 0; i < 100; ++i)
	{
		Estimator->Predict(FVector::ZeroVector, FVector::ZeroVector, 0.01f);
		Estimator->UpdateGPS(TruePos, TrueVel);
	}

	FUAVState Result = Estimator->GetEstimatedState();
	UAV_TEST_VECTOR_EQUAL(Result.Position, TruePos, 50.0f);
	UAV_TEST_VECTOR_EQUAL(Result.Velocity, TrueVel, 5.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
