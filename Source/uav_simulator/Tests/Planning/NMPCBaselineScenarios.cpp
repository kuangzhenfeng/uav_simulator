// Copyright Epic Games, Inc. All Rights Reserved.

#include "NMPCBaselineScenarios.h"

#if WITH_DEV_AUTOMATION_TESTS

void NMPCBaseline_GetScenarios(TArray<FNMPCBaselineScenario>& OutScenarios)
{
	OutScenarios.Empty();

	// ---- 场景 1：无障碍直线 ----
	{
		FNMPCBaselineScenario S;
		S.Name = TEXT("StraightLine_NoObstacle");
		S.StartPosition = FVector(0, 0, 0);
		S.StartVelocity = FVector(100, 0, 0);
		S.GoalPosition = FVector(5000, 0, 0);
		S.SimSteps = 200;
		S.MinClearanceRequired = 0.0f;
		S.MinProgressRequired = 500.0f;
		OutScenarios.Add(S);
	}

	// ---- 场景 2：单球正前方 ----
	{
		FNMPCBaselineScenario S;
		S.Name = TEXT("SingleSphere_Ahead");
		S.StartPosition = FVector(0, 0, 0);
		S.StartVelocity = FVector(200, 0, 0);
		S.GoalPosition = FVector(5000, 0, 0);
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(1500, 0, 0), 300.0f));
		S.SimSteps = 300;
		S.MinClearanceRequired = 0.0f;
		S.MinProgressRequired = 800.0f;
		OutScenarios.Add(S);
	}

	// ---- 场景 3：左右对称双障碍（走廊） ----
	{
		FNMPCBaselineScenario S;
		S.Name = TEXT("SymmetricCorridor");
		S.StartPosition = FVector(0, 0, 0);
		S.StartVelocity = FVector(200, 0, 0);
		S.GoalPosition = FVector(5000, 0, 0);
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(1500, 400, 0), 250.0f));
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(2, FVector(1500, -400, 0), 250.0f));
		S.SimSteps = 300;
		S.MinClearanceRequired = 0.0f;
		S.MinProgressRequired = 1000.0f;
		OutScenarios.Add(S);
	}

	// ---- 场景 4：对称封堵 ----
	{
		FNMPCBaselineScenario S;
		S.Name = TEXT("SymmetricBlockade");
		S.StartPosition = FVector(0, 0, 0);
		S.StartVelocity = FVector(200, 0, 0);
		S.GoalPosition = FVector(5000, 0, 0);
		// 前方密集放置，只留上下通过
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(1500, 0, 0), 400.0f));
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(2, FVector(1500, 600, 0), 300.0f));
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(3, FVector(1500, -600, 0), 300.0f));
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(4, FVector(1500, 0, 600), 300.0f));
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(5, FVector(1500, 0, -600), 300.0f));
		S.SimSteps = 400;
		S.MinClearanceRequired = 0.0f;
		S.MinProgressRequired = 800.0f;
		OutScenarios.Add(S);
	}

	// ---- 场景 5：U 形障碍 ----
	{
		FNMPCBaselineScenario S;
		S.Name = TEXT("UShapeObstacle");
		S.StartPosition = FVector(0, 0, 0);
		S.StartVelocity = FVector(200, 0, 0);
		S.GoalPosition = FVector(5000, 0, 0);
		// 三面围堵形成 U 形（开口朝 +Y）
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(1500, 0, 0), 400.0f));
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(2, FVector(1000, 600, 0), 350.0f));
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(3, FVector(2000, 600, 0), 350.0f));
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(4, FVector(1000, -600, 0), 350.0f));
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(5, FVector(2000, -600, 0), 350.0f));
		S.SimSteps = 400;
		S.MinClearanceRequired = 0.0f;
		S.MinProgressRequired = 800.0f;
		OutScenarios.Add(S);
	}

	// ---- 场景 6：窄通道 ----
	{
		FNMPCBaselineScenario S;
		S.Name = TEXT("NarrowPassage");
		S.StartPosition = FVector(0, 0, 0);
		S.StartVelocity = FVector(200, 0, 0);
		S.GoalPosition = FVector(5000, 0, 0);
		// 两障碍之间留约 400cm 通道
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(1500, 350, 0), 200.0f));
		S.Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(2, FVector(1500, -350, 0), 200.0f));
		S.SimSteps = 300;
		S.MinClearanceRequired = 0.0f;
		S.MinProgressRequired = 1000.0f;
		OutScenarios.Add(S);
	}

	// ---- 场景 7：动态横穿障碍 ----
	{
		FNMPCBaselineScenario S;
		S.Name = TEXT("DynamicCrossing");
		S.StartPosition = FVector(0, 0, 0);
		S.StartVelocity = FVector(200, 0, 0);
		S.GoalPosition = FVector(5000, 0, 0);
		FObstacleInfo DynObs = UAVTestHelpers::CreateSphereObstacle(1, FVector(1500, -1000, 0), 250.0f);
		DynObs.bIsDynamic = true;
		DynObs.Velocity = FVector(0, 300, 0); // 横穿
		S.Obstacles.Add(DynObs);
		S.SimSteps = 300;
		S.MinClearanceRequired = 0.0f;
		S.MinProgressRequired = 800.0f;
		OutScenarios.Add(S);
	}

	// ---- 场景 8：双机对向（模拟 CBF 场景，仅 NMPC 层） ----
	{
		FNMPCBaselineScenario S;
		S.Name = TEXT("DualAgentHeadOn");
		S.StartPosition = FVector(0, 0, 0);
		S.StartVelocity = FVector(200, 0, 0);
		S.GoalPosition = FVector(5000, 0, 0);
		// 将对向飞行的无人机建模为动态障碍物
		FObstacleInfo Agent2 = UAVTestHelpers::CreateSphereObstacle(1, FVector(3000, 0, 0), 300.0f);
		Agent2.bIsDynamic = true;
		Agent2.Velocity = FVector(-200, 0, 0);
		S.Obstacles.Add(Agent2);
		S.SimSteps = 400;
		S.MinClearanceRequired = 0.0f;
		S.MinProgressRequired = 500.0f;
		OutScenarios.Add(S);
	}
}

bool NMPCBaseline_RunScenario(
	const FNMPCBaselineScenario& Scenario,
	UNMPCAvoidance* NMPC,
	float& OutMinClearance,
	float& OutProgress)
{
	FVector Pos = Scenario.StartPosition;
	FVector Vel = Scenario.StartVelocity;
	const float dt = NMPC->Config.GetDt();

	// 初始化动态障碍物（可变状态，跨帧累加位移）
	TArray<FObstacleInfo> DynObstacles = Scenario.Obstacles;

	OutMinClearance = MAX_FLT;
	bool bHasNaN = false;

	for (int32 Step = 0; Step < Scenario.SimSteps; ++Step)
	{
		// 构建参考点（简单目标追踪）
		TArray<FVector> RefPoints;
		const int32 N = NMPC->Config.Solver.PredictionSteps;
		for (int32 k = 0; k <= N; ++k)
		{
			RefPoints.Add(Scenario.GoalPosition);
		}

		// 使用累积位置的动态障碍物（每帧推进 dt）
		for (FObstacleInfo& Obs : DynObstacles)
		{
			if (Obs.bIsDynamic)
			{
				Obs.Center = Obs.Center + Obs.Velocity * dt;
			}
		}

		// NMPC 求解
		FNMPCAvoidanceResult Result = NMPC->ComputeAvoidance(Pos, Vel, RefPoints, DynObstacles);

		// 检查 NaN
		if (Result.OptimalAcceleration.ContainsNaN() || Result.CorrectedTarget.ContainsNaN())
		{
			bHasNaN = true;
			break;
		}

		// Euler 积分推进状态
		Vel = Vel + Result.OptimalAcceleration * dt;

		// 速度限制
		float VelMag = Vel.Size();
		if (VelMag > NMPC->Config.Actuator.MaxVelocity)
		{
			Vel = Vel * (NMPC->Config.Actuator.MaxVelocity / VelMag);
		}

		Pos = Pos + Vel * dt;

		// 计算到障碍物最小距离
		for (const FObstacleInfo& Obs : DynObstacles)
		{
			float Dist = NMPC->CalculateDistanceToObstacle(Pos, Obs);
			OutMinClearance = FMath::Min(OutMinClearance, Dist);
		}
	}

	// 无障碍时 MinClearance 保持 MAX_FLT
	if (Scenario.Obstacles.Num() == 0)
	{
		OutMinClearance = MAX_FLT;
	}

	// 前进距离
	OutProgress = FVector::Dist(Pos, Scenario.StartPosition);

	// 验收判定
	bool bPassed = true;
	if (bHasNaN) bPassed = false;
	if (OutMinClearance < Scenario.MinClearanceRequired) bPassed = false;
	if (OutProgress < Scenario.MinProgressRequired) bPassed = false;

	return bPassed;
}

// ==================== 基准场景自动化测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCBaselineTest,
	"UAVSimulator.Planning.NMPCBaseline.AllScenarios",
	UAV_TEST_FLAGS)

bool FNMPCBaselineTest::RunTest(const FString& Parameters)
{
	TArray<FNMPCBaselineScenario> Scenarios;
	NMPCBaseline_GetScenarios(Scenarios);

	TestTrue(TEXT("Should have at least 8 scenarios"), Scenarios.Num() >= 8);

	int32 PassCount = 0;
	int32 FailCount = 0;

	for (const FNMPCBaselineScenario& Scenario : Scenarios)
	{
		UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

		float MinClearance = 0.0f;
		float Progress = 0.0f;
		bool bPassed = NMPCBaseline_RunScenario(Scenario, NMPC, MinClearance, Progress);

		FString StatusStr = bPassed ? TEXT("PASS") : TEXT("FAIL");
		UE_LOG(LogTemp, Log, TEXT("[Baseline] %s: %s  Clear=%.0f  Progress=%.0f"),
			*Scenario.Name, *StatusStr, MinClearance, Progress);

		if (bPassed)
		{
			++PassCount;
		}
		else
		{
			++FailCount;
			// 记录失败场景但不禁用其他场景
			UE_LOG(LogTemp, Warning, TEXT("[Baseline] FAILED: %s  Clear=%.0f(need>=%.0f)  Progress=%.0f(need>=%.0f)"),
				*Scenario.Name, MinClearance, Scenario.MinClearanceRequired,
				Progress, Scenario.MinProgressRequired);
		}

		// 每个场景独立测试
		FString ScenarioTestName = FString::Printf(TEXT("Scenario '%s' passed"), *Scenario.Name);
		TestTrue(*ScenarioTestName, bPassed);
	}

	UE_LOG(LogTemp, Log, TEXT("[Baseline] Summary: %d/%d passed"), PassCount, Scenarios.Num());

	return FailCount == 0;
}

#endif // WITH_DEV_AUTOMATION_TESTS
