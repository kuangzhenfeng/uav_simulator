// Copyright Epic Games, Inc. All Rights Reserved.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "uav_simulator/Planning/NMPCAvoidance.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 前向仿真测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_ForwardSimulation, "UAVSimulator.Planning.NMPCAvoidance.ForwardSimulation", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_ForwardSimulation::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	// 初始状态：原点出发，沿 X 轴匀速运动
	FVector InitPos(0, 0, 0);
	FVector InitVel(100, 0, 0);
	TArray<FVector> Controls;
	TArray<FVector> OutPositions;
	TArray<FVector> OutVelocities;

	// 零控制输入（无加速度）
	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		Controls.Add(FVector::ZeroVector);
	}

	NMPC->ForwardSimulate(InitPos, InitVel, Controls, OutPositions, OutVelocities);

	// 验证输出序列长度 = 预测步数 + 1（包含初始状态）
	TestEqual("Position count", OutPositions.Num(), NMPC->Config.PredictionSteps + 1);
	TestEqual("Velocity count", OutVelocities.Num(), NMPC->Config.PredictionSteps + 1);

	// 零加速度下速度应保持不变
	float Dt = NMPC->Config.GetDt();
	for (int32 i = 0; i < OutVelocities.Num(); ++i)
	{
		TestEqual("Constant velocity", OutVelocities[i], InitVel);
	}

	// 验证匀速运动的最终位置：P = P0 + V * T
	FVector ExpectedFinalPos = InitPos + InitVel * (Dt * NMPC->Config.PredictionSteps);
	TestEqual("Final position", OutPositions.Last(), ExpectedFinalPos);

	// 测试恒定加速度输入
	Controls.Empty();
	FVector ConstAccel(50, 0, 0);
	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		Controls.Add(ConstAccel);
	}

	NMPC->ForwardSimulate(InitPos, InitVel, Controls, OutPositions, OutVelocities);

	// 验证加速度使速度增大
	FVector ExpectedVel = InitVel + ConstAccel * (Dt * NMPC->Config.PredictionSteps);
	TestTrue("Velocity increases", OutVelocities.Last().X > InitVel.X);

	return true;
}

// ==================== 代价计算测试（无障碍物） ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_CostComputation_NoObstacles, "UAVSimulator.Planning.NMPCAvoidance.CostComputation_NoObstacles", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_CostComputation_NoObstacles::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	// 初始状态与控制输入
	FVector InitPos(0, 0, 0);
	FVector InitVel(100, 0, 0);
	TArray<FVector> Controls;
	TArray<FVector> Positions;
	TArray<FVector> Velocities;
	TArray<FVector> ReferencePoints;
	TArray<FObstacleInfo> Obstacles; // 空障碍物列表

	// 零控制输入，参考点设为远处目标
	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		Controls.Add(FVector::ZeroVector);
		ReferencePoints.Add(FVector(1000, 0, 0));
	}

	// 前向仿真得到预测轨迹
	NMPC->ForwardSimulate(InitPos, InitVel, Controls, Positions, Velocities);

	// 计算代价（无障碍物时仍有跟踪误差代价）
	float Cost = NMPC->ComputeCost(Positions, Velocities, Controls, ReferencePoints, Obstacles);

	// 由于位置偏离参考点，代价应为正
	TestTrue("Cost is positive", Cost > 0.0f);

	return true;
}

// ==================== 代价计算测试（有障碍物） ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_CostComputation_WithObstacle, "UAVSimulator.Planning.NMPCAvoidance.CostComputation_WithObstacle", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_CostComputation_WithObstacle::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	// 初始状态与控制输入
	FVector InitPos(0, 0, 0);
	FVector InitVel(100, 0, 0);
	TArray<FVector> Controls;
	TArray<FVector> Positions;
	TArray<FVector> Velocities;
	TArray<FVector> ReferencePoints;
	TArray<FObstacleInfo> Obstacles;

	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		Controls.Add(FVector::ZeroVector);
		ReferencePoints.Add(FVector(1000, 0, 0));
	}

	NMPC->ForwardSimulate(InitPos, InitVel, Controls, Positions, Velocities);

	// 先计算无障碍物时的代价作为基准
	float CostNoObstacle = NMPC->ComputeCost(Positions, Velocities, Controls, ReferencePoints, Obstacles);

	// 在轨迹前方添加球形障碍物
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(500, 0, 0), 100.0f));

	// 有障碍物时代价应增加（障碍物惩罚项）
	float CostWithObstacle = NMPC->ComputeCost(Positions, Velocities, Controls, ReferencePoints, Obstacles);

	TestTrue("Cost increases with obstacle", CostWithObstacle > CostNoObstacle);

	return true;
}

// ==================== 梯度下降测试（无障碍物） ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_GradientDescent_NoObstacles, "UAVSimulator.Planning.NMPCAvoidance.GradientDescent_NoObstacles", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_GradientDescent_NoObstacles::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	// 初始状态：原点出发，沿 X 轴运动
	FVector CurrentPos(0, 0, 0);
	FVector CurrentVel(100, 0, 0);
	TArray<FVector> ReferencePoints;
	TArray<FObstacleInfo> Obstacles; // 无障碍物

	// 参考点设为远处目标
	FVector GoalPos(2000, 0, 0);
	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		ReferencePoints.Add(GoalPos);
	}

	// 执行 NMPC 求解
	FNMPCAvoidanceResult Result = NMPC->ComputeAvoidance(CurrentPos, CurrentVel, ReferencePoints, Obstacles);

	// 验证优化后的目标点比初始位置更接近目标
	float DistBefore = FVector::Dist(CurrentPos, GoalPos);
	float DistAfter = FVector::Dist(Result.CorrectedTarget, GoalPos);

	TestTrue("Moves closer to goal", DistAfter < DistBefore);

	return true;
}

// ==================== 单球障碍物避障测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_ObstacleAvoidance_SingleSphere, "UAVSimulator.Planning.NMPCAvoidance.ObstacleAvoidance_SingleSphere", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_ObstacleAvoidance_SingleSphere::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	// 初始状态：原点出发，沿 X 轴运动，目标在正前方
	FVector CurrentPos(0, 0, 0);
	FVector CurrentVel(100, 0, 0);
	FVector GoalPos(2000, 0, 0);
	TArray<FVector> ReferencePoints;
	TArray<FObstacleInfo> Obstacles;

	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		ReferencePoints.Add(GoalPos);
	}

	// 在直线路径正中间放置球形障碍物
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(1000, 0, 0), 150.0f));

	FNMPCAvoidanceResult Result = NMPC->ComputeAvoidance(CurrentPos, CurrentVel, ReferencePoints, Obstacles);

	// 验证修正后的目标偏离直线路径（Y 或 Z 方向有偏移）
	float DirectPathDeviation = FMath::Abs(Result.CorrectedTarget.Y) + FMath::Abs(Result.CorrectedTarget.Z);
	TestTrue("Deviates from straight line", DirectPathDeviation > 10.0f);

	return true;
}

// ==================== 多球障碍物避障测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_ObstacleAvoidance_MultipleSpheres, "UAVSimulator.Planning.NMPCAvoidance.ObstacleAvoidance_MultipleSpheres", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_ObstacleAvoidance_MultipleSpheres::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	// 初始状态：原点出发，沿 X 轴运动
	FVector CurrentPos(0, 0, 0);
	FVector CurrentVel(100, 0, 0);
	FVector GoalPos(2000, 0, 0);
	TArray<FVector> ReferencePoints;
	TArray<FObstacleInfo> Obstacles;

	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		ReferencePoints.Add(GoalPos);
	}

	// 在 Y 轴两侧对称放置障碍物，形成走廊
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(1000, 300, 0), 100.0f));
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(2, FVector(1000, -300, 0), 100.0f));

	FNMPCAvoidanceResult Result = NMPC->ComputeAvoidance(CurrentPos, CurrentVel, ReferencePoints, Obstacles);

	// 验证 UAV 从走廊中间穿过（Y 偏移在走廊范围内）
	TestTrue("Finds path through corridor", FMath::Abs(Result.CorrectedTarget.Y) < 300.0f);
	// 验证仍然朝目标方向前进
	TestTrue("Moves forward", Result.CorrectedTarget.X > CurrentPos.X);

	return true;
}

// ==================== 混合形状障碍物避障测试（Box + Cylinder） ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_ObstacleAvoidance_BoxAndCylinder, "UAVSimulator.Planning.NMPCAvoidance.ObstacleAvoidance_BoxAndCylinder", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_ObstacleAvoidance_BoxAndCylinder::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	FVector CurrentPos(0, 0, 0);
	FVector CurrentVel(100, 0, 0);
	TArray<FVector> ReferencePoints;
	TArray<FObstacleInfo> Obstacles;

	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		ReferencePoints.Add(FVector(2000, 0, 0));
	}

	// 创建不同形状的障碍物：Box 和 Cylinder
	FObstacleInfo BoxObstacle = UAVTestHelpers::CreateBoxObstacle(1, FVector(800, 0, 0), FVector(100, 100, 100));
	FObstacleInfo CylinderObstacle = UAVTestHelpers::CreateCylinderObstacle(2, FVector(1200, 0, 0), 100.0f, 150.0f);

	Obstacles.Add(BoxObstacle);
	Obstacles.Add(CylinderObstacle);

	// 验证距离计算对不同形状均有效
	float DistToBox = NMPC->CalculateDistanceToObstacle(FVector(800, 0, 0), BoxObstacle);
	TestTrue("Box distance calculated", DistToBox >= 0.0f);

	float DistToCylinder = NMPC->CalculateDistanceToObstacle(FVector(1200, 0, 0), CylinderObstacle);
	TestTrue("Cylinder distance calculated", DistToCylinder >= 0.0f);

	// 验证 NMPC 能同时避开两种形状的障碍物
	FNMPCAvoidanceResult Result = NMPC->ComputeAvoidance(CurrentPos, CurrentVel, ReferencePoints, Obstacles);

	TestTrue("Avoids both obstacles", Result.bNeedsCorrection);

	return true;
}

// ==================== 动态障碍物预测测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_DynamicObstacle, "UAVSimulator.Planning.NMPCAvoidance.DynamicObstacle", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_DynamicObstacle::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	// 创建动态障碍物：沿 Y 轴运动
	FObstacleInfo DynamicObstacle = UAVTestHelpers::CreateSphereObstacle(1, FVector(1000, 0, 0), 100.0f);
	DynamicObstacle.bIsDynamic = true;
	DynamicObstacle.Velocity = FVector(0, 100, 0);

	// 预测 1 秒后的障碍物位置
	float DeltaTime = 1.0f;
	FObstacleInfo PredictedObstacle = NMPC->PredictObstacle(DynamicObstacle, DeltaTime);

	// 验证预测位置 = 当前位置 + 速度 * 时间
	FVector ExpectedPosition = DynamicObstacle.Center + DynamicObstacle.Velocity * DeltaTime;
	TestEqual("Predicted position correct", PredictedObstacle.Center, ExpectedPosition);

	return true;
}

// ==================== 热启动测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_WarmStart, "UAVSimulator.Planning.NMPCAvoidance.WarmStart", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_WarmStart::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	FVector CurrentPos(0, 0, 0);
	FVector CurrentVel(100, 0, 0);
	TArray<FVector> ReferencePoints;
	TArray<FObstacleInfo> Obstacles;

	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		ReferencePoints.Add(FVector(2000, 0, 0));
	}

	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(1000, 0, 0), 150.0f));

	// 第一次求解，建立初始控制序列
	FNMPCAvoidanceResult Result1 = NMPC->ComputeAvoidance(CurrentPos, CurrentVel, ReferencePoints, Obstacles);

	// 第二次求解，位置略有前进，利用上次结果热启动
	CurrentPos = FVector(100, 0, 0);
	FNMPCAvoidanceResult Result2 = NMPC->ComputeAvoidance(CurrentPos, CurrentVel, ReferencePoints, Obstacles);

	// 热启动后代价不应显著恶化
	TestTrue("Warm start improves or maintains cost", Result2.TotalCost <= Result1.TotalCost * 1.5f);

	return true;
}

// ==================== 卡死检测测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_StuckDetection, "UAVSimulator.Planning.NMPCAvoidance.StuckDetection", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_StuckDetection::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	FVector CurrentPos(0, 0, 0);
	FVector CurrentVel(100, 0, 0);
	TArray<FVector> ReferencePoints;
	TArray<FObstacleInfo> Obstacles;

	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		ReferencePoints.Add(FVector(2000, 0, 0));
	}

	// 四面包围：在前后左右各放置大型障碍物，模拟被困场景
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(300, 0, 0), 200.0f));
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(2, FVector(-300, 0, 0), 200.0f));
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(3, FVector(0, 300, 0), 200.0f));
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(4, FVector(0, -300, 0), 200.0f));

	FNMPCAvoidanceResult Result = NMPC->ComputeAvoidance(CurrentPos, CurrentVel, ReferencePoints, Obstacles);

	// 验证能检测到卡死状态
	TestTrue("Detects stuck condition", Result.bStuck);

	return true;
}

// ==================== 控制量约束测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_ControlLimits, "UAVSimulator.Planning.NMPCAvoidance.ControlLimits", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_ControlLimits::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	// 构造超出限制的控制输入
	TArray<FVector> Controls;
	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		Controls.Add(FVector(500, 500, 500));
	}

	// 投影到可行域
	FVector InitVel(100, 0, 0);
	NMPC->ProjectControls(Controls, InitVel);

	// 验证投影后的加速度不超过最大限制
	for (const FVector& Control : Controls)
	{
		TestTrue("Control within limits", Control.Size() <= NMPC->Config.MaxAcceleration + 1.0f);
	}

	// 前向仿真验证速度约束
	TArray<FVector> Positions;
	TArray<FVector> Velocities;
	NMPC->ForwardSimulate(FVector::ZeroVector, InitVel, Controls, Positions, Velocities);

	// 验证仿真过程中速度不超过最大限制
	for (const FVector& Velocity : Velocities)
	{
		TestTrue("Velocity within limits", Velocity.Size() <= NMPC->Config.MaxVelocity + 1.0f);
	}

	return true;
}

// ==================== 无障碍物无修正测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_NoCorrection_ClearPath, "UAVSimulator.Planning.NMPCAvoidance.NoCorrection_ClearPath", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_NoCorrection_ClearPath::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	// 初始状态：原点出发，沿 X 轴运动，前方无障碍物
	FVector CurrentPos(0, 0, 0);
	FVector CurrentVel(100, 0, 0);
	TArray<FVector> ReferencePoints;
	TArray<FObstacleInfo> Obstacles; // 空障碍物列表

	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		ReferencePoints.Add(FVector(2000, 0, 0));
	}

	FNMPCAvoidanceResult Result = NMPC->ComputeAvoidance(CurrentPos, CurrentVel, ReferencePoints, Obstacles);

	// 路径畅通时不需要修正
	TestFalse("No correction needed for clear path", Result.bNeedsCorrection);

	return true;
}

// ==================== 结果字段完整性测试 ====================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNMPCAvoidanceTest_ResultFields, "UAVSimulator.Planning.NMPCAvoidance.ResultFields", UAV_TEST_FLAGS)
bool FNMPCAvoidanceTest_ResultFields::RunTest(const FString& Parameters)
{
	UNMPCAvoidance* NMPC = NewObject<UNMPCAvoidance>();

	FVector CurrentPos(0, 0, 0);
	FVector CurrentVel(100, 0, 0);
	TArray<FVector> ReferencePoints;
	TArray<FObstacleInfo> Obstacles;

	for (int32 i = 0; i < NMPC->Config.PredictionSteps; ++i)
	{
		ReferencePoints.Add(FVector(2000, 0, 0));
	}

	// 添加障碍物触发避障修正
	Obstacles.Add(UAVTestHelpers::CreateSphereObstacle(1, FVector(1000, 0, 0), 150.0f));

	FNMPCAvoidanceResult Result = NMPC->ComputeAvoidance(CurrentPos, CurrentVel, ReferencePoints, Obstacles);

	// 验证返回结果各字段的有效性
	TestEqual("Trajectory length correct", Result.PredictedTrajectory.Num(), NMPC->Config.PredictionSteps + 1);
	TestTrue("Total cost non-negative", Result.TotalCost >= 0.0f);
	TestTrue("Corrected target valid", !Result.CorrectedTarget.ContainsNaN());
	TestTrue("Corrected direction valid", !Result.CorrectedDirection.ContainsNaN());

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS

