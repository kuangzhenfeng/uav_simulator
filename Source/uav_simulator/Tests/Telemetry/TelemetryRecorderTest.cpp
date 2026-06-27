// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Telemetry/TelemetryRecorder.h"
#include "../../Core/UAVPawn.h"
#include "../../Core/UAVTypes.h"
#include "../../Planning/PlanningVisualizer.h"

#if WITH_DEV_AUTOMATION_TESTS

// ==================== 降采样索引（纯函数契约）====================
// TelemetryRecorder::SampleIndices 是 static 纯函数，验证首尾入选 + 上限 + 等距。
// 这是未来轨迹数据契约的关键：Web 端依赖点数受控（≤ MaxTrajectoryPoints）。

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTelemetrySampleIndicesTest,
	"UAVSimulator.Telemetry.SampleIndices",
	UAV_TEST_FLAGS)

bool FTelemetrySampleIndicesTest::RunTest(const FString& Parameters)
{
	TArray<int32> Idx;

	// 点数 <= 上限：全选
	UTelemetryRecorder::SampleIndices(5, 32, Idx);
	TestEqual(TEXT("5点/上限32: 全选5个"), Idx.Num(), 5);
	TestEqual(TEXT("首点入选"), Idx[0], 0);
	TestEqual(TEXT("尾点入选"), Idx.Last(), 4);

	// 点数 == 上限
	UTelemetryRecorder::SampleIndices(32, 32, Idx);
	TestEqual(TEXT("32点/上限32: 全选"), Idx.Num(), 32);

	// 点数 > 上限：降采样到上限，首尾入选
	UTelemetryRecorder::SampleIndices(100, 32, Idx);
	TestEqual(TEXT("100点/上限32: 降采样到32"), Idx.Num(), 32);
	TestEqual(TEXT("降采样首点=0"), Idx[0], 0);
	TestEqual(TEXT("降采样尾点=99"), Idx.Last(), 99);
	// 索引单调递增
	bool bMonotonic = true;
	for (int32 i = 1; i < Idx.Num(); ++i)
	{
		if (Idx[i] <= Idx[i - 1]) { bMonotonic = false; break; }
	}
	TestTrue(TEXT("降采样索引单调递增"), bMonotonic);

	// 边界：0 点
	UTelemetryRecorder::SampleIndices(0, 32, Idx);
	TestEqual(TEXT("0点: 空结果"), Idx.Num(), 0);

	// 边界：1 点
	UTelemetryRecorder::SampleIndices(1, 32, Idx);
	TestEqual(TEXT("1点: 单点"), Idx.Num(), 1);

	return true;
}

// ==================== 规划路径门控（PlanningVisualizer 持久路径读回）====================
// traj_plan 的数据契约门控：仅当 PlanningVisualizer 有 ≥2 点的持久路径时，
// Recorder 才会写 traj_plan 行。验证 GetPersistentPath 的门控语义（点数不足返回 false）。
// 不依赖 SpawnActor<>/World（避开 automation worker 的 spawn 环境问题），直接测组件逻辑。

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTelemetryPlannedPathGateTest,
	"UAVSimulator.Telemetry.PlannedPathGate",
	UAV_TEST_FLAGS)

bool FTelemetryPlannedPathGateTest::RunTest(const FString& Parameters)
{
	// 直接构造 PlanningVisualizer 组件（Outer 用 GetTransientPackage，无需 World）
	UPlanningVisualizer* Vis = NewObject<UPlanningVisualizer>(GetTransientPackage());
	if (!TestNotNull(TEXT("PlanningVisualizer 创建成功"), Vis))
	{
		return false;
	}

	// 初始：无持久路径 → 门控不满足
	TArray<FVector> ReadBack;
	TestFalse(TEXT("初始无持久路径: 门控不满足"), Vis->GetPersistentPath(ReadBack));

	// 设置 1 点路径（不足 2 点）→ 门控仍不满足
	TArray<FVector> OnePoint = { FVector(0, 0, 200) };
	Vis->SetPersistentPath(OnePoint);
	TestFalse(TEXT("1点路径不足: 门控不满足"), Vis->GetPersistentPath(ReadBack));

	// 设置有效路径（≥2 点）→ 门控满足，读回一致
	TArray<FVector> PlanPath = { FVector(0, 0, 200), FVector(3000, 500, 200), FVector(9000, 0, 200) };
	Vis->SetPersistentPath(PlanPath);
	TestTrue(TEXT("3点路径: 门控满足"), Vis->GetPersistentPath(ReadBack));
	TestEqual(TEXT("读回点数一致"), ReadBack.Num(), 3);
	UAV_TEST_VECTOR_EQUAL_DEFAULT(ReadBack[0], FVector(0, 0, 200));
	UAV_TEST_VECTOR_EQUAL_DEFAULT(ReadBack.Last(), FVector(9000, 0, 200));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
