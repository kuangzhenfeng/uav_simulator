// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "TelemetryRecorder.generated.h"

class AUAVPawn;
class UWindField;
class IFileHandle;

/**
 * 仿真遥测记录器。
 *
 * 由 AMultiAgentGameMode 持有，按固定节拍把仿真状态追加写成 Logs/telemetry.ndjson
 * （每行一个 JSON 对象），作为可视化 Web 的专用数据源，取代从 UE 运行时日志
 * 用正则解析的脆弱方式。
 *
 * 设计要点：
 * - 单一追加式文件，每行一个事件，天然支持实时 tail 与事后回放；UE 崩溃前
 *   已落盘的行不丢失。
 * - 坐标/单位为 UE 原生（cm，左手系 X前/Y右/Z上），坐标系变换在消费端（Python）完成。
 * - frame 行按仿真秒节拍（默认 20Hz），metrics 行按仿真秒（1Hz）。
 * - 静态数据（spawn/obstacle/waypoint/wind_config）在首帧一次性写出。
 * - 零侵入拉取：通过 AUAVPawn 各 public getter 读指标，不修改仿真主循环。
 *
 * 写盘策略：BeginPlay 时以 share-read、truncate 打开持久 IFileHandle，
 * 每行直接 Write+Flush，外部 Python 可同时 tail 而不被阻塞；BeginDestroy 释放句柄。
 */
UCLASS(ClassGroup = (Telemetry), meta = (BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UTelemetryRecorder : public UActorComponent
{
	GENERATED_BODY()

public:
	UTelemetryRecorder();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void BeginDestroy() override;

	/** 注入共享风场单例（由 GameMode 提供） */
	void SetWindField(UWindField* InWindField);

	/**
	 * 覆盖默认输出路径（默认 ProjectDir/Logs/telemetry.ndjson）。
	 * 必须在文件句柄首次打开之前调用。供单元测试重定向输出，避免污染真实遥测文件。
	 */
	void SetOutputPath(const FString& InPath) { OutputPathOverride = InPath; }

	/**
	 * 显式初始化：打开输出文件句柄并重置采样累加器。
	 * 正常运行由 BeginPlay 自动调用；单元测试中组件未走完整注册流程，
	 * 显式调用本方法避免依赖 BeginPlay 的 bRegistered 断言。
	 * @return 文件句柄是否成功打开
	 */
	bool InitializeOutput();

	/**
	 * 写一帧判决行（周期快照或终局，由 ScenarioEvaluatorComponent 调用）。
	 * 同一份判决同时落 ndjson（这里）与 scenario_result.json（由调用方写）。
	 */
	void WriteVerdict(float SimTime, bool bFinal, int32 Reached, int32 Total,
		float ClearanceCm, float LateralDevCm, float ElapsedSec,
		bool bCollided, const TArray<FString>& Failures);

	/**
	 * 等距降采样索引（保证首尾点入选）。
	 * 纯函数：从 Total 个点中均匀取最多 MaxCount 个索引，保证首尾入选、单调递增。
	 * 公开供单元测试验证未来轨迹点数上限的数据契约。
	 */
	static void SampleIndices(int32 Total, int32 MaxCount, TArray<int32>& OutIndices);

protected:
	// 帧采样节拍（仿真秒），默认 20Hz
	float FrameIntervalSec = 0.05f;
	// 指标采样节拍（仿真秒），默认 1Hz
	float MetricsIntervalSec = 1.0f;
	// 未来轨迹采样节拍（仿真秒），默认 5Hz。三类未来轨迹共用，避免 ndjson 体积失控
	float TrajectoryIntervalSec = 0.2f;
	// 单条未来轨迹降采样后的最大点数
	int32 MaxTrajectoryPoints = 32;

private:
	// 持久追加写句柄（share-read，外部可同时读取）
	TUniquePtr<IFileHandle> FileHandle;

	// 注入的风场（弱引用，避免循环）
	TWeakObjectPtr<UWindField> WindField;

	// 输出路径覆盖（默认空 -> ProjectDir/Logs/telemetry.ndjson；测试可注入临时路径）
	FString OutputPathOverride;

	// 帧采样累加器（仿真秒）
	float FrameAccum = 0.0f;

	// 指标采样累加器（仿真秒）
	float MetricsAccum = 0.0f;

	// 未来轨迹采样累加器（仿真秒）
	float TrajectoryAccum = 0.0f;

	// 静态数据是否已写
	bool bStaticWritten = false;

	// 各 agent 最近已记录的 SimEventSeq，用于边沿检测（只输出变化后的事件）
	TMap<int32, int32> LastEventSeq;

	// 底层写一行（自动追加换行，UTF-8 编码）
	void WriteLine(const FString& JsonLine);

	// 收集场景内全部 AUAVPawn（按 AgentID 升序）
	void CollectAgents(TArray<AUAVPawn*>& OutAgents) const;

	// 首帧：写 meta/spawn/obstacle/waypoint/wind_config
	void WriteStaticOnce(float SimTime);

	// 一帧采样：遍历机队写 frame + 风速 + event 边沿
	void WriteFrame(float SimTime);

	// 指标采样：写 metrics（从各 Pawn public getter 拉累计指标）
	void WriteMetrics(float SimTime);

	// 未来轨迹采样：遍历机队写三类未来轨迹行
	//   traj_opt  —— TrajectoryTracker 正在跟踪的优化轨迹
	//   traj_plan —— PlanningVisualizer 持久化规划路径
	//   traj_nmpc —— NMPC 预测轨迹（含障碍代价 cost）
	// 仅当对应数据有效时写该 agent 的行，停止后自然停写（消费端覆盖式快照）
	void WriteFutureTrajectories(float SimTime);
};
