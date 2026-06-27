// Copyright Epic Games, Inc. All Rights Reserved.

#include "TelemetryRecorder.h"
#include "../uav_simulator.h"
#include "../Core/UAVPawn.h"
#include "../Core/UAVProductManager.h"
#include "../Core/UAVTypes.h"
#include "../Planning/ObstacleManager.h"
#include "../Planning/TrajectoryTracker.h"
#include "../Planning/PlanningVisualizer.h"
#include "../Planning/NMPCAvoidance.h"
#include "../Mission/MissionComponent.h"
#include "../Environment/WindField.h"
#include "../Environment/EnvironmentTypes.h"

#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"
#include "EngineUtils.h"
#include "GameFramework/WorldSettings.h"

DEFINE_LOG_CATEGORY_STATIC(LogTelemetry, Log, All);

UTelemetryRecorder::UTelemetryRecorder()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

// JSON 字符串转义（仅处理必要的控制字符与引号/反斜杠）
static FString JsonEscape(const FString& In)
{
	if (In.IsEmpty()) return TEXT("");
	return In.Replace(TEXT("\\"), TEXT("\\\\"))
		.Replace(TEXT("\""), TEXT("\\\""))
		.Replace(TEXT("\n"), TEXT("\\n"))
		.Replace(TEXT("\r"), TEXT("\\r"))
		.Replace(TEXT("\t"), TEXT("\\t"));
}

void UTelemetryRecorder::SetWindField(UWindField* InWindField)
{
	WindField = InWindField;
}

void UTelemetryRecorder::BeginPlay()
{
	Super::BeginPlay();
	InitializeOutput();
}

bool UTelemetryRecorder::InitializeOutput()
{
	const FString Path = OutputPathOverride.IsEmpty()
		? FPaths::Combine(FPaths::ProjectDir(), TEXT("Logs/telemetry.ndjson"))
		: OutputPathOverride;

	// 确保目录存在
	FPlatformFileManager::Get().GetPlatformFile().CreateDirectoryTree(*FPaths::GetPath(Path));

	// 打开 share-read + truncate 的持久句柄：覆盖旧文件，后续逐行追加写，
	// 外部 Python 可同时 tail 而不被独占阻塞。
	// 使用 FPlatformFile::OpenWrite(AllowRead=true) 跨平台打开持久 IFileHandle。
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	FileHandle.Reset(PlatformFile.OpenWrite(*Path, /*bAppend=*/false, /*bAllowRead=*/true));

	if (!FileHandle.IsValid())
	{
		UE_LOG(LogTelemetry, Error, TEXT("[Telemetry] Failed to open %s for writing"), *Path);
		return false;
	}

	UE_LOG(LogTelemetry, Log, TEXT("[Telemetry] Recording to %s"), *Path);
	FrameAccum = 0.0f;
	MetricsAccum = 0.0f;
	TrajectoryAccum = 0.0f;
	bStaticWritten = false;
	LastEventSeq.Reset();
	return true;
}

void UTelemetryRecorder::ResetForNewScenario()
{
	// 先关闭旧句柄（BeginPlay/上一次打开的），再 truncate 重开，确保文件被覆盖而非追加。
	FileHandle.Reset();
	InitializeOutput();
}

void UTelemetryRecorder::WriteReloadMarker()
{
	if (!FileHandle.IsValid()) return;
	// reload 标记无仿真时间意义，t 取 0；前端据此重置游标、刷新静态数据。
	WriteLine(TEXT("{\"type\":\"reload\",\"t\":0.0}"));
}

void UTelemetryRecorder::BeginDestroy()
{
	FileHandle.Reset();
	Super::BeginDestroy();
}

void UTelemetryRecorder::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// 关闭持久文件句柄：World 销毁时立即释放，避免句柄泄漏导致后续场景/测试 truncate 打开失败。
	// （此前只在 BeginDestroy 释放，但 BeginDestroy 在 GC 时才触发，测试间 World 快速销毁时
	//  旧句柄仍占着 telemetry.ndjson，导致下一个测试 OpenWrite 失败。）
	CloseOutput();
	Super::EndPlay(EndPlayReason);
}

void UTelemetryRecorder::CloseOutput()
{
	FileHandle.Reset();
}

void UTelemetryRecorder::WriteLine(const FString& JsonLine)
{
	if (!FileHandle.IsValid()) return;
	FTCHARToUTF8 Conv(*JsonLine);
	FileHandle->Write(reinterpret_cast<const uint8*>(Conv.Get()), Conv.Length());
	static const uint8 NewLine[] = { '\n' };
	FileHandle->Write(NewLine, 1);
	FileHandle->Flush();
}

void UTelemetryRecorder::CollectAgents(TArray<AUAVPawn*>& OutAgents) const
{
	OutAgents.Reset();
	if (UWorld* World = GetWorld())
	{
		for (TActorIterator<AUAVPawn> It(World); It; ++It)
		{
			AUAVPawn* Pawn = *It;
			if (Pawn && Pawn->GetAgentID() >= 0)
			{
				OutAgents.Add(Pawn);
			}
		}
	}
	OutAgents.Sort([](const AUAVPawn& A, const AUAVPawn& B)
	{
		return A.GetAgentID() < B.GetAgentID();
	});
}

void UTelemetryRecorder::WriteStaticOnce(float SimTime)
{
	TArray<AUAVPawn*> Agents;
	CollectAgents(Agents);

	// ---- meta ----
	float TimeDilation = 1.0f;
	if (UWorld* W = GetWorld())
	{
		if (AWorldSettings* WS = W->GetWorldSettings())
		{
			TimeDilation = WS->GetEffectiveTimeDilation();
		}
	}
	WriteLine(FString::Printf(
		TEXT("{\"type\":\"meta\",\"t\":%.3f,\"version\":1,\"slomo\":%.3f}"),
		SimTime, TimeDilation));

	// ---- wind_config ----
	if (WindField.IsValid())
	{
		const FWindConfig Cfg = WindField->GetWindConfig();
		const FVector Steady = Cfg.SteadyWindVelocity;
		WriteLine(FString::Printf(
			TEXT("{\"type\":\"wind_config\",\"t\":%.3f,\"windType\":%d,\"enabled\":%s,"
				"\"steady\":[%.2f,%.2f,%.2f]}"),
			SimTime, (int32)Cfg.WindType, Cfg.bEnabled ? TEXT("true") : TEXT("false"),
			Steady.X, Steady.Y, Steady.Z));
	}

	// 场景代理：取第一个有障碍/任务组件的 Pawn，用于读障碍与航点
	AUAVPawn* SceneProxy = nullptr;
	for (AUAVPawn* P : Agents)
	{
		if (P->GetObstacleManager() || P->GetMissionComponent())
		{
			SceneProxy = P;
			break;
		}
	}

	// ---- spawn（每个 agent 一行） ----
	for (AUAVPawn* P : Agents)
	{
		const FVector Pos = P->GetUAVState().Position;
		const FUAVModelSpec Spec = FUAVProductManager::GetModelSpec(P->GetModelID());
		WriteLine(FString::Printf(
			TEXT("{\"type\":\"spawn\",\"t\":%.3f,\"agent\":%d,\"model\":\"%s\",\"maxVelCm\":%.0f,"
				"\"collisionRadiusCm\":%.1f,\"pos\":[%.2f,%.2f,%.2f]}"),
			SimTime, P->GetAgentID(), *JsonEscape(Spec.ModelName), Spec.MaxVelocity,
			P->GetCollisionRadius(),
			Pos.X, Pos.Y, Pos.Z));
	}

	if (SceneProxy)
	{
		// ---- obstacle（仅场景静态障碍） ----
		if (UObstacleManager* ObsMgr = SceneProxy->GetObstacleManager())
		{
			for (const FObstacleInfo& O : ObsMgr->GetAllObstacles())
			{
				if (O.bIsPerceived) continue;
				WriteLine(FString::Printf(
					TEXT("{\"type\":\"obstacle\",\"t\":%.3f,\"id\":%d,\"oType\":%d,"
						"\"center\":[%.2f,%.2f,%.2f],\"extents\":[%.2f,%.2f,%.2f],"
						"\"actor\":\"%s\",\"dynamic\":%s}"),
					SimTime, O.ObstacleID, (int32)O.Type,
					O.Center.X, O.Center.Y, O.Center.Z,
					O.Extents.X, O.Extents.Y, O.Extents.Z,
					*JsonEscape(O.LinkedActor.IsValid() ? O.LinkedActor->GetName() : TEXT("")),
					O.bIsDynamic ? TEXT("true") : TEXT("false")));
			}
		}

		// ---- waypoint ----
		if (UMissionComponent* Mission = SceneProxy->GetMissionComponent())
		{
			const TArray<FMissionWaypoint>& Wps = Mission->GetMissionWaypoints();
			for (int32 i = 0; i < Wps.Num(); ++i)
			{
				const FVector W = Wps[i].Position;
				WriteLine(FString::Printf(
					TEXT("{\"type\":\"waypoint\",\"t\":%.3f,\"idx\":%d,\"pos\":[%.2f,%.2f,%.2f]}"),
					SimTime, i, W.X, W.Y, W.Z));
			}
		}
	}

	bStaticWritten = true;
}

void UTelemetryRecorder::WriteFrame(float SimTime)
{
	TArray<AUAVPawn*> Agents;
	CollectAgents(Agents);

	// ---- event 边沿（先于 frame 写出，保证时间顺序） ----
	for (AUAVPawn* P : Agents)
	{
		const int32 Seq = P->GetSimEventSeq();
		const int32* Last = LastEventSeq.Find(P->GetAgentID());
		if (!Last || *Last != Seq)
		{
			const FString& Evt = P->GetLastSimEvent();
			if (!Evt.IsEmpty())
			{
				const FVector Pos = P->GetUAVState().Position;
				WriteLine(FString::Printf(
					TEXT("{\"type\":\"event\",\"t\":%.3f,\"agent\":%d,\"event\":\"%s\","
						"\"pos\":[%.2f,%.2f,%.2f],\"crashed\":%s}"),
					SimTime, P->GetAgentID(), *JsonEscape(Evt),
					Pos.X, Pos.Y, Pos.Z,
					P->IsCrashed() ? TEXT("true") : TEXT("false")));
			}
			LastEventSeq.Add(P->GetAgentID(), Seq);
		}
	}

	// ---- frame（紧凑数组形式） ----
	FString AgentsArray;
	for (int32 i = 0; i < Agents.Num(); ++i)
	{
		AUAVPawn* P = Agents[i];
		const FUAVState S = P->GetUAVState();
		float Clearance = -1.0f; // -1 表示未算
		if (UObstacleManager* ObsMgr = P->GetObstacleManager())
		{
			FObstacleInfo Dummy;
			const float D = ObsMgr->GetDistanceToNearestObstacle(S.Position, Dummy);
			if (D >= 0.0f) Clearance = D;
		}
		if (i > 0) AgentsArray += TEXT(",");
		AgentsArray += FString::Printf(
			TEXT("{\"id\":%d,\"pos\":[%.2f,%.2f,%.2f],\"vel\":[%.2f,%.2f,%.2f],"
				"\"speedRatio\":%.3f,\"clearance\":%.2f,\"ctrl\":%d,\"crashed\":%s}"),
			P->GetAgentID(),
			S.Position.X, S.Position.Y, S.Position.Z,
			S.Velocity.X, S.Velocity.Y, S.Velocity.Z,
			P->GetSpeedRatio(), Clearance, (int32)P->GetControlMode(),
			P->IsCrashed() ? TEXT("true") : TEXT("false"));
	}

	// 风速真值（均匀风场，取原点等效）
	FString WindStr = TEXT("");
	if (WindField.IsValid())
	{
		const FVector W = WindField->GetCurrentWindVelocity();
		WindStr = FString::Printf(
			TEXT(",\"wind\":[%.2f,%.2f,%.2f]"), W.X, W.Y, W.Z);
	}

	WriteLine(FString::Printf(
		TEXT("{\"type\":\"frame\",\"t\":%.3f,\"agents\":[%s]%s}"),
		SimTime, *AgentsArray, *WindStr));
}

void UTelemetryRecorder::WriteMetrics(float SimTime)
{
	TArray<AUAVPawn*> Agents;
	CollectAgents(Agents);
	for (AUAVPawn* P : Agents)
	{
		WriteLine(FString::Printf(
			TEXT("{\"type\":\"metrics\",\"t\":%.3f,\"agent\":%d,\"speedRatio\":%.3f,"
				"\"lowSpeedDur\":%.2f,\"maxDev\":%.0f,\"maxRoll\":%.1f,\"maxPitch\":%.1f,"
				"\"instabTime\":%.2f,\"stuck\":%d,\"forceComplete\":%d}"),
			SimTime, P->GetAgentID(), P->GetSpeedRatio(),
			P->GetMaxLowSpeedDuration(), P->GetMaxCrossTrackDev(),
			P->GetMaxRoll(), P->GetMaxPitch(),
			P->GetAttitudeInstabilityTime(),
			P->GetNMPCStuckCount(), P->GetForceCompleteCount()));
	}
}

void UTelemetryRecorder::SampleIndices(int32 Total, int32 MaxCount, TArray<int32>& OutIndices)
{
	OutIndices.Reset();
	if (Total <= 0) return;
	if (Total <= MaxCount)
	{
		for (int32 i = 0; i < Total; ++i) OutIndices.Add(i);
		return;
	}
	// 等距降采样：首尾入选，中间均匀取点
	for (int32 i = 0; i < MaxCount; ++i)
	{
		const int32 Idx = FMath::Min(Total - 1, FMath::RoundToInt(static_cast<float>(i) * (Total - 1) / (MaxCount - 1)));
		OutIndices.Add(Idx);
	}
}

void UTelemetryRecorder::WriteFutureTrajectories(float SimTime)
{
	TArray<AUAVPawn*> Agents;
	CollectAgents(Agents);

	for (AUAVPawn* P : Agents)
	{
		const int32 AgentID = P->GetAgentID();

		// ---- traj_opt：优化轨迹（TrajectoryTracker 正在跟踪的 FTrajectory）----
		UTrajectoryTracker* Tracker = P->GetTrajectoryTracker();
		if (Tracker && Tracker->IsTracking())
		{
			const FTrajectory& Opt = Tracker->GetTrajectory();
			if (Opt.bIsValid && Opt.Points.Num() >= 2)
			{
				TArray<int32> Idx;
				SampleIndices(Opt.Points.Num(), MaxTrajectoryPoints, Idx);
				FString PtsJson;
				for (int32 k = 0; k < Idx.Num(); ++k)
				{
					const FVector& Pos = Opt.Points[Idx[k]].Position;
					PtsJson += FString::Printf(TEXT("%s[%.1f,%.1f,%.1f]"),
						k > 0 ? TEXT(",") : TEXT(""), Pos.X, Pos.Y, Pos.Z);
				}
				WriteLine(FString::Printf(
					TEXT("{\"type\":\"traj_opt\",\"t\":%.3f,\"agent\":%d,\"valid\":true,\"pts\":[%s]}"),
					SimTime, AgentID, *PtsJson));
			}
		}

		// ---- traj_plan：规划路径（PlanningVisualizer 持久化路径）----
		if (UPlanningVisualizer* Vis = P->GetPlanningVisualizer())
		{
			TArray<FVector> PlanPath;
			if (Vis->GetPersistentPath(PlanPath))
			{
				TArray<int32> Idx;
				SampleIndices(PlanPath.Num(), MaxTrajectoryPoints, Idx);
				FString PtsJson;
				for (int32 k = 0; k < Idx.Num(); ++k)
				{
					const FVector& Pos = PlanPath[Idx[k]];
					PtsJson += FString::Printf(TEXT("%s[%.1f,%.1f,%.1f]"),
						k > 0 ? TEXT(",") : TEXT(""), Pos.X, Pos.Y, Pos.Z);
				}
				WriteLine(FString::Printf(
					TEXT("{\"type\":\"traj_plan\",\"t\":%.3f,\"agent\":%d,\"pts\":[%s]}"),
					SimTime, AgentID, *PtsJson));
			}
		}

		// ---- traj_nmpc：NMPC 预测轨迹（含障碍代价 cost）----
		if (P->HasValidNMPCPrediction())
		{
			if (const FNMPCAvoidanceResult* Result = P->GetLastNMPCResult())
			{
				const TArray<FNMPCPredictionStep>& Pred = Result->PredictedTrajectory;
				if (Pred.Num() >= 2)
				{
					TArray<int32> Idx;
					SampleIndices(Pred.Num(), MaxTrajectoryPoints, Idx);
					FString PtsJson;
					for (int32 k = 0; k < Idx.Num(); ++k)
					{
						const FNMPCPredictionStep& Step = Pred[Idx[k]];
						PtsJson += FString::Printf(TEXT("%s[%.1f,%.1f,%.1f,%.2f]"),
							k > 0 ? TEXT(",") : TEXT(""),
							Step.Position.X, Step.Position.Y, Step.Position.Z,
							Step.ObstacleCost);
					}
					WriteLine(FString::Printf(
						TEXT("{\"type\":\"traj_nmpc\",\"t\":%.3f,\"agent\":%d,\"pts\":[%s]}"),
						SimTime, AgentID, *PtsJson));
				}
			}
		}
	}
}

void UTelemetryRecorder::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!FileHandle.IsValid()) return;

	const float SimTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;

	// 首帧写静态数据（此时机队/障碍/航点均已 BeginPlay 就绪）
	if (!bStaticWritten)
	{
		WriteStaticOnce(SimTime);
	}

	// 帧采样：仿真秒节拍（回放时间轴与判决时间一致）
	FrameAccum += DeltaTime;
	if (FrameAccum >= FrameIntervalSec)
	{
		FrameAccum = 0.0f;
		WriteFrame(SimTime);
	}

	// 指标采样：仿真秒节拍
	MetricsAccum += DeltaTime;
	if (MetricsAccum >= MetricsIntervalSec)
	{
		MetricsAccum = 0.0f;
		WriteMetrics(SimTime);
	}

	// 未来轨迹采样：低于帧率的节拍（默认 5Hz），避免 ndjson 体积失控
	TrajectoryAccum += DeltaTime;
	if (TrajectoryAccum >= TrajectoryIntervalSec)
	{
		TrajectoryAccum = 0.0f;
		WriteFutureTrajectories(SimTime);
	}
}

void UTelemetryRecorder::WriteVerdict(float SimTime, bool bFinal, int32 Reached, int32 Total,
	float ClearanceCm, float LateralDevCm, float ElapsedSec,
	bool bCollided, const TArray<FString>& Failures)
{
	if (!FileHandle.IsValid()) return;

	FString FailArray;
	for (int32 i = 0; i < Failures.Num(); ++i)
	{
		if (i > 0) FailArray += TEXT(",");
		FailArray += FString::Printf(TEXT("\"%s\""), *JsonEscape(Failures[i]));
	}

	WriteLine(FString::Printf(
		TEXT("{\"type\":\"verdict\",\"t\":%.3f,\"final\":%s,\"reached\":%d,\"total\":%d,"
			"\"clearanceCm\":%.0f,\"lateralDevCm\":%.0f,\"elapsedSec\":%.2f,"
			"\"collided\":%s,\"passed\":%s,\"failures\":[%s]}"),
		SimTime, bFinal ? TEXT("true") : TEXT("false"),
		Reached, Total,
		ClearanceCm, LateralDevCm, ElapsedSec,
		bCollided ? TEXT("true") : TEXT("false"),
		(Failures.Num() == 0) ? TEXT("true") : TEXT("false"),
		*FailArray));
}
