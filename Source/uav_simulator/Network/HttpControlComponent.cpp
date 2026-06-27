// Copyright Epic Games, Inc. All Rights Reserved.

#include "HttpControlComponent.h"
#include "HttpServerModule.h"
#include "IHttpRouter.h"
#include "HttpPath.h"
#include "HttpServerRequest.h"
#include "HttpServerResponse.h"
#include "Async/Async.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializerMacros.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GenericPlatform/GenericPlatformMisc.h"
#include "../MultiAgent/AgentManager.h"
#include "../Scenario/ScenarioDto.h"
#include "../Scenario/ScenarioFactory.h"
#include "../Core/UAVPawn.h"
#include "../Environment/WindField.h"
#include "../Environment/EnvironmentTypes.h"
#include "../Planning/NMPCAvoidance.h"
#include "../Core/UAVTypes.h"
#include "../MultiAgent/MultiAgentTypes.h"

DEFINE_LOG_CATEGORY_STATIC(LogHttpControl, Log, All);

// ============================================================================
// Web 契约 JSON（camelCase key + 数组坐标 [x,y,z]）→ FScenarioDto 手工解析
//
// 不使用 FJsonObjectConverter 的自动 UStruct 映射：它对 FVector 期望 {X,Y,Z}
// 对象、且 key 大小写/驼峰约定与 Web 前端不匹配，数组坐标会导致整结构解析失败。
// 手工按前端契约逐字段解析，缺失字段保留 DTO 默认值，坐标同时容忍数组与对象形式。
// 职责定位：HTTP 边界是 Web 契约的适配器（六边形端口），核心 ScenarioFactory
// (DTO→UScenario) 不依赖传输格式，保持纯 C++ 可测。
// ============================================================================

// 解析一个向量字段：优先 [x,y,z] 数组（Web 主契约），容错 {X,Y,Z} 对象。
// 用 HasTypedField 判类型，避开 FJsonValue::GetType()（UE5.7 protected）与 GetField 单参签名。
static FVector ParseWebVec(const TSharedPtr<FJsonObject>& Obj, const FString& Key, FVector Default)
{
	if (!Obj.IsValid()) return Default;
	if (Obj->HasTypedField<EJson::Array>(Key))
	{
		const TArray<TSharedPtr<FJsonValue>>& A = Obj->GetArrayField(Key);
		if (A.Num() >= 3)
		{
			return FVector(A[0]->AsNumber(), A[1]->AsNumber(), A[2]->AsNumber());
		}
	}
	else if (Obj->HasTypedField<EJson::Object>(Key))
	{
		const TSharedPtr<FJsonObject> O = Obj->GetObjectField(Key);
		if (O.IsValid())
		{
			return FVector(O->GetNumberField(TEXT("X")), O->GetNumberField(TEXT("Y")), O->GetNumberField(TEXT("Z")));
		}
	}
	return Default;
}

static FString WebStr(const TSharedPtr<FJsonObject>& O, const TCHAR* K, const FString& Def)
{
	return (O.IsValid() && O->HasField(K)) ? O->GetStringField(K) : Def;
}
static double WebNum(const TSharedPtr<FJsonObject>& O, const TCHAR* K, double Def)
{
	return (O.IsValid() && O->HasField(K)) ? O->GetNumberField(K) : Def;
}
static bool WebBool(const TSharedPtr<FJsonObject>& O, const TCHAR* K, bool Def)
{
	return (O.IsValid() && O->HasField(K)) ? O->GetBoolField(K) : Def;
}
static TSharedPtr<FJsonObject> WebObj(const TSharedPtr<FJsonObject>& O, const TCHAR* K)
{
	return (O.IsValid() && O->HasTypedField<EJson::Object>(K)) ? O->GetObjectField(K) : nullptr;
}

// 把 Web 契约 JSON 解析成 FScenarioDto（缺失字段保留默认值）。空 JSON 返回 false。
static bool BuildDtoFromWebJson(const TSharedPtr<FJsonObject>& Json, FScenarioDto& Out)
{
	if (!Json.IsValid()) return false;

	Out.Name = WebStr(Json, TEXT("name"), Out.Name);
	Out.RandomSeed = WebNum(Json, TEXT("randomSeed"), Out.RandomSeed);

	// ---- sim ----
	if (TSharedPtr<FJsonObject> Sim = WebObj(Json, TEXT("sim")))
	{
		Out.Sim.Slomo = WebNum(Sim, TEXT("slomo"), Out.Sim.Slomo);
		Out.Sim.DurationSec = WebNum(Sim, TEXT("durationSec"), Out.Sim.DurationSec);
		Out.Sim.ControlMode = WebStr(Sim, TEXT("controlMode"), Out.Sim.ControlMode);
		Out.Sim.MPCType = WebStr(Sim, TEXT("mpcType"), Out.Sim.MPCType);
	}

	// ---- wind ----
	if (TSharedPtr<FJsonObject> Wind = WebObj(Json, TEXT("wind")))
	{
		Out.Wind.Type = WebStr(Wind, TEXT("type"), Out.Wind.Type);
		Out.Wind.Steady = ParseWebVec(Wind, TEXT("steady"), Out.Wind.Steady);
		Out.Wind.GustAmplitude = WebNum(Wind, TEXT("gustAmplitude"), Out.Wind.GustAmplitude);
		Out.Wind.GustDuration = WebNum(Wind, TEXT("gustDuration"), Out.Wind.GustDuration);
		Out.Wind.GustFrequency = WebNum(Wind, TEXT("gustFrequency"), Out.Wind.GustFrequency);
		Out.Wind.TurbulenceIntensity = WebNum(Wind, TEXT("turbulenceIntensity"), Out.Wind.TurbulenceIntensity);
		Out.Wind.TurbulenceLengthScale = WebNum(Wind, TEXT("turbulenceLengthScale"), Out.Wind.TurbulenceLengthScale);
		Out.Wind.bEnabled = WebBool(Wind, TEXT("enabled"), Out.Wind.bEnabled);
	}

	// ---- acceptance ----
	if (TSharedPtr<FJsonObject> Acc = WebObj(Json, TEXT("acceptance")))
	{
		Out.Acceptance.bRequireAllWaypoints = WebBool(Acc, TEXT("requireAllWaypoints"), Out.Acceptance.bRequireAllWaypoints);
		Out.Acceptance.WaypointRadiusCm = WebNum(Acc, TEXT("waypointRadiusCm"), Out.Acceptance.WaypointRadiusCm);
		Out.Acceptance.MinClearanceCm = WebNum(Acc, TEXT("minClearanceCm"), Out.Acceptance.MinClearanceCm);
		Out.Acceptance.MaxLateralDeviationCm = WebNum(Acc, TEXT("maxLateralDeviationCm"), Out.Acceptance.MaxLateralDeviationCm);
		Out.Acceptance.TimeoutSec = WebNum(Acc, TEXT("timeoutSec"), Out.Acceptance.TimeoutSec);
		Out.Acceptance.EnergyBudget = WebNum(Acc, TEXT("energyBudget"), Out.Acceptance.EnergyBudget);
	}

	// ---- fleet ----
	if (Json->HasTypedField<EJson::Array>(TEXT("fleet")))
	{
		Out.Fleet.Reset();
		for (const TSharedPtr<FJsonValue>& Item : Json->GetArrayField(TEXT("fleet")))
		{
			TSharedPtr<FJsonObject> A = Item->AsObject();
			if (!A.IsValid()) continue;
			FScenarioDtoAgent& Agent = Out.Fleet.AddDefaulted_GetRef();
			Agent.Model = WebStr(A, TEXT("model"), Agent.Model);
			Agent.InitPos = ParseWebVec(A, TEXT("initPos"), Agent.InitPos);
			Agent.Yaw = WebNum(A, TEXT("yaw"), Agent.Yaw);
			Agent.bIsLeader = WebBool(A, TEXT("isLeader"), Agent.bIsLeader);
			Agent.Mode = WebStr(A, TEXT("mode"), Agent.Mode);
			if (A->HasTypedField<EJson::Array>(TEXT("waypoints")))
			{
				for (const TSharedPtr<FJsonValue>& W : A->GetArrayField(TEXT("waypoints")))
				{
					TSharedPtr<FJsonObject> WO = W->AsObject();
					if (!WO.IsValid()) continue;
					FScenarioDtoWaypoint& WP = Agent.Waypoints.AddDefaulted_GetRef();
					WP.Pos = ParseWebVec(WO, TEXT("pos"), WP.Pos);
					WP.Speed = WebNum(WO, TEXT("speed"), WP.Speed);
					WP.Hover = WebNum(WO, TEXT("hover"), WP.Hover);
				}
			}
		}
	}

	// ---- obstacles ----
	if (Json->HasTypedField<EJson::Array>(TEXT("obstacles")))
	{
		Out.Obstacles.Reset();
		for (const TSharedPtr<FJsonValue>& Item : Json->GetArrayField(TEXT("obstacles")))
		{
			TSharedPtr<FJsonObject> O = Item->AsObject();
			if (!O.IsValid()) continue;
			FScenarioDtoObstacle& Obs = Out.Obstacles.AddDefaulted_GetRef();
			Obs.Type = WebStr(O, TEXT("type"), Obs.Type);
			Obs.Center = ParseWebVec(O, TEXT("center"), Obs.Center);
			Obs.Extents = ParseWebVec(O, TEXT("extents"), Obs.Extents);
			Obs.SafetyMargin = WebNum(O, TEXT("safetyMargin"), Obs.SafetyMargin);
			Obs.Movement = WebStr(O, TEXT("movement"), Obs.Movement);
			Obs.Velocity = ParseWebVec(O, TEXT("velocity"), Obs.Velocity);
			Obs.PatrolSpeed = WebNum(O, TEXT("patrolSpeed"), Obs.PatrolSpeed);
			if (O->HasTypedField<EJson::Array>(TEXT("patrolPoints")))
			{
				for (const TSharedPtr<FJsonValue>& P : O->GetArrayField(TEXT("patrolPoints")))
				{
					// 每个巡逻点：数组 [x,y,z]（Web 主契约）。FJsonValue::GetType() 在 5.7 受保护，
					// 这里只接受数组形式（契约要求），非数组点跳过。
					const TArray<TSharedPtr<FJsonValue>>* PA = nullptr;
					if (P.IsValid() && P->TryGetArray(PA) && PA && PA->Num() >= 3)
					{
						Obs.PatrolPoints.Emplace((*PA)[0]->AsNumber(), (*PA)[1]->AsNumber(), (*PA)[2]->AsNumber());
					}
				}
			}
		}
	}

	return true;
}

UHttpControlComponent::UHttpControlComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	// 控制端在所有关卡/GameMode 生命周期都要工作，包括无编辑器的 -game headless。
}

void UHttpControlComponent::BeginPlay()
{
	Super::BeginPlay();
	StartServer();
}

void UHttpControlComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopServer();
	Super::EndPlay(EndPlayReason);
}

AMultiAgentGameMode* UHttpControlComponent::GetOwningGameMode() const
{
	if (AActor* Owner = GetOwner())
	{
		return Cast<AMultiAgentGameMode>(Owner);
	}
	return nullptr;
}

int32 UHttpControlComponent::ResolvePort() const
{
	int32 Port = DefaultPort;
	// 命令行 -ControlPort= 覆盖
	FParse::Value(FCommandLine::Get(), TEXT("-ControlPort="), Port);
	UE_LOG(LogHttpControl, Warning, TEXT("[HttpControl] ResolvePort base=%d"), Port);
	return Port;
}

void UHttpControlComponent::WritePortRegistry(int32 Port) const
{
	const FString Dir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT(".uav-ctrl"));
	FPlatformFileManager::Get().GetPlatformFile().CreateDirectoryTree(*Dir);
	const FString Path = FPaths::Combine(Dir, TEXT("port.json"));
	const FString Content = FString::Printf(TEXT("{\"port\":%d}"), Port);
	FFileHelper::SaveStringToFile(Content, *Path);
}

bool UHttpControlComponent::StartServer()
{
	if (bIsRunning)
	{
		return true;
	}

	BoundPort = ResolvePort();

	FHttpServerModule& HttpServerModule = FHttpServerModule::Get();
	HttpRouter = HttpServerModule.GetHttpRouter(BoundPort);
	if (!HttpRouter.IsValid())
	{
		UE_LOG(LogHttpControl, Error, TEXT("[HttpControl] Failed to get HTTP router for port %d"), BoundPort);
		return false;
	}

	RouteHandle = HttpRouter->BindRoute(
		FHttpPath(TEXT("/control")),
		EHttpServerRequestVerbs::VERB_POST | EHttpServerRequestVerbs::VERB_OPTIONS,
		FHttpRequestHandler::CreateUObject(this, &UHttpControlComponent::HandleRequest));

	if (!RouteHandle.IsValid())
	{
		UE_LOG(LogHttpControl, Error, TEXT("[HttpControl] Failed to bind /control route"));
		return false;
	}

	HttpServerModule.StartAllListeners();
	bIsRunning = true;

	WritePortRegistry(BoundPort);
	UE_LOG(LogHttpControl, Warning, TEXT("[HttpControl] Listening on http://127.0.0.1:%d/control"), BoundPort);
	return true;
}

void UHttpControlComponent::StopServer()
{
	if (!bIsRunning)
	{
		return;
	}
	if (HttpRouter.IsValid() && RouteHandle.IsValid())
	{
		HttpRouter->UnbindRoute(RouteHandle);
	}
	bIsRunning = false;
	UE_LOG(LogHttpControl, Log, TEXT("[HttpControl] Stopped"));
}

bool UHttpControlComponent::HandleRequest(const FHttpServerRequest& Request, const FHttpResultCallback& OnComplete)
{
	// CORS preflight（前端若直连可跨域；经 Python 反代则不需要）
	if (Request.Verb == EHttpServerRequestVerbs::VERB_OPTIONS)
	{
		TUniquePtr<FHttpServerResponse> Resp = FHttpServerResponse::Create(TEXT(""), TEXT("text/plain"));
		Resp->Headers.Add(TEXT("Access-Control-Allow-Origin"), {TEXT("*")});
		Resp->Headers.Add(TEXT("Access-Control-Allow-Methods"), {TEXT("POST, OPTIONS")});
		Resp->Headers.Add(TEXT("Access-Control-Allow-Headers"), {TEXT("Content-Type")});
		Resp->Code = EHttpServerResponseCodes::NoContent;
		OnComplete(MoveTemp(Resp));
		return true;
	}

	if (Request.Verb != EHttpServerRequestVerbs::VERB_POST)
	{
		TUniquePtr<FHttpServerResponse> Resp = FHttpServerResponse::Create(TEXT("{\"error\":\"method not allowed\"}"), TEXT("application/json"));
		Resp->Code = EHttpServerResponseCodes::BadRequest;
		Resp->Headers.Add(TEXT("Access-Control-Allow-Origin"), {TEXT("*")});
		OnComplete(MoveTemp(Resp));
		return true;
	}

	// 解析 body
	FString Body;
	if (Request.Body.Num() > 0)
	{
		FUTF8ToTCHAR Convert(reinterpret_cast<const ANSICHAR*>(Request.Body.GetData()), Request.Body.Num());
		Body = FString(Convert.Length(), Convert.Get());
	}

	// 子路径决定命令：/control, /control/reload, /control/slomo, ...
	const FString Path = Request.RelativePath.GetPath();

	// 切到 GameThread 执行（UE API 与热重载要求）
	AsyncTask(ENamedThreads::GameThread, [this, Path, Body, OnComplete]()
	{
		TSharedPtr<FJsonObject> BodyObj = MakeShareable(new FJsonObject);
		if (!Body.IsEmpty())
		{
			TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Body);
			if (!FJsonSerializer::Deserialize(Reader, BodyObj) || !BodyObj.IsValid())
			{
				BodyObj = MakeShareable(new FJsonObject); // 解析失败用空对象，部分命令不需要 body
			}
		}

		TSharedPtr<FJsonObject> Result = DispatchCommand(Path, BodyObj);

		FString JsonStr;
		TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonStr);
		FJsonSerializer::Serialize(Result.ToSharedRef(), Writer);

		TUniquePtr<FHttpServerResponse> Resp = FHttpServerResponse::Create(JsonStr, TEXT("application/json"));
		Resp->Headers.Add(TEXT("Access-Control-Allow-Origin"), {TEXT("*")});
		const bool bOk = Result->HasField(TEXT("ok")) ? Result->GetBoolField(TEXT("ok")) : true;
		Resp->Code = bOk ? EHttpServerResponseCodes::Ok : EHttpServerResponseCodes::BadRequest;
		OnComplete(MoveTemp(Resp));
	});

	return true;
}

TSharedPtr<FJsonObject> UHttpControlComponent::DispatchCommand(const FString& Path, const TSharedPtr<FJsonObject>& Body)
{
	if (Path.EndsWith(TEXT("reload")))
	{
		return HandleReload(Body);
	}
	if (Path.EndsWith(TEXT("slomo")))
	{
		return HandleSlomo(Body);
	}
	if (Path.EndsWith(TEXT("wind")))
	{
		return HandleWind(Body);
	}
	if (Path.EndsWith(TEXT("params")))
	{
		return HandleParams(Body);
	}
	if (Path.EndsWith(TEXT("status")))
	{
		return HandleStatus();
	}
	if (Path.EndsWith(TEXT("exit")))
	{
		return HandleExit(Body);
	}

	TSharedPtr<FJsonObject> Err = MakeShareable(new FJsonObject);
	Err->SetBoolField(TEXT("ok"), false);
	Err->SetStringField(TEXT("error"), FString::Printf(TEXT("unknown command path: %s"), *Path));
	return Err;
}

TSharedPtr<FJsonObject> UHttpControlComponent::HandleReload(const TSharedPtr<FJsonObject>& Body)
{
	TSharedPtr<FJsonObject> Resp = MakeShareable(new FJsonObject);
	AMultiAgentGameMode* GM = GetOwningGameMode();
	if (!GM)
	{
		Resp->SetBoolField(TEXT("ok"), false);
		Resp->SetStringField(TEXT("error"), TEXT("no game mode"));
		return Resp;
	}

	// 从 Web 契约 JSON 反序列化为 FScenarioDto
	// （手工解析：FJsonObjectConverter 不兼容数组坐标 [x,y,z]，见文件顶部说明）
	FScenarioDto Dto;
	if (!BuildDtoFromWebJson(Body, Dto))
	{
		Resp->SetBoolField(TEXT("ok"), false);
		Resp->SetStringField(TEXT("error"), TEXT("invalid scenario dto"));
		return Resp;
	}

	UScenario* Scenario = UScenarioFactory::BuildFromDto(Dto, GM);
	if (!Scenario)
	{
		Resp->SetBoolField(TEXT("ok"), false);
		Resp->SetStringField(TEXT("error"), TEXT("factory build failed"));
		return Resp;
	}

	const bool bOk = GM->ReloadScenario(Scenario);
	Resp->SetBoolField(TEXT("ok"), bOk);
	Resp->SetNumberField(TEXT("fleetCount"), GM->GetScenarioFleet().Num());
	Resp->SetStringField(TEXT("scenario"), Dto.Name);
	if (!bOk)
	{
		Resp->SetStringField(TEXT("error"), TEXT("reload rejected (already reloading or null)"));
	}
	return Resp;
}

TSharedPtr<FJsonObject> UHttpControlComponent::HandleSlomo(const TSharedPtr<FJsonObject>& Body)
{
	TSharedPtr<FJsonObject> Resp = MakeShareable(new FJsonObject);
	AMultiAgentGameMode* GM = GetOwningGameMode();
	if (!GM)
	{
		Resp->SetBoolField(TEXT("ok"), false);
		Resp->SetStringField(TEXT("error"), TEXT("no game mode"));
		return Resp;
	}

	const float Scale = Body->GetNumberField(TEXT("scale"));
	GM->SetSlomo(Scale);
	Resp->SetBoolField(TEXT("ok"), true);
	Resp->SetNumberField(TEXT("scale"), Scale);
	return Resp;
}

TSharedPtr<FJsonObject> UHttpControlComponent::HandleWind(const TSharedPtr<FJsonObject>& Body)
{
	TSharedPtr<FJsonObject> Resp = MakeShareable(new FJsonObject);
	AMultiAgentGameMode* GM = GetOwningGameMode();
	if (!GM || !GM->GetWindField())
	{
		Resp->SetBoolField(TEXT("ok"), false);
		Resp->SetStringField(TEXT("error"), TEXT("no wind field"));
		return Resp;
	}

	FWindConfig Cfg = GM->GetWindField()->GetWindConfig();
	// 按 DTO 风字段覆盖（缺省保留原值）
	if (Body->HasField(TEXT("type")))
	{
		const FString T = Body->GetStringField(TEXT("type"));
		if (T == TEXT("None")) Cfg.WindType = EWindFieldType::None;
		else if (T == TEXT("Constant")) Cfg.WindType = EWindFieldType::Constant;
		else if (T == TEXT("Gust")) Cfg.WindType = EWindFieldType::Gust;
		else if (T == TEXT("Turbulent")) Cfg.WindType = EWindFieldType::Turbulent;
	}
	if (Body->HasField(TEXT("gustAmplitude"))) Cfg.GustAmplitude = Body->GetNumberField(TEXT("gustAmplitude"));
	if (Body->HasField(TEXT("turbulenceIntensity"))) Cfg.TurbulenceIntensity = Body->GetNumberField(TEXT("turbulenceIntensity"));
	if (Body->HasField(TEXT("enabled"))) Cfg.bEnabled = Body->GetBoolField(TEXT("enabled"));
	if (Body->HasTypedField<EJson::Array>(TEXT("steady")))
	{
		const TArray<TSharedPtr<FJsonValue>>* Arr;
		if (Body->TryGetArrayField(TEXT("steady"), Arr) && Arr->Num() >= 3)
		{
			Cfg.SteadyWindVelocity = FVector(
				(*Arr)[0]->AsNumber(), (*Arr)[1]->AsNumber(), (*Arr)[2]->AsNumber());
		}
	}

	GM->GetWindField()->SetWindConfig(Cfg);
	Resp->SetBoolField(TEXT("ok"), true);
	return Resp;
}

TSharedPtr<FJsonObject> UHttpControlComponent::HandleParams(const TSharedPtr<FJsonObject>& Body)
{
	// 实时调参：按 target 分发到 fleet/leader/单机。
	// body 形如 {"target":"fleet","attitudeRoll":{"kp":0.05,"ki":0.005,"kd":0.05},...}
	TSharedPtr<FJsonObject> Resp = MakeShareable(new FJsonObject);
	AMultiAgentGameMode* GM = GetOwningGameMode();
	if (!GM)
	{
		Resp->SetBoolField(TEXT("ok"), false);
		Resp->SetStringField(TEXT("error"), TEXT("no game mode"));
		return Resp;
	}

	const FString Target = Body->HasField(TEXT("target")) ? Body->GetStringField(TEXT("target")) : TEXT("fleet");

	int32 Applied = 0;
	for (AUAVPawn* Pawn : GM->GetScenarioFleet())
	{
		if (!Pawn) continue;
		// target 过滤：fleet 全体；leader 仅 AgentID==0（场景装配 0 号为首机）；
		// 否则解析为指定 agentId
		if (Target == TEXT("leader"))
		{
			if (Pawn->GetAgentID() != 0) continue;
		}
		else if (Target != TEXT("fleet"))
		{
			const int32 AgentId = FCString::Atoi(*Target);
			if (Pawn->GetAgentID() != AgentId) continue;
		}

		// 姿态 PID
		const auto ApplyPid = [&](const TCHAR* Key, int32 Axis)
		{
			if (Body->HasTypedField<EJson::Object>(Key))
			{
				const TSharedPtr<FJsonObject>* P;
				if (Body->TryGetObjectField(Key, P) && P->IsValid())
				{
					Pawn->SetAttitudePID(Axis,
						(*P)->GetNumberField(TEXT("kp")),
						(*P)->GetNumberField(TEXT("ki")),
						(*P)->GetNumberField(TEXT("kd")));
				}
			}
		};
		ApplyPid(TEXT("attitudeRoll"), 0);
		ApplyPid(TEXT("attitudePitch"), 1);
		ApplyPid(TEXT("attitudeYaw"), 2);

		if (Body->HasTypedField<EJson::Object>(TEXT("positionPID")))
		{
			const TSharedPtr<FJsonObject>* P;
			if (Body->TryGetObjectField(TEXT("positionPID"), P) && P->IsValid())
			{
				Pawn->SetPositionPIDGains(
					(*P)->GetNumberField(TEXT("kp")),
					(*P)->GetNumberField(TEXT("ki")),
					(*P)->GetNumberField(TEXT("kd")));
			}
		}
		if (Body->HasTypedField<EJson::Object>(TEXT("velocityPID")))
		{
			const TSharedPtr<FJsonObject>* P;
			if (Body->TryGetObjectField(TEXT("velocityPID"), P) && P->IsValid())
			{
				Pawn->SetVelocityPIDGains(
					(*P)->GetNumberField(TEXT("kp")),
					(*P)->GetNumberField(TEXT("ki")),
					(*P)->GetNumberField(TEXT("kd")));
			}
		}
		if (Body->HasTypedField<EJson::Object>(TEXT("cbfqp")))
		{
			const TSharedPtr<FJsonObject>* P;
			if (Body->TryGetObjectField(TEXT("cbfqp"), P) && P->IsValid())
			{
				FCBFQPConfig Cfg = Pawn->GetCBFQPConfig();
				if ((*P)->HasField(TEXT("dSafe"))) Cfg.DSafe = (*P)->GetNumberField(TEXT("dSafe"));
				if ((*P)->HasField(TEXT("alpha0"))) Cfg.Alpha0 = (*P)->GetNumberField(TEXT("alpha0"));
				if ((*P)->HasField(TEXT("mode"))) Cfg.Mode = static_cast<ECBFMode>((*P)->GetIntegerField(TEXT("mode")));
				Pawn->SetCBFQPConfig(Cfg);
			}
		}

		++Applied;
	}

	Resp->SetBoolField(TEXT("ok"), true);
	Resp->SetNumberField(TEXT("applied"), Applied);
	return Resp;
}

TSharedPtr<FJsonObject> UHttpControlComponent::HandleStatus() const
{
	TSharedPtr<FJsonObject> Resp = MakeShareable(new FJsonObject);
	AMultiAgentGameMode* GM = GetOwningGameMode();
	Resp->SetStringField(TEXT("state"), GM && GM->IsReloading() ? TEXT("reloading") : TEXT("running"));
	Resp->SetNumberField(TEXT("fleet"), GM ? GM->GetScenarioFleet().Num() : 0);
	Resp->SetNumberField(TEXT("port"), BoundPort);
	return Resp;
}

TSharedPtr<FJsonObject> UHttpControlComponent::HandleExit(const TSharedPtr<FJsonObject>& Body)
{
	TSharedPtr<FJsonObject> Resp = MakeShareable(new FJsonObject);
	Resp->SetBoolField(TEXT("ok"), true);
	Resp->SetStringField(TEXT("message"), TEXT("exit requested"));
	// 延迟一点退出，确保响应已发回
	AsyncTask(ENamedThreads::GameThread, []()
	{
		GLog->Flush();
		FGenericPlatformMisc::RequestExit(false);
	});
	return Resp;
}
