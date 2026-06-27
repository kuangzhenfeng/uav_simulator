// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "IHttpRouter.h"
#include "HttpControlComponent.generated.h"

class AMultiAgentGameMode;
class UScenario;

class AMultiAgentGameMode;
class UScenario;

/**
 * Web 控制面板的 UE 侧 HTTP 控制端。
 *
 * 在独立端口（默认 8770）监听 POST 命令，把外部（Python 反代）的控制意图切到 GameThread
 * 执行，驱动 AMultiAgentGameMode 的热重载 / 实时调参能力。
 *
 * 设计要点：
 * - 独立于 SoftUEBridge 插件进程（后者在 -unattended 会跳过启动），本组件显式在
 *   headless(-game -NullRHI) 模式下工作 —— 命令面板正是为自动化仿真场景设计。
 * - 复用引擎 HttpServerModule 的 IHttpRouter（SoftUEBridge 同款范式），自管路由与端口。
 * - 所有副作用在 GameThread 执行（AsyncTask），网络线程仅解析 JSON。
 * - 端口被占用自动 +1..+9 避让，端口写入 Saved/.uav-ctrl/port.json 供 Python 发现。
 * - 仅监听 127.0.0.1，本机单用户工具，不做鉴权。
 */
UCLASS(ClassGroup = (Network), meta = (BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UHttpControlComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UHttpControlComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** 控制端是否已启动监听 */
	bool IsRunning() const { return bIsRunning; }

	/** 实际监听端口（端口避让后可能与配置不同） */
	int32 GetBoundPort() const { return BoundPort; }

private:
	// 启动 HTTP 路由并绑定路由，返回是否成功
	bool StartServer();

	// 停止监听、解绑路由
	void StopServer();

	// 选择可用端口：从 BasePort 起尝试 +1..+9，写 port.json
	int32 ResolvePort() const;

	// 把端口写入 Saved/.uav-ctrl/port.json 供 Python 发现
	void WritePortRegistry(int32 Port) const;

	// 请求处理（在 HTTP 线程）
	bool HandleRequest(const FHttpServerRequest& Request, const FHttpResultCallback& OnComplete);

	// 派发具体命令（切到 GameThread 后调用），返回 JSON 结果体
	TSharedPtr<FJsonObject> DispatchCommand(const FString& Path, const TSharedPtr<FJsonObject>& Body);

	// 命令处理器
	TSharedPtr<FJsonObject> HandleReload(const TSharedPtr<FJsonObject>& Body);
	TSharedPtr<FJsonObject> HandleSlomo(const TSharedPtr<FJsonObject>& Body);
	TSharedPtr<FJsonObject> HandleWind(const TSharedPtr<FJsonObject>& Body);
	TSharedPtr<FJsonObject> HandleParams(const TSharedPtr<FJsonObject>& Body);
	TSharedPtr<FJsonObject> HandleStatus() const;
	TSharedPtr<FJsonObject> HandleExit(const TSharedPtr<FJsonObject>& Body);
	TSharedPtr<FJsonObject> HandleStop(const TSharedPtr<FJsonObject>& Body);
	TSharedPtr<FJsonObject> HandlePause(const TSharedPtr<FJsonObject>& Body);

	// 取所属 GameMode
	AMultiAgentGameMode* GetOwningGameMode() const;

	// 路由句柄
	TSharedPtr<IHttpRouter> HttpRouter;
	FHttpRouteHandle RouteHandle;

	// 监听端口
	int32 BoundPort = 0;

	// 默认端口（可经命令行 -ControlPort= 覆盖）
	static constexpr int32 DefaultPort = 8770;

	// 最大端口尝试次数
	static constexpr int32 MaxPortAttempts = 10;

	bool bIsRunning = false;
};
