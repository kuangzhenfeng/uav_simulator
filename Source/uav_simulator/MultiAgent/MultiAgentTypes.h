// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "../Core/UAVTypes.h"
#include "../Planning/NMPCAvoidance.h"
#include "MultiAgentTypes.generated.h"

/**
 * 编队类型枚举
 */
UENUM(BlueprintType)
enum class EFormationType : uint8
{
	None UMETA(DisplayName = "None"),
	Line UMETA(DisplayName = "Line"),
	VShape UMETA(DisplayName = "V-Shape"),
	Circle UMETA(DisplayName = "Circle"),
	Diamond UMETA(DisplayName = "Diamond")
};

/**
 * Agent 状态快照 (用于多机通信)
 */
USTRUCT(BlueprintType)
struct FAgentStateSnapshot
{
	GENERATED_BODY()

	// Agent 唯一标识
	UPROPERTY(BlueprintReadOnly, Category = "MultiAgent")
	int32 AgentID = -1;

	// 完整飞行状态
	UPROPERTY(BlueprintReadOnly, Category = "MultiAgent")
	FUAVState State;

	// 当前目标位置
	UPROPERTY(BlueprintReadOnly, Category = "MultiAgent")
	FVector TargetPosition;

	// 最近一次 NMPC 输出加速度 (供邻居 CBF-QP 使用)
	UPROPERTY(BlueprintReadOnly, Category = "MultiAgent")
	FVector NMPCAcceleration;

	// 快照时间戳
	UPROPERTY(BlueprintReadOnly, Category = "MultiAgent")
	double Timestamp = 0.0;

	FAgentStateSnapshot()
		: TargetPosition(FVector::ZeroVector)
		, NMPCAcceleration(FVector::ZeroVector)
	{}
};

/**
 * Agent 间通信消息
 */
USTRUCT(BlueprintType)
struct FAgentMessage
{
	GENERATED_BODY()

	// 发送方 AgentID
	UPROPERTY(BlueprintReadOnly, Category = "MultiAgent")
	int32 SenderID = -1;

	// 接收方 AgentID (-1 表示广播)
	UPROPERTY(BlueprintReadOnly, Category = "MultiAgent")
	int32 ReceiverID = -1;

	// 消息载荷
	UPROPERTY(BlueprintReadOnly, Category = "MultiAgent")
	FAgentStateSnapshot Payload;

	// 发送时间
	UPROPERTY(BlueprintReadOnly, Category = "MultiAgent")
	double SendTime = 0.0;
};


/**
 * CBF-QP 工作模式
 */
UENUM(BlueprintType)
enum class ECBFMode : uint8
{
	Disabled	UMETA(DisplayName = "Disabled"),
	ShadowLog	UMETA(DisplayName = "Shadow Log"),
	Active		UMETA(DisplayName = "Active")
};

/**
 * CBF-QP 求解状态
 */
UENUM(BlueprintType)
enum class ECBFQPStatus : uint8
{
	Solved			UMETA(DisplayName = "Solved"),
	SolvedWithSlack	UMETA(DisplayName = "Solved With Slack"),
	MaxIterations	UMETA(DisplayName = "Max Iterations"),
	NumericalFailure UMETA(DisplayName = "Numerical Failure")
};

/**
 * CBF-QP 安全滤波器配置
 */
USTRUCT(BlueprintType)
struct FCBFQPConfig
{
	GENERATED_BODY()

	// 机间安全距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP")
	float DSafe = 500.0f;

	// CBF 类-K 函数系数 (相对阶数 2, 两个系数)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP")
	float Alpha0 = 1.0f;

	// CBF 一阶导数系数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP")
	float Alpha1 = 2.0f;

	// QP 求解器最大迭代次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|Solver")
	int32 MaxIterations = 30;

	// 收敛容差
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|Solver")
	float ConvergenceTolerance = 0.1f;

	// ---- 静态障碍 CBF 参数 ----

	// 静态障碍安全距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|Static")
	float DSafeStatic = 200.0f;

	// 静态障碍 CBF α₀
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|Static")
	float StaticAlpha0 = 1.0f;

	// 静态障碍 CBF α₁
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|Static")
	float StaticAlpha1 = 2.0f;

	// 静态障碍 CBF 影响距离 (cm)，仅在该距离内的障碍物生成约束
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|Static")
	float StaticInfluenceDistance = 1500.0f;

	// ---- Slack 惩罚权重 ----

	// 静态 slack 惩罚权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|Slack")
	float RhoStatic = 100.0f;

	// 机间 slack 惩罚权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|Slack")
	float RhoAgent = 100.0f;

	// ---- 工作模式 ----

	// CBF-QP 工作模式: Disabled/ShadowLog/Active
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|Mode")
	ECBFMode Mode = ECBFMode::Active;

	// ---- QP 求解器参数 ----

	// 加速度上界 (cm/s²)，用于 QP 内约束
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|QP")
	float MaxAccelerationQP = 800.0f;

	// 速度包络上界 (cm/s)，用于 QP 内沿速度方向主动制动
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|QP")
	float MaxVelocity = 2000.0f;

	// QP 内可执行倾角上界 (deg)，需与位置控制器倾角限制保持一致
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|QP")
	float MaxTiltAngleDeg = 30.0f;

	// QP 最大迭代次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|QP")
	int32 QPMaxIterations = 50;

	// QP KKT 容差
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|QP")
	float QPKKTTolerance = 1e-4f;

	// 是否启用 QP 热启动
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CBFQP|QP")
	bool bEnableWarmStart = true;
};

/**
 * 编队配置
 */
USTRUCT(BlueprintType)
struct FFormationConfig
{
	GENERATED_BODY()

	// 编队类型
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Formation")
	EFormationType Type = EFormationType::Line;

	// 机间间距 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Formation")
	float Spacing = 1000.0f;

	// 队形切换过渡时间 (秒)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Formation")
	float TransitionDuration = 2.0f;

	// Leader AgentID (-1 表示质心模式)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Formation")
	int32 LeaderID = 0;
};

/**
 * 联合 NMPC 配置 (扩展单机 FNMPCConfig)
 */
USTRUCT(BlueprintType)
struct FJointNMPCConfig
{
	GENERATED_BODY()

	// 基础 NMPC 配置 (复用单机参数)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JointNMPC")
	FNMPCConfig BaseConfig;

	// 编队偏差权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JointNMPC|Weights")
	float WeightFormation = 0.15f;

	// 机间碰撞代价权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JointNMPC|Weights")
	float WeightInterAgentCollision = 50.0f;

	// 机间安全距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JointNMPC")
	float InterAgentSafeDistance = 500.0f;

	// 机间碰撞影响距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JointNMPC")
	float InterAgentInfluenceDistance = 3000.0f;

	// 最大 Agent 数量
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JointNMPC")
	int32 MaxAgents = 4;

	// Joint NMPC 求解频率 (Hz, 低于单机 20Hz)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JointNMPC")
	float SolveFrequency = 5.0f;
};
