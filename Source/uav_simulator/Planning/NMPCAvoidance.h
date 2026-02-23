// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "../Core/UAVTypes.h"
#include "NMPCAvoidance.generated.h"

/**
 * NMPC 配置参数
 */
USTRUCT(BlueprintType)
struct FNMPCConfig
{
	GENERATED_BODY()

	// 预测步数 N
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC")
	int32 PredictionSteps = 10;

	// 预测时域 T_h (秒)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC")
	float PredictionHorizon = 3.5f;

	// 最大加速度 (cm/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC")
	float MaxAcceleration = 800.0f;

	// 最大速度 (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC")
	float MaxVelocity = 2000.0f;

	// 参考跟踪权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Weights")
	float WeightReference = 0.3f;

	// 速度跟踪权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Weights")
	float WeightVelocity = 0.3f;

	// 控制输入权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Weights")
	float WeightControl = 0.0001f;

	// 障碍物代价权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Weights")
	float WeightObstacle = 1.5f;

	// 终端代价权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Weights")
	float WeightTerminal = 2.0f;

	// 障碍物势垒衰减系数 α
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	float ObstacleAlpha = 0.5f;

	// 障碍物安全距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	float ObstacleSafeDistance = 300.0f;

	// 障碍物影响距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	float ObstacleInfluenceDistance = 2000.0f;

	// 求解器最大迭代次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	int32 MaxIterations = 15;

	// 收敛容差
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float ConvergenceTolerance = 0.1f;

	// 有限差分步长 (cm/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float FiniteDiffEpsilon = 1.0f;

	// 初始学习率
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float InitialStepSize = 500.0f;

	// 回溯线搜索缩减因子
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float BacktrackFactor = 0.5f;

	// 回溯线搜索最大次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	int32 MaxBacktrackSteps = 8;

	// Stuck 检测: 合力阈值
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC")
	float StuckForceThreshold = 1.5f;

	// 到达目标距离阈值 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC")
	float GoalReachedThreshold = 100.0f;

	// 单步障碍物代价上限 (防止 exp 数值爆炸)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	float MaxObstacleCostPerStep = 200.0f;

	// 横向扰动幅度 (cm/s^2, 用于跳出局部最优)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float LateralPerturbationMagnitude = 150.0f;

	// 连续代价上升次数阈值，超过后重置温启动
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	int32 WarmStartResetThreshold = 25;

	// reset 后免疫帧数，防止立即重触发（~150ms）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	int32 WarmStartResetImmunity = 200;

	// 代价连续上升判定为卡死的阈值
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	int32 CostRiseStuckThreshold = 8;

	// 每次求解最小有效前进距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float MinProgressPerSolve = 5.0f;

	// 控制接近饱和的比例阈值 (相对 MaxAcceleration)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float SaturationAccelRatio = 0.9f;

	// 障碍物代价死区: 小于该值视为无障碍影响
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	float ObstacleCostDeadband = 0.3f;

	// 纠偏目标最小位移阈值 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC")
	float MinCorrectionDistance = 5.0f;

	// 纠偏目标前瞻步数 (显式欧拉中 step 1 不含控制量，需用更远的步)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC", meta = (ClampMin = "2", ClampMax = "10"))
	int32 CorrectionLookaheadSteps = 5;

	// 获取时间步长 dt
	float GetDt() const { return PredictionHorizon / FMath::Max(PredictionSteps, 1); }
};

/**
 * NMPC 预测步数据 (可视化用)
 */
USTRUCT(BlueprintType)
struct FNMPCPredictionStep
{
	GENERATED_BODY()

	// 预测位置
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	FVector Position;

	// 预测速度
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	FVector Velocity;

	// 控制输入 (加速度)
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	FVector ControlInput;

	// 该步的障碍物代价
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	float ObstacleCost;

	FNMPCPredictionStep()
		: Position(FVector::ZeroVector)
		, Velocity(FVector::ZeroVector)
		, ControlInput(FVector::ZeroVector)
		, ObstacleCost(0.0f)
	{}
};

/**
 * NMPC 避障结果
 */
USTRUCT(BlueprintType)
struct FNMPCAvoidanceResult
{
	GENERATED_BODY()

	// 修正后目标位置 (第一步控制输入应用后的位置)
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	FVector CorrectedTarget;

	// 修正方向 (归一化)
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	FVector CorrectedDirection;

	// 合力向量 (从第一步控制输入映射, 可视化用)
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	FVector TotalForce;

	// 引力分量 (指向参考点方向, 可视化用)
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	FVector AttractiveForce;

	// 斥力分量 (控制输入 - 引力, 可视化用)
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	FVector RepulsiveForce;

	// 是否需要修正
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	bool bNeedsCorrection;

	// 是否陷入困境 (四面包围, 无法找到可行解)
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	bool bStuck;

	// 预测轨迹 (可视化用)
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	TArray<FNMPCPredictionStep> PredictedTrajectory;

	// 总代价
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	float TotalCost;

	// 第一步最优控制量 u*[0]（cm/s²），直接用于控制层
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	FVector OptimalAcceleration;

	FNMPCAvoidanceResult()
		: CorrectedTarget(FVector::ZeroVector)
		, CorrectedDirection(FVector::ForwardVector)
		, TotalForce(FVector::ZeroVector)
		, AttractiveForce(FVector::ZeroVector)
		, RepulsiveForce(FVector::ZeroVector)
		, bNeedsCorrection(false)
		, bStuck(false)
		, TotalCost(0.0f)
		, OptimalAcceleration(FVector::ZeroVector)
	{}
};

/**
 * NMPC (非线性模型预测控制) 局部避障器
 *
 * 通过滚动优化预测窗口内的控制序列实现避障:
 * - 状态向量: [px, py, pz, vx, vy, vz]
 * - 控制向量: [ax, ay, az] (期望加速度)
 * - 预测模型: 离散欧拉积分
 * - 代价函数: 参考跟踪 + 速度跟踪 + 控制代价 + 障碍物势垒 + 终端代价
 * - 求解器: 投影梯度下降 + 回溯线搜索 + 温启动
 */
UCLASS(BlueprintType)
class UAV_SIMULATOR_API UNMPCAvoidance : public UObject
{
	GENERATED_BODY()

public:
	UNMPCAvoidance();

	/**
	 * 计算 NMPC 避障修正
	 * @param CurrentPosition 当前位置 (cm)
	 * @param CurrentVelocity 当前速度 (cm/s)
	 * @param ReferencePoints 参考轨迹点 (N+1 个, 从当前时刻开始)
	 * @param Obstacles 附近障碍物列表
	 * @return 避障结果
	 */
	UFUNCTION(BlueprintCallable, Category = "NMPC Avoidance")
	FNMPCAvoidanceResult ComputeAvoidance(
		const FVector& CurrentPosition,
		const FVector& CurrentVelocity,
		const TArray<FVector>& ReferencePoints,
		const TArray<FObstacleInfo>& Obstacles);

	// NMPC 配置
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC")
	FNMPCConfig Config;

	// ---- 内部方法 (public for testing) ----

	/**
	 * 前向仿真: 给定初始状态和控制序列, 计算预测轨迹
	 * @param InitPos 初始位置
	 * @param InitVel 初始速度
	 * @param Controls 控制序列 (N 个加速度向量)
	 * @param OutPositions 输出位置序列 (N+1 个)
	 * @param OutVelocities 输出速度序列 (N+1 个)
	 */
	void ForwardSimulate(
		const FVector& InitPos,
		const FVector& InitVel,
		const TArray<FVector>& Controls,
		TArray<FVector>& OutPositions,
		TArray<FVector>& OutVelocities) const;

	/**
	 * 计算总代价
	 * @param Positions 位置序列 (N+1)
	 * @param Velocities 速度序列 (N+1)
	 * @param Controls 控制序列 (N)
	 * @param ReferencePoints 参考点 (N+1)
	 * @param Obstacles 障碍物列表
	 * @return 总代价
	 */
	float ComputeCost(
		const TArray<FVector>& Positions,
		const TArray<FVector>& Velocities,
		const TArray<FVector>& Controls,
		const TArray<FVector>& ReferencePoints,
		const TArray<FObstacleInfo>& Obstacles) const;

	/**
	 * 计算单个障碍物的代价 (指数势垒)
	 * @param Position 查询位置
	 * @param Obstacle 障碍物信息
	 * @return 障碍物代价
	 */
	float ComputeObstacleCost(const FVector& Position, const FObstacleInfo& Obstacle) const;

	/**
	 * 计算点到障碍物表面的距离
	 * @param Point 查询点
	 * @param Obstacle 障碍物信息
	 * @return 距离 (正值=外部, 负值=内部)
	 */
	float CalculateDistanceToObstacle(const FVector& Point, const FObstacleInfo& Obstacle) const;

	/**
	 * 预测动态障碍物未来位置
	 * @param Obstacle 障碍物信息
	 * @param DeltaTime 预测时间 (秒)
	 * @return 预测后的障碍物信息
	 */
	FObstacleInfo PredictObstacle(const FObstacleInfo& Obstacle, float DeltaTime) const;

	/**
	 * 投影控制序列到可行域 (加速度和速度约束)
	 * @param Controls 控制序列 (会被修改)
	 * @param InitVel 初始速度
	 */
	void ProjectControls(TArray<FVector>& Controls, const FVector& InitVel) const;

	/**
	 * 温启动: 将上次最优控制序列左移一步
	 */
	void WarmStart();

	/**
	 * 计算前方障碍物的最小距离 (用于主动减速)
	 * @param CurrentPosition 当前位置
	 * @param LookAheadDistance 前瞻距离 (cm)
	 * @param Obstacles 障碍物列表
	 * @return 最小距离 (cm)，MAX_FLT 表示无障碍物
	 */
	float GetMinObstacleDistance(
		const FVector& CurrentPosition,
		float LookAheadDistance,
		const TArray<FObstacleInfo>& Obstacles) const;

private:
	// 上次最优控制序列 (用于温启动)
	TArray<FVector> PreviousControls;

	// 是否有上次的控制序列
	bool bHasPreviousControls = false;

	// 上次总代价 (用于代价上升趋势统计)
	float PreviousTotalCost = MAX_FLT;

	// 连续代价上升计数
	int32 ConsecutiveCostRiseCount = 0;

	// reset 后剩余免疫帧数
	int32 WarmStartResetImmunityCount = 0;

	// 上一帧状态 (用于日志状态转换检测)
	bool bPrevNeedsCorrection = false;
	bool bPrevStuck = false;

	// 滞后计数器：NeedsCorrection 触发后最少保持的帧数
	int32 NeedsCorrectionHoldCount = 0;
	// 冷却计数器：Y->N 后最少保持 N 的帧数，防止立即重新触发
	int32 NeedsCorrectionCooldownCount = 0;
	// MaxHorizonObs EMA 平滑值（滤除 NMPC 求解噪声）
	float SmoothedMaxHorizonObs = 0.0f;
	// 上次周期性日志时间戳（秒）
	double LastPeriodicLogTime = 0.0;

	/**
	 * 有限差分计算梯度
	 * @param InitPos 初始位置
	 * @param InitVel 初始速度
	 * @param Controls 当前控制序列
	 * @param ReferencePoints 参考点
	 * @param Obstacles 障碍物
	 * @param OutGradient 输出梯度 (N 个向量)
	 */
	void ComputeGradient(
		const FVector& InitPos,
		const FVector& InitVel,
		const TArray<FVector>& Controls,
		const TArray<FVector>& ReferencePoints,
		const TArray<FObstacleInfo>& Obstacles,
		TArray<FVector>& OutGradient) const;
};
