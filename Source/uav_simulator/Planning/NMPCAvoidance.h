// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "../Core/UAVTypes.h"
#include "NMPCAvoidance.generated.h"

/**
 * MPC类型枚举
 */
UENUM(BlueprintType)
enum class EMPCType : uint8
{
	Nonlinear UMETA(DisplayName = "Nonlinear MPC"),
	Linear UMETA(DisplayName = "Linear MPC")
};

// ==================== 嵌套配置结构 ====================

/**
 * NMPC 求解器配置
 * 关系: PredictionHorizon / PredictionSteps = dt
 */
USTRUCT(BlueprintType)
struct FNMPCSolverConfig
{
	GENERATED_BODY()

	// 预测步数 N
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	int32 PredictionSteps = 10;

	// 预测时域 T_h (秒)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float PredictionHorizon = 2.0f;

	// 求解器最大迭代次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	int32 MaxIterations = 25;

	// 收敛容差
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float ConvergenceTolerance = 0.1f;

	// 有限差分步长 (cm/s²)，应按控制尺度设置
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float FiniteDiffEpsilon = 20.0f;

	// 梯度下降步长 (cm/s²)：归一化梯度方向上的步进量
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float InitialStepSize = 100.0f;

	// 回溯线搜索缩减因子
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	float BacktrackFactor = 0.5f;

	// 回溯线搜索最大次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	int32 MaxBacktrackSteps = 8;

	// 连续代价上升次数阈值，超过后重置温启动
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	int32 WarmStartResetThreshold = 30;

	// reset 后免疫帧数，防止立即重触发（~150ms）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	int32 WarmStartResetImmunity = 200;
};

/**
 * NMPC 代价权重配置
 * 注意: 改为平方误差后，旧权重不能直接复用（量纲从 cm 变为 cm²）
 */
USTRUCT(BlueprintType)
struct FNMPCCostConfig
{
	GENERATED_BODY()

	// 参考跟踪权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
	float WeightReference = 0.3f;

	// 速度跟踪权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
	float WeightVelocity = 0.2f;

	// 控制输入权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
	float WeightControl = 0.001f;

	// 时序一致性权重：惩罚与上一帧解的偏差，抑制帧间振荡
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
	float WeightTemporalConsistency = 0.01f;

	// 障碍物代价权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
	float WeightObstacle = 3.0f;

	// 终端代价权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
	float WeightTerminal = 2.0f;

		// 横向跟踪权重 (Frenet 分解)
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
		float WeightLateral = 0.5f;

		// 纵向滞后权重 (Frenet 分解)
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
		float WeightLag = 0.3f;

		// 路径进度权重
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
		float WeightProgress = 0.1f;

		// 反向运动惩罚权重
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
		float WeightReverse = 0.5f;
};

/**
 * NMPC 障碍物配置
 * 约束: ObstacleInfluenceDistance > ObstacleSafeDistance
 */
USTRUCT(BlueprintType)
struct FNMPCObstacleConfig
{
	GENERATED_BODY()

	// 障碍物势垒衰减系数 α
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	float ObstacleAlpha = 0.4f;

	// 障碍物安全距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	float ObstacleSafeDistance = 300.0f;

	// 障碍物影响距离 (cm)，必须大于安全距离
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	float ObstacleInfluenceDistance = 1000.0f;

	// 单步障碍物代价上限 (防止 exp 数值爆炸)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	float MaxObstacleCostPerStep = 200.0f;

	// 障碍物代价死区: 小于该值视为无障碍影响
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	float ObstacleCostDeadband = 0.3f;

		// Smooth hinge 参数 β
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
		float SmoothHingeBeta = 2.0f;

		// 是否使用 smooth hinge 障碍代价（A/B 对比开关）
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
		bool bUseSmoothHinge = false;
};

/**
 * NMPC 初始化与卡死检测配置
 */
USTRUCT(BlueprintType)
struct FNMPCInitializationConfig
{
	GENERATED_BODY()

	// Stuck 检测: 合力阈值
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Init")
	float StuckForceThreshold = 1.5f;

	// 纠偏目标最小位移阈值 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Init")
	float MinCorrectionDistance = 5.0f;

	// 纠偏目标前瞻步数 (显式欧拉中 step 1 不含控制量，需用更远的步)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Init", meta = (ClampMin = "2", ClampMax = "10"))
	int32 CorrectionLookaheadSteps = 5;

	// 横向扰动幅度 (cm/s^2, 用于跳出局部最优)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Init")
	float LateralPerturbationMagnitude = 150.0f;
};

/**
 * 执行器约束配置
 * 约束: 制动距离 = v²/(2*a_max)，与 MaxVelocity 和 MaxAcceleration 一致
 */
USTRUCT(BlueprintType)
struct FActuatorConstraintConfig
{
	GENERATED_BODY()

	// 最大加速度 (cm/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Actuator")
	float MaxAcceleration = 400.0f;

	// 最大速度 (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Actuator")
	float MaxVelocity = 2000.0f;

		// Dykstra 投影最大迭代数
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Actuator")
		int32 MaxProjectionIterations = 10;

		// Dykstra 投影可行性容差
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Actuator")
		float ProjectionTolerance = 1.0f;

		// 是否使用 Dykstra 投影（false = 旧顺序投影）
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Actuator")
		bool bUseDykstraProjection = true;
};

/**
 * NMPC 诊断配置（预留）
 */
USTRUCT(BlueprintType)
struct FNMPCDiagnosticsConfig
{
	GENERATED_BODY()
};

// ==================== 顶层配置 ====================

/**
 * NMPC 配置参数
 */
USTRUCT(BlueprintType)
struct FNMPCConfig
{
	GENERATED_BODY()

	// MPC类型
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MPC")
	EMPCType MPCType = EMPCType::Nonlinear;

	// 求解器参数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Solver")
	FNMPCSolverConfig Solver;

	// 代价权重
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Cost")
	FNMPCCostConfig Cost;

	// 障碍物参数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Obstacle")
	FNMPCObstacleConfig Obstacle;

	// 初始化与卡死检测
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Init")
	FNMPCInitializationConfig Init;

	// 执行器约束
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Actuator")
	FActuatorConstraintConfig Actuator;

	// 诊断参数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NMPC|Diag")
	FNMPCDiagnosticsConfig Diag;

	// 获取时间步长 dt
	float GetDt() const { return Solver.PredictionHorizon / FMath::Max(Solver.PredictionSteps, 1); }

	// 线性MPC权重访问器（复用NMPC权重）
	float GetPositionWeight() const { return Cost.WeightReference; }
	float GetControlWeight() const { return Cost.WeightControl; }
	float GetObstacleWeight() const { return Cost.WeightObstacle; }
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
 * NMPC 求解诊断信息
 */
USTRUCT(BlueprintType)
struct FNMPCSolveDiagnostics
{
	GENERATED_BODY()

	// 初始代价
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	float InitialCost = MAX_FLT;

	// 最终代价
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	float FinalCost = MAX_FLT;

	// 相对代价下降率
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	float RelativeCostDecrease = 0.0f;

	// 梯度范数
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	float GradientNorm = 0.0f;

	// 迭代次数
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	int32 Iterations = 0;

	// 回溯失败次数
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	int32 BacktrackFailCount = 0;

	// 求解失败原因（拆分为具体标志）
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	bool bMaxIterReached = false;       // 达到最大迭代次数未收敛
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	bool bLineSearchFailed = false;     // 线搜索完全失败
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	bool bClearanceInsufficient = false; // 收敛但预测净空不足
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	bool bProgressInsufficient = false; // 收敛但路径进度不足
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	bool bNaNOrInf = false;             // 代价出现 NaN 或 Inf

	// 初值类型
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	FString InitType = TEXT("Unknown");

	// 最小预测净空 (cm) — 基于全时域预测轨迹
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	float MinPredictedClearance = MAX_FLT;

	// 预测路径进度 (cm)
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	float PredictedPathProgress = 0.0f;

	// 求解时间 (ms)
	UPROPERTY(BlueprintReadOnly, Category = "NMPC|Diag")
	float SolveTimeMs = 0.0f;
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

	// 求解诊断信息
	UPROPERTY(BlueprintReadOnly, Category = "NMPC")
	FNMPCSolveDiagnostics Diagnostics;

	FNMPCAvoidanceResult()
		: CorrectedTarget(FVector::ZeroVector)
		, CorrectedDirection(FVector::ForwardVector)
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
 * - 求解器: 投影梯度下降 + 回溯线搜索 + 温启动 + 确定性多初值
 */
/**
 * 绕行侧记忆 (Homotopy class)
 */
enum class EAvoidanceHomotopy : uint8
{
	None,
	Left,
	Right,
	Above,
	Below
};

/**
 * NMPC 初始化候选类型
 */
enum class EInitCandidateType : uint8
{
	Warm,
	Nominal,
	Left,
	Right,
	Up,
	Down,
	Brake
};

/**
 * NMPC 初始化候选
 */
struct FInitCandidate
{
	EInitCandidateType Type = EInitCandidateType::Nominal;
	TArray<FVector> Controls;
	float Cost = MAX_FLT;
	float MinClearance = MAX_FLT;
	float PathProgress = 0.0f;
};


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
	 * 计算有符号距离的梯度 ∇d(p)
	 * 梯度方向: 从障碍物表面指向查询点（外部时指向外，内部时指向最小穿透方向）
	 * @param Point 查询点
	 * @param Obstacle 障碍物信息
	 * @return 梯度向量 (归一化或接近归一化)
	 */
	FVector ComputeDistanceGradient(const FVector& Point, const FObstacleInfo& Obstacle) const;
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
		 * Dykstra 投影: 同时满足加速度和速度约束
		 */
		void DykstraProjectStep(
			FVector& Control,
			const FVector& CurrentVel,
			float dt) const;

	/**
	 * 温启动: 将上次最优控制序列左移一步
	 */
	void WarmStart();

		/**
		 * Frenet 坐标分解: 将位置误差分解为纵向/横向
		 */

		/**
		 * 生成多初值候选
		 */
		TArray<FInitCandidate> GenerateCandidates(
			const TArray<FVector>& ReferencePoints,
			const FVector& CurrentVelocity) const;

		/**
		 * 从 2-4 个控制 knot 插值生成完整控制序列
		 */
		void InterpolateFromKnots(
			TArray<FVector>& OutControls,
			const TArray<FVector>& Knots,
			const TArray<int32>& KnotIndices) const;
		void ComputeFrenetCoordinates(
			const FVector& Position,
			const FVector& RefPoint,
			const FVector& NextRefPoint,
			FVector& OutTangent,
			float& OutParallel,
			FVector& OutPerpendicular) const;

private:
	// 上次最优控制序列 (用于温启动)
	TArray<FVector> PreviousControls;

	// 是否有上次的控制序列
	bool bHasPreviousControls = false;

	// ---- Homotopy 绕行侧记忆 ----
	EAvoidanceHomotopy CurrentHomotopy = EAvoidanceHomotopy::None;
	float HomotopyCost = MAX_FLT;
	int32 HomotopyObstacleID = -1;

	// 上一帧状态 (用于日志状态转换检测)
	bool bPrevNeedsCorrection = false;
	bool bPrevStuck = false;

	// 滞后计数器：NeedsCorrection 触发后最少保持的帧数
	int32 NeedsCorrectionHoldCount = 0;
	// MaxHorizonObs EMA 平滑值（滤除 NMPC 求解噪声）
	float SmoothedMaxHorizonObs = 0.0f;
	// 卡死逃逸模式：触发后持续多帧强制突破
	int32 StuckEscapeCount = 0;

	// 位置基准卡死检测
	FVector StuckCheckPosition = FVector::ZeroVector;
	int32 SlowProgressCount = 0;

	// 振荡卡死检测：长窗口内净位移极小但持续在动
	FVector OscillationAnchor = FVector::ZeroVector;
	int32 OscillationSolveCount = 0;
	float OscillationPathLength = 0.0f;
	FVector OscillationPrevPos = FVector::ZeroVector;
	bool bOscillationInitialized = false;

	// WarmStart 初始化
	void InitializeControls(
		TArray<FVector>& OutControls,
		const TArray<FVector>& ReferencePoints);
	bool ShouldUseWarmStart() const;
	FVector ComputeInitialControl(const TArray<FVector>& ReferencePoints) const;

	// 振荡卡死检测
	bool DetectOscillationStuck(const FVector& CurrentPosition, bool bNeedsCorrection);
	void ResetOscillationDetection(const FVector& CurrentPosition);
	void UpdateOscillationMetrics(const FVector& CurrentPosition);

	// 卡死检测
	bool DetectPositionStuck(const FVector& CurrentPosition, bool bNeedsCorrection);
	void ResetStuckDetection(const FVector& CurrentPosition);

	// 圆柱体距离计算
	float CalculateCylinderDistance(
		const FVector& Point,
		const FObstacleInfo& Obstacle) const;

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
