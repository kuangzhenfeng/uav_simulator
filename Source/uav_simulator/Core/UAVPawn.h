// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "UAVTypes.h"
#include "UAVProductTypes.h"
#include "UAVProductManager.h"
#include "../Planning/NMPCAvoidance.h"
#include "../MultiAgent/MultiAgentTypes.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "UAVPawn.generated.h"

class UUAVDynamics;
class USensorBase;
class UAttitudeController;
class UPositionController;
class UDebugVisualizer;
class AUAVAIController;
class UTrajectoryTracker;
class UObstacleManager;
class UPlanningVisualizer;
class UMissionComponent;
class UObstacleDetector;
class UStabilityScorer;
class UControlParameterTuner;
class UNMPCAvoidance;
class ULinearMPCAvoidance;
class UAgentCommunicationComponent;
class UFormationComponent;
class UCBFQPFilter;
class AMultiAgentGameMode;
class UWindField;
class UBarometerSensor;
class UMagnetometerSensor;
class UAnemometerSensor;

/**
 * 无人机Pawn类
 * 继承自APawn以支持AI控制器
 * 负责整合物理模型、传感器、控制器等组件
 */
UCLASS()
class UAV_SIMULATOR_API AUAVPawn : public APawn
{
	GENERATED_BODY()

public:
	AUAVPawn();

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;

	// 获取当前无人机状态
	UFUNCTION(BlueprintCallable, Category = "UAV")
	FUAVState GetUAVState() const { return CurrentState; }

	// 设置目标姿态 (用于姿态控制)
	UFUNCTION(BlueprintCallable, Category = "UAV")
	void SetTargetAttitude(FRotator InTargetAttitude);

	// 设置目标位置 (用于位置控制)
	UFUNCTION(BlueprintCallable, Category = "UAV")
	void SetTargetPosition(FVector InTargetPosition);

	// 获取目标位置
	UFUNCTION(BlueprintCallable, Category = "UAV")
	FVector GetTargetPosition() const { return TargetPosition; }

	// 检查是否到达目标位置
	UFUNCTION(BlueprintCallable, Category = "UAV")
	bool HasReachedTarget(float Tolerance = 100.0f) const;

	// 设置轨迹并开始跟踪
	UFUNCTION(BlueprintCallable, Category = "UAV|Trajectory")
	void SetTrajectory(const FTrajectory& InTrajectory);

	// 开始轨迹跟踪
	UFUNCTION(BlueprintCallable, Category = "UAV|Trajectory")
	void StartTrajectoryTracking();

	// 停止轨迹跟踪
	UFUNCTION(BlueprintCallable, Category = "UAV|Trajectory")
	void StopTrajectoryTracking();

	// 获取轨迹跟踪进度
	UFUNCTION(BlueprintCallable, Category = "UAV|Trajectory")
	float GetTrajectoryProgress() const;

	// 检查轨迹跟踪是否完成
	UFUNCTION(BlueprintCallable, Category = "UAV|Trajectory")
	bool IsTrajectoryComplete() const;

	// 检查轨迹跟踪是否因超时完成
	UFUNCTION(BlueprintCallable, Category = "UAV|Trajectory")
	bool IsTrajectoryTimedOut() const;

	// 设置预定义航点
	UFUNCTION(BlueprintCallable, Category = "UAV|Waypoints", meta = (DeprecatedFunction, DeprecationMessage = "Use MissionComponent->SetWaypoints instead"))
	void SetWaypoints(const TArray<FVector>& InWaypoints);

	// 获取预定义航点
	UFUNCTION(BlueprintCallable, Category = "UAV|Waypoints", meta = (DeprecatedFunction, DeprecationMessage = "Use MissionComponent->GetWaypointPositions instead"))
	TArray<FVector> GetWaypoints() const;

	// 检查是否有航点
	UFUNCTION(BlueprintCallable, Category = "UAV|Waypoints", meta = (DeprecatedFunction, DeprecationMessage = "Use MissionComponent->HasWaypoints instead"))
	bool HasWaypoints() const;

	// 清除航点
	UFUNCTION(BlueprintCallable, Category = "UAV|Waypoints", meta = (DeprecatedFunction, DeprecationMessage = "Use MissionComponent->ClearWaypoints instead"))
	void ClearWaypoints();

	// 设置控制模式
	UFUNCTION(BlueprintCallable, Category = "UAV|Control")
	void SetControlMode(EUAVControlMode NewMode);

	// 获取控制模式
	UFUNCTION(BlueprintCallable, Category = "UAV|Control")
	EUAVControlMode GetControlMode() const { return ControlMode; }

	// 获取飞行状态
	UFUNCTION(BlueprintCallable, Category = "UAV|State")
	EFlightState GetFlightState() const { return FlightState; }

	// 是否已炸机
	UFUNCTION(BlueprintCallable, Category = "UAV|State")
	bool IsCrashed() const { return FlightState == EFlightState::Crashed; }

	// 获取轨迹跟踪组件
	UFUNCTION(BlueprintCallable, Category = "UAV|Components")
	UTrajectoryTracker* GetTrajectoryTracker() const { return TrajectoryTrackerComponent; }

	// 获取障碍物管理器
	UFUNCTION(BlueprintCallable, Category = "UAV|Components")
	UObstacleManager* GetObstacleManager() const { return ObstacleManagerComponent; }

	// 获取规划可视化器
	UFUNCTION(BlueprintCallable, Category = "UAV|Components")
	UPlanningVisualizer* GetPlanningVisualizer() const { return PlanningVisualizerComponent; }

	// 获取任务管理组件
	UFUNCTION(BlueprintCallable, Category = "UAV|Components")
	UMissionComponent* GetMissionComponent() const { return MissionComponent; }

	// 获取障碍物感知传感器
	UFUNCTION(BlueprintCallable, Category = "UAV|Components")
	UObstacleDetector* GetObstacleDetector() const { return ObstacleDetectorComponent; }

	// 获取稳定性评分组件
	UFUNCTION(BlueprintCallable, Category = "UAV|Components")
	UStabilityScorer* GetStabilityScorer() const { return StabilityScorerComponent; }

	// NMPC stuck 状态查询
	bool IsNMPCStuck() const { return bNMPCStuck; }


	// 获取当前 NMPC 加速度（供 AgentManager 状态缓存传播到 CBF-QP）
	FVector GetNMPCAcceleration() const { return SmoothedNMPCAcceleration; }
		// 获取 UAV 碰撞半径（cm），基于 ArmLength + 安全余量
		// 用于障碍物感知时替代 GetActorBounds 返回的过大包围盒
		float GetCollisionRadius() const
		{
			// ArmLength 单位是 m，转换为 cm，乘以 2.5 得到机体等效半径（含桨叶）
			const FUAVModelSpec Spec = FUAVProductManager::GetModelSpec(ModelID);
			return Spec.ArmLength * 100.0f * 2.5f;
		}

	// ---- 多机协同接口 ----

	// 获取 AgentID（-1 表示未注册/单机模式）
	UFUNCTION(BlueprintCallable, Category = "MultiAgent")
	int32 GetAgentID() const { return AgentID; }

	// 是否处于多机模式
	UFUNCTION(BlueprintCallable, Category = "MultiAgent")
	bool IsMultiAgentMode() const { return bIsMultiAgentMode; }

	// 获取通信组件
	UFUNCTION(BlueprintCallable, Category = "MultiAgent")
	UAgentCommunicationComponent* GetCommunicationComponent() const { return CommunicationComponent; }

	// 获取编队控制组件
	UFUNCTION(BlueprintCallable, Category = "MultiAgent")
	UFormationComponent* GetFormationComponent() const { return FormationComponent; }

protected:
	// 根组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<USceneComponent> RootSceneComponent;

	// 物理模型组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UUAVDynamics> DynamicsComponent;

	// 姿态控制器组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UAttitudeController> AttitudeControllerComponent;

	// 位置控制器组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UPositionController> PositionControllerComponent;

	// 传感器列表
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TArray<TObjectPtr<USensorBase>> Sensors;

	// 调试可视化组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UDebugVisualizer> DebugVisualizerComponent;

	// 轨迹跟踪组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UTrajectoryTracker> TrajectoryTrackerComponent;

	// 障碍物管理组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UObstacleManager> ObstacleManagerComponent;

	// 规划可视化组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UPlanningVisualizer> PlanningVisualizerComponent;

	// 任务管理组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UMissionComponent> MissionComponent;

	// 障碍物感知传感器组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UObstacleDetector> ObstacleDetectorComponent;

	// 稳定性评分组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UStabilityScorer> StabilityScorerComponent;

	// PID调参组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UControlParameterTuner> ParameterTunerComponent;

	// 多机通信组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MultiAgent Components")
	TObjectPtr<UAgentCommunicationComponent> CommunicationComponent;

	// 编队控制组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MultiAgent Components")
	TObjectPtr<UFormationComponent> FormationComponent;

	// 风场组件（Phase 14: 环境模拟）
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Environment Components")
	TObjectPtr<UWindField> WindFieldComponent;

	// 气压计传感器
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensor Components")
	TObjectPtr<UBarometerSensor> BarometerSensor;

	// 磁力计传感器
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensor Components")
	TObjectPtr<UMagnetometerSensor> MagnetometerSensor;

	// 风速计传感器
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensor Components")
	TObjectPtr<UAnemometerSensor> AnemometerSensor;

	// 跟随相机系统
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<USpringArmComponent> CameraBoom;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV Components")
	TObjectPtr<UCameraComponent> FollowCamera;

	// NMPC 避障组件（全程接管轨迹跟踪+避障）
	UPROPERTY()
	TObjectPtr<UNMPCAvoidance> NMPCComponent;

	// 线性MPC 避障组件（可选，轻量级替代）
	UPROPERTY()
	TObjectPtr<ULinearMPCAvoidance> LinearMPCComponent;

	// 当前状态
	UPROPERTY(BlueprintReadOnly, Category = "UAV State")
	FUAVState CurrentState;

	// 目标姿态
	UPROPERTY(BlueprintReadWrite, Category = "UAV Control")
	FRotator TargetAttitude;

	// 目标位置
	UPROPERTY(BlueprintReadWrite, Category = "UAV Control")
	FVector TargetPosition;

	// 控制模式
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Control")
	EUAVControlMode ControlMode = EUAVControlMode::Position;

	// 飞行状态
	UPROPERTY(BlueprintReadOnly, Category = "UAV State")
	EFlightState FlightState = EFlightState::Flying;

	// 预定义航点数组（已废弃，请使用 MissionComponent）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV|Waypoints", meta = (DeprecatedProperty, DeprecationMessage = "Use MissionComponent instead"))
	TArray<FVector> Waypoints;

	// 设置载荷质量（运行时可变，如喷洒消耗药液）
	UFUNCTION(BlueprintCallable, Category = "UAV Model")
	void SetPayloadMass(float NewPayloadMass);

protected:
	// 型号选择
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Model")
	EUAVModelID ModelID = EUAVModelID::Agri_AG20;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Model")
	float CurrentPayloadMass = 0.0f;	// kg，运行时可变

private:
	bool bNMPCStuck = false;

	// NMPC 调用节流：缓存上次结果，避免每个子步都调用
	float NMPCSolveAccumulator = 0.0f;
	FVector CachedNMPCAcceleration = FVector::ZeroVector;
	FVector SmoothedNMPCAcceleration = FVector::ZeroVector;
	bool bHasCachedNMPC = false;

	// stuck 逃逸冷却：stuck 时延长不求解时间，让逃逸加速度持续生效
	float StuckEscapeCooldown = 0.0f;


	// 缓存最近障碍物距离（cm），用于偏差保护和减速决策
	float CachedNearestObsDist = MAX_FLT;

	// 缓存 NMPC 修正目标位置，用于障碍物感知偏差计算
	FVector CachedNMPCCorrectedTarget = FVector::ZeroVector;
	bool bHasCachedNMPCTarget = false;

	// ---- 多机协同私有状态 ----

	// Agent 唯一标识（由 AgentManager 分配）
	int32 AgentID = -1;

	// 是否处于多机模式
	bool bIsMultiAgentMode = false;

	// CBF-QP 安全滤波器实例
	UPROPERTY()
	TObjectPtr<UCBFQPFilter> CBFQPFilter;

	// CBF-QP 配置
	FCBFQPConfig CBFQPConfig;

	// ---- 原有私有方法 ----

	// 更新传感器数据
	void UpdateSensors(float DeltaTime);

	// 更新控制器
	void UpdateController(float DeltaTime);

	// 更新物理模型
	void UpdatePhysics(float DeltaTime);

	// 碰撞检测（与障碍物穿透时触发炸机）
	void CheckCollision();

	// 触发炸机逻辑
	void TriggerCrash();

	// NMPC 求解相关
	bool ShouldSolveNMPC(float DeltaTime);
	void SolveNMPCAvoidance(float DeltaTime);
	void PrepareReferencePoints(TArray<FVector>& OutReferencePoints);
	void FilterNearbyObstacles(TArray<FObstacleInfo>& OutObstacles);
	void FixReferencePointsPenetratingObstacles(
		TArray<FVector>& ReferencePoints,
		const TArray<FObstacleInfo>& Obstacles);

	// 逃逸逻辑
	void HandleStuckEscape(
		const FNMPCAvoidanceResult& Result,
		const TArray<FObstacleInfo>& NearbyObstacles);
	FVector CalculateEscapeDirection(const TArray<FObstacleInfo>& NearbyObstacles);
	float CalculateEscapeAcceleration(float NearestObsDist);

	// 偏差保护
	FVector ApplyDeviationProtection(const FVector& NMPCAcceleration);
	FVector LimitLateralAcceleration(const FVector& Acceleration);
	FVector ApplyPDCorrection(const FVector& Acceleration);
	FVector ApplyHardLimitCorrection(const FVector& Acceleration, float CrossTrackDev);
	void UpdateSpeedScaleForObstacles();

	// 紧急制动
	FVector ApplyEmergencyBraking(const FVector& Acceleration);

	// 速度钳位
	FVector ApplyVelocityClamp(const FVector& Acceleration);

	// 位置保持
	void ExecutePositionHold();
	FVector GetTrajectoryEndpointOrTarget();

	// 从线性加速度计算期望角加速度（用于前馈控制）
	// 基于 Mellinger & Kumar 微分平坦方法，通过 jerk 解析推导角速度，再数值微分得角加速度
	FRotator ComputeAngularAccelerationFromLinearAccel(
		const FVector& LinearAccel,
		float CurrentYaw);

	// 前馈数值微分历史状态
	FVector PrevFeedforwardAccel = FVector::ZeroVector;     // 上一帧线性加速度
	FVector PrevDesiredAngularVel = FVector::ZeroVector;    // 上一帧期望角速度 (rad/s)
	FVector PrevFilteredAngularAccel = FVector::ZeroVector; // 上一帧滤波后角加速度 (rad/s²)
	int32 FeedforwardWarmupCount = 0;                       // 预热计数器（需2帧积累历史）
};
