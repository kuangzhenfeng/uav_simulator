// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "UAVTypes.h"
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

	// 预定义航点数组（已废弃，请使用 MissionComponent）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV|Waypoints", meta = (DeprecatedProperty, DeprecationMessage = "Use MissionComponent instead"))
	TArray<FVector> Waypoints;

	// 向后兼容：控制模式（已废弃，请使用 ControlMode）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Control", meta = (DeprecatedProperty, DeprecationMessage = "Use ControlMode instead"))
	bool bUsePositionControl = true;

protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Parameters")
	float Mass = 1.5f; // kg

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Parameters")
	float ArmLength = 0.225f; // m

private:
	// 更新传感器数据
	void UpdateSensors(float DeltaTime);

	// 更新控制器
	void UpdateController(float DeltaTime);

	// 更新物理模型
	void UpdatePhysics(float DeltaTime);
};
