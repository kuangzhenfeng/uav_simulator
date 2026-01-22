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

	// 当前状态
	UPROPERTY(BlueprintReadOnly, Category = "UAV State")
	FUAVState CurrentState;

	// 目标姿态
	UPROPERTY(BlueprintReadWrite, Category = "UAV Control")
	FRotator TargetAttitude;

	// 目标位置
	UPROPERTY(BlueprintReadWrite, Category = "UAV Control")
	FVector TargetPosition;

	// 控制模式：true=位置控制，false=姿态控制
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Control")
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
