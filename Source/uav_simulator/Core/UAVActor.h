// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UAVTypes.h"
#include "UAVActor.generated.h"

class UUAVDynamics;
class USensorBase;
class UAttitudeController;
class UPositionController;
class UDebugVisualizer;

/**
 * 无人机Actor基类
 * 负责整合物理模型、传感器、控制器等组件
 */
UCLASS()
class UAV_SIMULATOR_API AUAVActor : public AActor
{
	GENERATED_BODY()

public:
	AUAVActor();

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

protected:
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
