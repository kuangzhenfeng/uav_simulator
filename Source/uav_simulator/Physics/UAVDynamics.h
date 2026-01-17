// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "UAVDynamics.generated.h"

/**
 * 四旋翼无人机动力学模型组件
 * 实现基于牛顿-欧拉方程的六自由度动力学模型
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UUAVDynamics : public UActorComponent
{
	GENERATED_BODY()

public:
	UUAVDynamics();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 更新动力学模型
	UFUNCTION(BlueprintCallable, Category = "UAV Dynamics")
	FUAVState UpdateDynamics(const FUAVState& CurrentState, float DeltaTime);

	// 设置电机推力值 (0-1归一化)
	UFUNCTION(BlueprintCallable, Category = "UAV Dynamics")
	void SetMotorThrusts(const TArray<float>& InThrusts);

	// 获取当前电机推力
	UFUNCTION(BlueprintCallable, Category = "UAV Dynamics")
	TArray<float> GetMotorThrusts() const { return MotorThrusts; }

protected:
	// 物理参数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Parameters")
	float Mass = 1.5f; // kg

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Parameters")
	float ArmLength = 0.225f; // m

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Parameters")
	FVector MomentOfInertia = FVector(0.029f, 0.029f, 0.055f); // kg·m²

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Parameters")
	float MaxThrust = 15.0f; // N (单个电机最大推力)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Parameters")
	float DragCoefficient = 0.1f; // 空气阻力系数

	// 电机推力 (0-1归一化)
	UPROPERTY(BlueprintReadOnly, Category = "Motor State")
	TArray<float> MotorThrusts;

private:
	// 计算总推力和力矩
	void ComputeForcesAndTorques(FVector& OutForce, FVector& OutTorque) const;

	// 重力常量 (UE5使用cm/s², 需要转换)
	static constexpr float GravityAcceleration = 980.0f; // cm/s²
};
