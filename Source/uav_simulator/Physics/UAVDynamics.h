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

	// 获取线性加速度（机体坐标系，用于 IMU 仿真）
	UFUNCTION(BlueprintCallable, Category = "UAV Dynamics")
	FVector GetLinearAcceleration() const { return LinearAcceleration; }

	// 批量设置物理参数（供型号注册表调用）
	void SetPhysicsParams(float InMass, float InArmLength, const FVector& InMomentOfInertia, float InMaxThrust)
	{
		Mass = InMass; ArmLength = InArmLength;
		MomentOfInertia = InMomentOfInertia; MaxThrust = InMaxThrust;

		// 根据最大推力和最大转速计算推力系数
		// T = k_t * ω²  =>  k_t = T_max / ω_max²
		ThrustCoefficient = MaxThrust / (MaxMotorSpeed * MaxMotorSpeed);

		// 反扭矩系数通常是推力系数的 1.5-2% (经验值)
		TorqueCoefficient = ThrustCoefficient * 0.016f;
	}

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
	float DragCoefficient = 0.1f; // 线性空气阻力系数

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Parameters")
	float QuadraticDragCoefficient = 0.0002f; // 二次空气阻力系数

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Physics Parameters")
	float MotorTorqueCoefficient = 0.0045f; // 电机扭矩系数 (N·m / N)

	// 电机动力学参数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Parameters")
	float ThrustCoefficient = 1.0e-5f; // 推力系数 k_t (N / (rad/s)²)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Parameters")
	float TorqueCoefficient = 1.6e-7f; // 反扭矩系数 k_m (N·m / (rad/s)²)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Parameters")
	float MotorTimeConstant = 0.02f; // 电机时间常数 (s)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Parameters")
	float MaxMotorSpeed = 838.0f; // 最大电机转速 (rad/s, ~8000 RPM)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor Parameters")
	float MinMotorSpeed = 100.0f; // 最小电机转速 (rad/s, ~955 RPM)

	// 电机推力 (0-1归一化)
	UPROPERTY(BlueprintReadOnly, Category = "Motor State")
	TArray<float> MotorThrusts;

	// 电机转速 (rad/s)
	UPROPERTY(BlueprintReadOnly, Category = "Motor State")
	TArray<float> MotorSpeeds;

	// 线性加速度（机体坐标系，用于 IMU 仿真）
	FVector LinearAcceleration = FVector::ZeroVector;

private:
	// 计算总推力和力矩
	void ComputeForcesAndTorques(FVector& OutForce, FVector& OutTorque) const;

	// 更新电机动力学（一阶惯性环节）
	void UpdateMotorDynamics(float DeltaTime);

	// 从期望推力反解电机转速
	float SolveMotorSpeedFromThrust(float DesiredThrust) const;

	// RK4积分器
	FUAVState IntegrateRK4(const FUAVState& State, float DeltaTime);

	// 计算状态导数
	FUAVState ComputeDerivative(const FUAVState& State, float Time);

	// 计算角加速度（包含陀螺效应）
	FVector ComputeAngularAcceleration(const FVector& Torque, const FVector& AngularVelocity) const;

	// 状态向量操作辅助函数
	FUAVState AddStates(const FUAVState& A, const FUAVState& B) const;
	FUAVState ScaleState(const FUAVState& State, float Scale) const;

	// 重力常量 (UE5使用cm/s², 需要转换)
	static constexpr float GravityAcceleration = 980.0f; // cm/s²
};
