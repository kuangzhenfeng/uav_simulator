// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "EnvironmentTypes.h"
#include "WindField.generated.h"

/**
 * 风场模拟组件
 * 提供恒定风、阵风和 Dryden 湍流模型
 * 应挂载到 GameMode 或 Level Actor 上，供所有 UAV 共享
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UWindField : public UActorComponent
{
	GENERATED_BODY()

public:
	UWindField();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 获取当前风速 (cm/s，世界坐标系)
	UFUNCTION(BlueprintCallable, Category = "Wind")
	FVector GetCurrentWindVelocity() const { return CurrentState.WindVelocity; }

	// 获取完整风场状态
	UFUNCTION(BlueprintCallable, Category = "Wind")
	FWindState GetWindState() const { return CurrentState; }

	// 获取风场配置
	UFUNCTION(BlueprintCallable, Category = "Wind")
	FWindConfig GetWindConfig() const { return Config; }

	// 设置风场配置
	UFUNCTION(BlueprintCallable, Category = "Wind")
	void SetWindConfig(const FWindConfig& InConfig);

	// 获取指定位置的风速（支持空间变化，当前实现为均匀风场）
	UFUNCTION(BlueprintCallable, Category = "Wind")
	FVector GetWindAtPosition(const FVector& WorldPosition) const;

	// 计算风阻力 (cm/s²，返回加速度)
	UFUNCTION(BlueprintCallable, Category = "Wind")
	FVector ComputeWindDragAcceleration(
		const FVector& UAVVelocity,
		const FVector& UAVPosition,
		float UAVMass) const;

protected:
	virtual void BeginPlay() override;

private:
	// 更新阵风模型
	void UpdateGustModel(float DeltaTime);

	// 更新 Dryden 湍流模型
	void UpdateTurbulenceModel(float DeltaTime);

	// Dryden 湍流频谱滤波（白噪声 → Dryden 相关噪声）
	FVector DrydenFilter(const FVector& WhiteNoise, float DeltaTime);

	// 风场配置
	UPROPERTY(EditAnywhere, Category = "Wind")
	FWindConfig Config;

	// 当前风场状态
	FWindState CurrentState;

	// ---- 阵风模型状态 ----
	bool bGustActive = false;
	float GustTimer = 0.0f;
	FVector CurrentGustVelocity = FVector::ZeroVector;
	FVector TargetGustVelocity = FVector::ZeroVector;
	float NextGustTime = 0.0f;

	// ---- Dryden 湍流模型状态 ----
	// 三轴独立的一阶低通滤波器状态
	FVector DrydenStateX = FVector::ZeroVector;
	FVector DrydenStateY = FVector::ZeroVector;
	FVector DrydenStateZ = FVector::ZeroVector;

	// Dryden 滤波器时间常数
	float DrydenTauX = 0.0f;
	float DrydenTauY = 0.0f;
	float DrydenTauZ = 0.0f;

	// 随机数生成器（确定性，可选固定种子）
	bool bUseFixedSeed = false;
	int32 RandomSeed = 42;

	// 生成高斯白噪声
	FVector GenerateWhiteNoise();
	float GaussianRandom();
};
