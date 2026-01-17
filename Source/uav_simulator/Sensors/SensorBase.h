// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "SensorBase.generated.h"

/**
 * 传感器基类
 * 所有传感器都继承自此类
 */
UCLASS(Abstract, ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API USensorBase : public UActorComponent
{
	GENERATED_BODY()

public:
	USensorBase();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 更新传感器数据（由子类实现）
	UFUNCTION(BlueprintCallable, Category = "Sensor")
	virtual void UpdateSensor(const FUAVState& TrueState, float DeltaTime);

	// 获取传感器是否启用
	UFUNCTION(BlueprintCallable, Category = "Sensor")
	bool IsEnabled() const { return bEnabled; }

	// 设置传感器启用状态
	UFUNCTION(BlueprintCallable, Category = "Sensor")
	void SetEnabled(bool bInEnabled) { bEnabled = bInEnabled; }

protected:
	// 传感器是否启用
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor")
	bool bEnabled = true;

	// 传感器更新频率 (Hz)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor")
	float UpdateRate = 100.0f;

	// 累计时间（用于控制更新频率）
	float AccumulatedTime = 0.0f;
};
