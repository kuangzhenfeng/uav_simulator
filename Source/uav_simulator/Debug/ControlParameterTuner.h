// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ControlParameterTuner.generated.h"

class UAttitudeController;
class UPositionController;

/**
 * 控制参数调试组件
 * 用于运行时调整和保存控制器参数
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UControlParameterTuner : public UActorComponent
{
	GENERATED_BODY()

public:
	UControlParameterTuner();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 设置姿态控制器引用
	UFUNCTION(BlueprintCallable, Category = "Parameter Tuner")
	void SetAttitudeController(UAttitudeController* Controller);

	// 设置位置控制器引用
	UFUNCTION(BlueprintCallable, Category = "Parameter Tuner")
	void SetPositionController(UPositionController* Controller);

	// 保存参数到文件
	UFUNCTION(BlueprintCallable, Category = "Parameter Tuner")
	bool SaveParameters(const FString& FileName);

	// 从文件加载参数
	UFUNCTION(BlueprintCallable, Category = "Parameter Tuner")
	bool LoadParameters(const FString& FileName);

	// 重置为默认参数
	UFUNCTION(BlueprintCallable, Category = "Parameter Tuner")
	void ResetToDefaults();

protected:
	// 控制器引用
	UPROPERTY()
	TObjectPtr<UAttitudeController> AttitudeControllerRef;

	UPROPERTY()
	TObjectPtr<UPositionController> PositionControllerRef;

	// 是否启用实时调试
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bEnableDebugDisplay = true;
};
