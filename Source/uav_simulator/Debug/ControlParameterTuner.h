// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "HAL/IConsoleManager.h"
#include "ControlParameterTuner.generated.h"

class UAttitudeController;
class UPositionController;
class AUAVPawn;
class AUAVHUD;

enum class EAutoTunePhase : uint8 { Idle, PosLoop };

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

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

protected:
	// 控制器引用
	UPROPERTY()
	TObjectPtr<UAttitudeController> AttitudeControllerRef;

	UPROPERTY()
	TObjectPtr<UPositionController> PositionControllerRef;

	// 是否启用实时调试
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bEnableDebugDisplay = true;

	UPROPERTY()
	TObjectPtr<AUAVPawn> UAVPawnRef;

private:
	// 阶跃测试状态
	bool bStepTestActive = false;
	float StepMagnitude = 200.0f;
	float StepTargetZ = 0.0f;
	float StepElapsed = 0.0f;
	float StepPeakOvershoot = 0.0f;
	float SettleTimer = 0.0f;

	TArray<IConsoleObject*> ConsoleCommands;

	// 自动调参状态
	EAutoTunePhase AutoTunePhase = EAutoTunePhase::Idle;
	float LastSettleTime = 0.0f, LastOvershootPct = 0.0f;
	bool bStepResultReady = false;
	float StepBaseZ = 0.0f;
	float AT_DeadTime = 0.0f;   // θ
	float AT_TimeConst = 0.0f;  // τ

	void RegisterConsoleCommands();
	void HandleSetParam(const TArray<FString>& Args);
	void HandleStep(const TArray<FString>& Args);
	void TriggerStepTest(float Magnitude);
	void TickStepTest(float DeltaTime);
	void HandleAutoTune(const TArray<FString>& Args);
	void TickAutoTune();
};
