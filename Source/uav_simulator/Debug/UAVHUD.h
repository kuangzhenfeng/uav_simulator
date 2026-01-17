// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "../Core/UAVTypes.h"
#include "UAVHUD.generated.h"

/**
 * UAV HUD类
 * 显示无人机状态信息
 */
UCLASS()
class UAV_SIMULATOR_API AUAVHUD : public AHUD
{
	GENERATED_BODY()

public:
	AUAVHUD();

	virtual void DrawHUD() override;

	// 设置要显示的UAV状态
	UFUNCTION(BlueprintCallable, Category = "UAV HUD")
	void SetUAVState(const FUAVState& InState);

	// 设置电机推力
	UFUNCTION(BlueprintCallable, Category = "UAV HUD")
	void SetMotorThrusts(const TArray<float>& InThrusts);

protected:
	// 当前UAV状态
	FUAVState CurrentState;

	// 电机推力
	TArray<float> MotorThrusts;

	// 是否显示UAV信息
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HUD Settings")
	bool bShowUAVInfo = true;

private:
	// 绘制状态信息
	void DrawStateInfo();

	// 绘制电机推力
	void DrawMotorInfo();
};
