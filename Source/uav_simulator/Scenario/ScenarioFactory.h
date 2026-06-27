// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "ScenarioTypes.h"
#include "ScenarioDto.h"
#include "ScenarioFactory.generated.h"

/**
 * 场景工厂（ScenarioFactory）。
 *
 * 把 Web 控制面板的统一数据契约 FScenarioDto 转成内存 UScenario（含全部子资产）。
 * 纯运行时工具：不落盘、不读资产，全部用 NewObject 构造强引用子资产，
 * 直接赋值给 UScenario 的 TSoftObjectPtr 字段（隐式从硬指针构造）。
 *
 * 返回的 UScenario 可直接喂给 UScenarioLoader::Assemble* 系列方法装配进世界。
 */
UCLASS()
class UAV_SIMULATOR_API UScenarioFactory : public UObject
{
	GENERATED_BODY()

public:
	/**
	 * 把 DTO 转成内存 UScenario（含全部子资产，outer 指定）。
	 * 不落盘。返回的 UScenario 可直接喂给 UScenarioLoader::Assemble*。
	 *
	 * @param Dto   Web 控制面板的统一数据契约
	 * @param Outer 子资产的外部所有者（可为 GetTransientPackage()）
	 * @return 装配好的 UScenario；Dto 异常时仍返回一个最小可用的 UScenario
	 */
	UFUNCTION(BlueprintCallable, Category = "Scenario")
	static UScenario* BuildFromDto(const FScenarioDto& Dto, UObject* Outer);

private:
	// ---- 字符串 -> 枚举 helper（大小写敏感，不匹配返回合理默认值）----

	static EObstacleType ParseObstacleType(const FString& InName);
	static EObstacleMovementType ParseObstacleMovementType(const FString& InName);
	static EWindFieldType ParseWindFieldType(const FString& InName);
	static EUAVModelID ParseUAVModelID(const FString& InName);
	static EMissionMode ParseMissionMode(const FString& InName);
	static EMPCType ParseMPCType(const FString& InName);
	static EUAVControlMode ParseControlMode(const FString& InName);
};
