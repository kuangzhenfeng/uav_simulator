// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "DataLogger.generated.h"

/**
 * 飞行数据记录器组件
 * 记录飞行状态、控制输出等数据，并导出为CSV格式
 */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UDataLogger : public UActorComponent
{
	GENERATED_BODY()

public:
	UDataLogger();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 开始记录
	UFUNCTION(BlueprintCallable, Category = "Data Logger")
	void StartRecording();

	// 停止记录
	UFUNCTION(BlueprintCallable, Category = "Data Logger")
	void StopRecording();

	// 记录一帧数据
	UFUNCTION(BlueprintCallable, Category = "Data Logger")
	void LogFrame(const FUAVState& State, const TArray<float>& MotorThrusts);

	// 导出数据到CSV
	UFUNCTION(BlueprintCallable, Category = "Data Logger")
	bool ExportToCSV(const FString& FileName);

	// 清除记录的数据
	UFUNCTION(BlueprintCallable, Category = "Data Logger")
	void ClearData();

	// 是否正在记录
	UFUNCTION(BlueprintCallable, Category = "Data Logger")
	bool IsRecording() const { return bIsRecording; }

protected:
	// 是否正在记录
	UPROPERTY(BlueprintReadOnly, Category = "Logger State")
	bool bIsRecording = false;

	// 记录频率（Hz）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Logger Settings")
	float RecordingFrequency = 100.0f;

	// 自动导出
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Logger Settings")
	bool bAutoExportOnStop = true;

private:
	// 数据记录结构
	struct FLogEntry
	{
		float Time;
		FVector Position;
		FVector Velocity;
		FRotator Rotation;
		FVector AngularVelocity;
		TArray<float> MotorThrusts;
	};

	// 记录的数据
	TArray<FLogEntry> LoggedData;

	// 累计时间
	float AccumulatedTime = 0.0f;

	// 记录开始时间
	float RecordingStartTime = 0.0f;
};
