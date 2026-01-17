// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UAVTypes.generated.h"

/**
 * 无人机状态数据结构
 */
USTRUCT(BlueprintType)
struct FUAVState
{
	GENERATED_BODY()

	// 位置 (世界坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "UAV State")
	FVector Position;

	// 速度 (世界坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "UAV State")
	FVector Velocity;

	// 姿态 (欧拉角: Roll, Pitch, Yaw)
	UPROPERTY(BlueprintReadWrite, Category = "UAV State")
	FRotator Rotation;

	// 角速度 (机体坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "UAV State")
	FVector AngularVelocity;

	FUAVState()
		: Position(FVector::ZeroVector)
		, Velocity(FVector::ZeroVector)
		, Rotation(FRotator::ZeroRotator)
		, AngularVelocity(FVector::ZeroVector)
	{}
};

/**
 * 电机输出数据
 */
USTRUCT(BlueprintType)
struct FMotorOutput
{
	GENERATED_BODY()

	// 4个电机的推力值 (0-1归一化)
	UPROPERTY(BlueprintReadWrite, Category = "Motor")
	TArray<float> Thrusts;

	FMotorOutput()
	{
		Thrusts.Init(0.0f, 4);
	}
};
