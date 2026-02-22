// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "../Control/AttitudeController.h"
#include "UAVProductTypes.generated.h"

/** UAV产品类型 */
UENUM(BlueprintType)
enum class EUAVProductType : uint8
{
	Agricultural	UMETA(DisplayName = "Agricultural UAV"),	// 农业喷洒
	Mapping			UMETA(DisplayName = "Mapping UAV"),			// 测绘航测
};

/** UAV型号ID */
UENUM(BlueprintType)
enum class EUAVModelID : uint8
{
	Agri_AG20		UMETA(DisplayName = "AG-20 (20L)"),
	Agri_AG60		UMETA(DisplayName = "AG-60 (60L)"),
	Agri_AG100		UMETA(DisplayName = "AG-100 (100L)"),
	Map_SVPro		UMETA(DisplayName = "SV-Pro (RGB)"),
	Map_SVLiDAR		UMETA(DisplayName = "SV-LiDAR"),
};

/** 型号参数预设（对应各组件的 UPROPERTY 字段） */
USTRUCT(BlueprintType)
struct FUAVModelSpec
{
	GENERATED_BODY()

	// 产品信息
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model")
	EUAVProductType ProductType = EUAVProductType::Agricultural;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model")
	FString ModelName;

	// 物理参数（→ UUAVDynamics）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Physics")
	float Mass = 1.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Physics")
	float ArmLength = 0.225f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Physics")
	FVector MomentOfInertia = FVector(0.029f, 0.029f, 0.055f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Physics")
	float MaxThrust = 15.0f;	// 单电机，N

	// 姿态控制参数（→ UAttitudeController）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Attitude")
	float HoverThrust = 0.245f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Attitude")
	FPIDParams RollPID = FPIDParams(0.008f, 0.0f, 0.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Attitude")
	FPIDParams PitchPID = FPIDParams(0.008f, 0.0f, 0.0f);

	// 位置控制参数（→ UPositionController）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Position")
	float MaxVelocity = 2000.0f;	// cm/s

	// 载荷元数据
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Payload")
	float MaxPayloadKg = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Payload")
	float SprayWidthM = 0.0f;	// 农业喷幅（m），测绘为 0

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Model|Payload")
	FString PayloadDescription;
};
