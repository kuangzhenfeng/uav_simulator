// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UAVProductTypes.h"

class AUAVPawn;
class UUAVDynamics;
class UAttitudeController;
class UPositionController;

/** 型号注册表：静态查询与参数应用 */
class FUAVProductManager
{
public:
	/** 获取指定型号的参数预设 */
	static FUAVModelSpec GetModelSpec(EUAVModelID ModelID)
	{
		FUAVModelSpec S;
		switch (ModelID)
		{
		// ---- 农业系列 ----
		case EUAVModelID::Agri_AG20:
			S.ProductType = EUAVProductType::Agricultural; S.ModelName = TEXT("AG-20");
			S.Mass = 12.0f; S.MaxThrust = 100.0f; S.ArmLength = 0.45f;
			S.MomentOfInertia = FVector(0.28f, 0.28f, 0.50f);
			S.HoverThrust = 0.294f;
			S.RollPID = FPIDParams(0.015f, 0.0f, 0.008f); S.PitchPID = S.RollPID;
			S.MaxVelocity = 1200.0f;
			S.MaxPayloadKg = 20.0f; S.SprayWidthM = 6.0f;
			S.PayloadDescription = TEXT("20L喷洒系统，喷幅6m");
			break;

		case EUAVModelID::Agri_AG60:
			S.ProductType = EUAVProductType::Agricultural; S.ModelName = TEXT("AG-60");
			S.Mass = 22.0f; S.MaxThrust = 250.0f; S.ArmLength = 0.60f;
			S.MomentOfInertia = FVector(0.80f, 0.80f, 1.40f);
			S.HoverThrust = 0.216f;
			S.RollPID = FPIDParams(0.012f, 0.0f, 0.006f); S.PitchPID = S.RollPID;
			S.MaxVelocity = 800.0f;
			S.MaxPayloadKg = 60.0f; S.SprayWidthM = 10.0f;
			S.PayloadDescription = TEXT("60L喷洒系统，喷幅10m");
			break;

		case EUAVModelID::Agri_AG100:
			S.ProductType = EUAVProductType::Agricultural; S.ModelName = TEXT("AG-100");
			S.Mass = 35.0f; S.MaxThrust = 400.0f; S.ArmLength = 0.75f;
			S.MomentOfInertia = FVector(1.80f, 1.80f, 3.20f);
			S.HoverThrust = 0.214f;
			S.RollPID = FPIDParams(0.010f, 0.0f, 0.004f); S.PitchPID = S.RollPID;
			S.MaxVelocity = 600.0f;
			S.MaxPayloadKg = 100.0f; S.SprayWidthM = 14.0f;
			S.PayloadDescription = TEXT("100L喷洒系统，喷幅14m");
			break;

		// ---- 测绘系列 ----
		case EUAVModelID::Agri_X100Demo:
        {
			S=GetModelSpec(EUAVModelID::Agri_AG60);
			S.ModelName=TEXT("X100-inspired engineering model");
			S.MaxPayloadKg=50; S.SprayWidthM=6;
            // 根据工程模型的惯量及力矩增益配置 8 rad/s、阻尼比 0.9 的姿态闭环。
            const float Gain=4*S.MaxThrust*S.ArmLength*0.707f;
            S.RollPID=FPIDParams(S.MomentOfInertia.X*64/Gain*PI/180,0,2*.9f*S.MomentOfInertia.X*8/Gain*PI/180);
            S.PitchPID=S.RollPID;
			S.PayloadDescription=TEXT("50L农业作业工程模型，未标定厂家动力学");
			break;
        }

		// ---- 测绘系列 ----
		case EUAVModelID::Map_SVPro:
			S.ProductType = EUAVProductType::Mapping; S.ModelName = TEXT("SV-Pro");
			S.Mass = 3.2f;  S.MaxThrust = 18.0f; S.ArmLength = 0.28f;
			S.MomentOfInertia = FVector(0.045f, 0.045f, 0.082f);
			S.HoverThrust = 0.436f;
			S.RollPID = FPIDParams(0.018f, 0.0f, 0.0f); S.PitchPID = S.RollPID;
			S.MaxVelocity = 2000.0f;
			S.MaxPayloadKg = 0.8f; S.SprayWidthM = 0.0f;
			S.PayloadDescription = TEXT("5000万像素RGB相机，GSD 2cm@100m");
			break;

		case EUAVModelID::Map_SVLiDAR:
			S.ProductType = EUAVProductType::Mapping; S.ModelName = TEXT("SV-LiDAR");
			S.Mass = 4.5f;  S.MaxThrust = 20.0f; S.ArmLength = 0.30f;
			S.MomentOfInertia = FVector(0.060f, 0.060f, 0.110f);
			S.HoverThrust = 0.551f;
			S.RollPID = FPIDParams(0.007f, 0.0f, 0.0f); S.PitchPID = S.RollPID;
			S.MaxVelocity = 1800.0f;
			S.MaxPayloadKg = 2.0f; S.SprayWidthM = 0.0f;
			S.PayloadDescription = TEXT("激光雷达 Livox Mid-360，精度±2cm");
			break;

		default:
			break;	// 返回默认值（1.5kg基准机）
		}
		return S;
	}

};
