// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * UAV模拟器日志类别
 * 
 * 使用方式：
 *   UE_LOG(LogUAVPosition, Verbose, TEXT("Target: %s"), *Pos.ToString());
 *   UE_LOG(LogUAVAttitude, Warning, TEXT("Error: %.2f"), Error);
 * 
 * 运行时控制（控制台命令）：
 *   Log LogUAVPosition Verbose    // 显示详细日志
 *   Log LogUAVPosition Warning    // 只显示警告及以上
 *   Log LogUAVPosition Off        // 关闭日志
 * 
 * 日志级别（从低到高）：
 *   Verbose  - 详细调试信息，发布版自动移除
 *   Log      - 一般信息
 *   Warning  - 警告
 *   Error    - 错误
 *   Fatal    - 致命错误，会导致崩溃
 */

// ============== 日志类别声明 ==============

// 默认日志
DECLARE_LOG_CATEGORY_EXTERN(LogDefault, Log, All);

// 位置控制器日志
DECLARE_LOG_CATEGORY_EXTERN(LogUAVPosition, Log, All);

// 姿态控制器日志
DECLARE_LOG_CATEGORY_EXTERN(LogUAVAttitude, Log, All);

// 物理动力学日志
DECLARE_LOG_CATEGORY_EXTERN(LogUAVDynamics, Log, All);

// UAVActor日志
DECLARE_LOG_CATEGORY_EXTERN(LogUAVActor, Log, All);

// 传感器日志
DECLARE_LOG_CATEGORY_EXTERN(LogUAVSensor, Log, All);
