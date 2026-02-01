// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * UAV Simulator Log Categories
 *
 * Usage:
 *   UE_LOG(LogUAVPosition, Verbose, TEXT("Target: %s"), *Pos.ToString());
 *   UE_LOG(LogUAVPlanning, Warning, TEXT("Path planning failed"));
 *
 * Runtime Control (Console Commands):
 *   Log LogUAVPlanning Verbose    // Show verbose logs
 *   Log LogUAVPlanning Warning    // Show warnings and above only
 *   Log LogUAVPlanning Off        // Disable logs
 *
 * Log Levels (Low to High):
 *   Verbose  - Detailed debug info, removed in shipping builds
 *   Log      - General information
 *   Warning  - Warnings
 *   Error    - Errors
 *   Fatal    - Fatal errors, causes crash
 */

// ============== Log Category Declarations ==============

// Default log
DECLARE_LOG_CATEGORY_EXTERN(LogDefault, Log, All);

// Position controller log
DECLARE_LOG_CATEGORY_EXTERN(LogUAVPosition, Log, All);

// Attitude controller log
DECLARE_LOG_CATEGORY_EXTERN(LogUAVAttitude, Log, All);

// Physics dynamics log
DECLARE_LOG_CATEGORY_EXTERN(LogUAVDynamics, Log, All);

// UAVActor log
DECLARE_LOG_CATEGORY_EXTERN(LogUAVActor, Log, All);

// Sensor log
DECLARE_LOG_CATEGORY_EXTERN(LogUAVSensor, Log, All);

// Path planning and trajectory log
DECLARE_LOG_CATEGORY_EXTERN(LogUAVPlanning, Log, All);

// AI behavior tree log
DECLARE_LOG_CATEGORY_EXTERN(LogUAVAI, Log, All);

// Mission management log
DECLARE_LOG_CATEGORY_EXTERN(LogUAVMission, Log, All);
