// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "UAVTestGameMode.generated.h"

/**
 * UAV测试GameMode
 * 用于测试无人机功能
 */
UCLASS()
class UAV_SIMULATOR_API AUAVTestGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AUAVTestGameMode();
};
