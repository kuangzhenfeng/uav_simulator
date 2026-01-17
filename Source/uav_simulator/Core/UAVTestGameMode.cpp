// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVTestGameMode.h"
#include "../Debug/UAVHUD.h"

AUAVTestGameMode::AUAVTestGameMode()
{
	// 设置默认HUD类
	HUDClass = AUAVHUD::StaticClass();
}
