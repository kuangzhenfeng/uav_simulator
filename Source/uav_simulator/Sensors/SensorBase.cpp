// Copyright Epic Games, Inc. All Rights Reserved.

#include "SensorBase.h"
#include "../uav_simulator.h"

USensorBase::USensorBase()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void USensorBase::BeginPlay()
{
	Super::BeginPlay();
}

void USensorBase::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	SCOPE_CYCLE_COUNTER(STAT_SensorBase);
}

void USensorBase::UpdateSensor(const FUAVState& TrueState, float DeltaTime)
{
	// 基类实现为空，由子类重写
}
