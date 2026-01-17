// Copyright Epic Games, Inc. All Rights Reserved.

#include "SensorBase.h"

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
}

void USensorBase::UpdateSensor(const FUAVState& TrueState, float DeltaTime)
{
	// 基类实现为空，由子类重写
}
