// Copyright Epic Games, Inc. All Rights Reserved.

#include "FormationComponent.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UFormationComponent::UFormationComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void UFormationComponent::SetTargetOffset(const FVector& NewOffset)
{
	if (!TargetFormationOffset.Equals(NewOffset, 1.0f))
	{
		TargetFormationOffset = NewOffset;
		bIsTransitioning = true;
	}
}

void UFormationComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bIsTransitioning)
	{
		return;
	}

	// 平滑插值：向目标偏移量移动
	FVector Diff = TargetFormationOffset - CurrentFormationOffset;
	float Dist = Diff.Size();

	if (Dist < 1.0f)
	{
		// 已到达目标
		CurrentFormationOffset = TargetFormationOffset;
		bIsTransitioning = false;
		return;
	}

	// 以 TransitionSpeed 的速度向目标移动
	float Step = FMath::Min(TransitionSpeed * DeltaTime, Dist);
	CurrentFormationOffset += Diff.GetSafeNormal() * Step;
}
