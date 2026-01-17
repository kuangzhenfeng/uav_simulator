// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVHUD.h"
#include "Engine/Canvas.h"

AUAVHUD::AUAVHUD()
{
	MotorThrusts.Init(0.0f, 4);
}

void AUAVHUD::DrawHUD()
{
	Super::DrawHUD();

	if (!bShowHUD)
		return;

	DrawStateInfo();
	DrawMotorInfo();
}

void AUAVHUD::SetUAVState(const FUAVState& InState)
{
	CurrentState = InState;
}

void AUAVHUD::SetMotorThrusts(const TArray<float>& InThrusts)
{
	if (InThrusts.Num() == 4)
	{
		MotorThrusts = InThrusts;
	}
}

void AUAVHUD::DrawStateInfo()
{
	if (!Canvas)
		return;

	float YPos = 50.0f;
	float XPos = 50.0f;
	float LineHeight = 20.0f;

	// 标题
	FString Title = TEXT("UAV State");
	DrawText(Title, FColor::White, XPos, YPos, nullptr, 1.2f);
	YPos += LineHeight * 1.5f;

	// 位置
	FString PosText = FString::Printf(TEXT("Position: X=%.1f Y=%.1f Z=%.1f"),
		CurrentState.Position.X, CurrentState.Position.Y, CurrentState.Position.Z);
	DrawText(PosText, FColor::Green, XPos, YPos);
	YPos += LineHeight;

	// 速度
	FString VelText = FString::Printf(TEXT("Velocity: X=%.1f Y=%.1f Z=%.1f"),
		CurrentState.Velocity.X, CurrentState.Velocity.Y, CurrentState.Velocity.Z);
	DrawText(VelText, FColor::Yellow, XPos, YPos);
	YPos += LineHeight;

	// 姿态
	FString RotText = FString::Printf(TEXT("Rotation: R=%.1f P=%.1f Y=%.1f"),
		CurrentState.Rotation.Roll, CurrentState.Rotation.Pitch, CurrentState.Rotation.Yaw);
	DrawText(RotText, FColor::Cyan, XPos, YPos);
	YPos += LineHeight;

	// 角速度
	FString AngVelText = FString::Printf(TEXT("AngVel: X=%.2f Y=%.2f Z=%.2f"),
		CurrentState.AngularVelocity.X, CurrentState.AngularVelocity.Y, CurrentState.AngularVelocity.Z);
	DrawText(AngVelText, FColor::Magenta, XPos, YPos);
}

void AUAVHUD::DrawMotorInfo()
{
	if (!Canvas || MotorThrusts.Num() != 4)
		return;

	float YPos = 200.0f;
	float XPos = 50.0f;
	float LineHeight = 20.0f;

	// 标题
	FString Title = TEXT("Motor Thrusts");
	DrawText(Title, FColor::White, XPos, YPos, nullptr, 1.2f);
	YPos += LineHeight * 1.5f;

	// 电机推力
	for (int32 i = 0; i < 4; i++)
	{
		FString MotorText = FString::Printf(TEXT("Motor %d: %.2f"), i, MotorThrusts[i]);
		DrawText(MotorText, FColor::Orange, XPos, YPos);
		YPos += LineHeight;
	}
}
