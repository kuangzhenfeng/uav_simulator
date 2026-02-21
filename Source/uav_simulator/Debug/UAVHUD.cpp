// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVHUD.h"
#include "Engine/Canvas.h"
#include "../Control/AttitudeController.h"
#include "../Control/PositionController.h"

AUAVHUD::AUAVHUD()
{
	MotorThrusts.Init(0.0f, 4);
}

void AUAVHUD::DrawHUD()
{
	Super::DrawHUD();

	if (!bShowUAVInfo)
		return;

	DrawStateInfo();
	DrawMotorInfo();
	DrawControllerParams();
	DrawStepTestResult();
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

void AUAVHUD::SetControllerParams(UAttitudeController* AttitudeCtrl, UPositionController* PositionCtrl)
{
	AttitudeController = AttitudeCtrl;
	PositionController = PositionCtrl;
}

void AUAVHUD::SetStepTestResult(float SettleTime, float OvershootPct, float SteadyErr)
{
	bHasStepResult = true;
	StepSettleTime = SettleTime;
	StepOvershootPct = OvershootPct;
	StepSteadyError = SteadyErr;
}

void AUAVHUD::DrawStepTestResult()
{
	if (!Canvas || !bHasStepResult) return;

	float XPos = Canvas->SizeX - 280.0f;
	float YPos = 50.0f;
	float LineHeight = 20.0f;

	DrawText(TEXT("Step Test Result"), FColor::White, XPos, YPos, nullptr, 1.2f);
	YPos += LineHeight * 1.5f;

	DrawText(FString::Printf(TEXT("Settle Time: %.2f s"), StepSettleTime), FColor::Green, XPos, YPos);
	YPos += LineHeight;

	FColor OvershootColor = (StepOvershootPct > 20.0f) ? FColor::Red : FColor::Yellow;
	DrawText(FString::Printf(TEXT("Overshoot:   %.1f %%"), StepOvershootPct), OvershootColor, XPos, YPos);
	YPos += LineHeight;

	DrawText(FString::Printf(TEXT("Steady Err:  %.1f cm"), StepSteadyError), FColor::Cyan, XPos, YPos);
}

void AUAVHUD::DrawControllerParams()
{
	if (!Canvas)
		return;

	float YPos = 350.0f;
	float XPos = 50.0f;
	float LineHeight = 20.0f;

	// 姿态控制器参数
	if (AttitudeController)
	{
		FString Title = TEXT("Attitude PID");
		DrawText(Title, FColor::White, XPos, YPos, nullptr, 1.2f);
		YPos += LineHeight * 1.5f;

		FString RollText = FString::Printf(TEXT("Roll: Kp=%.3f Ki=%.3f Kd=%.3f"),
			AttitudeController->RollPID.Kp, AttitudeController->RollPID.Ki, AttitudeController->RollPID.Kd);
		DrawText(RollText, FColor::Cyan, XPos, YPos);
		YPos += LineHeight;

		FString PitchText = FString::Printf(TEXT("Pitch: Kp=%.3f Ki=%.3f Kd=%.3f"),
			AttitudeController->PitchPID.Kp, AttitudeController->PitchPID.Ki, AttitudeController->PitchPID.Kd);
		DrawText(PitchText, FColor::Cyan, XPos, YPos);
		YPos += LineHeight;

		FString YawText = FString::Printf(TEXT("Yaw: Kp=%.3f Ki=%.3f Kd=%.3f"),
			AttitudeController->YawPID.Kp, AttitudeController->YawPID.Ki, AttitudeController->YawPID.Kd);
		DrawText(YawText, FColor::Cyan, XPos, YPos);
		YPos += LineHeight * 1.5f;
	}

	// 位置控制器参数
	if (PositionController)
	{
		FString Title = TEXT("Position PID");
		DrawText(Title, FColor::White, XPos, YPos, nullptr, 1.2f);
		YPos += LineHeight * 1.5f;

		FString PosText = FString::Printf(TEXT("Pos: Kp=%.2f Ki=%.2f Kd=%.2f"),
			PositionController->Kp_Position, PositionController->Ki_Position, PositionController->Kd_Position);
		DrawText(PosText, FColor::Yellow, XPos, YPos);
		YPos += LineHeight;

		FString VelText = FString::Printf(TEXT("Vel: Kp=%.2f Ki=%.2f Kd=%.2f"),
			PositionController->Kp_Velocity, PositionController->Ki_Velocity, PositionController->Kd_Velocity);
		DrawText(VelText, FColor::Yellow, XPos, YPos);
	}
}
