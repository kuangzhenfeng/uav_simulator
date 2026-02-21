// Copyright Epic Games, Inc. All Rights Reserved.

#include "ControlParameterTuner.h"
#include "../Control/AttitudeController.h"
#include "../Control/PositionController.h"
#include "../Core/UAVPawn.h"
#include "UAVHUD.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"
#include "GameFramework/PlayerController.h"

UControlParameterTuner::UControlParameterTuner()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UControlParameterTuner::BeginPlay()
{
	Super::BeginPlay();
	UAVPawnRef = Cast<AUAVPawn>(GetOwner());
	RegisterConsoleCommands();
}

void UControlParameterTuner::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	for (IConsoleObject* Cmd : ConsoleCommands)
		IConsoleManager::Get().UnregisterConsoleObject(Cmd);
	ConsoleCommands.Empty();
	Super::EndPlay(EndPlayReason);
}

void UControlParameterTuner::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (bStepTestActive)
		TickStepTest(DeltaTime);
	if (AutoTunePhase != EAutoTunePhase::Idle)
		TickAutoTune();
}

void UControlParameterTuner::SetAttitudeController(UAttitudeController* Controller)
{
	AttitudeControllerRef = Controller;
}

void UControlParameterTuner::SetPositionController(UPositionController* Controller)
{
	PositionControllerRef = Controller;
}

bool UControlParameterTuner::SaveParameters(const FString& FileName)
{
	FString SavePath = FPaths::ProjectSavedDir() + TEXT("Config/") + FileName + TEXT(".ini");

	TArray<FString> Lines;
	Lines.Add(TEXT("[AttitudeController]"));

	if (AttitudeControllerRef)
	{
		Lines.Add(FString::Printf(TEXT("Kp_Roll=%.3f"), AttitudeControllerRef->RollPID.Kp));
		Lines.Add(FString::Printf(TEXT("Ki_Roll=%.3f"), AttitudeControllerRef->RollPID.Ki));
		Lines.Add(FString::Printf(TEXT("Kd_Roll=%.3f"), AttitudeControllerRef->RollPID.Kd));
		Lines.Add(FString::Printf(TEXT("Kp_Pitch=%.3f"), AttitudeControllerRef->PitchPID.Kp));
		Lines.Add(FString::Printf(TEXT("Ki_Pitch=%.3f"), AttitudeControllerRef->PitchPID.Ki));
		Lines.Add(FString::Printf(TEXT("Kd_Pitch=%.3f"), AttitudeControllerRef->PitchPID.Kd));
		Lines.Add(FString::Printf(TEXT("Kp_Yaw=%.3f"), AttitudeControllerRef->YawPID.Kp));
		Lines.Add(FString::Printf(TEXT("Ki_Yaw=%.3f"), AttitudeControllerRef->YawPID.Ki));
		Lines.Add(FString::Printf(TEXT("Kd_Yaw=%.3f"), AttitudeControllerRef->YawPID.Kd));
	}

	Lines.Add(TEXT(""));
	Lines.Add(TEXT("[PositionController]"));

	if (PositionControllerRef)
	{
		Lines.Add(FString::Printf(TEXT("Kp_Position=%.3f"), PositionControllerRef->Kp_Position));
		Lines.Add(FString::Printf(TEXT("Ki_Position=%.3f"), PositionControllerRef->Ki_Position));
		Lines.Add(FString::Printf(TEXT("Kd_Position=%.3f"), PositionControllerRef->Kd_Position));
		Lines.Add(FString::Printf(TEXT("Kp_Velocity=%.3f"), PositionControllerRef->Kp_Velocity));
		Lines.Add(FString::Printf(TEXT("Ki_Velocity=%.3f"), PositionControllerRef->Ki_Velocity));
		Lines.Add(FString::Printf(TEXT("Kd_Velocity=%.3f"), PositionControllerRef->Kd_Velocity));
	}

	FString Content = FString::Join(Lines, TEXT("\n"));
	return FFileHelper::SaveStringToFile(Content, *SavePath);
}

bool UControlParameterTuner::LoadParameters(const FString& FileName)
{
	FString LoadPath = FPaths::ProjectSavedDir() + TEXT("Config/") + FileName + TEXT(".ini");

	FString Content;
	if (!FFileHelper::LoadFileToString(Content, *LoadPath))
	{
		return false;
	}

	TArray<FString> Lines;
	Content.ParseIntoArrayLines(Lines);

	FString CurrentSection;
	for (const FString& Line : Lines)
	{
		FString TrimmedLine = Line.TrimStartAndEnd();

		if (TrimmedLine.StartsWith(TEXT("[")))
		{
			CurrentSection = TrimmedLine;
			continue;
		}

		FString Key, Value;
		if (TrimmedLine.Split(TEXT("="), &Key, &Value))
		{
			float FloatValue = FCString::Atof(*Value);

			if (CurrentSection == TEXT("[AttitudeController]") && AttitudeControllerRef)
			{
				if (Key == TEXT("Kp_Roll")) AttitudeControllerRef->RollPID.Kp = FloatValue;
				else if (Key == TEXT("Ki_Roll")) AttitudeControllerRef->RollPID.Ki = FloatValue;
				else if (Key == TEXT("Kd_Roll")) AttitudeControllerRef->RollPID.Kd = FloatValue;
				else if (Key == TEXT("Kp_Pitch")) AttitudeControllerRef->PitchPID.Kp = FloatValue;
				else if (Key == TEXT("Ki_Pitch")) AttitudeControllerRef->PitchPID.Ki = FloatValue;
				else if (Key == TEXT("Kd_Pitch")) AttitudeControllerRef->PitchPID.Kd = FloatValue;
				else if (Key == TEXT("Kp_Yaw")) AttitudeControllerRef->YawPID.Kp = FloatValue;
				else if (Key == TEXT("Ki_Yaw")) AttitudeControllerRef->YawPID.Ki = FloatValue;
				else if (Key == TEXT("Kd_Yaw")) AttitudeControllerRef->YawPID.Kd = FloatValue;
			}
			else if (CurrentSection == TEXT("[PositionController]") && PositionControllerRef)
			{
				if (Key == TEXT("Kp_Position")) PositionControllerRef->Kp_Position = FloatValue;
				else if (Key == TEXT("Ki_Position")) PositionControllerRef->Ki_Position = FloatValue;
				else if (Key == TEXT("Kd_Position")) PositionControllerRef->Kd_Position = FloatValue;
				else if (Key == TEXT("Kp_Velocity")) PositionControllerRef->Kp_Velocity = FloatValue;
				else if (Key == TEXT("Ki_Velocity")) PositionControllerRef->Ki_Velocity = FloatValue;
				else if (Key == TEXT("Kd_Velocity")) PositionControllerRef->Kd_Velocity = FloatValue;
			}
		}
	}

	return true;
}

void UControlParameterTuner::ResetToDefaults()
{
	if (AttitudeControllerRef)
	{
		AttitudeControllerRef->RollPID = FPIDParams(0.05f, 0.005f, 0.05f);
		AttitudeControllerRef->PitchPID = FPIDParams(0.05f, 0.005f, 0.05f);
		AttitudeControllerRef->YawPID = FPIDParams(0.04f, 0.002f, 0.04f);
	}

	if (PositionControllerRef)
	{
		PositionControllerRef->Kp_Position = 1.0f;
		PositionControllerRef->Ki_Position = 0.0f;
		PositionControllerRef->Kd_Position = 0.5f;
		PositionControllerRef->Kp_Velocity = 2.0f;
		PositionControllerRef->Ki_Velocity = 0.1f;
		PositionControllerRef->Kd_Velocity = 0.1f;
	}
}

void UControlParameterTuner::RegisterConsoleCommands()
{
	ConsoleCommands.Add(IConsoleManager::Get().RegisterConsoleCommand(
		TEXT("pid.set"),
		TEXT("Set PID param. Usage: pid.set <param> <value>  (e.g. pid.set pos.kp 1.5)"),
		FConsoleCommandWithArgsDelegate::CreateUObject(this, &UControlParameterTuner::HandleSetParam)
	));
	ConsoleCommands.Add(IConsoleManager::Get().RegisterConsoleCommand(
		TEXT("pid.step"),
		TEXT("Trigger Z-axis step test. Usage: pid.step [magnitude=200]"),
		FConsoleCommandWithArgsDelegate::CreateUObject(this, &UControlParameterTuner::HandleStep)
	));
	ConsoleCommands.Add(IConsoleManager::Get().RegisterConsoleCommand(
		TEXT("pid.save"),
		TEXT("Save PID parameters to file"),
		FConsoleCommandDelegate::CreateLambda([this]() { SaveParameters(TEXT("PIDParams")); })
	));
	ConsoleCommands.Add(IConsoleManager::Get().RegisterConsoleCommand(
		TEXT("pid.reset"),
		TEXT("Reset PID parameters to defaults"),
		FConsoleCommandDelegate::CreateLambda([this]() { ResetToDefaults(); })
	));
	ConsoleCommands.Add(IConsoleManager::Get().RegisterConsoleCommand(
		TEXT("pid.autotune"),
		TEXT("Auto-tune position and velocity PID parameters"),
		FConsoleCommandWithArgsDelegate::CreateUObject(this, &UControlParameterTuner::HandleAutoTune)
	));
}

void UControlParameterTuner::HandleSetParam(const TArray<FString>& Args)
{
	if (Args.Num() < 2) return;
	const FString& Param = Args[0];
	float Value = FCString::Atof(*Args[1]);

	if (PositionControllerRef)
	{
		if      (Param == TEXT("pos.kp")) PositionControllerRef->Kp_Position = Value;
		else if (Param == TEXT("pos.ki")) PositionControllerRef->Ki_Position = Value;
		else if (Param == TEXT("pos.kd")) PositionControllerRef->Kd_Position = Value;
		else if (Param == TEXT("vel.kp")) PositionControllerRef->Kp_Velocity = Value;
		else if (Param == TEXT("vel.ki")) PositionControllerRef->Ki_Velocity = Value;
		else if (Param == TEXT("vel.kd")) PositionControllerRef->Kd_Velocity = Value;
	}
	if (AttitudeControllerRef)
	{
		if      (Param == TEXT("roll.kp"))  AttitudeControllerRef->RollPID.Kp = Value;
		else if (Param == TEXT("roll.ki"))  AttitudeControllerRef->RollPID.Ki = Value;
		else if (Param == TEXT("roll.kd"))  AttitudeControllerRef->RollPID.Kd = Value;
		else if (Param == TEXT("pitch.kp")) AttitudeControllerRef->PitchPID.Kp = Value;
		else if (Param == TEXT("pitch.ki")) AttitudeControllerRef->PitchPID.Ki = Value;
		else if (Param == TEXT("pitch.kd")) AttitudeControllerRef->PitchPID.Kd = Value;
		else if (Param == TEXT("yaw.kp"))   AttitudeControllerRef->YawPID.Kp = Value;
		else if (Param == TEXT("yaw.ki"))   AttitudeControllerRef->YawPID.Ki = Value;
		else if (Param == TEXT("yaw.kd"))   AttitudeControllerRef->YawPID.Kd = Value;
	}
	UE_LOG(LogTemp, Log, TEXT("[PIDTuner] Set %s = %.4f"), *Param, Value);
}

void UControlParameterTuner::HandleStep(const TArray<FString>& Args)
{
	StepMagnitude = (Args.Num() > 0) ? FCString::Atof(*Args[0]) : 200.0f;
	TriggerStepTest(StepMagnitude);
}

void UControlParameterTuner::TriggerStepTest(float Magnitude)
{
	if (!UAVPawnRef) return;
	StepMagnitude = Magnitude;
	FVector CurPos = UAVPawnRef->GetUAVState().Position;
	StepTargetZ   = CurPos.Z + StepMagnitude;
	StepBaseZ     = CurPos.Z;
	StepElapsed   = 0.0f;
	StepPeakOvershoot = 0.0f;
	SettleTimer   = 0.0f;
	AT_DeadTime   = 0.0f;
	AT_TimeConst  = 0.0f;
	bStepResultReady = false;
	bStepTestActive = true;
	UAVPawnRef->SetTargetPosition(FVector(CurPos.X, CurPos.Y, StepTargetZ));
	UE_LOG(LogTemp, Log, TEXT("[PIDTuner] Step test started: baseZ=%.1f targetZ=%.1f mag=%.1f"),
		CurPos.Z, StepTargetZ, StepMagnitude);
}

void UControlParameterTuner::TickStepTest(float DeltaTime)
{
	if (!UAVPawnRef) return;
	StepElapsed += DeltaTime;

	float CurrentZ = UAVPawnRef->GetUAVState().Position.Z;
	float Error    = FMath::Abs(CurrentZ - StepTargetZ);
	float Threshold = FMath::Abs(StepMagnitude) * 0.05f;

	float Overshoot = (StepMagnitude >= 0.0f) ? (CurrentZ - StepTargetZ) : (StepTargetZ - CurrentZ);
	if (Overshoot > StepPeakOvershoot)
		StepPeakOvershoot = Overshoot;

	// FOPDT 参数采样
	float Progress = (StepMagnitude != 0.0f) ? ((CurrentZ - StepBaseZ) / StepMagnitude) : 0.0f;
	if (AT_DeadTime == 0.0f && Progress > 0.02f)
		AT_DeadTime = StepElapsed;
	if (AT_TimeConst == 0.0f && AT_DeadTime > 0.0f && Progress >= 0.632f)
		AT_TimeConst = StepElapsed - AT_DeadTime;

	if (Error < Threshold)
	{
		SettleTimer += DeltaTime;
		if (SettleTimer >= 0.5f)
		{
			float OvershootPct = (FMath::Abs(StepMagnitude) > 0.0f) ? (StepPeakOvershoot / FMath::Abs(StepMagnitude) * 100.0f) : 0.0f;
			float SteadyErr = Error;
			UE_LOG(LogTemp, Log, TEXT("[PIDTuner] Step result: SettleTime=%.2fs Overshoot=%.1f%% SteadyErr=%.1fcm"),
				StepElapsed, OvershootPct, SteadyErr);

			LastSettleTime = StepElapsed;
			LastOvershootPct = OvershootPct;
			bStepResultReady = true;

			if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
			{
				if (AUAVHUD* HUD = PC->GetHUD<AUAVHUD>())
					HUD->SetStepTestResult(StepElapsed, OvershootPct, SteadyErr);
			}
			bStepTestActive = false;
		}
	}
	else
	{
		SettleTimer = 0.0f;
	}

	if (StepElapsed > 30.0f)
	{
		UE_LOG(LogTemp, Warning, TEXT("[PIDTuner] Step test timed out"));
		LastSettleTime = 30.0f;
		LastOvershootPct = 0.0f;
		bStepResultReady = true;
		bStepTestActive = false;
	}
}

void UControlParameterTuner::HandleAutoTune(const TArray<FString>& Args)
{
	if (!UAVPawnRef || !PositionControllerRef) return;
	AutoTunePhase = EAutoTunePhase::PosLoop;
	UE_LOG(LogTemp, Log, TEXT("[AutoTune] Starting IMC tuning..."));
	TriggerStepTest(200.0f);
}

void UControlParameterTuner::TickAutoTune()
{
	if (bStepTestActive || !bStepResultReady) return;
	bStepResultReady = false;

	float theta  = FMath::Max(AT_DeadTime, 0.016f);
	float tau    = FMath::Max(AT_TimeConst, 0.1f);
	float lambda = FMath::Max(theta, tau / 3.0f);
	float PosKp  = tau / (lambda + theta);
	float PosKd  = PosKp * theta / 2.0f;

	PositionControllerRef->Kp_Position = PosKp;
	PositionControllerRef->Kd_Position = PosKd;
	PositionControllerRef->Kp_Velocity = PosKp * 2.0f;
	PositionControllerRef->Kd_Velocity = PosKd * 0.5f;

	UE_LOG(LogTemp, Log, TEXT("[AutoTune] theta=%.3fs tau=%.3fs lambda=%.3fs -> pos.kp=%.3f pos.kd=%.3f vel.kp=%.3f vel.kd=%.3f"),
		theta, tau, lambda, PosKp, PosKd, PosKp * 2.0f, PosKd * 0.5f);
	SaveParameters(TEXT("PIDParams_AutoTune"));
	AutoTunePhase = EAutoTunePhase::Idle;
}
