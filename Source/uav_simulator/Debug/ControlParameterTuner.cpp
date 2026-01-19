// Copyright Epic Games, Inc. All Rights Reserved.

#include "ControlParameterTuner.h"
#include "../Control/AttitudeController.h"
#include "../Control/PositionController.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"

UControlParameterTuner::UControlParameterTuner()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UControlParameterTuner::BeginPlay()
{
	Super::BeginPlay();
}

void UControlParameterTuner::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
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
