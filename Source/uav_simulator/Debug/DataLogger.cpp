// Copyright Epic Games, Inc. All Rights Reserved.

#include "DataLogger.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"

UDataLogger::UDataLogger()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UDataLogger::BeginPlay()
{
	Super::BeginPlay();
}

void UDataLogger::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UDataLogger::StartRecording()
{
	if (!bIsRecording)
	{
		bIsRecording = true;
		RecordingStartTime = GetWorld()->GetTimeSeconds();
		ClearData();
	}
}

void UDataLogger::StopRecording()
{
	if (bIsRecording)
	{
		bIsRecording = false;

		if (bAutoExportOnStop)
		{
			FString FileName = FString::Printf(TEXT("FlightLog_%s"), *FDateTime::Now().ToString());
			ExportToCSV(FileName);
		}
	}
}

void UDataLogger::LogFrame(const FUAVState& State, const TArray<float>& MotorThrusts)
{
	if (!bIsRecording)
		return;

	AccumulatedTime += GetWorld()->GetDeltaSeconds();
	float RecordInterval = 1.0f / RecordingFrequency;

	if (AccumulatedTime >= RecordInterval)
	{
		FLogEntry Entry;
		Entry.Time = GetWorld()->GetTimeSeconds() - RecordingStartTime;
		Entry.Position = State.Position;
		Entry.Velocity = State.Velocity;
		Entry.Rotation = State.Rotation;
		Entry.AngularVelocity = State.AngularVelocity;
		Entry.MotorThrusts = MotorThrusts;

		LoggedData.Add(Entry);
		AccumulatedTime = 0.0f;
	}
}

bool UDataLogger::ExportToCSV(const FString& FileName)
{
	if (LoggedData.Num() == 0)
		return false;

	FString SavePath = FPaths::ProjectSavedDir() + TEXT("Logs/") + FileName + TEXT(".csv");

	TArray<FString> Lines;

	// CSV 头部
	Lines.Add(TEXT("Time,PosX,PosY,PosZ,VelX,VelY,VelZ,Roll,Pitch,Yaw,AngVelX,AngVelY,AngVelZ,Motor0,Motor1,Motor2,Motor3"));

	// 数据行
	for (const FLogEntry& Entry : LoggedData)
	{
		FString Line = FString::Printf(TEXT("%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f"),
			Entry.Time,
			Entry.Position.X, Entry.Position.Y, Entry.Position.Z,
			Entry.Velocity.X, Entry.Velocity.Y, Entry.Velocity.Z,
			Entry.Rotation.Roll, Entry.Rotation.Pitch, Entry.Rotation.Yaw,
			Entry.AngularVelocity.X, Entry.AngularVelocity.Y, Entry.AngularVelocity.Z,
			Entry.MotorThrusts.Num() > 0 ? Entry.MotorThrusts[0] : 0.0f,
			Entry.MotorThrusts.Num() > 1 ? Entry.MotorThrusts[1] : 0.0f,
			Entry.MotorThrusts.Num() > 2 ? Entry.MotorThrusts[2] : 0.0f,
			Entry.MotorThrusts.Num() > 3 ? Entry.MotorThrusts[3] : 0.0f
		);
		Lines.Add(Line);
	}

	FString Content = FString::Join(Lines, TEXT("\n"));
	return FFileHelper::SaveStringToFile(Content, *SavePath);
}

void UDataLogger::ClearData()
{
	LoggedData.Empty();
	AccumulatedTime = 0.0f;
}
