// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug.h"
#include "HAL/PlatformStackWalk.h"

DEFINE_LOG_CATEGORY(LogUAVDebug);

void UDebug::PrintCallStack(const FString& Message)
{
	if (!Message.IsEmpty())
	{
		UE_LOG(LogUAVDebug, Log, TEXT("=== CALL STACK: %s ==="), *Message);
	}
	else
	{
		UE_LOG(LogUAVDebug, Log, TEXT("=== CALL STACK ==="));
	}

	const int32 MaxFrames = 64;
	uint64 StackFrames[MaxFrames];
	int32 NumFrames = FPlatformStackWalk::CaptureStackBackTrace(StackFrames, MaxFrames);

	for (int32 i = 1; i < NumFrames; ++i) // 跳过当前函数
	{
		ANSICHAR Buffer[512];
		Buffer[0] = '\0';
		FPlatformStackWalk::ProgramCounterToHumanReadableString(i - 1, StackFrames[i], Buffer, sizeof(Buffer));
		UE_LOG(LogUAVDebug, Log, TEXT("  [%d] %s"), i - 1, ANSI_TO_TCHAR(Buffer));
	}

	UE_LOG(LogUAVDebug, Log, TEXT("=== END STACK ==="));
}

FString UDebug::GetCallStackString()
{
	FString Result;

	const int32 MaxFrames = 64;
	uint64 StackFrames[MaxFrames];
	int32 NumFrames = FPlatformStackWalk::CaptureStackBackTrace(StackFrames, MaxFrames);

	for (int32 i = 1; i < NumFrames; ++i)
	{
		ANSICHAR Buffer[512];
		Buffer[0] = '\0';
		FPlatformStackWalk::ProgramCounterToHumanReadableString(i - 1, StackFrames[i], Buffer, sizeof(Buffer));
		Result += FString::Printf(TEXT("[%d] %s\n"), i - 1, ANSI_TO_TCHAR(Buffer));
	}

	return Result;
}
