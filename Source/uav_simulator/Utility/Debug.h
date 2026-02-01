// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Debug.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogUAVDebug, Log, All);

/**
 * 调试工具类
 */
UCLASS()
class UAV_SIMULATOR_API UDebug : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * 输出当前调用堆栈到日志
	 * @param Message 附加消息
	 */
	UFUNCTION(BlueprintCallable, Category = "Debug")
	static void PrintCallStack(const FString& Message = TEXT(""));

	/**
	 * 获取当前调用堆栈字符串
	 * @return 堆栈字符串
	 */
	UFUNCTION(BlueprintCallable, Category = "Debug")
	static FString GetCallStackString();
};

/** 堆栈输出宏 */
#define UAV_PRINT_STACK(Message) UDebug::PrintCallStack(FString::Printf(TEXT("%s [%s:%d]"), TEXT(Message), TEXT(__FUNCTION__), __LINE__))
