#pragma once

#include "CoreMinimal.h"
#include "HAL/PlatformTime.h"

/**
 * 通用计数过滤器
 * 用法：每 N 次调用允许执行一次
 * 应用场景：日志节流、传感器降采样、周期性任务等
 */
class FCountFilter
{
public:
	explicit FCountFilter(uint32 InCount) : Counter(0), Count(InCount) {}

	// 返回 true 表示允许执行
	bool ShouldExecute()
	{
		return (++Counter % Count == 0);
	}

	void Reset() { Counter = 0; }

private:
	uint32 Counter;
	uint32 Count;
};

/**
 * 通用时间过滤器
 * 用法：每 N 秒允许执行一次
 * 应用场景：日志节流、定时任务、状态更新等
 */
class FTimeFilter
{
public:
	explicit FTimeFilter(double InIntervalSeconds)
		: LastExecuteTime(0.0)
		, IntervalSeconds(InIntervalSeconds)
	{}

	// 返回 true 表示允许执行
	bool ShouldExecute()
	{
		const double Now = FPlatformTime::Seconds();
		if (Now - LastExecuteTime >= IntervalSeconds)
		{
			LastExecuteTime = Now;
			return true;
		}
		return false;
	}

	void Reset() { LastExecuteTime = FPlatformTime::Seconds(); }

private:
	double LastExecuteTime;
	double IntervalSeconds;
};

/**
 * 便捷宏：时间过滤执行
 * 用法：EXECUTE_THROTTLE(1.0, { your code here; })
 */
#define EXECUTE_THROTTLE(Seconds, Code) \
	do \
	{ \
		static FTimeFilter Filter##__LINE__(Seconds); \
		if (Filter##__LINE__.ShouldExecute()) \
		{ \
			Code \
		} \
	} while(0)

/**
 * 便捷宏：计数过滤执行
 * 用法：EXECUTE_EVERY_N(100, { your code here; })
 */
#define EXECUTE_EVERY_N(Count, Code) \
	do \
	{ \
		static FCountFilter Filter##__LINE__(Count); \
		if (Filter##__LINE__.ShouldExecute()) \
		{ \
			Code \
		} \
	} while(0)

/**
 * 便捷宏：时间过滤日志（日志专用简化版）
 * 用法：UE_LOG_THROTTLE(1.0, LogCategory, Verbosity, TEXT("Message"))
 */
#define UE_LOG_THROTTLE(Seconds, CategoryName, Verbosity, Format, ...) \
	EXECUTE_THROTTLE(Seconds, UE_LOG(CategoryName, Verbosity, Format, ##__VA_ARGS__);)

/**
 * 便捷宏：计数过滤日志（日志专用简化版）
 * 用法：UE_LOG_EVERY_N(100, LogCategory, Verbosity, TEXT("Message"))
 */
#define UE_LOG_EVERY_N(Count, CategoryName, Verbosity, Format, ...) \
	EXECUTE_EVERY_N(Count, UE_LOG(CategoryName, Verbosity, Format, ##__VA_ARGS__);)
