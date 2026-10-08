#pragma once

#include "CoreMinimal.h"
#include "../Core/UAVTypes.h"

/** 统一生成剩余条带的巡航、加减速与地头转弯，运行时只跟踪一条轨迹。 */
struct UAV_SIMULATOR_API FCoveragePlanner
{
    static float HeadlandDistance(float Speed, float Acceleration);
    static bool Build(const TArray<FVector>& StripPoints, int32 FirstEndPoint, float CoveredCm,
        const FTrajectory& Approach, float Speed, float Acceleration,
        FTrajectory& OutTrajectory, TMap<int32,float>& OutStripEndTimes);
};
