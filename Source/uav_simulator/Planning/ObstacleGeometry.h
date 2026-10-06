#pragma once

#include "../Core/UAVTypes.h"

// 所有规划、安全滤波和碰撞检查共享的几何查询，距离包含 SafetyMargin。
namespace ObstacleGeometry
{
	UAV_SIMULATOR_API float SignedDistance(const FVector& Point, const FObstacleInfo& Obstacle);
	UAV_SIMULATOR_API FVector Gradient(const FVector& Point, const FObstacleInfo& Obstacle);
}
