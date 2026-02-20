// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UAVTypes.generated.h"

/**
 * 无人机状态数据结构
 */
USTRUCT(BlueprintType)
struct FUAVState
{
	GENERATED_BODY()

	// 位置 (世界坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "UAV State")
	FVector Position;

	// 速度 (世界坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "UAV State")
	FVector Velocity;

	// 姿态 (欧拉角: Roll, Pitch, Yaw)
	UPROPERTY(BlueprintReadWrite, Category = "UAV State")
	FRotator Rotation;

	// 角速度 (机体坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "UAV State")
	FVector AngularVelocity;

	FUAVState()
		: Position(FVector::ZeroVector)
		, Velocity(FVector::ZeroVector)
		, Rotation(FRotator::ZeroRotator)
		, AngularVelocity(FVector::ZeroVector)
	{}
};

/**
 * 电机输出数据
 */
USTRUCT(BlueprintType)
struct FMotorOutput
{
	GENERATED_BODY()

	// 4个电机的推力值 (0-1归一化)
	UPROPERTY(BlueprintReadWrite, Category = "Motor")
	TArray<float> Thrusts;

	FMotorOutput()
	{
		Thrusts.Init(0.0f, 4);
	}
};

/**
 * 轨迹点数据结构
 * 包含位置、速度、加速度和时间戳
 */
USTRUCT(BlueprintType)
struct FTrajectoryPoint
{
	GENERATED_BODY()

	// 位置 (世界坐标系, cm)
	UPROPERTY(BlueprintReadWrite, Category = "Trajectory")
	FVector Position;

	// 速度 (世界坐标系, cm/s)
	UPROPERTY(BlueprintReadWrite, Category = "Trajectory")
	FVector Velocity;

	// 加速度 (世界坐标系, cm/s²)
	UPROPERTY(BlueprintReadWrite, Category = "Trajectory")
	FVector Acceleration;

	// 时间戳 (秒)
	UPROPERTY(BlueprintReadWrite, Category = "Trajectory")
	float TimeStamp;

	// 期望偏航角 (度)
	UPROPERTY(BlueprintReadWrite, Category = "Trajectory")
	float Yaw;

	FTrajectoryPoint()
		: Position(FVector::ZeroVector)
		, Velocity(FVector::ZeroVector)
		, Acceleration(FVector::ZeroVector)
		, TimeStamp(0.0f)
		, Yaw(0.0f)
	{}

	FTrajectoryPoint(const FVector& InPosition, float InTimeStamp)
		: Position(InPosition)
		, Velocity(FVector::ZeroVector)
		, Acceleration(FVector::ZeroVector)
		, TimeStamp(InTimeStamp)
		, Yaw(0.0f)
	{}

	FTrajectoryPoint(const FVector& InPosition, const FVector& InVelocity,
					 const FVector& InAcceleration, float InTimeStamp, float InYaw = 0.0f)
		: Position(InPosition)
		, Velocity(InVelocity)
		, Acceleration(InAcceleration)
		, TimeStamp(InTimeStamp)
		, Yaw(InYaw)
	{}
};

/**
 * 轨迹数据结构
 * 包含轨迹点序列和元数据
 */
USTRUCT(BlueprintType)
struct FTrajectory
{
	GENERATED_BODY()

	// 轨迹点序列
	UPROPERTY(BlueprintReadWrite, Category = "Trajectory")
	TArray<FTrajectoryPoint> Points;

	// 轨迹总时长 (秒)
	UPROPERTY(BlueprintReadWrite, Category = "Trajectory")
	float TotalDuration;

	// 轨迹是否有效
	UPROPERTY(BlueprintReadWrite, Category = "Trajectory")
	bool bIsValid;

	FTrajectory()
		: TotalDuration(0.0f)
		, bIsValid(false)
	{}

	// 获取轨迹点数量
	int32 Num() const { return Points.Num(); }

	// 判断轨迹是否为空
	bool IsEmpty() const { return Points.Num() == 0; }

	// 清空轨迹
	void Clear()
	{
		Points.Empty();
		TotalDuration = 0.0f;
		bIsValid = false;
	}

	// 添加轨迹点
	void AddPoint(const FTrajectoryPoint& Point)
	{
		Points.Add(Point);
		if (Points.Num() > 0)
		{
			TotalDuration = Points.Last().TimeStamp;
		}
	}
};

/**
 * 障碍物类型枚举
 */
UENUM(BlueprintType)
enum class EObstacleType : uint8
{
	Sphere		UMETA(DisplayName = "Sphere"),
	Box			UMETA(DisplayName = "Box"),
	Cylinder	UMETA(DisplayName = "Cylinder"),
	Custom		UMETA(DisplayName = "Custom")
};

/**
 * 障碍物信息结构
 */
USTRUCT(BlueprintType)
struct FObstacleInfo
{
	GENERATED_BODY()

	// 障碍物ID
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	int32 ObstacleID;

	// 障碍物类型
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	EObstacleType Type;

	// 中心位置 (世界坐标系)
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	FVector Center;

	// 尺寸 (取决于类型: Sphere=半径, Box=半尺寸, Cylinder=半径+高度)
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	FVector Extents;

	// 旋转
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	FRotator Rotation;

	// 是否为动态障碍物
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	bool bIsDynamic;

	// 速度 (仅对动态障碍物有效)
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	FVector Velocity;

	// 安全边距
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	float SafetyMargin;

	// 是否为感知检测到的障碍物（区别于预注册障碍物）
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	bool bIsPerceived;

	// 最后一次被感知确认的时间戳
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	float LastPerceivedTime;

	// 关联的Actor
	UPROPERTY(BlueprintReadWrite, Category = "Obstacle")
	TWeakObjectPtr<AActor> LinkedActor;

	FObstacleInfo()
		: ObstacleID(-1)
		, Type(EObstacleType::Sphere)
		, Center(FVector::ZeroVector)
		, Extents(FVector(100.0f))
		, Rotation(FRotator::ZeroRotator)
		, bIsDynamic(false)
		, Velocity(FVector::ZeroVector)
		, SafetyMargin(50.0f)
		, bIsPerceived(false)
		, LastPerceivedTime(0.0f)
	{}

	FObstacleInfo(int32 InID, EObstacleType InType, const FVector& InCenter, const FVector& InExtents)
		: ObstacleID(InID)
		, Type(InType)
		, Center(InCenter)
		, Extents(InExtents)
		, Rotation(FRotator::ZeroRotator)
		, bIsDynamic(false)
		, Velocity(FVector::ZeroVector)
		, SafetyMargin(50.0f)
		, bIsPerceived(false)
		, LastPerceivedTime(0.0f)
	{}
};

/**
 * 路径规划算法枚举
 */
UENUM(BlueprintType)
enum class EPathPlanningAlgorithm : uint8
{
	AStar		UMETA(DisplayName = "A*"),
	RRT			UMETA(DisplayName = "RRT"),
	RRTStar		UMETA(DisplayName = "RRT*")
};

/**
 * UAV控制模式枚举
 */
UENUM(BlueprintType)
enum class EUAVControlMode : uint8
{
	Attitude	UMETA(DisplayName = "Attitude Control"),
	Position	UMETA(DisplayName = "Position Control"),
	Trajectory	UMETA(DisplayName = "Trajectory Tracking")
};

/**
 * 规划配置参数
 */
USTRUCT(BlueprintType)
struct FPlanningConfig
{
	GENERATED_BODY()

	// 路径规划算法
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planning")
	EPathPlanningAlgorithm Algorithm;

	// 是否启用轨迹优化
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planning")
	bool bEnableTrajectoryOptimization;

	// 最大速度 (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planning")
	float MaxVelocity;

	// 最大加速度 (cm/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planning")
	float MaxAcceleration;

	// 安全边距 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planning")
	float SafetyMargin;

	// A*网格分辨率 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "A* Settings")
	float GridResolution;

	// RRT步长 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT Settings")
	float RRTStepSize;

	// RRT最大迭代次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT Settings")
	int32 RRTMaxIterations;

	// RRT目标偏置概率 (0-1)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RRT Settings")
	float RRTGoalBias;

	// 是否启用动态避障
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance")
	bool bEnableDynamicAvoidance;

	// 重规划距离阈值 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance")
	float ReplanningThreshold;

	FPlanningConfig()
		: Algorithm(EPathPlanningAlgorithm::AStar)
		, bEnableTrajectoryOptimization(true)
		, MaxVelocity(800.0f)
		, MaxAcceleration(400.0f)
		, SafetyMargin(50.0f)
		, GridResolution(50.0f)
		, RRTStepSize(100.0f)
		, RRTMaxIterations(5000)
		, RRTGoalBias(0.1f)
		, bEnableDynamicAvoidance(true)
		, ReplanningThreshold(200.0f)
	{}
};
