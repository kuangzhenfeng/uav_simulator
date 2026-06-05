// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "../Core/UAVTypes.h"
#include "../MultiAgent/MultiAgentTypes.h"
#include "../MultiAgent/TaskAllocationTypes.h"

/**
 * UAV Simulator 测试通用工具
 * 提供测试分类常量、浮点数/向量近似相等检查宏和辅助函数
 */

// ==================== 测试分类常量 ====================

namespace UAVTestCategories
{
	// 主分类
	static const TCHAR* Root = TEXT("UAVSimulator");

	// 模块分类
	static const TCHAR* Planning = TEXT("UAVSimulator.Planning");
	static const TCHAR* Control = TEXT("UAVSimulator.Control");
	static const TCHAR* Physics = TEXT("UAVSimulator.Physics");
	static const TCHAR* Mission = TEXT("UAVSimulator.Mission");
	static const TCHAR* Core = TEXT("UAVSimulator.Core");

	// Planning 子分类
	static const TCHAR* AStar = TEXT("UAVSimulator.Planning.AStar");
	static const TCHAR* RRT = TEXT("UAVSimulator.Planning.RRT");
	static const TCHAR* TrajectoryOptimizer = TEXT("UAVSimulator.Planning.TrajectoryOptimizer");
	static const TCHAR* TrajectoryTracker = TEXT("UAVSimulator.Planning.TrajectoryTracker");
	static const TCHAR* ObstacleManager = TEXT("UAVSimulator.Planning.ObstacleManager");
	static const TCHAR* NMPCAvoidance = TEXT("UAVSimulator.Planning.NMPCAvoidance");

	// Control 子分类
	static const TCHAR* AttitudeController = TEXT("UAVSimulator.Control.AttitudeController");
	static const TCHAR* PositionController = TEXT("UAVSimulator.Control.PositionController");

	// Physics 子分类
	static const TCHAR* UAVDynamics = TEXT("UAVSimulator.Physics.UAVDynamics");

	// Mission 子分类
	static const TCHAR* MissionComponent = TEXT("UAVSimulator.Mission.MissionComponent");

	// Core 子分类
	static const TCHAR* UAVTypes = TEXT("UAVSimulator.Core.UAVTypes");

	// MultiAgent 子分类
	static const TCHAR* MultiAgent = TEXT("UAVSimulator.MultiAgent");
	static const TCHAR* CBFQPFilter = TEXT("UAVSimulator.MultiAgent.CBFQPFilter");
	static const TCHAR* JointNMPCSolver = TEXT("UAVSimulator.MultiAgent.JointNMPCSolver");
	static const TCHAR* MILPSolverCat = TEXT("UAVSimulator.MultiAgent.MILPSolver");
	static const TCHAR* TaskAllocatorCat = TEXT("UAVSimulator.MultiAgent.TaskAllocator");
	static const TCHAR* TaskMonitorCat = TEXT("UAVSimulator.MultiAgent.TaskMonitor");

	// Environment 子分类
	static const TCHAR* Environment = TEXT("UAVSimulator.Environment");
	static const TCHAR* WindField = TEXT("UAVSimulator.Environment.WindField");

	// Sensors 新增子分类
	static const TCHAR* Barometer = TEXT("UAVSimulator.Sensors.Barometer");
	static const TCHAR* Magnetometer = TEXT("UAVSimulator.Sensors.Magnetometer");
	static const TCHAR* Anemometer = TEXT("UAVSimulator.Sensors.Anemometer");
}

// ==================== 测试标志组合 ====================

// 命令行测试标志（支持 -ExecCmds 运行）
// 使用 ClientContext 支持命令行和编辑器运行
#define UAV_TEST_FLAGS (EAutomationTestFlags::ClientContext | EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

// 高优先级测试标志
#define UAV_TEST_FLAGS_HIGH_PRIORITY (EAutomationTestFlags::ClientContext | EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter | EAutomationTestFlags::HighPriority)

// ==================== 浮点数近似相等检查 ====================

// 默认浮点数容差
#define UAV_FLOAT_TOLERANCE 1e-4f

// 检查两个浮点数是否近似相等
#define UAV_TEST_FLOAT_EQUAL(Actual, Expected, Tolerance) \
	TestTrue(FString::Printf(TEXT("Expected %f to equal %f (tolerance: %f)"), Actual, Expected, Tolerance), \
		FMath::IsNearlyEqual(Actual, Expected, Tolerance))

// 使用默认容差检查浮点数相等
#define UAV_TEST_FLOAT_EQUAL_DEFAULT(Actual, Expected) \
	UAV_TEST_FLOAT_EQUAL(Actual, Expected, UAV_FLOAT_TOLERANCE)

// 检查浮点数是否在指定范围内
#define UAV_TEST_FLOAT_IN_RANGE(Value, Min, Max) \
	TestTrue(FString::Printf(TEXT("Expected %f to be in range [%f, %f]"), Value, Min, Max), \
		Value >= Min && Value <= Max)

// ==================== 向量近似相等检查 ====================

// 默认向量容差
#define UAV_VECTOR_TOLERANCE 1.0f

// 检查两个向量是否近似相等
#define UAV_TEST_VECTOR_EQUAL(Actual, Expected, Tolerance) \
	TestTrue(FString::Printf(TEXT("Expected (%f, %f, %f) to equal (%f, %f, %f) (tolerance: %f)"), \
		Actual.X, Actual.Y, Actual.Z, Expected.X, Expected.Y, Expected.Z, Tolerance), \
		Actual.Equals(Expected, Tolerance))

// 使用默认容差检查向量相等
#define UAV_TEST_VECTOR_EQUAL_DEFAULT(Actual, Expected) \
	UAV_TEST_VECTOR_EQUAL(Actual, Expected, UAV_VECTOR_TOLERANCE)

// 检查向量长度是否近似相等
#define UAV_TEST_VECTOR_SIZE_EQUAL(Actual, ExpectedSize, Tolerance) \
	TestTrue(FString::Printf(TEXT("Expected vector size %f to equal %f (tolerance: %f)"), \
		Actual.Size(), ExpectedSize, Tolerance), \
		FMath::IsNearlyEqual(Actual.Size(), ExpectedSize, Tolerance))

// ==================== 旋转近似相等检查 ====================

// 默认旋转容差（度）
#define UAV_ROTATOR_TOLERANCE 0.1f

// 检查两个旋转是否近似相等
#define UAV_TEST_ROTATOR_EQUAL(Actual, Expected, Tolerance) \
	TestTrue(FString::Printf(TEXT("Expected (P:%f, Y:%f, R:%f) to equal (P:%f, Y:%f, R:%f) (tolerance: %f)"), \
		Actual.Pitch, Actual.Yaw, Actual.Roll, Expected.Pitch, Expected.Yaw, Expected.Roll, Tolerance), \
		Actual.Equals(Expected, Tolerance))

// ==================== 辅助函数 ====================

namespace UAVTestHelpers
{
	/**
	 * 创建测试用球体障碍物
	 * @param ID 障碍物ID
	 * @param Center 中心位置
	 * @param Radius 半径
	 * @param SafetyMargin 安全边距
	 * @return 障碍物信息
	 */
	inline FObstacleInfo CreateSphereObstacle(int32 ID, const FVector& Center, float Radius, float SafetyMargin = 50.0f)
	{
		FObstacleInfo Obstacle;
		Obstacle.ObstacleID = ID;
		Obstacle.Type = EObstacleType::Sphere;
		Obstacle.Center = Center;
		Obstacle.Extents = FVector(Radius);
		Obstacle.SafetyMargin = SafetyMargin;
		Obstacle.bIsDynamic = false;
		return Obstacle;
	}

	/**
	 * 创建测试用盒体障碍物
	 * @param ID 障碍物ID
	 * @param Center 中心位置
	 * @param HalfExtents 半尺寸
	 * @param Rotation 旋转
	 * @param SafetyMargin 安全边距
	 * @return 障碍物信息
	 */
	inline FObstacleInfo CreateBoxObstacle(int32 ID, const FVector& Center, const FVector& HalfExtents,
		const FRotator& Rotation = FRotator::ZeroRotator, float SafetyMargin = 50.0f)
	{
		FObstacleInfo Obstacle;
		Obstacle.ObstacleID = ID;
		Obstacle.Type = EObstacleType::Box;
		Obstacle.Center = Center;
		Obstacle.Extents = HalfExtents;
		Obstacle.Rotation = Rotation;
		Obstacle.SafetyMargin = SafetyMargin;
		Obstacle.bIsDynamic = false;
		return Obstacle;
	}

	/**
	 * 创建测试用圆柱体障碍物
	 * @param ID 障碍物ID
	 * @param Center 中心位置
	 * @param Radius 半径
	 * @param HalfHeight 半高度
	 * @param SafetyMargin 安全边距
	 * @return 障碍物信息
	 */
	inline FObstacleInfo CreateCylinderObstacle(int32 ID, const FVector& Center, float Radius, float HalfHeight,
		float SafetyMargin = 50.0f)
	{
		FObstacleInfo Obstacle;
		Obstacle.ObstacleID = ID;
		Obstacle.Type = EObstacleType::Cylinder;
		Obstacle.Center = Center;
		Obstacle.Extents = FVector(Radius, Radius, HalfHeight);
		Obstacle.SafetyMargin = SafetyMargin;
		Obstacle.bIsDynamic = false;
		return Obstacle;
	}

	/**
	 * 创建测试用轨迹点
	 * @param Position 位置
	 * @param Velocity 速度
	 * @param Acceleration 加速度
	 * @param TimeStamp 时间戳
	 * @param Yaw 偏航角
	 * @return 轨迹点
	 */
	inline FTrajectoryPoint CreateTrajectoryPoint(const FVector& Position, const FVector& Velocity = FVector::ZeroVector,
		const FVector& Acceleration = FVector::ZeroVector, float TimeStamp = 0.0f, float Yaw = 0.0f)
	{
		return FTrajectoryPoint(Position, Velocity, Acceleration, TimeStamp, Yaw);
	}

	/**
	 * 创建简单的测试轨迹（直线）
	 * @param Start 起点
	 * @param End 终点
	 * @param Duration 持续时间
	 * @param NumPoints 采样点数
	 * @return 轨迹
	 */
	inline FTrajectory CreateLinearTrajectory(const FVector& Start, const FVector& End, float Duration, int32 NumPoints = 10)
	{
		FTrajectory Trajectory;
		Trajectory.bIsValid = true;
		Trajectory.TotalDuration = Duration;

		const FVector Direction = (End - Start).GetSafeNormal();
		const float Distance = FVector::Dist(Start, End);
		const FVector Velocity = Direction * (Distance / Duration);

		for (int32 i = 0; i < NumPoints; ++i)
		{
			float Alpha = static_cast<float>(i) / static_cast<float>(NumPoints - 1);
			float Time = Alpha * Duration;
			FVector Position = FMath::Lerp(Start, End, Alpha);

			FTrajectoryPoint Point;
			Point.Position = Position;
			Point.Velocity = Velocity;
			Point.Acceleration = FVector::ZeroVector;
			Point.TimeStamp = Time;
			Point.Yaw = 0.0f;

			Trajectory.Points.Add(Point);
		}

		return Trajectory;
	}

	/**
	 * 创建圆形测试轨迹
	 * @param Center 圆心
	 * @param Radius 半径
	 * @param Duration 持续时间
	 * @param NumPoints 采样点数
	 * @return 轨迹
	 */
	inline FTrajectory CreateCircularTrajectory(const FVector& Center, float Radius, float Duration, int32 NumPoints = 36)
	{
		FTrajectory Trajectory;
		Trajectory.bIsValid = true;
		Trajectory.TotalDuration = Duration;

		const float AngularVelocity = 2.0f * PI / Duration;

		for (int32 i = 0; i < NumPoints; ++i)
		{
			float Alpha = static_cast<float>(i) / static_cast<float>(NumPoints - 1);
			float Time = Alpha * Duration;
			float Angle = Alpha * 2.0f * PI;

			FVector Position = Center + FVector(
				Radius * FMath::Cos(Angle),
				Radius * FMath::Sin(Angle),
				0.0f
			);

			FVector Velocity = FVector(
				-Radius * AngularVelocity * FMath::Sin(Angle),
				Radius * AngularVelocity * FMath::Cos(Angle),
				0.0f
			);

			FVector Acceleration = FVector(
				-Radius * AngularVelocity * AngularVelocity * FMath::Cos(Angle),
				-Radius * AngularVelocity * AngularVelocity * FMath::Sin(Angle),
				0.0f
			);

			FTrajectoryPoint Point;
			Point.Position = Position;
			Point.Velocity = Velocity;
			Point.Acceleration = Acceleration;
			Point.TimeStamp = Time;
			Point.Yaw = FMath::RadiansToDegrees(Angle + PI / 2.0f);

			Trajectory.Points.Add(Point);
		}

		return Trajectory;
	}

	/**
	 * 创建测试用 UAV 状态
	 * @param Position 位置
	 * @param Velocity 速度
	 * @param Rotation 旋转
	 * @param AngularVelocity 角速度
	 * @return UAV 状态
	 */
	inline FUAVState CreateUAVState(const FVector& Position = FVector::ZeroVector,
		const FVector& Velocity = FVector::ZeroVector,
		const FRotator& Rotation = FRotator::ZeroRotator,
		const FVector& AngularVelocity = FVector::ZeroVector)
	{
		FUAVState State;
		State.Position = Position;
		State.Velocity = Velocity;
		State.Rotation = Rotation;
		State.AngularVelocity = AngularVelocity;
		return State;
	}

	/**
	 * 创建测试用航点数组
	 * @param Positions 位置数组
	 * @return 航点位置数组
	 */
	inline TArray<FVector> CreateWaypoints(std::initializer_list<FVector> Positions)
	{
		TArray<FVector> Waypoints;
		for (const FVector& Pos : Positions)
		{
			Waypoints.Add(Pos);
		}
		return Waypoints;
		}

		// ==================== MultiAgent 辅助函数 ====================

		/**
		 * 创建测试用 Agent 状态快照
		 */
		inline FAgentStateSnapshot CreateAgentSnapshot(
			int32 AgentID,
			const FVector& Position = FVector::ZeroVector,
			const FVector& Velocity = FVector::ZeroVector,
			const FRotator& Rotation = FRotator::ZeroRotator,
			const FVector& TargetPosition = FVector::ZeroVector,
			const FVector& NMPCAcceleration = FVector::ZeroVector)
		{
			FAgentStateSnapshot Snapshot;
			Snapshot.AgentID = AgentID;
			Snapshot.State.Position = Position;
			Snapshot.State.Velocity = Velocity;
			Snapshot.State.Rotation = Rotation;
			Snapshot.TargetPosition = TargetPosition;
			Snapshot.NMPCAcceleration = NMPCAcceleration;
			Snapshot.Timestamp = 0.0;
			return Snapshot;
		}

		/**
		 * 创建测试用任务描述
		 */
		inline FTaskDescriptor CreateTaskDescriptor(
			int32 TaskID,
			const FVector& TargetLocation = FVector::ZeroVector,
			ETaskPriority Priority = ETaskPriority::Normal,
			float EstimatedDuration = 10.0f,
			float Reward = 1.0f,
			float RequiredPayload = 0.0f,
			int32 RequiredCapabilities = 0)
		{
			FTaskDescriptor Task;
			Task.TaskID = TaskID;
			Task.TargetLocation = TargetLocation;
			Task.Priority = Priority;
			Task.EstimatedDuration = EstimatedDuration;
			Task.Reward = Reward;
			Task.RequiredPayload = RequiredPayload;
			Task.RequiredCapabilities = RequiredCapabilities;
			return Task;
		}

		/**
		 * 创建测试用 UAV 能力描述
		 */
		inline FUAVCapability CreateUAVCapability(
			int32 AgentID,
			const FVector& Position = FVector::ZeroVector,
			float MaxSpeed = 2000.0f,
			float RemainingFlightTime = 600.0f,
			float MaxPayloadKg = 10.0f,
			int32 CapabilityMask = 0,
			float CurrentPayloadMass = 0.0f)
		{
			FUAVCapability Cap;
			Cap.AgentID = AgentID;
			Cap.CurrentPosition = Position;
			Cap.MaxSpeed = MaxSpeed;
			Cap.RemainingFlightTime = RemainingFlightTime;
			Cap.MaxPayloadKg = MaxPayloadKg;
			Cap.CapabilityMask = CapabilityMask;
			Cap.CurrentPayloadMass = CurrentPayloadMass;
			return Cap;
		}

		/**
		 * 创建测试用任务分配条目
		 */
		inline FTaskAssignment CreateTaskAssignment(
			int32 TaskID,
			int32 AgentID,
			float StartTime = 0.0f,
			float CompletionTime = 0.0f,
			float TravelDistance = 0.0f)
		{
			FTaskAssignment Assignment;
			Assignment.TaskID = TaskID;
			Assignment.AgentID = AgentID;
			Assignment.EstimatedStartTime = StartTime;
			Assignment.EstimatedCompletionTime = CompletionTime;
			Assignment.EstimatedTravelDistance = TravelDistance;
			return Assignment;
		}

		/**
		 * 创建测试用分配结果
		 */
		inline FTaskAllocationResult CreateAllocationResult(
			const TArray<FTaskAssignment>& Assignments)
		{
			FTaskAllocationResult Result;
			Result.Assignments = Assignments;
			Result.bIsFeasible = true;
			Result.TotalCost = 0.0f;
			Result.Makespan = 0.0f;
			return Result;
		}
	}
