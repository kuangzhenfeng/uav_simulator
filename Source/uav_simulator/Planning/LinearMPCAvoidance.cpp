// Copyright Epic Games, Inc. All Rights Reserved.

#include "LinearMPCAvoidance.h"
#include "../Debug/UAVLogConfig.h"

ULinearMPCAvoidance::ULinearMPCAvoidance()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void ULinearMPCAvoidance::BeginPlay()
{
	Super::BeginPlay();
}

FNMPCAvoidanceResult ULinearMPCAvoidance::ComputeAvoidance(
	const FVector& CurrentPosition,
	const FVector& CurrentVelocity,
	const TArray<FVector>& ReferencePoints,
	const TArray<FObstacleInfo>& Obstacles)
{
	FNMPCAvoidanceResult Result;

	const int32 N = Config.PredictionSteps;
	if (ReferencePoints.Num() < N + 1)
	{
		return Result;
	}

	// 初始状态
	FVector X0 = CurrentPosition;
	FVector V0 = CurrentVelocity;

	// 参考轨迹
	TArray<FVector> Xref;
	for (int32 i = 0; i <= N; ++i)
	{
		Xref.Add(ReferencePoints[i]);
	}

	// 求解QP
	TArray<FVector> U;
	const double StartTime = FPlatformTime::Seconds();
	bool bSolved = SolveQP(X0, Xref, Obstacles, U);
	const double SolveTime = (FPlatformTime::Seconds() - StartTime) * 1000.0;

	if (!bSolved || U.Num() == 0)
	{
		return Result;
	}

	// 返回结果
	Result.OptimalAcceleration = U[0];

	return Result;
}

float ULinearMPCAvoidance::CalculateDistanceToObstacle(const FVector& Position, const FObstacleInfo& Obstacle) const
{
	// 复用NMPC的障碍物距离计算
	// 简化：使用球体近似
	FVector Delta = Position - Obstacle.Center;
	float Dist = Delta.Size();
	float Radius = Obstacle.Extents.GetMax();
	return FMath::Max(0.0f, Dist - Radius);
}

void ULinearMPCAvoidance::ComputeLinearization(
	const FVector& Position,
	const FVector& Velocity,
	float Dt,
	TArray<float>& OutA,
	TArray<float>& OutB)
{
	// 状态空间模型：x = [px, py, pz, vx, vy, vz]^T
	// 控制输入：u = [ax, ay, az]^T
	// 离散化（欧拉法）：x[k+1] = A*x[k] + B*u[k]
	//
	// A = [I3  dt*I3]  (6x6)
	//     [0   I3   ]
	//
	// B = [0.5*dt²*I3]  (6x3)
	//     [dt*I3      ]

	OutA.SetNum(36); // 6x6
	OutB.SetNum(18); // 6x3

	// 初始化为零
	for (int32 i = 0; i < 36; ++i) OutA[i] = 0.0f;
	for (int32 i = 0; i < 18; ++i) OutB[i] = 0.0f;

	// A矩阵
	// 上半部分：I3 + dt*I3（位置更新）
	OutA[0] = 1.0f;  OutA[1] = 0.0f;  OutA[2] = 0.0f;  OutA[3] = Dt;    OutA[4] = 0.0f;  OutA[5] = 0.0f;
	OutA[6] = 0.0f;  OutA[7] = 1.0f;  OutA[8] = 0.0f;  OutA[9] = 0.0f;  OutA[10] = Dt;   OutA[11] = 0.0f;
	OutA[12] = 0.0f; OutA[13] = 0.0f; OutA[14] = 1.0f; OutA[15] = 0.0f; OutA[16] = 0.0f; OutA[17] = Dt;
	// 下半部分：I3（速度保持）
	OutA[18] = 0.0f; OutA[19] = 0.0f; OutA[20] = 0.0f; OutA[21] = 1.0f; OutA[22] = 0.0f; OutA[23] = 0.0f;
	OutA[24] = 0.0f; OutA[25] = 0.0f; OutA[26] = 0.0f; OutA[27] = 0.0f; OutA[28] = 1.0f; OutA[29] = 0.0f;
	OutA[30] = 0.0f; OutA[31] = 0.0f; OutA[32] = 0.0f; OutA[33] = 0.0f; OutA[34] = 0.0f; OutA[35] = 1.0f;

	// B矩阵
	const float Dt2 = 0.5f * Dt * Dt;
	// 上半部分：0.5*dt²*I3（位置受加速度影响）
	OutB[0] = Dt2;   OutB[1] = 0.0f;  OutB[2] = 0.0f;
	OutB[3] = 0.0f;  OutB[4] = Dt2;   OutB[5] = 0.0f;
	OutB[6] = 0.0f;  OutB[7] = 0.0f;  OutB[8] = Dt2;
	// 下半部分：dt*I3（速度受加速度影响）
	OutB[9] = Dt;    OutB[10] = 0.0f; OutB[11] = 0.0f;
	OutB[12] = 0.0f; OutB[13] = Dt;   OutB[14] = 0.0f;
	OutB[15] = 0.0f; OutB[16] = 0.0f; OutB[17] = Dt;
}

bool ULinearMPCAvoidance::SolveQP(
	const FVector& X0,
	const TArray<FVector>& Xref,
	const TArray<FObstacleInfo>& Obstacles,
	TArray<FVector>& OutU)
{
	const int32 N = Config.PredictionSteps;
	const float Dt = Config.GetDt();

	// 计算线性化
	ComputeLinearization(X0, FVector::ZeroVector, Dt, A_matrix, B_matrix);

	// 初始化控制序列（热启动：指向参考点）
	OutU.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		FVector TargetAccel = (Xref[i + 1] - X0) / (Dt * (i + 1));
		OutU[i] = TargetAccel.GetClampedToMaxSize(Config.MaxAcceleration);
	}

	// 投影梯度下降
	const int32 MaxIter = Config.MaxIterations;
	const float StepSize = 0.1f; // 步长

	for (int32 Iter = 0; Iter < MaxIter; ++Iter)
	{
		// 前向仿真
		TArray<FVector> X;
		ForwardSimulateLinear(X0, OutU, X);

		// 计算梯度
		TArray<FVector> GradU;
		ComputeAnalyticGradient(X, OutU, Xref, Obstacles, GradU);

		// 梯度下降更新
		for (int32 i = 0; i < N; ++i)
		{
			OutU[i] -= StepSize * GradU[i];
		}

		// 投影到约束集
		ProjectToConstraints(OutU);
	}

	return true;
}

void ULinearMPCAvoidance::ComputeAnalyticGradient(
	const TArray<FVector>& X,
	const TArray<FVector>& U,
	const TArray<FVector>& Xref,
	const TArray<FObstacleInfo>& Obstacles,
	TArray<FVector>& OutGradU)
{
	const int32 N = Config.PredictionSteps;
	OutGradU.SetNum(N);

	// 简化梯度计算：∇J_u = 2*R*u
	// 忽略状态项的梯度（需要反向传播，计算复杂）
	const float R = Config.GetControlWeight();

	for (int32 i = 0; i < N; ++i)
	{
		// 控制代价梯度
		OutGradU[i] = 2.0f * R * U[i];

		// 障碍物代价梯度（简化：基于当前位置）
		if (i < X.Num())
		{
			for (const FObstacleInfo& Obs : Obstacles)
			{
				float Dist = CalculateDistanceToObstacle(X[i], Obs);
				if (Dist < Config.ObstacleInfluenceDistance)
				{
					FVector GradDir = (X[i] - Obs.Center).GetSafeNormal();
					float Penalty = Config.GetObstacleWeight() * FMath::Exp(-Dist / Config.ObstacleSafeDistance);
					OutGradU[i] += Penalty * GradDir;
				}
			}
		}
	}
}

void ULinearMPCAvoidance::ForwardSimulateLinear(
	const FVector& X0,
	const TArray<FVector>& U,
	TArray<FVector>& OutX)
{
	const int32 N = U.Num();
	OutX.SetNum(N + 1);
	OutX[0] = X0;

	FVector CurrentPos = X0;
	FVector CurrentVel = FVector::ZeroVector;

	const float Dt = Config.GetDt();

	for (int32 i = 0; i < N; ++i)
	{
		// 线性模型：x[k+1] = A*x[k] + B*u[k]
		// 简化实现：直接用运动学方程
		FVector Accel = U[i];
		CurrentVel += Accel * Dt;
		CurrentPos += CurrentVel * Dt + 0.5f * Accel * Dt * Dt;

		OutX[i + 1] = CurrentPos;
	}
}

void ULinearMPCAvoidance::ProjectToConstraints(TArray<FVector>& U)
{
	const float MaxAccel = Config.MaxAcceleration;
	const float MaxVel = Config.MaxVelocity;

	for (FVector& Accel : U)
	{
		// 加速度约束
		Accel = Accel.GetClampedToMaxSize(MaxAccel);
	}

	// 速度约束（简化：不显式处理，依赖加速度约束）
}

float ULinearMPCAvoidance::ComputeCost(
	const TArray<FVector>& X,
	const TArray<FVector>& U,
	const TArray<FVector>& Xref,
	const TArray<FObstacleInfo>& Obstacles)
{
	float Cost = 0.0f;
	const int32 N = U.Num();

	// 跟踪代价
	for (int32 i = 0; i <= N; ++i)
	{
		if (i < X.Num() && i < Xref.Num())
		{
			float TrackError = FVector::Dist(X[i], Xref[i]);
			Cost += Config.GetPositionWeight() * TrackError * TrackError;
		}
	}

	// 控制代价
	for (int32 i = 0; i < N; ++i)
	{
		Cost += Config.GetControlWeight() * U[i].SizeSquared();
	}

	// 障碍物代价
	for (int32 i = 0; i < X.Num(); ++i)
	{
		for (const FObstacleInfo& Obs : Obstacles)
		{
			float Dist = CalculateDistanceToObstacle(X[i], Obs);
			if (Dist < Config.ObstacleInfluenceDistance)
			{
				float Penalty = Config.GetObstacleWeight() * FMath::Exp(-Dist / Config.ObstacleSafeDistance);
				Cost += Penalty;
			}
		}
	}

	return Cost;
}


