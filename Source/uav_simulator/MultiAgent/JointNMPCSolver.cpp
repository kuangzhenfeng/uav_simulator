// Copyright Epic Games, Inc. All Rights Reserved.

#include "JointNMPCSolver.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UJointNMPCSolver::UJointNMPCSolver()
{
}

FJointNMPCSolveResult UJointNMPCSolver::Solve(
	const TArray<FAgentStateSnapshot>& AgentStates,
	const TArray<TArray<FVector>>& ReferencePointsPerAgent,
	const TArray<FObstacleInfo>& StaticObstacles,
	const FJointNMPCConfig& Config)
{
	FJointNMPCSolveResult Result;

	int32 NumAgents = AgentStates.Num();
	if (NumAgents == 0 || NumAgents > Config.MaxAgents)
	{
		UE_LOG(LogUAVMultiAgent, Warning, TEXT("[JointNMPC] Invalid agent count: %d"), NumAgents);
		return Result;
	}

	int32 N = Config.BaseConfig.PredictionSteps;
	float Dt = Config.BaseConfig.GetDt();
	float MaxAccel = Config.BaseConfig.MaxAcceleration;
	float MaxVel = Config.BaseConfig.MaxVelocity;

	// 初始化或温启动控制序列
	TArray<TArray<FVector>> AllControls;
	if (bHasPreviousSolve && PreviousAllControls.Num() == NumAgents)
	{
		// 温启动：左移一步
		for (int32 i = 0; i < NumAgents; ++i)
		{
			TArray<FVector> WarmControls;
			if (PreviousAllControls[i].Num() == N)
			{
				for (int32 k = 1; k < N; ++k)
				{
					WarmControls.Add(PreviousAllControls[i][k]);
				}
				// 最后一步重复末尾
				WarmControls.Add(PreviousAllControls[i][N - 1]);
			}
			else
			{
				for (int32 k = 0; k < N; ++k)
				{
					WarmControls.Add(FVector::ZeroVector);
				}
			}
			AllControls.Add(WarmControls);
		}
	}
	else
	{
		// 冷启动：用参考轨迹方向初始化
		for (int32 i = 0; i < NumAgents; ++i)
		{
			TArray<FVector> AgentControls;
			for (int32 k = 0; k < N; ++k)
			{
				FVector InitControl = FVector::ZeroVector;
				if (ReferencePointsPerAgent.IsValidIndex(i) &&
					ReferencePointsPerAgent[i].Num() > 1)
				{
					FVector DesiredVel = (ReferencePointsPerAgent[i][1] - ReferencePointsPerAgent[i][0]) / Dt;
					InitControl = DesiredVel.GetClampedToMaxSize(MaxAccel) * 0.5f;
				}
				AgentControls.Add(InitControl);
			}
			AllControls.Add(AgentControls);
		}
	}

	// 计算编队目标位置
	TArray<FVector> FormationTargets;
	for (int32 i = 0; i < NumAgents; ++i)
	{
		if (ReferencePointsPerAgent.IsValidIndex(i) && ReferencePointsPerAgent[i].Num() > 0)
		{
			FormationTargets.Add(ReferencePointsPerAgent[i].Last());
		}
		else
		{
			FormationTargets.Add(AgentStates[i].State.Position);
		}
	}

	// 准备参考点（确保每 Agent 有 N+1 个点）
	TArray<TArray<FVector>> PaddedRefs;
	for (int32 i = 0; i < NumAgents; ++i)
	{
		TArray<FVector> Refs;
		if (ReferencePointsPerAgent.IsValidIndex(i))
		{
			Refs = ReferencePointsPerAgent[i];
		}
		while (Refs.Num() < N + 1)
		{
			Refs.Add(Refs.Num() > 0 ? Refs.Last() : AgentStates[i].State.Position);
		}
		PaddedRefs.Add(Refs);
	}

	// 投影梯度下降主循环
	float StepSize = Config.BaseConfig.InitialStepSize;

	for (int32 Iter = 0; Iter < Config.BaseConfig.MaxIterations; ++Iter)
	{
		// 投影控制到可行域
		for (int32 i = 0; i < NumAgents; ++i)
		{
			ProjectAgentControls(AllControls[i], AgentStates[i].State.Velocity, MaxAccel, MaxVel);
		}

		// 前向仿真所有 Agent
		TArray<TArray<FVector>> AllPositions, AllVelocities;
		for (int32 i = 0; i < NumAgents; ++i)
		{
			TArray<FVector> Positions, Velocities;
			ForwardSimulateAgent(
				AgentStates[i].State.Position, AgentStates[i].State.Velocity,
				AllControls[i], Dt, MaxVel, Positions, Velocities);
			AllPositions.Add(Positions);
			AllVelocities.Add(Velocities);
		}

		// 计算代价
		float CurrentCost = ComputeJointCost(
			AllPositions, AllVelocities, AllControls, PaddedRefs,
			StaticObstacles, FormationTargets, Config);

		// 收敛检查
		if (Iter > 0 && FMath::Abs(CurrentCost - PreviousTotalCost) < Config.BaseConfig.ConvergenceTolerance)
		{
			Result.TotalCost = CurrentCost;
			break;
		}
		PreviousTotalCost = CurrentCost;
		Result.TotalCost = CurrentCost;

		// 计算梯度
		TArray<TArray<FVector>> Gradient;
		ComputeJointGradient(AgentStates, AllControls, AllPositions, PaddedRefs,
			StaticObstacles, FormationTargets, Config, Gradient);

		// 回溯线搜索
		float BestCost = CurrentCost;
		TArray<TArray<FVector>> BestControls = AllControls;
		float CurrentStep = StepSize;

		for (int32 BtStep = 0; BtStep < Config.BaseConfig.MaxBacktrackSteps; ++BtStep)
		{
			// 试探更新
			TArray<TArray<FVector>> TrialControls;
			for (int32 i = 0; i < NumAgents; ++i)
			{
				TArray<FVector> AgentTrial;
				for (int32 k = 0; k < N; ++k)
				{
					FVector Updated = AllControls[i][k] - CurrentStep * Gradient[i][k];
					AgentTrial.Add(Updated);
				}
				TrialControls.Add(AgentTrial);
			}

			// 前向仿真试探
			TArray<TArray<FVector>> TrialPositions, TrialVelocities;
			for (int32 i = 0; i < NumAgents; ++i)
			{
				TArray<FVector> Pos, Vel;
				ForwardSimulateAgent(
					AgentStates[i].State.Position, AgentStates[i].State.Velocity,
					TrialControls[i], Dt, MaxVel, Pos, Vel);
				TrialPositions.Add(Pos);
				TrialVelocities.Add(Vel);
			}

			float TrialCost = ComputeJointCost(
				TrialPositions, TrialVelocities, TrialControls, PaddedRefs,
				StaticObstacles, FormationTargets, Config);

			if (TrialCost < BestCost)
			{
				BestCost = TrialCost;
				BestControls = TrialControls;
				break;
			}

			CurrentStep *= Config.BaseConfig.BacktrackFactor;
		}

		AllControls = BestControls;
		StepSize = FMath::Min(CurrentStep * 1.2f, Config.BaseConfig.InitialStepSize);
	}

	// 提取每 Agent 的第一步最优加速度
	Result.bConverged = true;
	for (int32 i = 0; i < NumAgents; ++i)
	{
		if (AllControls[i].Num() > 0)
		{
			Result.OptimalAccelerations.Add(AllControls[i][0]);
		}
		else
		{
			Result.OptimalAccelerations.Add(FVector::ZeroVector);
		}
	}

	// 保存用于温启动
	PreviousAllControls = AllControls;
	bHasPreviousSolve = true;

	return Result;
}

void UJointNMPCSolver::ForwardSimulateAgent(
	const FVector& InitPos, const FVector& InitVel,
	const TArray<FVector>& Controls, float Dt, float MaxVel,
	TArray<FVector>& OutPositions, TArray<FVector>& OutVelocities) const
{
	int32 N = Controls.Num();
	OutPositions.SetNum(N + 1);
	OutVelocities.SetNum(N + 1);

	OutPositions[0] = InitPos;
	OutVelocities[0] = InitVel;

	for (int32 k = 0; k < N; ++k)
	{
		OutVelocities[k + 1] = OutVelocities[k] + Controls[k] * Dt;
		// 速度约束
		if (OutVelocities[k + 1].Size() > MaxVel)
		{
			OutVelocities[k + 1] = OutVelocities[k + 1].GetClampedToMaxSize(MaxVel);
		}
		OutPositions[k + 1] = OutPositions[k] + OutVelocities[k + 1] * Dt;
	}
}

float UJointNMPCSolver::ComputeJointCost(
	const TArray<TArray<FVector>>& AllPositions,
	const TArray<TArray<FVector>>& AllVelocities,
	const TArray<TArray<FVector>>& AllControls,
	const TArray<TArray<FVector>>& AllReferences,
	const TArray<FObstacleInfo>& StaticObstacles,
	const TArray<FVector>& FormationTargets,
	const FJointNMPCConfig& Config) const
{
	float TotalCost = 0.0f;
	int32 NumAgents = AllPositions.Num();
	int32 N = Config.BaseConfig.PredictionSteps;
	float Dt = Config.BaseConfig.GetDt();

	// 单机代价：参考跟踪 + 速度跟踪 + 控制代价
	for (int32 i = 0; i < NumAgents; ++i)
	{
		for (int32 k = 0; k < N; ++k)
		{
			// 参考跟踪
			float RefCost = FVector::DistSquared(AllPositions[i][k], AllReferences[i][k]);
			TotalCost += Config.BaseConfig.WeightReference * RefCost;

			// 速度跟踪
			FVector DesiredVel = (AllReferences[i][k + 1] - AllReferences[i][k]) / Dt;
			float VelCost = FVector::DistSquared(AllVelocities[i][k], DesiredVel);
			TotalCost += Config.BaseConfig.WeightVelocity * VelCost;

			// 控制代价
			TotalCost += Config.BaseConfig.WeightControl * AllControls[i][k].SizeSquared();

			// 静态障碍物代价（指数势垒）
			for (const FObstacleInfo& Obs : StaticObstacles)
			{
				float Dist = FMath::Sqrt(FVector::DistSquared(AllPositions[i][k], Obs.Center)) - Obs.Extents.GetMax();
				float SafeDist = Config.BaseConfig.ObstacleSafeDistance;
				float InfluenceDist = Config.BaseConfig.ObstacleInfluenceDistance;
				float Alpha = Config.BaseConfig.ObstacleAlpha;

				if (Dist < InfluenceDist)
				{
					float Exponent = -Alpha * (Dist - SafeDist);
					if (Exponent <= 20.0f)
					{
						float ObsCost = FMath::Max(0.0f, Exponent);
						TotalCost += Config.BaseConfig.WeightObstacle * ObsCost;
					}
				}
			}
		}

		// 终端代价
		float TerminalRefCost = FVector::DistSquared(AllPositions[i][N], AllReferences[i][N]);
		TotalCost += Config.BaseConfig.WeightTerminal * TerminalRefCost;
	}

	// 机间碰撞代价（指数势垒）
	for (int32 i = 0; i < NumAgents; ++i)
	{
		for (int32 j = i + 1; j < NumAgents; ++j)
		{
			for (int32 k = 0; k <= N; ++k)
			{
				float Cost = ComputeInterAgentCollisionCost(
					AllPositions[i][k], AllPositions[j][k],
					Config.InterAgentSafeDistance,
					Config.InterAgentInfluenceDistance,
					Config.BaseConfig.ObstacleAlpha);
				TotalCost += Config.WeightInterAgentCollision * Cost;
			}
		}
	}

	// 编队偏差代价
	if (FormationTargets.Num() == NumAgents)
	{
		for (int32 i = 0; i < NumAgents; ++i)
		{
			for (int32 k = 0; k <= N; ++k)
			{
				float FormCost = ComputeFormationCost(AllPositions[i][k], FormationTargets[i]);
				TotalCost += Config.WeightFormation * FormCost;
			}
		}
	}

	return TotalCost;
}

float UJointNMPCSolver::ComputeInterAgentCollisionCost(
	const FVector& PosA, const FVector& PosB,
	float SafeDist, float InfluenceDist, float Alpha) const
{
	float Dist = FVector::Dist(PosA, PosB);

	if (Dist > InfluenceDist)
	{
		return 0.0f;
	}

	if (Dist < KINDA_SMALL_NUMBER)
	{
		return 1000.0f; // 重叠时的极大代价
	}

	float Exponent = -Alpha * (Dist - SafeDist);
	if (Exponent > 20.0f)
	{
		return Exponent; // 线性外推防止数值溢出
	}
	return FMath::Max(0.0f, Exponent);
}

float UJointNMPCSolver::ComputeFormationCost(
	const FVector& ActualPos, const FVector& DesiredPos) const
{
	return FVector::DistSquared(ActualPos, DesiredPos);
}

void UJointNMPCSolver::ProjectAgentControls(
	TArray<FVector>& Controls, const FVector& InitVel,
	float MaxAccel, float MaxVel) const
{
	for (FVector& U : Controls)
	{
		// 加速度约束
		if (U.Size() > MaxAccel)
		{
			U = U.GetClampedToMaxSize(MaxAccel);
		}
	}
}

void UJointNMPCSolver::ComputeJointGradient(
	const TArray<FAgentStateSnapshot>& AgentStates,
	const TArray<TArray<FVector>>& AllControls,
	const TArray<TArray<FVector>>& AllPositions,
	const TArray<TArray<FVector>>& AllReferences,
	const TArray<FObstacleInfo>& StaticObstacles,
	const TArray<FVector>& FormationTargets,
	const FJointNMPCConfig& Config,
	TArray<TArray<FVector>>& OutGradient) const
{
	int32 NumAgents = AgentStates.Num();
	int32 N = Config.BaseConfig.PredictionSteps;
	float Epsilon = Config.BaseConfig.FiniteDiffEpsilon;
	float Dt = Config.BaseConfig.GetDt();
	float MaxVel = Config.BaseConfig.MaxVelocity;

	OutGradient.SetNum(NumAgents);

	for (int32 i = 0; i < NumAgents; ++i)
	{
		OutGradient[i].SetNum(N);

		for (int32 k = 0; k < N; ++k)
		{
			FVector GradK = FVector::ZeroVector;

			// 对每个分量做中心差分
			for (int32 Dim = 0; Dim < 3; ++Dim)
			{
				TArray<FVector> ControlsPlus = AllControls[i];
				TArray<FVector> ControlsMinus = AllControls[i];

				ControlsPlus[k][Dim] += Epsilon;
				ControlsMinus[k][Dim] -= Epsilon;

				// 前向仿真 Agent i
				TArray<FVector> PosPlus, VelPlus, PosMinus, VelMinus;
				ForwardSimulateAgent(
					AgentStates[i].State.Position, AgentStates[i].State.Velocity,
					ControlsPlus, Dt, MaxVel, PosPlus, VelPlus);
				ForwardSimulateAgent(
					AgentStates[i].State.Position, AgentStates[i].State.Velocity,
					ControlsMinus, Dt, MaxVel, PosMinus, VelMinus);

				// 构造包含扰动后的所有 Agent 位置数组（其余 Agent 保持不变）
				TArray<TArray<FVector>> TrialPosPlus = AllPositions;
				TArray<TArray<FVector>> TrialVelPlus; // 不需要重新计算其余 Agent
				TArray<TArray<FVector>> TrialPosMinus = AllPositions;
				TrialPosPlus[i] = PosPlus;
				TrialPosMinus[i] = PosMinus;

				TArray<TArray<FVector>> TrialCtrlPlus = AllControls;
				TArray<TArray<FVector>> TrialCtrlMinus = AllControls;
				TrialCtrlPlus[i] = ControlsPlus;
				TrialCtrlMinus[i] = ControlsMinus;

				float CostPlus = ComputeJointCost(
					TrialPosPlus, TrialVelPlus, TrialCtrlPlus, AllReferences,
					StaticObstacles, FormationTargets, Config);
				float CostMinus = ComputeJointCost(
					TrialPosMinus, TrialVelPlus, TrialCtrlMinus, AllReferences,
					StaticObstacles, FormationTargets, Config);

				GradK[Dim] = (CostPlus - CostMinus) / (2.0f * Epsilon);
			}

			OutGradient[i][k] = GradK;
		}
	}
}
