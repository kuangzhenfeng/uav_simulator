// Copyright Epic Games, Inc. All Rights Reserved.

#include "JointNMPCSolver.h"
#include "../Planning/ObstacleGeometry.h"
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

	int32 N = Config.BaseConfig.Solver.PredictionSteps;
	float Dt = Config.BaseConfig.GetDt();
	float MaxAccel = Config.BaseConfig.Actuator.MaxAcceleration;
	float MaxVel = Config.BaseConfig.Actuator.MaxVelocity;

	TArray<int32> AgentIDs;
	for (const auto& State : AgentStates) AgentIDs.Add(State.AgentID);

	// 飞行集合变化时重新初始化，避免将另一架飞机的控制序列用于温启动。
	TArray<TArray<FVector>> AllControls;
	if (bHasPreviousSolve && PreviousAllControls.Num() == NumAgents && PreviousAgentIDs == AgentIDs)
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
		// 先取出填充值，避免 Refs.Add(Refs.Last()) 在扩容时引用悬挂
		FVector FillValue = Refs.Num() > 0 ? Refs.Last() : AgentStates[i].State.Position;
		while (Refs.Num() < N + 1)
		{
			Refs.Add(FillValue);
		}
		PaddedRefs.Add(Refs);
	}

	// 投影梯度下降主循环
	float StepSize = Config.BaseConfig.Solver.InitialStepSize;
	PreviousTotalCost = MAX_FLT;

	for (int32 Iter = 0; Iter < Config.BaseConfig.Solver.MaxIterations; ++Iter)
	{
		// 投影控制到可行域
		for (int32 i = 0; i < NumAgents; ++i)
		{
			ProjectAgentControls(AllControls[i], AgentStates[i].State.Velocity, MaxAccel, MaxVel, Dt);
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
		double CurrentCost = ComputeJointCost(
			AllPositions, AllVelocities, AllControls, PaddedRefs,
			StaticObstacles, FormationTargets, Config);

		// 收敛检查
		if (Iter > 0 && FMath::Abs(CurrentCost - PreviousTotalCost) < Config.BaseConfig.Solver.ConvergenceTolerance)
		{
			Result.TotalCost = CurrentCost;
			Result.bConverged = true;
			break;
		}
		PreviousTotalCost = CurrentCost;
		Result.TotalCost = CurrentCost;

		// 计算梯度
		TArray<TArray<FVector>> Gradient;
		ComputeJointGradient(AgentStates, AllControls, AllPositions, AllVelocities, PaddedRefs,
			StaticObstacles, FormationTargets, Config, Gradient);

		double GradientNormSquared = 0.0;
		for (const auto& AgentGradient : Gradient) for (const FVector& G : AgentGradient) GradientNormSquared += G.SizeSquared();
		const double GradientNorm = FMath::Sqrt(GradientNormSquared);
		if (!FMath::IsFinite(GradientNorm)) break;
		if (GradientNorm < 1.e-6) { Result.bConverged = true; break; }
		for (auto& AgentGradient : Gradient) for (FVector& G : AgentGradient) G /= GradientNorm;

		// 回溯线搜索
		double BestCost = CurrentCost;
		TArray<TArray<FVector>> BestControls = AllControls;
		float CurrentStep = StepSize;

		for (int32 BtStep = 0; BtStep < Config.BaseConfig.Solver.MaxBacktrackSteps; ++BtStep)
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
				ProjectAgentControls(AgentTrial, AgentStates[i].State.Velocity, MaxAccel, MaxVel, Dt);
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

			double TrialCost = ComputeJointCost(
				TrialPositions, TrialVelocities, TrialControls, PaddedRefs,
				StaticObstacles, FormationTargets, Config);

			if (TrialCost < BestCost)
			{
				BestCost = TrialCost;
				BestControls = TrialControls;
				break;
			}

			CurrentStep *= Config.BaseConfig.Solver.BacktrackFactor;
		}

		if (BestCost == CurrentCost) break;
		AllControls = BestControls;
		Result.TotalCost = BestCost;
		StepSize = FMath::Min(CurrentStep * 1.2f, Config.BaseConfig.Solver.InitialStepSize);
	}

	// 提取每 Agent 的第一步最优加速度
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

	Result.bUsableControls=FMath::IsFinite(Result.TotalCost) && Result.OptimalAccelerations.Num()==NumAgents;
    for(const FVector& A:Result.OptimalAccelerations)
        if(A.ContainsNaN() || A.Size()>MaxAccel+0.1f) Result.bUsableControls=false;

    // 保存用于温启动
	PreviousAllControls = AllControls;
	PreviousAgentIDs = AgentIDs;
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

double UJointNMPCSolver::ComputeJointCost(
	const TArray<TArray<FVector>>& AllPositions,
	const TArray<TArray<FVector>>& AllVelocities,
	const TArray<TArray<FVector>>& AllControls,
	const TArray<TArray<FVector>>& AllReferences,
	const TArray<FObstacleInfo>& StaticObstacles,
	const TArray<FVector>& FormationTargets,
	const FJointNMPCConfig& Config) const
{
	double TotalCost = 0.0;
	int32 NumAgents = AllPositions.Num();
	int32 N = Config.BaseConfig.Solver.PredictionSteps;
	float Dt = Config.BaseConfig.GetDt();

	// 单机代价：参考跟踪 + 速度跟踪 + 控制代价
	for (int32 i = 0; i < NumAgents; ++i)
	{
		for (int32 k = 0; k < N; ++k)
		{
			// 参考跟踪
			float RefCost = FVector::DistSquared(AllPositions[i][k], AllReferences[i][k]);
			TotalCost += Config.BaseConfig.Cost.WeightReference * RefCost;

			// 速度跟踪
			FVector DesiredVel = (AllReferences[i][k + 1] - AllReferences[i][k]) / Dt;
			float VelCost = FVector::DistSquared(AllVelocities[i][k], DesiredVel);
			TotalCost += Config.BaseConfig.Cost.WeightVelocity * VelCost;

			// 控制代价
			TotalCost += Config.BaseConfig.Cost.WeightControl * AllControls[i][k].SizeSquared();

			// 静态障碍物代价（指数势垒）
			for (const FObstacleInfo& Obs : StaticObstacles)
			{
				float Dist = ObstacleGeometry::SignedDistance(AllPositions[i][k], Obs);
				float SafeDist = Config.BaseConfig.Obstacle.ObstacleSafeDistance;
				float InfluenceDist = Config.BaseConfig.Obstacle.ObstacleInfluenceDistance;
				float Alpha = Config.BaseConfig.Obstacle.ObstacleAlpha;

				if (Dist < InfluenceDist)
				{
					float Exponent = -Alpha * (Dist - SafeDist);
					if (FMath::IsFinite(Exponent))
					{
						float ObsCost = FMath::Max(0.0f, Exponent);
						TotalCost += Config.BaseConfig.Cost.WeightObstacle * ObsCost;
					}
				}
			}
		}

		// 终端代价
		float TerminalRefCost = FVector::DistSquared(AllPositions[i][N], AllReferences[i][N]);
		TotalCost += Config.BaseConfig.Cost.WeightTerminal * TerminalRefCost;
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
					Config.BaseConfig.Obstacle.ObstacleAlpha);
				TotalCost += Config.WeightInterAgentCollision * Cost;
			}
		}
	}

    // 编队代价约束机间相对位置，避免用未来末点把整队拉向同一固定目标。
    if (FormationTargets.Num()==NumAgents && Config.WeightFormation>0)
        for(int32 I=0;I<NumAgents;++I) for(int32 J=I+1;J<NumAgents;++J)
            for(int32 K=0;K<=N;++K)
                TotalCost+=Config.WeightFormation*((AllPositions[I][K]-AllPositions[J][K])-(AllReferences[I][K]-AllReferences[J][K])).SizeSquared();

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
	float MaxAccel, float MaxVel, float Dt) const
{
    FVector Velocity=InitVel;
	for (FVector& U : Controls)
	{
		// 加速度约束
        // 交替投影到加速度球和下一步速度球。
        for(int32 Iter=0;Iter<20;++Iter)
        {
            U=U.GetClampedToMaxSize(MaxAccel);
            const FVector NextVelocity=(Velocity+U*Dt).GetClampedToMaxSize(MaxVel);
            U=(NextVelocity-Velocity)/FMath::Max(Dt,SMALL_NUMBER);
        }
        Velocity+=U*Dt;
	}
}

void UJointNMPCSolver::ComputeJointGradient(
	const TArray<FAgentStateSnapshot>& AgentStates,
	const TArray<TArray<FVector>>& AllControls,
	const TArray<TArray<FVector>>& AllPositions,
	const TArray<TArray<FVector>>& AllVelocities,
	const TArray<TArray<FVector>>& AllReferences,
	const TArray<FObstacleInfo>& StaticObstacles,
	const TArray<FVector>& FormationTargets,
	const FJointNMPCConfig& Config,
	TArray<TArray<FVector>>& OutGradient) const
{
    const int32 Count=AgentStates.Num(), N=Config.BaseConfig.Solver.PredictionSteps;
    const double Dt=Config.BaseConfig.GetDt();
    TArray<TArray<FVector>> GP,GV;
    GP.SetNum(Count);GV.SetNum(Count);OutGradient.SetNum(Count);
    for(int32 I=0;I<Count;++I)
    {
        GP[I].Init(FVector::ZeroVector,N+1);GV[I].Init(FVector::ZeroVector,N+1);OutGradient[I].SetNum(N);
        for(int32 K=0;K<N;++K)
        {
            GP[I][K]+=2*Config.BaseConfig.Cost.WeightReference*(AllPositions[I][K]-AllReferences[I][K]);
            const FVector DesiredVel=(AllReferences[I][K+1]-AllReferences[I][K])/Dt;
            GV[I][K]+=2*Config.BaseConfig.Cost.WeightVelocity*(AllVelocities[I][K]-DesiredVel);
            for(const auto& Obs:StaticObstacles)
                if(ObstacleGeometry::SignedDistance(AllPositions[I][K],Obs)<Config.BaseConfig.Obstacle.ObstacleSafeDistance)
                    GP[I][K]-=Config.BaseConfig.Cost.WeightObstacle*Config.BaseConfig.Obstacle.ObstacleAlpha*ObstacleGeometry::Gradient(AllPositions[I][K],Obs);
        }
        GP[I][N]+=2*Config.BaseConfig.Cost.WeightTerminal*(AllPositions[I][N]-AllReferences[I][N]);
    }
    for(int32 I=0;I<Count;++I) for(int32 J=I+1;J<Count;++J) for(int32 K=0;K<=N;++K)
    {
        const FVector Difference=AllPositions[I][K]-AllPositions[J][K];
        const double Distance=Difference.Size();
        FVector G=FVector::ZeroVector;
        if(Distance<Config.InterAgentSafeDistance)
            G-=Config.WeightInterAgentCollision*Config.BaseConfig.Obstacle.ObstacleAlpha*(Distance>1.e-6 ? Difference/Distance : FVector(1,0,0));
        if(FormationTargets.Num()==Count)
            G+=2*Config.WeightFormation*(Difference-(AllReferences[I][K]-AllReferences[J][K]));
        GP[I][K]+=G;GP[J][K]-=G;
    }
    // 半隐式双积分模型的伴随梯度，复杂度与预测步数线性相关。
    for(int32 I=0;I<Count;++I)
    {
        FVector LP=GP[I][N],LV=GV[I][N];
        for(int32 K=N-1;K>=0;--K)
        {
            OutGradient[I][K]=2*Config.BaseConfig.Cost.WeightControl*AllControls[I][K]+Dt*LV+Dt*Dt*LP;
            LV=GV[I][K]+LV+Dt*LP;
            LP=GP[I][K]+LP;
        }
    }
}
