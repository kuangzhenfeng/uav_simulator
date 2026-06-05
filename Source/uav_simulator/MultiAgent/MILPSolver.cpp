// Copyright Epic Games, Inc. All Rights Reserved.

#include "MILPSolver.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UMILPSolver::UMILPSolver()
	: IncumbentObjective(MAX_FLT)
	, bHasIncumbent(false)
	, NodesExplored(0)
	, SolveStartTime(0.0)
	, TimeLimit(2.0f)
{
}

FMILPResult UMILPSolver::Solve(
	const TArray<float>& c,
	const TArray<TArray<float>>& AIneq,
	const TArray<float>& bIneq,
	const TArray<TArray<float>>& AEq,
	const TArray<float>& bEq,
	const TArray<float>& LB,
	const TArray<float>& UB,
	const TArray<int32>& IntegerIndices,
	const FMILPSolverConfig& Config)
{
	FMILPResult Result;
	double StartTime = FPlatformTime::Seconds();

	// 如果没有整数变量，直接求解 LP
	if (IntegerIndices.Num() == 0)
	{
		FLPResult LPResult = SolveLP(c, AIneq, bIneq, AEq, bEq, LB, UB, Config);
		Result.Solution = LPResult.Solution;
		Result.ObjectiveValue = LPResult.ObjectiveValue;
		Result.bIsFeasible = LPResult.bIsFeasible;
		Result.OptimalityGap = 0.0f;
		Result.NodesExplored = 1;
		Result.SolveTimeSeconds = (float)(FPlatformTime::Seconds() - StartTime);
		return Result;
	}

	// 分支定界求解
	Result = BranchAndBound(c, AIneq, bIneq, AEq, bEq, LB, UB, IntegerIndices, Config);
	Result.SolveTimeSeconds = (float)(FPlatformTime::Seconds() - StartTime);
	return Result;
}

FLPResult UMILPSolver::SolveLP(
	const TArray<float>& c,
	const TArray<TArray<float>>& AIneq,
	const TArray<float>& bIneq,
	const TArray<TArray<float>>& AEq,
	const TArray<float>& bEq,
	const TArray<float>& LB,
	const TArray<float>& UB,
	const FMILPSolverConfig& Config)
{
	FLPResult Result;

	int32 NumVars = c.Num();
	if (NumVars == 0)
	{
		return Result;
	}

	// 构造标准形式 LP:
	// min c^T x  s.t.  A_ineq * x <= b_ineq,  lb <= x <= ub
	// 使用投影梯度法求解简化版 LP
	// 对于小规模问题（< 200 变量），投影梯度法足够高效

	// 初始化为可行域中点
	TArray<float> X;
	X.SetNum(NumVars);
	for (int32 i = 0; i < NumVars; ++i)
	{
		float Lower = (i < LB.Num()) ? LB[i] : 0.0f;
		float Upper = (i < UB.Num()) ? UB[i] : 1.0f;
		X[i] = (Lower + Upper) * 0.5f;
	}

	float StepSize = 0.1f;
	float PrevObj = MAX_FLT;

	for (int32 Iter = 0; Iter < Config.LPMaxIterations; ++Iter)
	{
		// 计算目标函数梯度 (c 本身)
		// 梯度下降步
		for (int32 i = 0; i < NumVars; ++i)
		{
			X[i] -= StepSize * c[i];
		}

		// 投影到可行域：先投影到变量界
		for (int32 i = 0; i < NumVars; ++i)
		{
			float Lower = (i < LB.Num()) ? LB[i] : 0.0f;
			float Upper = (i < UB.Num()) ? UB[i] : 1.0f;
			X[i] = FMath::Clamp(X[i], Lower, Upper);
		}

		// 投影到不等式约束（逐个投影）
		for (int32 Row = 0; Row < AIneq.Num() && Row < bIneq.Num(); ++Row)
		{
			// 计算 AIneq[Row] * X
			float Violation = -bIneq[Row];
			const TArray<float>& RowCoeffs = AIneq[Row];
			for (int32 i = 0; i < RowCoeffs.Num() && i < NumVars; ++i)
			{
				Violation += RowCoeffs[i] * X[i];
			}

			// 如果约束违反 (Violation > 0)，投影到约束面
			if (Violation > Config.LPConvergenceTolerance)
			{
				float NormSq = 0.0f;
				for (int32 i = 0; i < RowCoeffs.Num() && i < NumVars; ++i)
				{
					NormSq += RowCoeffs[i] * RowCoeffs[i];
				}
				if (NormSq > KINDA_SMALL_NUMBER)
				{
					float Scale = Violation / NormSq;
					for (int32 i = 0; i < RowCoeffs.Num() && i < NumVars; ++i)
					{
						X[i] -= Scale * RowCoeffs[i];
					}
				}
			}
		}

		// 再次投影到变量界
		for (int32 i = 0; i < NumVars; ++i)
		{
			float Lower = (i < LB.Num()) ? LB[i] : 0.0f;
			float Upper = (i < UB.Num()) ? UB[i] : 1.0f;
			X[i] = FMath::Clamp(X[i], Lower, Upper);
		}

		// 计算目标函数值
		float Obj = 0.0f;
		for (int32 i = 0; i < NumVars; ++i)
		{
			Obj += c[i] * X[i];
		}

		// 收敛检查
		if (FMath::Abs(Obj - PrevObj) < Config.LPConvergenceTolerance)
		{
			Result.ObjectiveValue = Obj;
			break;
		}
		PrevObj = Obj;

		// 自适应步长
		StepSize *= 0.95f;
		StepSize = FMath::Max(StepSize, 1e-6f);
	}

	// 计算最终目标函数值
	float FinalObj = 0.0f;
	for (int32 i = 0; i < NumVars; ++i)
	{
		FinalObj += c[i] * X[i];
	}

	Result.Solution = X;
	Result.ObjectiveValue = FinalObj;
	Result.bIsFeasible = IsFeasible(X, AIneq, bIneq, AEq, bEq, LB, UB, 0.01f);
	return Result;
}

FMILPResult UMILPSolver::BranchAndBound(
	const TArray<float>& c,
	const TArray<TArray<float>>& AIneq,
	const TArray<float>& bIneq,
	const TArray<TArray<float>>& AEq,
	const TArray<float>& bEq,
	const TArray<float>& LB,
	const TArray<float>& UB,
	const TArray<int32>& IntegerIndices,
	const FMILPSolverConfig& Config)
{
	FMILPResult Result;
	IncumbentSolution.Empty();
	IncumbentObjective = MAX_FLT;
	bHasIncumbent = false;
	NodesExplored = 0;
	SolveStartTime = FPlatformTime::Seconds();
	TimeLimit = Config.TimeLimitSeconds;

	// 简单的栈式分支定界
	struct FBBNode
	{
		TArray<float> NodeLB;
		TArray<float> NodeUB;
	};

	TArray<FBBNode> NodeStack;

	// 根节点
	FBBNode Root;
	Root.NodeLB = LB;
	Root.NodeUB = UB;
	NodeStack.Add(Root);

	while (NodeStack.Num() > 0 && NodesExplored < Config.MaxBranchAndBoundNodes)
	{
		// 时间限制检查
		if ((float)(FPlatformTime::Seconds() - SolveStartTime) > TimeLimit)
		{
			break;
		}

		// 取出最佳节点（DFS）
		FBBNode Current = NodeStack.Pop();
		NodesExplored++;

		// 求解 LP 松弛
		FLPResult LPResult = SolveLP(c, AIneq, bIneq, AEq, bEq, Current.NodeLB, Current.NodeUB, Config);

		// 剪枝：LP 不可行
		if (!LPResult.bIsFeasible)
		{
			continue;
		}

		// 剪枝：LP 下界 >= 当前最优（定界）
		if (bHasIncumbent && LPResult.ObjectiveValue >= IncumbentObjective * (1.0f - Config.MIPGap))
		{
			continue;
		}

		// 检查整数可行性
		if (IsIntegerFeasible(LPResult.Solution, IntegerIndices))
		{
			// 更新最优解
			if (!bHasIncumbent || LPResult.ObjectiveValue < IncumbentObjective)
			{
				IncumbentSolution = LPResult.Solution;
				IncumbentObjective = LPResult.ObjectiveValue;
				bHasIncumbent = true;
			}
			continue;
		}

		// 分支：选择最接近 0.5 的整数变量
		int32 BranchVar = SelectBranchingVariable(LPResult.Solution, IntegerIndices);
		if (BranchVar < 0 || BranchVar >= c.Num())
		{
			continue;
		}

		float BranchValue = LPResult.Solution[BranchVar];

		// 左子节点：x[BranchVar] <= floor(BranchValue)
		{
			FBBNode LeftNode;
			LeftNode.NodeLB = Current.NodeLB;
			LeftNode.NodeUB = Current.NodeUB;
			LeftNode.NodeUB[BranchVar] = FMath::FloorToFloat(BranchValue);
			// 检查上下界一致性
			if (LeftNode.NodeLB[BranchVar] <= LeftNode.NodeUB[BranchVar])
			{
				NodeStack.Add(LeftNode);
			}
		}

		// 右子节点：x[BranchVar] >= ceil(BranchValue)
		{
			FBBNode RightNode;
			RightNode.NodeLB = Current.NodeLB;
			RightNode.NodeUB = Current.NodeUB;
			RightNode.NodeLB[BranchVar] = FMath::CeilToFloat(BranchValue);
			if (RightNode.NodeLB[BranchVar] <= RightNode.NodeUB[BranchVar])
			{
				NodeStack.Add(RightNode);
			}
		}
	}

	// 返回结果
	if (bHasIncumbent)
	{
		Result.Solution = IncumbentSolution;
		Result.ObjectiveValue = IncumbentObjective;
		Result.bIsFeasible = true;
		// 计算间隙（近似）
		Result.OptimalityGap = 0.0f; // 简化处理
	}
	Result.NodesExplored = NodesExplored;

	return Result;
}

int32 UMILPSolver::SelectBranchingVariable(
	const TArray<float>& LPSolution,
	const TArray<int32>& IntegerIndices) const
{
	int32 BestVar = -1;
	float BestFractional = 2.0f; // 距离 0.5 的距离

	for (int32 Idx : IntegerIndices)
	{
		if (Idx < 0 || Idx >= LPSolution.Num())
		{
			continue;
		}

		float Value = LPSolution[Idx];
		float Fractional = FMath::Abs(Value - FMath::RoundToFloat(Value));

		// 已经是整数
		if (Fractional < 0.01f)
		{
			continue;
		}

		// 选择最接近 0.5 的（最分数化分支）
		float DistanceToHalf = FMath::Abs(Fractional - 0.5f);
		if (DistanceToHalf < BestFractional)
		{
			BestFractional = DistanceToHalf;
			BestVar = Idx;
		}
	}

	return BestVar;
}

TArray<float> UMILPSolver::RoundSolution(
	const TArray<float>& LPSolution,
	const TArray<int32>& IntegerIndices) const
{
	TArray<float> Rounded = LPSolution;
	for (int32 Idx : IntegerIndices)
	{
		if (Idx >= 0 && Idx < Rounded.Num())
		{
			Rounded[Idx] = FMath::RoundToFloat(Rounded[Idx]);
		}
	}
	return Rounded;
}

bool UMILPSolver::IsIntegerFeasible(
	const TArray<float>& Solution,
	const TArray<int32>& IntegerIndices,
	float Tolerance) const
{
	for (int32 Idx : IntegerIndices)
	{
		if (Idx < 0 || Idx >= Solution.Num())
		{
			return false;
		}
		float Fractional = FMath::Abs(Solution[Idx] - FMath::RoundToFloat(Solution[Idx]));
		if (Fractional > Tolerance)
		{
			return false;
		}
	}
	return true;
}

bool UMILPSolver::IsFeasible(
	const TArray<float>& Solution,
	const TArray<TArray<float>>& AIneq,
	const TArray<float>& bIneq,
	const TArray<TArray<float>>& AEq,
	const TArray<float>& bEq,
	const TArray<float>& LB,
	const TArray<float>& UB,
	float Tolerance) const
{
	int32 NumVars = Solution.Num();

	// 检查变量界
	for (int32 i = 0; i < NumVars; ++i)
	{
		float Lower = (i < LB.Num()) ? LB[i] : 0.0f;
		float Upper = (i < UB.Num()) ? UB[i] : 1.0f;
		if (Solution[i] < Lower - Tolerance || Solution[i] > Upper + Tolerance)
		{
			return false;
		}
	}

	// 检查不等式约束
	for (int32 Row = 0; Row < AIneq.Num() && Row < bIneq.Num(); ++Row)
	{
		float Sum = 0.0f;
		const TArray<float>& RowCoeffs = AIneq[Row];
		for (int32 i = 0; i < RowCoeffs.Num() && i < NumVars; ++i)
		{
			Sum += RowCoeffs[i] * Solution[i];
		}
		if (Sum > bIneq[Row] + Tolerance)
		{
			return false;
		}
	}

	// 检查等式约束
	for (int32 Row = 0; Row < AEq.Num() && Row < bEq.Num(); ++Row)
	{
		float Sum = 0.0f;
		const TArray<float>& RowCoeffs = AEq[Row];
		for (int32 i = 0; i < RowCoeffs.Num() && i < NumVars; ++i)
		{
			Sum += RowCoeffs[i] * Solution[i];
		}
		if (FMath::Abs(Sum - bEq[Row]) > Tolerance)
		{
			return false;
		}
	}

	return true;
}
