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

namespace
{
// 两阶段单纯形：第一阶段消除人工变量，第二阶段优化真实目标。
class FSimplexTableau
{
public:
    int32 M, N, Iterations = 0, MaxIterations;
    TArray<TArray<double>> D;
    TArray<int32> Basis, NonBasis;
    static constexpr double Eps = 1e-8;
    FSimplexTableau(const TArray<TArray<double>>& A, const TArray<double>& B,
        const TArray<float>& C, int32 Limit) : M(B.Num()), N(C.Num()), MaxIterations(Limit)
    {
        D.SetNum(M+2); for(auto& Row:D) Row.Init(0.0,N+2);
        Basis.SetNum(M); NonBasis.SetNum(N+1);
        for(int32 I=0;I<M;++I)
        {
            for(int32 J=0;J<N;++J) D[I][J]=A[I][J];
            Basis[I]=N+I; D[I][N]=-1; D[I][N+1]=B[I];
        }
        for(int32 J=0;J<N;++J) {NonBasis[J]=J;D[M][J]=C[J];}
        NonBasis[N]=-1; D[M+1][N]=1;
    }
    void Pivot(int32 R,int32 S)
    {
        const double Inv=1.0/D[R][S];
        for(int32 I=0;I<M+2;++I) if(I!=R)
            for(int32 J=0;J<N+2;++J) if(J!=S) D[I][J]-=D[R][J]*D[I][S]*Inv;
        for(int32 J=0;J<N+2;++J) if(J!=S) D[R][J]*=Inv;
        for(int32 I=0;I<M+2;++I) if(I!=R) D[I][S]*=-Inv;
        D[R][S]=Inv; Swap(Basis[R],NonBasis[S]);
    }
    bool Optimize(int32 Phase)
    {
        const int32 Obj=Phase==1 ? M+1 : M;
        while(++Iterations<=MaxIterations)
        {
            int32 S=INDEX_NONE;
            for(int32 J=0;J<=N;++J)
            {
                if(Phase==2 && NonBasis[J]==-1) continue;
                if(S==INDEX_NONE || D[Obj][J]<D[Obj][S]-Eps ||
                    (FMath::Abs(D[Obj][J]-D[Obj][S])<=Eps && NonBasis[J]<NonBasis[S])) S=J;
            }
            if(S==INDEX_NONE || D[Obj][S]>=-Eps) return true;
            int32 R=INDEX_NONE;
            for(int32 I=0;I<M;++I)
            {
                if(D[I][S]<=Eps) continue;
                const double Ratio=D[I][N+1]/D[I][S];
                const double Best=R==INDEX_NONE ? MAX_dbl : D[R][N+1]/D[R][S];
                if(R==INDEX_NONE || Ratio<Best-Eps || (FMath::Abs(Ratio-Best)<=Eps && Basis[I]<Basis[R])) R=I;
            }
            if(R==INDEX_NONE) return false;
            Pivot(R,S);
        }
        return false;
    }
    bool Solve(TArray<float>& X)
    {
        int32 R=INDEX_NONE;
        for(int32 I=0;I<M;++I) if(R==INDEX_NONE || D[I][N+1]<D[R][N+1]) R=I;
        if(R!=INDEX_NONE && D[R][N+1]<-Eps)
        {
            Pivot(R,N);
            if(!Optimize(1) || FMath::Abs(D[M+1][N+1])>Eps) return false;
            for(int32 I=0;I<M;++I) if(Basis[I]==-1)
            {
                int32 S=INDEX_NONE;
                for(int32 J=0;J<N;++J) if(FMath::Abs(D[I][J])>Eps &&
                    (S==INDEX_NONE || NonBasis[J]<NonBasis[S])) S=J;
                if(S!=INDEX_NONE) Pivot(I,S);
            }
        }
        if(!Optimize(2)) return false;
        X.Init(0.0f,N);
        for(int32 I=0;I<M;++I) if(Basis[I]>=0 && Basis[I]<N) X[Basis[I]]=float(D[I][N+1]);
        return true;
    }
};
}

FLPResult UMILPSolver::SolveLP(
    const TArray<float>& c, const TArray<TArray<float>>& AIneq, const TArray<float>& bIneq,
    const TArray<TArray<float>>& AEq, const TArray<float>& bEq,
    const TArray<float>& LB, const TArray<float>& UB, const FMILPSolverConfig& Config)
{
    FLPResult Result;
    const int32 N=c.Num(); if(N==0) return Result;
    TArray<double> Lower; Lower.Init(0.0,N);
    for(int32 J=0;J<N;++J) if(LB.IsValidIndex(J)) Lower[J]=LB[J];
    TArray<TArray<double>> Rows; TArray<double> Bounds;
    auto AddRow=[&](const TArray<float>& Coeff,float Bound,double Sign)
    {
        TArray<double> Row;Row.Init(0.0,N);double B=Sign*Bound;
        for(int32 J=0;J<N;++J) if(Coeff.IsValidIndex(J)) {Row[J]=Sign*Coeff[J];B-=Row[J]*Lower[J];}
        Rows.Add(Row);Bounds.Add(B);
    };
    for(int32 I=0;I<AIneq.Num() && I<bIneq.Num();++I) AddRow(AIneq[I],bIneq[I],1);
    for(int32 I=0;I<AEq.Num() && I<bEq.Num();++I)
    { AddRow(AEq[I],bEq[I],1); AddRow(AEq[I],bEq[I],-1); }
    for(int32 J=0;J<N;++J) if(UB.IsValidIndex(J))
    {
        if(UB[J]<Lower[J]-1e-8) return Result;
        TArray<double> Row;Row.Init(0.0,N);Row[J]=1;Rows.Add(Row);Bounds.Add(UB[J]-Lower[J]);
    }
    FSimplexTableau LP(Rows,Bounds,c,Config.LPMaxIterations);
    if(!LP.Solve(Result.Solution)) return Result;
    Result.ObjectiveValue=0;
    for(int32 J=0;J<N;++J)
    {
        Result.Solution[J]+=float(Lower[J]);
        Result.ObjectiveValue+=c[J]*Result.Solution[J];
    }
    Result.bIsFeasible=IsFeasible(Result.Solution,AIneq,bIneq,AEq,bEq,LB,UB,Config.LPConvergenceTolerance*10);
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
		if (bHasIncumbent && LPResult.ObjectiveValue >= IncumbentObjective - FMath::Abs(IncumbentObjective) * Config.MIPGap)
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
