// Copyright Epic Games, Inc. All Rights Reserved.

#include "StateEstimator.h"

UStateEstimator::UStateEstimator()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UStateEstimator::BeginPlay()
{
	Super::BeginPlay();

	// 初始化估计状态
	EstimatedState = FUAVState();

	// 初始化矩阵
	InitializeMatrices();
}

void UStateEstimator::InitializeMatrices()
{
	// 初始化 6×6 协方差矩阵 P（单位矩阵）
	CovarianceMatrix.SetNum(36);
	for (int32 i = 0; i < 36; ++i)
	{
		CovarianceMatrix[i] = 0.0f;
	}
	// 对角元素设为初始值
	CovarianceMatrix[0] = 1.0f;  // P_xx
	CovarianceMatrix[7] = 1.0f;  // P_yy
	CovarianceMatrix[14] = 1.0f; // P_zz
	CovarianceMatrix[21] = 1.0f; // P_vxvx
	CovarianceMatrix[28] = 1.0f; // P_vyvy
	CovarianceMatrix[35] = 1.0f; // P_vzvz

	// 初始化过程噪声矩阵 Q（对角矩阵）
	ProcessNoiseMatrix.SetNum(36);
	for (int32 i = 0; i < 36; ++i)
	{
		ProcessNoiseMatrix[i] = 0.0f;
	}
	ProcessNoiseMatrix[0] = ProcessNoisePosition;
	ProcessNoiseMatrix[7] = ProcessNoisePosition;
	ProcessNoiseMatrix[14] = ProcessNoisePosition;
	ProcessNoiseMatrix[21] = ProcessNoiseVelocity;
	ProcessNoiseMatrix[28] = ProcessNoiseVelocity;
	ProcessNoiseMatrix[35] = ProcessNoiseVelocity;

	// 初始化测量噪声矩阵 R（对角矩阵）
	MeasurementNoiseMatrix.SetNum(36);
	for (int32 i = 0; i < 36; ++i)
	{
		MeasurementNoiseMatrix[i] = 0.0f;
	}
	MeasurementNoiseMatrix[0] = MeasurementNoisePosition;
	MeasurementNoiseMatrix[7] = MeasurementNoisePosition;
	MeasurementNoiseMatrix[14] = MeasurementNoisePosition;
	MeasurementNoiseMatrix[21] = MeasurementNoiseVelocity;
	MeasurementNoiseMatrix[28] = MeasurementNoiseVelocity;
	MeasurementNoiseMatrix[35] = MeasurementNoiseVelocity;
}

void UStateEstimator::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UStateEstimator::Predict(const FVector& Acceleration, const FVector& AngularVelocity, float DeltaTime)
{
	// EKF预测步骤

	// 1. 状态预测
	// 位置预测: p = p + v*dt + 0.5*a*dt^2
	EstimatedState.Position += EstimatedState.Velocity * DeltaTime + 0.5f * Acceleration * DeltaTime * DeltaTime;

	// 速度预测: v = v + a*dt
	EstimatedState.Velocity += Acceleration * DeltaTime;

	// 姿态预测: 使用角速度积分
	FVector AngularVelDeg = AngularVelocity * (180.0f / PI);
	FRotator DeltaRotation(AngularVelDeg.Y * DeltaTime, AngularVelDeg.Z * DeltaTime, AngularVelDeg.X * DeltaTime);
	EstimatedState.Rotation = (EstimatedState.Rotation.Quaternion() * DeltaRotation.Quaternion()).Rotator();

	// 角速度更新
	EstimatedState.AngularVelocity = AngularVelocity;

	// 2. 协方差预测（完整版本）
	// P = F*P*F^T + Q*dt

	// 计算雅可比矩阵 F (6×6)
	TArray<float> F;
	ComputeJacobianF(F, DeltaTime);

	// 计算 F^T
	TArray<float> FT;
	FT.SetNum(36);
	MatrixTranspose(F, FT, 6, 6);

	// 计算 F*P
	TArray<float> FP;
	FP.SetNum(36);
	MatrixMultiply(F, CovarianceMatrix, FP, 6, 6, 6);

	// 计算 F*P*F^T
	TArray<float> FPFT;
	FPFT.SetNum(36);
	MatrixMultiply(FP, FT, FPFT, 6, 6, 6);

	// 计算 Q*dt
	TArray<float> Qdt;
	Qdt.SetNum(36);
	for (int32 i = 0; i < 36; ++i)
	{
		Qdt[i] = ProcessNoiseMatrix[i] * DeltaTime;
	}

	// P = F*P*F^T + Q*dt
	MatrixAdd(FPFT, Qdt, CovarianceMatrix, 36);
}

void UStateEstimator::UpdateGPS(const FVector& GPSPosition, const FVector& GPSVelocity)
{
	// EKF更新步骤（使用GPS测量）

	// 1. 计算观测矩阵 H (6×6，单位矩阵，因为直接观测位置和速度)
	TArray<float> H;
	ComputeJacobianH(H);

	// 2. 计算 H^T
	TArray<float> HT;
	HT.SetNum(36);
	MatrixTranspose(H, HT, 6, 6);

	// 3. 计算 H*P
	TArray<float> HP;
	HP.SetNum(36);
	MatrixMultiply(H, CovarianceMatrix, HP, 6, 6, 6);

	// 4. 计算 H*P*H^T
	TArray<float> HPHT;
	HPHT.SetNum(36);
	MatrixMultiply(HP, HT, HPHT, 6, 6, 6);

	// 5. 计算 H*P*H^T + R
	TArray<float> S;
	S.SetNum(36);
	MatrixAdd(HPHT, MeasurementNoiseMatrix, S, 36);

	// 6. 计算 S 的逆矩阵
	TArray<float> SInv;
	SInv.SetNum(36);
	bool bInvSuccess = MatrixInverse(S, SInv, 6);
	if (!bInvSuccess)
	{
		// 矩阵求逆失败，跳过此次更新
		return;
	}

	// 7. 计算 P*H^T
	TArray<float> PHT;
	PHT.SetNum(36);
	MatrixMultiply(CovarianceMatrix, HT, PHT, 6, 6, 6);

	// 8. 计算卡尔曼增益 K = P*H^T * S^-1
	TArray<float> K;
	K.SetNum(36);
	MatrixMultiply(PHT, SInv, K, 6, 6, 6);

	// 9. 计算新息（innovation）y = z - H*x
	FVector PositionInnovation = GPSPosition - EstimatedState.Position;
	FVector VelocityInnovation = GPSVelocity - EstimatedState.Velocity;

	// 10. 状态更新 x = x + K*y
	// K 是 6×6，y 是 6×1
	EstimatedState.Position.X += K[0] * PositionInnovation.X + K[1] * PositionInnovation.Y + K[2] * PositionInnovation.Z +
	                              K[3] * VelocityInnovation.X + K[4] * VelocityInnovation.Y + K[5] * VelocityInnovation.Z;
	EstimatedState.Position.Y += K[6] * PositionInnovation.X + K[7] * PositionInnovation.Y + K[8] * PositionInnovation.Z +
	                              K[9] * VelocityInnovation.X + K[10] * VelocityInnovation.Y + K[11] * VelocityInnovation.Z;
	EstimatedState.Position.Z += K[12] * PositionInnovation.X + K[13] * PositionInnovation.Y + K[14] * PositionInnovation.Z +
	                              K[15] * VelocityInnovation.X + K[16] * VelocityInnovation.Y + K[17] * VelocityInnovation.Z;

	EstimatedState.Velocity.X += K[18] * PositionInnovation.X + K[19] * PositionInnovation.Y + K[20] * PositionInnovation.Z +
	                              K[21] * VelocityInnovation.X + K[22] * VelocityInnovation.Y + K[23] * VelocityInnovation.Z;
	EstimatedState.Velocity.Y += K[24] * PositionInnovation.X + K[25] * PositionInnovation.Y + K[26] * PositionInnovation.Z +
	                              K[27] * VelocityInnovation.X + K[28] * VelocityInnovation.Y + K[29] * VelocityInnovation.Z;
	EstimatedState.Velocity.Z += K[30] * PositionInnovation.X + K[31] * PositionInnovation.Y + K[32] * PositionInnovation.Z +
	                              K[33] * VelocityInnovation.X + K[34] * VelocityInnovation.Y + K[35] * VelocityInnovation.Z;

	// 11. 协方差更新 P = (I - K*H) * P
	// 计算 K*H
	TArray<float> KH;
	KH.SetNum(36);
	MatrixMultiply(K, H, KH, 6, 6, 6);

	// 计算 I - K*H
	TArray<float> IKH;
	IKH.SetNum(36);
	for (int32 i = 0; i < 6; ++i)
	{
		for (int32 j = 0; j < 6; ++j)
		{
			IKH[i * 6 + j] = (i == j ? 1.0f : 0.0f) - KH[i * 6 + j];
		}
	}

	// 计算 (I - K*H) * P
	TArray<float> NewP;
	NewP.SetNum(36);
	MatrixMultiply(IKH, CovarianceMatrix, NewP, 6, 6, 6);

	CovarianceMatrix = NewP;
}

void UStateEstimator::Reset(const FUAVState& InitialState)
{
	EstimatedState = InitialState;

	// 重新初始化矩阵
	InitializeMatrices();
}

// 矩阵辅助函数实现

void UStateEstimator::MatrixMultiply(const TArray<float>& A, const TArray<float>& B, TArray<float>& Result, int M, int N, int P) const
{
	// A: M×N, B: N×P, Result: M×P
	for (int32 i = 0; i < M; ++i)
	{
		for (int32 j = 0; j < P; ++j)
		{
			float Sum = 0.0f;
			for (int32 k = 0; k < N; ++k)
			{
				Sum += A[i * N + k] * B[k * P + j];
			}
			Result[i * P + j] = Sum;
		}
	}
}

void UStateEstimator::MatrixTranspose(const TArray<float>& A, TArray<float>& Result, int Rows, int Cols) const
{
	for (int32 i = 0; i < Rows; ++i)
	{
		for (int32 j = 0; j < Cols; ++j)
		{
			Result[j * Rows + i] = A[i * Cols + j];
		}
	}
}

void UStateEstimator::MatrixAdd(const TArray<float>& A, const TArray<float>& B, TArray<float>& Result, int Size) const
{
	for (int32 i = 0; i < Size; ++i)
	{
		Result[i] = A[i] + B[i];
	}
}

bool UStateEstimator::MatrixInverse(const TArray<float>& A, TArray<float>& Result, int N) const
{
	// 使用高斯-约旦消元法求逆矩阵
	// 创建增广矩阵 [A | I]
	TArray<float> Aug;
	Aug.SetNum(N * N * 2);

	// 初始化增广矩阵
	for (int32 i = 0; i < N; ++i)
	{
		for (int32 j = 0; j < N; ++j)
		{
			Aug[i * (N * 2) + j] = A[i * N + j];
			Aug[i * (N * 2) + N + j] = (i == j) ? 1.0f : 0.0f;
		}
	}

	// 高斯-约旦消元
	for (int32 i = 0; i < N; ++i)
	{
		// 寻找主元
		int32 MaxRow = i;
		float MaxVal = FMath::Abs(Aug[i * (N * 2) + i]);
		for (int32 k = i + 1; k < N; ++k)
		{
			float Val = FMath::Abs(Aug[k * (N * 2) + i]);
			if (Val > MaxVal)
			{
				MaxVal = Val;
				MaxRow = k;
			}
		}

		// 检查是否奇异
		if (MaxVal < 1e-10f)
		{
			return false;
		}

		// 交换行
		if (MaxRow != i)
		{
			for (int32 j = 0; j < N * 2; ++j)
			{
				float Temp = Aug[i * (N * 2) + j];
				Aug[i * (N * 2) + j] = Aug[MaxRow * (N * 2) + j];
				Aug[MaxRow * (N * 2) + j] = Temp;
			}
		}

		// 归一化当前行
		float Pivot = Aug[i * (N * 2) + i];
		for (int32 j = 0; j < N * 2; ++j)
		{
			Aug[i * (N * 2) + j] /= Pivot;
		}

		// 消元
		for (int32 k = 0; k < N; ++k)
		{
			if (k != i)
			{
				float Factor = Aug[k * (N * 2) + i];
				for (int32 j = 0; j < N * 2; ++j)
				{
					Aug[k * (N * 2) + j] -= Factor * Aug[i * (N * 2) + j];
				}
			}
		}
	}

	// 提取逆矩阵
	for (int32 i = 0; i < N; ++i)
	{
		for (int32 j = 0; j < N; ++j)
		{
			Result[i * N + j] = Aug[i * (N * 2) + N + j];
		}
	}

	return true;
}

void UStateEstimator::ComputeJacobianF(TArray<float>& F, float DeltaTime) const
{
	// 状态转移矩阵 F (6×6)
	// 状态向量: [px, py, pz, vx, vy, vz]^T
	// 线性化模型: x[k+1] = F*x[k] + B*u[k]
	//
	// F = [I3  dt*I3]
	//     [0   I3   ]
	//
	// 其中 I3 是 3×3 单位矩阵

	F.SetNum(36);
	for (int32 i = 0; i < 36; ++i)
	{
		F[i] = 0.0f;
	}

	// 上半部分：I3 + dt*I3（位置对位置和速度的导数）
	F[0] = 1.0f;  F[1] = 0.0f;  F[2] = 0.0f;  F[3] = DeltaTime;  F[4] = 0.0f;      F[5] = 0.0f;
	F[6] = 0.0f;  F[7] = 1.0f;  F[8] = 0.0f;  F[9] = 0.0f;       F[10] = DeltaTime; F[11] = 0.0f;
	F[12] = 0.0f; F[13] = 0.0f; F[14] = 1.0f; F[15] = 0.0f;      F[16] = 0.0f;      F[17] = DeltaTime;

	// 下半部分：I3（速度对速度的导数）
	F[18] = 0.0f; F[19] = 0.0f; F[20] = 0.0f; F[21] = 1.0f; F[22] = 0.0f; F[23] = 0.0f;
	F[24] = 0.0f; F[25] = 0.0f; F[26] = 0.0f; F[27] = 0.0f; F[28] = 1.0f; F[29] = 0.0f;
	F[30] = 0.0f; F[31] = 0.0f; F[32] = 0.0f; F[33] = 0.0f; F[34] = 0.0f; F[35] = 1.0f;
}

void UStateEstimator::ComputeJacobianH(TArray<float>& H) const
{
	// 观测矩阵 H (6×6)
	// 观测向量: [px, py, pz, vx, vy, vz]^T
	// 直接观测位置和速度，所以 H = I6（单位矩阵）

	H.SetNum(36);
	for (int32 i = 0; i < 36; ++i)
	{
		H[i] = 0.0f;
	}

	// 对角元素为 1
	for (int32 i = 0; i < 6; ++i)
	{
		H[i * 6 + i] = 1.0f;
	}
}
