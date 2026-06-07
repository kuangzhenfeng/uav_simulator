// Copyright Epic Games, Inc. All Rights Reserved.

#include "WindField.h"
#include "../uav_simulator.h"
#include "../Debug/UAVLogConfig.h"

// 环境日志类别（仅在 WindField 中使用时局部声明）
DEFINE_LOG_CATEGORY_STATIC(LogUAVWind, Log, All);

UWindField::UWindField()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UWindField::BeginPlay()
{
	Super::BeginPlay();

	// 初始化阵风定时器
	NextGustTime = 60.0f / FMath::Max(Config.GustFrequency, 0.1f);

	// 初始化 Dryden 滤波器时间常数
	if (Config.WindType == EWindFieldType::Turbulent)
	{
		// Dryden 滤波器时间常数: τ = L / V_ref
		// 使用一个参考速度（默认 500 cm/s ≈ 5 m/s）
		const float RefSpeed = FMath::Max(Config.SteadyWindVelocity.Size(), 100.0f);
		const float LengthScaleM = Config.TurbulenceLengthScale * 0.01f; // cm → m
		const float RefSpeedM = RefSpeed * 0.01f; // cm/s → m/s

		DrydenTauX = LengthScaleM / FMath::Max(RefSpeedM, 0.5f);
		DrydenTauY = LengthScaleM / FMath::Max(RefSpeedM, 0.5f) * 2.0f;
		DrydenTauZ = LengthScaleM / FMath::Max(RefSpeedM, 0.5f) * 2.0f;
	}

	UE_LOG(LogUAVWind, Log, TEXT("[WindField] Initialized: type=%d steady=(%d,%d,%d)"),
		static_cast<int32>(Config.WindType),
		(int32)Config.SteadyWindVelocity.X, (int32)Config.SteadyWindVelocity.Y,
		(int32)Config.SteadyWindVelocity.Z);
}

void UWindField::SetWindConfig(const FWindConfig& InConfig)
{
	Config = InConfig;

	// 重新计算 Dryden 参数
	const float RefSpeed = FMath::Max(Config.SteadyWindVelocity.Size(), 100.0f);
	const float LengthScaleM = Config.TurbulenceLengthScale * 0.01f;
	const float RefSpeedM = RefSpeed * 0.01f;

	DrydenTauX = LengthScaleM / FMath::Max(RefSpeedM, 0.5f);
	DrydenTauY = LengthScaleM / FMath::Max(RefSpeedM, 0.5f) * 2.0f;
	DrydenTauZ = LengthScaleM / FMath::Max(RefSpeedM, 0.5f) * 2.0f;

	// 立即更新稳态风速，不依赖 TickComponent
	if (Config.bEnabled)
	{
		CurrentState.SteadyComponent = Config.SteadyWindVelocity;
		CurrentState.WindVelocity = CurrentState.SteadyComponent
			+ CurrentState.GustComponent + CurrentState.TurbulenceComponent;
		CurrentState.WindSpeed = CurrentState.WindVelocity.Size();
	}
	else
	{
		CurrentState.WindVelocity = FVector::ZeroVector;
		CurrentState.WindSpeed = 0.0f;
	}

	UE_LOG(LogUAVWind, Log, TEXT("[WindField] Config updated: type=%d"), static_cast<int32>(Config.WindType));
}

void UWindField::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	SCOPE_CYCLE_COUNTER(STAT_WindField);

	if (!Config.bEnabled)
	{
		CurrentState.WindVelocity = FVector::ZeroVector;
		return;
	}

	// 更新各风分量
	CurrentState.SteadyComponent = Config.SteadyWindVelocity;

	if (Config.WindType == EWindFieldType::Gust || Config.WindType == EWindFieldType::Turbulent)
	{
		UpdateGustModel(DeltaTime);
	}

	if (Config.WindType == EWindFieldType::Turbulent)
	{
		UpdateTurbulenceModel(DeltaTime);
	}
	else
	{
		CurrentState.TurbulenceComponent = FVector::ZeroVector;
	}

	if (Config.WindType == EWindFieldType::None)
	{
		CurrentState.SteadyComponent = FVector::ZeroVector;
		CurrentState.GustComponent = FVector::ZeroVector;
		CurrentState.TurbulenceComponent = FVector::ZeroVector;
	}

	// 合成总风速
	CurrentState.WindVelocity = CurrentState.SteadyComponent
		+ CurrentState.GustComponent
		+ CurrentState.TurbulenceComponent;

	// 计算风速标量和风向
	CurrentState.WindSpeed = CurrentState.WindVelocity.Size();
	if (CurrentState.WindSpeed > KINDA_SMALL_NUMBER)
	{
		// 风向: 使用 XY 平面投影计算，0 度 = X 正方向（北）
		CurrentState.WindDirection = FMath::Atan2(CurrentState.WindVelocity.Y, CurrentState.WindVelocity.X) * (180.0f / PI);
		if (CurrentState.WindDirection < 0.0f)
		{
			CurrentState.WindDirection += 360.0f;
		}
	}
}

void UWindField::UpdateGustModel(float DeltaTime)
{
	if (!bGustActive)
	{
		GustTimer += DeltaTime;
		if (GustTimer >= NextGustTime)
		{
			// 触发新阵风
			bGustActive = true;
			GustTimer = 0.0f;

			// 随机阵风方向和幅度
			float Angle = FMath::FRandRange(0.0f, 2.0f * PI);
			float Amplitude = FMath::FRandRange(Config.GustAmplitude * 0.3f, Config.GustAmplitude);
			TargetGustVelocity = FVector(
				FMath::Cos(Angle) * Amplitude,
				FMath::Sin(Angle) * Amplitude,
				FMath::FRandRange(-Config.GustAmplitude * 0.2f, Config.GustAmplitude * 0.2f));
		}
	}
	else
	{
		// 平滑插值到目标阵风速度
		float Alpha = FMath::Clamp(DeltaTime / FMath::Max(Config.GustDuration * 0.3f, 0.1f), 0.0f, 1.0f);
		CurrentGustVelocity = FMath::Lerp(CurrentGustVelocity, TargetGustVelocity, Alpha);

		GustTimer += DeltaTime;
		if (GustTimer >= Config.GustDuration)
		{
			// 阵风结束
			bGustActive = false;
			GustTimer = 0.0f;
			TargetGustVelocity = FVector::ZeroVector;
			NextGustTime = 60.0f / FMath::Max(Config.GustFrequency, 0.1f);
			NextGustTime *= FMath::FRandRange(0.7f, 1.3f); // 随机化
		}
	}

	// 阵风衰减（非活跃时逐渐消失）
	if (!bGustActive)
	{
		CurrentGustVelocity *= FMath::Exp(-DeltaTime * 2.0f);
	}

	CurrentState.GustComponent = CurrentGustVelocity;
}

void UWindField::UpdateTurbulenceModel(float DeltaTime)
{
	// 生成高斯白噪声
	FVector WhiteNoise = GenerateWhiteNoise();

	// Dryden 滤波：白噪声 → 时间相关湍流
	CurrentState.TurbulenceComponent = DrydenFilter(WhiteNoise, DeltaTime);

	// 缩放到配置的湍流强度
	CurrentState.TurbulenceComponent *= Config.TurbulenceIntensity;
}

FVector UWindField::DrydenFilter(const FVector& WhiteNoise, float DeltaTime)
{
	// Dryden 湍流模型：一阶低通滤波器
	// x(k+1) = x(k) * exp(-dt/τ) + σ * sqrt(1 - exp(-2*dt/τ)) * white_noise
	// 三轴独立处理

	auto FilterAxis = [](float State, float Noise, float Tau, float Dt) -> float
	{
		if (Tau < KINDA_SMALL_NUMBER)
		{
			return Noise;
		}
		float Decay = FMath::Exp(-Dt / Tau);
		float Sigma = FMath::Sqrt(1.0f - Decay * Decay);
		return State * Decay + Sigma * Noise;
	};

	FVector Result;
	Result.X = FilterAxis(DrydenStateX.X, WhiteNoise.X, DrydenTauX, DeltaTime);
	Result.Y = FilterAxis(DrydenStateX.Y, WhiteNoise.Y, DrydenTauY, DeltaTime);
	Result.Z = FilterAxis(DrydenStateX.Z, WhiteNoise.Z, DrydenTauZ, DeltaTime);

	DrydenStateX = Result;

	return Result;
}

FVector UWindField::GetWindAtPosition(const FVector& WorldPosition) const
{
	// 当前实现为均匀风场，所有位置风速相同
	// 后续可扩展为空间变化风场（如对数风廓线）
	return CurrentState.WindVelocity;
}

FVector UWindField::ComputeWindDragAcceleration(
	const FVector& UAVVelocity,
	const FVector& UAVPosition,
	float UAVMass) const
{
	if (!Config.bEnabled || UAVMass < KINDA_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}

	// 获取该位置的风速
	FVector WindVelocity = GetWindAtPosition(UAVPosition);

	// 相对风速 = 风速 - UAV速度
	FVector RelativeWind = WindVelocity - UAVVelocity;
	float RelSpeed = RelativeWind.Size();

	if (RelSpeed < KINDA_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}

	// 风阻力加速度 (m/s² → cm/s²)
	// F_drag = 0.5 * ρ * Cd * A * |v_rel|² * v_rel_dir
	// a_drag = F_drag / m
	// 单位注意: WindVelocity 和 UAVVelocity 是 cm/s，需转换为 m/s 计算
	float RelSpeedM = RelSpeed * 0.01f; // cm/s → m/s
	FVector RelWindDir = RelativeWind / RelSpeed;

	float DragForceMag = 0.5f * Config.AirDensity * Config.DragCoefficient
		* Config.DragArea * RelSpeedM * RelSpeedM; // N

	// 转换为 UE5 加速度 (cm/s²)
	float DragAccel = (DragForceMag / UAVMass) * 100.0f; // m/s² → cm/s²

	return RelWindDir * DragAccel;
}

FVector UWindField::GenerateWhiteNoise()
{
	return FVector(GaussianRandom(), GaussianRandom(), GaussianRandom());
}

float UWindField::GaussianRandom()
{
	// Box-Muller 变换生成标准正态分布
	static bool bHasSpare = false;
	static float Spare = 0.0f;

	if (bHasSpare)
	{
		bHasSpare = false;
		return Spare;
	}

	float U1, U2;
	do
	{
		U1 = FMath::FRand();
	} while (U1 < KINDA_SMALL_NUMBER);
	U2 = FMath::FRand();

	float Radius = FMath::Sqrt(-2.0f * FMath::Loge(U1));
	float Theta = 2.0f * PI * U2;

	Spare = Radius * FMath::Sin(Theta);
	bHasSpare = true;

	return Radius * FMath::Cos(Theta);
}
