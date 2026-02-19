// Copyright Epic Games, Inc. All Rights Reserved.

#include "StabilityScorer.h"
#include "../Planning/ObstacleManager.h"
#include "DrawDebugHelpers.h"

UStabilityScorer::UStabilityScorer()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UStabilityScorer::UpdateFlightScore(const FUAVState& State, UObstacleManager* ObstacleManager)
{
	// 1. 姿态评分: 惩罚过大的 Roll/Pitch
	float Tilt = FMath::Abs(State.Rotation.Roll) + FMath::Abs(State.Rotation.Pitch);
	float RawAttitude = FMath::Clamp(100.f - Tilt / MaxTiltDeg * 100.f, 0.f, 100.f);

	// 2. 角速度评分: 惩罚高角速率
	float RawAngVel = FMath::Clamp(100.f - State.AngularVelocity.Size() / MaxAngVelMag * 100.f, 0.f, 100.f);

	// 3. 速度平滑度评分: 惩罚帧间速度突变
	float RawVelSmooth = 100.f;
	if (bHasPrevVelocity)
	{
		float VelChange = (State.Velocity - PrevVelocity).Size();
		RawVelSmooth = FMath::Clamp(100.f - VelChange / MaxVelChange * 100.f, 0.f, 100.f);
	}
	PrevVelocity = State.Velocity;
	bHasPrevVelocity = true;

	// 4. 障碍物距离评分
	float RawObsDist = 100.f;
	if (ObstacleManager && ObstacleManager->GetAllObstacles().Num() > 0)
	{
		FObstacleInfo Nearest;
		float Dist = ObstacleManager->GetDistanceToNearestObstacle(State.Position, Nearest);
		RawObsDist = Dist <= 0.f ? 0.f : FMath::Clamp(Dist / SafeDistance * 100.f, 0.f, 100.f);
	}

	// EMA 平滑
	Score.AttitudeScore = EMA(Score.AttitudeScore, RawAttitude);
	Score.AngularVelocityScore = EMA(Score.AngularVelocityScore, RawAngVel);
	Score.VelocitySmoothnessScore= EMA(Score.VelocitySmoothnessScore,RawVelSmooth);
	Score.ObstacleDistanceScore = EMA(Score.ObstacleDistanceScore, RawObsDist);

	Score.FlightScore = (Score.AttitudeScore + Score.AngularVelocityScore + Score.VelocitySmoothnessScore) / 3.f;
	Score.AvoidanceScore = (Score.ObstacleDistanceScore + Score.AvoidanceSmoothnessScore) / 2.f;
	Score.CompositeScore = Score.FlightScore * 0.5f + Score.AvoidanceScore * 0.5f;
}

void UStabilityScorer::UpdateAvoidanceScore(const FNMPCAvoidanceResult& AvoidanceResult)
{
	float RawSmooth = 100.f;
	if (AvoidanceResult.bStuck)
	{
		RawSmooth = 0.f;
	}
	else if (AvoidanceResult.bNeedsCorrection)
	{
		RawSmooth = FMath::Clamp(100.f - AvoidanceResult.RepulsiveForce.Size() / MaxRepulsiveMag * 100.f, 0.f, 100.f);
	}

	Score.AvoidanceSmoothnessScore = EMA(Score.AvoidanceSmoothnessScore, RawSmooth);
	Score.AvoidanceScore = (Score.ObstacleDistanceScore + Score.AvoidanceSmoothnessScore) / 2.f;
	Score.CompositeScore = Score.FlightScore * 0.5f + Score.AvoidanceScore * 0.5f;
}

void UStabilityScorer::DrawScoreHUD(const FVector& WorldLocation) const
{
	UWorld* World = GetWorld();
	if (!World) return;

	const FVector Base = WorldLocation + FVector(0.f, 0.f, 260.f);
	const float Step = 30.f;

	auto Draw = [&](const FString& Text, float S, int32 Line)
	{
		DrawDebugString(World, Base + FVector(0.f, 0.f, Line * Step), Text, nullptr, ToColor(S), 0.f, true, 1.1f);
	};

	Draw(FString::Printf(TEXT("■ 综合: %.1f"), Score.CompositeScore), Score.CompositeScore, 0);
	Draw(FString::Printf(TEXT("  飞行: %.1f  [姿态:%.0f  角速:%.0f  平滑:%.0f]"),
		Score.FlightScore, Score.AttitudeScore, Score.AngularVelocityScore, Score.VelocitySmoothnessScore),
		Score.FlightScore, -1);
	Draw(FString::Printf(TEXT("  避障: %.1f  [距离:%.0f  修正:%.0f]"),
		Score.AvoidanceScore, Score.ObstacleDistanceScore, Score.AvoidanceSmoothnessScore),
		Score.AvoidanceScore, -2);
}
