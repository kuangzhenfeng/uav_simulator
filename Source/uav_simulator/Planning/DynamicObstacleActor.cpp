// Copyright Epic Games, Inc. All Rights Reserved.

#include "DynamicObstacleActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "uav_simulator/Debug/UAVLogConfig.h"
#include "uav_simulator/Utility/Filter.h"

ADynamicObstacleActor::ADynamicObstacleActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	ObstacleMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ObstacleMesh"));
	SetRootComponent(ObstacleMesh);
	ObstacleMesh->SetMobility(EComponentMobility::Movable);
	ObstacleMesh->SetCollisionProfileName(TEXT("BlockAllDynamic"));
	ObstacleMesh->SetGenerateOverlapEvents(false);

}

void ADynamicObstacleActor::BeginPlay()
{
	Super::BeginPlay();
}

void ADynamicObstacleActor::Configure(const FScenarioObstacleEntry& Entry)
{
	Definition = Entry;
	const TCHAR* MeshPath = TEXT("/Engine/BasicShapes/Cube.Cube");
	FVector Size = Entry.Extents * 2.0f;
	if (Entry.Type == EObstacleType::Sphere)
	{
		MeshPath = TEXT("/Engine/BasicShapes/Sphere.Sphere");
		Size = FVector(Entry.Extents.X * 2.0f);
	}
	else if (Entry.Type == EObstacleType::Cylinder)
	{
		MeshPath = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
		Size = FVector(Entry.Extents.X * 2.0f, Entry.Extents.X * 2.0f, Entry.Extents.Z * 2.0f);
	}
	if (UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, MeshPath))
	{
		ObstacleMesh->SetStaticMesh(Mesh);
		const FVector NativeSize = Mesh->GetBoundingBox().GetSize();
		ObstacleMesh->SetRelativeScale3D(Size / NativeSize);
	}
	MovementType = Entry.MovementType;
	LinearVelocity = Entry.Velocity;
	PatrolPoints = Entry.PatrolPoints;
	PatrolSpeed = FMath::Max(0.0f, Entry.PatrolSpeed);

	// 初始位姿：巡逻类落第一个航点（若有），其余落声明中心。
	const FVector InitialLocation = ((MovementType == EObstacleMovementType::PatrolLoop || MovementType == EObstacleMovementType::PatrolPingPong) && PatrolPoints.Num() > 0) ? PatrolPoints[0] : Entry.Center;
	SetActorLocation(InitialLocation, false, nullptr, ETeleportType::ResetPhysics);
	SetActorRotation(Entry.Rotation, ETeleportType::ResetPhysics);

	// 运行时状态重置
	CurrentSegmentIndex = 0;
	SegmentDistance = 0.0f;
	bForward = true;
	FVector NormalizedPosition = InitialLocation, Velocity;
	AdvancePatrolState(0.0f, CurrentSegmentIndex, SegmentDistance, bForward, NormalizedPosition, Velocity);
	SetActorLocation(NormalizedPosition, false, nullptr, ETeleportType::ResetPhysics);

	UE_LOG(LogUAVPlanning, Log, TEXT("[DynamicObstacle] Configured: MovementType=%d, InitialLoc=%s, PatrolPoints=%d, PatrolSpeed=%.1f"),
		(int32)MovementType, *InitialLocation.ToString(), PatrolPoints.Num(), PatrolSpeed);
}

void ADynamicObstacleActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	switch (MovementType)
	{
	case EObstacleMovementType::LinearVelocity:
	{
		// 匀速直线：出界处理留给上层（本任务不销毁，避免破坏 LinkedActor 弱引用链路）。
		if (!LinearVelocity.IsNearlyZero())
		{
			const FVector Next = GetActorLocation() + LinearVelocity * DeltaTime;
			SetActorLocation(Next, false, nullptr, ETeleportType::ResetPhysics);
		}
		break;
	}
	case EObstacleMovementType::PatrolLoop:
	case EObstacleMovementType::PatrolPingPong:
		AdvanceAlongPoints(DeltaTime);
		break;
	case EObstacleMovementType::Static:
	default:
		// 静态障碍保持声明位姿。
		break;
	}
}

void ADynamicObstacleActor::AdvancePatrolState(float DeltaTime, int32& Index, float& Distance,
	bool& Forward, FVector& Position, FVector& Velocity) const
{
	const int32 Num = PatrolPoints.Num();
	Velocity = FVector::ZeroVector;
	if (Num < 2 || PatrolSpeed <= KINDA_SMALL_NUMBER) return;
	float CycleLength = 0.0f;
	for (int32 i = 1; i < Num; ++i) CycleLength += FVector::Distance(PatrolPoints[i - 1], PatrolPoints[i]);
	CycleLength = MovementType == EObstacleMovementType::PatrolLoop
		? CycleLength + FVector::Distance(PatrolPoints.Last(), PatrolPoints[0]) : CycleLength * 2.0f;
	if (CycleLength <= KINDA_SMALL_NUMBER) return;
	// 完整周期不改变状态，保留跨段残差，避免大时间步丢失运动距离。
	float Remaining = FMath::Fmod(PatrolSpeed * FMath::Max(DeltaTime, 0.0f), CycleLength);
	for (int32 Guard = 0; Guard < Num * 2 + 4; ++Guard)
	{
		if (MovementType == EObstacleMovementType::PatrolPingPong)
		{
			if (Index == Num - 1) Forward = false;
			if (Index == 0) Forward = true;
		}
		const int32 Next = MovementType == EObstacleMovementType::PatrolLoop
			? (Index + 1) % Num : Index + (Forward ? 1 : -1);
		const FVector Delta = PatrolPoints[Next] - PatrolPoints[Index];
		const float Length = Delta.Size();
		if (Length > KINDA_SMALL_NUMBER && Remaining < Length - Distance)
		{
			Distance += Remaining;
			Velocity = Delta / Length * PatrolSpeed;
			Position = PatrolPoints[Index] + Delta / Length * Distance;
			return;
		}
		Remaining = FMath::Max(0.0f, Remaining - FMath::Max(0.0f, Length - Distance));
		Index = Next;
		Distance = 0.0f;
	}
}

void ADynamicObstacleActor::AdvanceAlongPoints(float DeltaTime)
{
	FVector Position = GetActorLocation(), Velocity;
	AdvancePatrolState(DeltaTime, CurrentSegmentIndex, SegmentDistance, bForward, Position, Velocity);
	SetActorLocation(Position, false, nullptr, ETeleportType::ResetPhysics);
	UE_LOG_THROTTLE(2.0f, LogUAVPlanning, Log,
		TEXT("[DynamicObstacle] Tick: MoveType=%d, Loc=%s, SegIdx=%d, SegDist=%.1f, Forward=%d"),
		(int32)MovementType, *Position.ToString(), CurrentSegmentIndex, SegmentDistance, bForward ? 1 : 0);
}

FObstacleInfo ADynamicObstacleActor::PredictObstacleSnapshot(float SecondsAhead) const
{
	FObstacleInfo Snapshot = GetObstacleSnapshot();
	if (MovementType == EObstacleMovementType::LinearVelocity)
		Snapshot.Center += LinearVelocity * FMath::Max(SecondsAhead, 0.0f);
	else if (MovementType == EObstacleMovementType::PatrolLoop || MovementType == EObstacleMovementType::PatrolPingPong)
	{
		int32 Index = CurrentSegmentIndex;
		float Distance = SegmentDistance;
		bool Forward = bForward;
		AdvancePatrolState(SecondsAhead, Index, Distance, Forward, Snapshot.Center, Snapshot.Velocity);
	}
	return Snapshot;
}

FVector ADynamicObstacleActor::GetVelocity() const
{
	if (MovementType == EObstacleMovementType::LinearVelocity) return LinearVelocity;
	if (MovementType == EObstacleMovementType::PatrolLoop || MovementType == EObstacleMovementType::PatrolPingPong)
	{
		int32 Index = CurrentSegmentIndex;
		float Distance = SegmentDistance;
		bool Forward = bForward;
		FVector Position = GetActorLocation(), Velocity;
		AdvancePatrolState(0.0f, Index, Distance, Forward, Position, Velocity);
		return Velocity;
	}
	return FVector::ZeroVector;
}

FObstacleInfo ADynamicObstacleActor::GetObstacleSnapshot() const
{
	FObstacleInfo Snapshot;
	Snapshot.Type = Definition.Type;
	Snapshot.Center = GetActorLocation();
	Snapshot.Extents = Definition.Extents;
	Snapshot.Rotation = GetActorRotation();
	Snapshot.SafetyMargin = Definition.SafetyMargin;
	Snapshot.bIsDynamic = MovementType != EObstacleMovementType::Static;
	Snapshot.Velocity = GetVelocity();
	Snapshot.LinkedActor = const_cast<ADynamicObstacleActor*>(this);
	return Snapshot;
}
