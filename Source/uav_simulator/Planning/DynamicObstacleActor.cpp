// Copyright Epic Games, Inc. All Rights Reserved.

#include "DynamicObstacleActor.h"
#include "Components/SceneComponent.h"
#include "uav_simulator/Debug/UAVLogConfig.h"
#include "uav_simulator/Utility/Filter.h"

ADynamicObstacleActor::ADynamicObstacleActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	// 仅持有一个可移动的根组件，保证 SetActorLocation 生效；
	// 可视化壳由蓝图负责（与裸 AActor 作 LinkedActor 的方案一致）。
	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
}

void ADynamicObstacleActor::BeginPlay()
{
	Super::BeginPlay();
}

void ADynamicObstacleActor::Configure(const FScenarioObstacleEntry& Entry)
{
	MovementType = Entry.MovementType;
	LinearVelocity = Entry.Velocity;
	PatrolPoints = Entry.PatrolPoints;
	PatrolSpeed = FMath::Max(0.0f, Entry.PatrolSpeed);

	// 初始位姿：巡逻类落第一个航点（若有），其余落声明中心。
	const FVector InitialLocation = (PatrolPoints.Num() > 0) ? PatrolPoints[0] : Entry.Center;
	SetActorLocation(InitialLocation, false, nullptr, ETeleportType::ResetPhysics);
	SetActorRotation(Entry.Rotation, ETeleportType::ResetPhysics);

	// 运行时状态重置
	CurrentSegmentIndex = 0;
	SegmentDistance = 0.0f;
	bForward = true;
	bSegmentValid = false;
	RecomputeCurrentSegment();

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
		// 静态：ScenarioLoader 不应 Spawn 本 Actor，防御性无操作。
		break;
	}
}

void ADynamicObstacleActor::RecomputeCurrentSegment()
{
	bSegmentValid = false;
	const int32 Num = PatrolPoints.Num();
	if (Num < 2)
	{
		return;
	}

	int32 NextIndex;
	if (MovementType == EObstacleMovementType::PatrolLoop)
	{
		NextIndex = (CurrentSegmentIndex + 1) % Num;
	}
	else // PatrolPingPong：方向由 bForward 决定，端点翻转在 Advance 到达端点时完成。
	{
		NextIndex = bForward ? (CurrentSegmentIndex + 1) : (CurrentSegmentIndex - 1);
	}

	if (NextIndex < 0 || NextIndex >= Num)
	{
		return;
	}

	const FVector& Start = PatrolPoints[CurrentSegmentIndex];
	const FVector& End = PatrolPoints[NextIndex];
	const FVector Dir = End - Start;
	SegmentLength = Dir.Size();
	if (SegmentLength < KINDA_SMALL_NUMBER)
	{
		// 退化为零长段：标记有效但方向置零，由 Advance 立刻跳到下一段。
		SegmentDirection = FVector::ZeroVector;
		bSegmentValid = true;
		return;
	}
	SegmentDirection = Dir / SegmentLength;
	bSegmentValid = true;
}

void ADynamicObstacleActor::AdvanceAlongPoints(float DeltaTime)
{
	const int32 Num = PatrolPoints.Num();
	if (Num < 2 || PatrolSpeed <= KINDA_SMALL_NUMBER || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	if (!bSegmentValid)
	{
		RecomputeCurrentSegment();
		if (!bSegmentValid)
		{
			return;
		}
	}

	// 本帧推进距离（残差跨段累加）。
	float Remaining = PatrolSpeed * DeltaTime;

	// 多段推进：处理一帧跨过多段或连续零长段的极端情形。
	int32 SafetyGuard = Num * 2 + 4;
	while (Remaining > KINDA_SMALL_NUMBER && SafetyGuard-- > 0)
	{
		if (SegmentLength < KINDA_SMALL_NUMBER)
		{
			// 零长段：不消耗距离，直接切到下一段。
		}
		else
		{
			const float LeftOnSegment = SegmentLength - SegmentDistance;
			if (Remaining < LeftOnSegment)
			{
				SegmentDistance += Remaining;
				Remaining = 0.0f;
				break;
			}
			// 走完当前段，消耗对应距离。
			Remaining -= LeftOnSegment;
			SegmentDistance = SegmentLength;
		}

		// 段切换：新起点 = 当前段终点索引。
		if (MovementType == EObstacleMovementType::PatrolLoop)
		{
			CurrentSegmentIndex = (CurrentSegmentIndex + 1) % Num;
		}
		else // PatrolPingPong
		{
			// 先把起点推进到当前段终点。
			CurrentSegmentIndex = bForward ? (CurrentSegmentIndex + 1) : (CurrentSegmentIndex - 1);
			// 抵达端点（末位/首位）时翻转方向，使下一段沿相反方向返回。
			if (bForward && CurrentSegmentIndex == Num - 1)
			{
				bForward = false;
			}
			else if (!bForward && CurrentSegmentIndex == 0)
			{
				bForward = true;
			}
		}

		SegmentDistance = 0.0f;
		RecomputeCurrentSegment();
		if (!bSegmentValid)
		{
			break;
		}
	}

	// 位置 = 当前段起点 + 归一化方向 * 已走距离。
	if (bSegmentValid && SegmentLength >= KINDA_SMALL_NUMBER)
	{
		const FVector Start = PatrolPoints[CurrentSegmentIndex];
		const FVector NextLocation = Start + SegmentDirection * SegmentDistance;
		SetActorLocation(NextLocation, false, nullptr, ETeleportType::ResetPhysics);
	}

	UE_LOG_THROTTLE(2.0f, LogUAVPlanning, Log,
		TEXT("[DynamicObstacle] Tick: MoveType=%d, Loc=%s, SegIdx=%d, SegDist=%.1f, Forward=%d"),
		(int32)MovementType, *GetActorLocation().ToString(), CurrentSegmentIndex, SegmentDistance, bForward ? 1 : 0);
}
