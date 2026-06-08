// Copyright Epic Games, Inc. All Rights Reserved.

#include "ObstacleManager.h"
#include "../uav_simulator.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "Logging/LogVerbosity.h"
#include "uav_simulator/Debug/UAVLogConfig.h"
#include "uav_simulator/Utility/Filter.h"
#include "../Core/UAVPawn.h"

UObstacleManager::UObstacleManager()
{
	PrimaryComponentTick.bCanEverTick = true;
	NextObstacleID = 1;
	DefaultSafetyMargin = 50.0f;
	bAutoUpdateDynamicObstacles = true;
	CollisionWarningDistance = 200.0f;
	bShowDebug = false;
}
void UObstacleManager::OnRegister()
{
	Super::OnRegister();

	if (bAutoDiscoverNamedStaticObstacles)
	{
		DiscoverNamedStaticObstacles();
	}
}

void UObstacleManager::BeginPlay()
{
	Super::BeginPlay();

	if (bAutoDiscoverNamedStaticObstacles)
	{
		DiscoverNamedStaticObstacles();
	}
}

void UObstacleManager::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	SCOPE_CYCLE_COUNTER(STAT_ObstacleManager);

	if (bAutoUpdateDynamicObstacles)
	{
		UpdateDynamicObstacles(DeltaTime);
	}

	// 自动清理过期的感知障碍物
	if (bAutoRemoveStalePerceived)
	{
		RemoveStalePerceivedObstacles(PerceivedObstacleMaxAge);
	}

	if (bShowDebug)
	{
		DrawDebugObstacles();
	}
}
int32 UObstacleManager::RegisterObstacle(const FObstacleInfo& Obstacle)
{
	FObstacleInfo NewObstacle = Obstacle;
	NewObstacle.ObstacleID = NextObstacleID++;

	Obstacles.Add(NewObstacle);
	OnObstacleDetected.Broadcast(NewObstacle);

	UE_LOG(LogUAVPlanning, Log, TEXT("Obstacle registered: ID=%d, Type=%d, Center=%s, Extents=%s, Actor=%s, Total=%d"),
		NewObstacle.ObstacleID,
		(int32)NewObstacle.Type,
		*NewObstacle.Center.ToString(),
		*NewObstacle.Extents.ToString(),
		NewObstacle.LinkedActor.IsValid() ? *NewObstacle.LinkedActor->GetName() : TEXT("none"),
		Obstacles.Num());

	return NewObstacle.ObstacleID;
}

int32 UObstacleManager::RegisterObstacleFromActor(AActor* Actor, EObstacleType Type, float SafetyMargin)
{
	if (!Actor)
	{
		return -1;
	}
	if (Actor == GetOwner())
	{
		return -1;
	}

	// UAV 特殊处理：GetActorBounds 包含 CameraBoom 等子组件导致包围盒过大
	const AUAVPawn* UAVActor = Cast<AUAVPawn>(Actor);
	FVector Origin, BoxExtent;
	float CollisionRadius = 0.0f;

	if (UAVActor)
	{
		CollisionRadius = UAVActor->GetCollisionRadius();
		Origin = Actor->GetActorLocation();
		BoxExtent = FVector(CollisionRadius);
	}
	else
	{
		FVector UnusedOrigin;
		Actor->GetActorBounds(false, UnusedOrigin, BoxExtent);
		Origin = Actor->GetActorLocation();
	}

	FObstacleInfo Obstacle;
	Obstacle.Type = Type;
	Obstacle.Center = Origin;
	Obstacle.Rotation = Actor->GetActorRotation();
	Obstacle.SafetyMargin = SafetyMargin;
	Obstacle.LinkedActor = Actor;
	Obstacle.Extents = ComputeObstacleExtentsFromBounds(Type, BoxExtent);

	return RegisterObstacle(Obstacle);
}

FVector UObstacleManager::ComputeObstacleExtentsFromBounds(EObstacleType Type, const FVector& BoxExtent) const
{
	switch (Type)
	{
	case EObstacleType::Sphere:
		return FVector(BoxExtent.GetMax());
	case EObstacleType::Box:
		return BoxExtent;
	case EObstacleType::Cylinder:
		return FVector(FMath::Max(BoxExtent.X, BoxExtent.Y), 0.0f, BoxExtent.Z);
	default:
		return BoxExtent;
	}
}

void UObstacleManager::DiscoverNamedStaticObstacles()
{
	UWorld* World = GetWorld();
	if (!World || StaticObstacleNamePrefix.IsEmpty())
	{
		return;
	}

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!IsNamedStaticObstacleActor(Actor))
		{
			continue;
		}

		bool bAlreadyRegistered = false;
		for (const FObstacleInfo& Obstacle : Obstacles)
		{
			if (Obstacle.LinkedActor.Get() == Actor)
			{
				bAlreadyRegistered = true;
				break;
			}
		}

		if (!bAlreadyRegistered)
		{
			RegisterObstacleFromActor(Actor, EObstacleType::Box, DefaultSafetyMargin);
		}
	}
}

bool UObstacleManager::IsNamedStaticObstacleActor(const AActor* Actor) const
{
	if (!Actor || Actor == GetOwner())
	{
		return false;
	}

	return Actor->GetName().StartsWith(StaticObstacleNamePrefix);
}

bool UObstacleManager::RemoveObstacle(int32 ObstacleID)
{
	for (int32 i = 0; i < Obstacles.Num(); ++i)
	{
		if (Obstacles[i].ObstacleID == ObstacleID)
		{
			UE_LOG(LogUAVPlanning, Log, TEXT("Obstacle removed: ID=%d, Total=%d"), ObstacleID, Obstacles.Num() - 1);
			Obstacles.RemoveAt(i);
			return true;
		}
	}
	return false;
}

void UObstacleManager::ClearAllObstacles()
{
	Obstacles.Empty();
}

bool UObstacleManager::GetObstacle(int32 ObstacleID, FObstacleInfo& OutObstacle) const
{
	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		if (Obstacle.ObstacleID == ObstacleID)
		{
			OutObstacle = Obstacle;
			return true;
		}
	}
	return false;
}

bool UObstacleManager::CheckCollision(const FVector& Point, float Radius) const
{
	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		if (IsPointInObstacle(Point, Obstacle, Radius))
		{
			UE_LOG(LogUAVPlanning, Warning, TEXT("[ObstacleManager] Collision detected: Point=%s, Radius=%.1f, ObstacleID=%d, ObstacleCenter=%s"),
				*Point.ToString(), Radius, Obstacle.ObstacleID, *Obstacle.Center.ToString());
			return true;
		}
	}
	return false;
}

bool UObstacleManager::CheckLineCollision(const FVector& Start, const FVector& End, float Radius) const
{
	FVector Direction = End - Start;
	float Distance = Direction.Size();

	if (Distance < KINDA_SMALL_NUMBER)
	{
		return CheckCollision(Start, Radius);
	}

	Direction.Normalize();

	// 沿线段采样
	float StepSize = FMath::Min(50.0f, Distance * 0.1f);
	int32 NumSteps = FMath::CeilToInt(Distance / StepSize);

	for (int32 i = 0; i <= NumSteps; ++i)
	{
		float t = (float)i / (float)NumSteps;
		FVector SamplePoint = FMath::Lerp(Start, End, t);

		if (CheckCollision(SamplePoint, Radius))
		{
			return true;
		}
	}

	return false;
}

float UObstacleManager::GetDistanceToNearestObstacle(const FVector& Point, FObstacleInfo& OutNearestObstacle) const
{
	float MinDistance = FLT_MAX;
	int32 NearestIndex = -1;

	for (int32 i = 0; i < Obstacles.Num(); ++i)
	{
		float Distance = CalculateDistanceToObstacle(Point, Obstacles[i]);
		if (Distance < MinDistance)
		{
			MinDistance = Distance;
			NearestIndex = i;
		}
	}

	if (NearestIndex >= 0)
	{
		OutNearestObstacle = Obstacles[NearestIndex];
	}

	return MinDistance;
}

TArray<FObstacleInfo> UObstacleManager::GetObstaclesInRange(const FVector& Center, float Radius) const
{
	TArray<FObstacleInfo> Result;

	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		float Distance = CalculateDistanceToObstacle(Center, Obstacle);
		if (Distance <= Radius)
		{
			Result.Add(Obstacle);
		}
	}

	return Result;
}

void UObstacleManager::ScanForObstacles(const FVector& ScanCenter, float ScanRadius, const TArray<TSubclassOf<AActor>>& ActorClasses)
{
	if (!GetWorld())
	{
		return;
	}

	for (const TSubclassOf<AActor>& ActorClass : ActorClasses)
	{
		for (TActorIterator<AActor> It(GetWorld(), ActorClass); It; ++It)
		{
			AActor* Actor = *It;
			if (Actor && Actor != GetOwner())
			{
				float Distance = FVector::Dist(Actor->GetActorLocation(), ScanCenter);
				if (Distance <= ScanRadius)
				{
					// 检查是否已经注册
					bool bAlreadyRegistered = false;
					for (const FObstacleInfo& Obstacle : Obstacles)
					{
						if (Obstacle.LinkedActor.Get() == Actor)
						{
							bAlreadyRegistered = true;
							break;
						}
					}

					if (!bAlreadyRegistered)
					{
						RegisterObstacleFromActor(Actor, EObstacleType::Box, DefaultSafetyMargin);
					}
				}
			}
		}
	}
}

void UObstacleManager::UpdateDynamicObstacles(float DeltaTime)
{
	for (FObstacleInfo& Obstacle : Obstacles)
	{
		if (Obstacle.bIsDynamic && Obstacle.LinkedActor.IsValid())
		{
			AActor* Actor = Obstacle.LinkedActor.Get();
			FVector Origin, BoxExtent;

			// UAV 特殊处理
			const AUAVPawn* UAVActor = Cast<AUAVPawn>(Actor);
			if (UAVActor)
			{
				BoxExtent = FVector(UAVActor->GetCollisionRadius());
				Origin = Actor->GetActorLocation();
			}
			else
			{
				FVector UnusedOrigin;
				Actor->GetActorBounds(false, UnusedOrigin, BoxExtent);
				Origin = Actor->GetActorLocation();
			}

			const FVector PreviousCenter = Obstacle.Center;
			Obstacle.Center = Origin;
			Obstacle.Rotation = Actor->GetActorRotation();
			Obstacle.Extents = ComputeObstacleExtentsFromBounds(Obstacle.Type, BoxExtent);

			if (DeltaTime > KINDA_SMALL_NUMBER)
			{
				Obstacle.Velocity = (Obstacle.Center - PreviousCenter) / DeltaTime;
			}
			else
			{
				Obstacle.Velocity = Actor->GetVelocity();
			}
		}
	}
}

// ========== 感知障碍物管理 ==========

int32 UObstacleManager::RegisterPerceivedObstacle(const FObstacleInfo& Obstacle)
{
	FObstacleInfo NewObstacle = Obstacle;
	NewObstacle.bIsPerceived = true;
	NewObstacle.LastPerceivedTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	return RegisterObstacle(NewObstacle);
}

// ========== 障碍物更新辅助方法 ==========

void UObstacleManager::UpdateObstacleVelocity(
	FObstacleInfo& Obstacle,
	const FVector& PrevCenter,
	float DeltaTime,
	AActor* Actor)
{
	if (DeltaTime > KINDA_SMALL_NUMBER)
	{
		Obstacle.Velocity = (Obstacle.Center - PrevCenter) / DeltaTime;
	}
	else
	{
		Obstacle.Velocity = Actor->GetVelocity();
	}
}

bool UObstacleManager::TryUpdateExistingObstacle(
	AActor* Actor,
	EObstacleType Type,
	const FVector& Origin,
	const FVector& BoxExtent,
	float SafetyMargin,
	float CurrentTime)
{
	for (FObstacleInfo& Obstacle : Obstacles)
	{
		if (Obstacle.LinkedActor.Get() == Actor)
		{
			const FVector PrevCenter = Obstacle.Center;
			const float PrevTime = Obstacle.LastPerceivedTime;
			const float DeltaTime = CurrentTime - PrevTime;

			Obstacle.Type = Type;
			Obstacle.Center = Origin;
			Obstacle.Rotation = Actor->GetActorRotation();
			Obstacle.Extents = ComputeObstacleExtentsFromBounds(Type, BoxExtent);
			Obstacle.SafetyMargin = SafetyMargin;
			Obstacle.bIsPerceived = true;
			Obstacle.bIsDynamic = true;
			Obstacle.LastPerceivedTime = CurrentTime;

			UpdateObstacleVelocity(Obstacle, PrevCenter, DeltaTime, Actor);

			UE_LOG(LogUAVPlanning, Log, TEXT("[ObstacleManager] Refreshed perceived obstacle: ID=%d, Center=%s, Extents=%s, Velocity=%s"),
				Obstacle.ObstacleID, *Obstacle.Center.ToString(), *Obstacle.Extents.ToString(), *Obstacle.Velocity.ToString());
			return true;
		}
	}
	return false;
}

int32 UObstacleManager::RegisterPerceivedObstacleFromActor(AActor* Actor, EObstacleType Type, float SafetyMargin)
{
	// Early return: 无效Actor
	if (!Actor || Actor == GetOwner())
	{
		return -1;
	}

	UE_LOG_THROTTLE(5.0, LogUAVPlanning, Log, TEXT("[ObstacleManager] Registering perceived obstacle from actor: %s"), *Actor->GetName());

	// UAV 特殊处理：GetActorBounds 包含子组件导致包围盒过大
	const AUAVPawn* UAVActor = Cast<AUAVPawn>(Actor);
	FVector Origin, BoxExtent;
	if (UAVActor)
	{
		BoxExtent = FVector(UAVActor->GetCollisionRadius());
		Origin = Actor->GetActorLocation();
	}
	else
	{
		// 使用 GetActorBounds 获取包围盒尺寸，但中心位置用 GetActorLocation
		// GetActorBounds 的 Origin 是包围盒几何中心，可能因子组件偏移而与 Actor 位置不同
		FVector UnusedOrigin;
		Actor->GetActorBounds(false, UnusedOrigin, BoxExtent);
		Origin = Actor->GetActorLocation();
	}
	const float CurrentTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;

	// 尝试更新已存在的障碍物：
	// 优先按 LinkedActor 匹配；若 Actor 指针失效，则按中心距离匹配（同一物理物体）
	constexpr float MergeDistSq = 200.0f * 200.0f; // 200cm 内视为同一障碍物
	for (FObstacleInfo& Obstacle : Obstacles)
	{
		bool bSameActor = false;

		if (Obstacle.LinkedActor.IsValid() && Obstacle.LinkedActor.Get() == Actor)
		{
			bSameActor = true;
		}
		else if (Obstacle.bIsPerceived && FVector::DistSquared(Obstacle.Center, Origin) < MergeDistSq)
		{
			// Actor 指针失效或不同，但空间位置重合 — 同一物理障碍物
			bSameActor = true;
		}

		if (bSameActor)
		{
			const FVector PrevCenter = Obstacle.Center;
			const float PrevTime = Obstacle.LastPerceivedTime;
			const float DeltaTime = CurrentTime - PrevTime;

			Obstacle.Type = Type;
			Obstacle.Center = Origin;
			Obstacle.Rotation = Actor->GetActorRotation();
			Obstacle.Extents = ComputeObstacleExtentsFromBounds(Type, BoxExtent);
			Obstacle.SafetyMargin = SafetyMargin;
			Obstacle.LinkedActor = Actor; // 刷新指针
			Obstacle.bIsPerceived = true;
			Obstacle.bIsDynamic = true;
			Obstacle.LastPerceivedTime = CurrentTime;

			UpdateObstacleVelocity(Obstacle, PrevCenter, DeltaTime, Actor);

			return Obstacle.ObstacleID;
		}
	}

	// 添加新障碍物
	FObstacleInfo Obstacle;
	Obstacle.Type = Type;
	Obstacle.Center = Origin;
	Obstacle.Rotation = Actor->GetActorRotation();
	Obstacle.SafetyMargin = SafetyMargin;
	Obstacle.LinkedActor = Actor;
	Obstacle.bIsPerceived = true;
	Obstacle.bIsDynamic = true;
	Obstacle.Velocity = Actor->GetVelocity();
	Obstacle.Extents = ComputeObstacleExtentsFromBounds(Type, BoxExtent);

	// 输出调试日志：Actor边界信息
	UE_LOG_THROTTLE(5.0, LogUAVPlanning, Log, TEXT("[ObstacleManager] Actor Bounds: Origin=%s, Extent=%s"),
		*Origin.ToString(), *BoxExtent.ToString());

	return RegisterPerceivedObstacle(Obstacle);
}

// 刷新已感知障碍物的最后感知时间，防止被误判为过期
void UObstacleManager::RefreshPerceivedObstacle(int32 ObstacleID)
{
	for (FObstacleInfo& Obstacle : Obstacles)
	{
		if (Obstacle.ObstacleID == ObstacleID && Obstacle.bIsPerceived)
		{
			Obstacle.LastPerceivedTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
			return;
		}
	}
}

// 移除超过 MaxAge 秒未被刷新的感知障碍物
int32 UObstacleManager::RemoveStalePerceivedObstacles(float MaxAge)
{
	float CurrentTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
	int32 RemovedCount = 0;

	for (int32 i = Obstacles.Num() - 1; i >= 0; --i)
	{
		if (Obstacles[i].bIsPerceived && (CurrentTime - Obstacles[i].LastPerceivedTime) > MaxAge)
		{
			UE_LOG_THROTTLE(5.0, LogUAVPlanning, Log, TEXT("[ObstacleManager] Removing stale perceived obstacle: ID=%d, Age=%.1fs"),
				Obstacles[i].ObstacleID, CurrentTime - Obstacles[i].LastPerceivedTime);
			Obstacles.RemoveAt(i);
			RemovedCount++;
		}
	}

	if (RemovedCount > 0)
	{
		UE_LOG(LogUAVPlanning, Log, TEXT("[ObstacleManager] Removed %d stale obstacles, remaining=%d"),
			RemovedCount, Obstacles.Num());
	}

	return RemovedCount;
}

// 返回所有通过感知器动态检测到的障碍物
TArray<FObstacleInfo> UObstacleManager::GetPerceivedObstacles() const
{
	TArray<FObstacleInfo> Result;
	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		if (Obstacle.bIsPerceived)
		{
			Result.Add(Obstacle);
		}
	}
	return Result;
}

// 返回所有预注册的静态障碍物（非感知器检测）
TArray<FObstacleInfo> UObstacleManager::GetPreregisteredObstacles() const
{
	TArray<FObstacleInfo> Result;
	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		if (!Obstacle.bIsPerceived)
		{
			Result.Add(Obstacle);
		}
	}
	return Result;
}

// 计算点到障碍物表面的有符号距离（负值 = 穿透），已减去 SafetyMargin
float UObstacleManager::CalculateDistanceToObstacle(const FVector& Point, const FObstacleInfo& Obstacle) const
{
	switch (Obstacle.Type)
	{
	case EObstacleType::Sphere:
		// 球体：中心距离 - 半径 - 安全裕度
		return FVector::Dist(Point, Obstacle.Center) - Obstacle.Extents.X - Obstacle.SafetyMargin;

	case EObstacleType::Box:
		{
			// 盒体：转到局部坐标系，找最近点，计算距离
			FVector LocalPoint = Obstacle.Rotation.UnrotateVector(Point - Obstacle.Center);
			FVector ClosestPoint;
			ClosestPoint.X = FMath::Clamp(LocalPoint.X, -Obstacle.Extents.X, Obstacle.Extents.X);
			ClosestPoint.Y = FMath::Clamp(LocalPoint.Y, -Obstacle.Extents.Y, Obstacle.Extents.Y);
			ClosestPoint.Z = FMath::Clamp(LocalPoint.Z, -Obstacle.Extents.Z, Obstacle.Extents.Z);

			float Distance = FVector::Dist(LocalPoint, ClosestPoint);
			return Distance - Obstacle.SafetyMargin;
		}

	case EObstacleType::Cylinder:
		{
			// 圆柱体：分别计算水平径向和垂向距离
			FVector LocalPoint = Point - Obstacle.Center;
			float HorizontalDist = FVector2D(LocalPoint.X, LocalPoint.Y).Size();
			float VerticalDist = FMath::Abs(LocalPoint.Z);

			float HorizontalPenetration = HorizontalDist - Obstacle.Extents.X;
			float VerticalPenetration = VerticalDist - Obstacle.Extents.Z;

			// 四种情况：内部(双负)、纯水平外、纯垂直外、角点外
			if (HorizontalPenetration < 0 && VerticalPenetration < 0)
			{
				return FMath::Max(HorizontalPenetration, VerticalPenetration) - Obstacle.SafetyMargin;
			}
			else if (HorizontalPenetration < 0)
			{
				return VerticalPenetration - Obstacle.SafetyMargin;
			}
			else if (VerticalPenetration < 0)
			{
				return HorizontalPenetration - Obstacle.SafetyMargin;
			}
			else
			{
				// 角点：到圆柱表面最近点的欧氏距离
				return FMath::Sqrt(HorizontalPenetration * HorizontalPenetration +
								   VerticalPenetration * VerticalPenetration) - Obstacle.SafetyMargin;
			}
		}

	default:
		// 未知类型：保守估计为球体
		return FVector::Dist(Point, Obstacle.Center) - Obstacle.Extents.GetMax() - Obstacle.SafetyMargin;
	}
}

// 检查指定点（含附加半径）是否在障碍物内部
bool UObstacleManager::IsPointInObstacle(const FVector& Point, const FObstacleInfo& Obstacle, float Radius) const
{
	// 总判定半径 = 查询半径 + 安全裕度
	float TotalRadius = Radius + Obstacle.SafetyMargin;

	switch (Obstacle.Type)
	{
	case EObstacleType::Sphere:
		{
			float Distance = FVector::Dist(Point, Obstacle.Center);
			float Threshold = Obstacle.Extents.X + TotalRadius;
			bool bInside = Distance < Threshold;
			UE_LOG_THROTTLE(2.0, LogUAVPlanning, Log, TEXT("[ObstacleManager] IsPointInObstacle: Point=%s, Dist=%.1f, Threshold=%.1f, Inside=%s"),
				*Point.ToString(), Distance, Threshold, bInside ? TEXT("YES") : TEXT("NO"));
			return bInside;
		}

	case EObstacleType::Box:
		{
			FVector LocalPoint = Obstacle.Rotation.UnrotateVector(Point - Obstacle.Center);
			FVector ExpandedExtents = Obstacle.Extents + FVector(TotalRadius);

			return FMath::Abs(LocalPoint.X) < ExpandedExtents.X &&
				   FMath::Abs(LocalPoint.Y) < ExpandedExtents.Y &&
				   FMath::Abs(LocalPoint.Z) < ExpandedExtents.Z;
		}

	case EObstacleType::Cylinder:
		{
			FVector LocalPoint = Point - Obstacle.Center;
			float HorizontalDist = FVector2D(LocalPoint.X, LocalPoint.Y).Size();

			return HorizontalDist < (Obstacle.Extents.X + TotalRadius) &&
				   FMath::Abs(LocalPoint.Z) < (Obstacle.Extents.Z + TotalRadius);
		}

	default:
		return false;
	}
}

void UObstacleManager::DrawDebugObstacles() const
{
	if (!GetWorld())
	{
		return;
	}

	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		FColor Color = Obstacle.bIsDynamic ? FColor::Orange : FColor::Red;

		switch (Obstacle.Type)
		{
		case EObstacleType::Sphere:
			DrawDebugSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.X, 16, Color, false, -1.0f, 0, 2.0f);
			DrawDebugSphere(GetWorld(), Obstacle.Center, Obstacle.Extents.X + Obstacle.SafetyMargin, 16,
				FColor(Color.R, Color.G, Color.B, 128), false, -1.0f, 0, 1.0f);
			break;

		case EObstacleType::Box:
			DrawDebugBox(GetWorld(), Obstacle.Center, Obstacle.Extents, Obstacle.Rotation.Quaternion(),
				Color, false, -1.0f, 0, 2.0f);
			break;

		case EObstacleType::Cylinder:
			// 近似用多条线绘制圆柱
			{
				int32 NumSegments = 16;
				for (int32 i = 0; i < NumSegments; ++i)
				{
					float Angle1 = (float)i / NumSegments * 2.0f * PI;
					float Angle2 = (float)(i + 1) / NumSegments * 2.0f * PI;

					FVector P1Top = Obstacle.Center + FVector(FMath::Cos(Angle1) * Obstacle.Extents.X,
						FMath::Sin(Angle1) * Obstacle.Extents.X, Obstacle.Extents.Z);
					FVector P2Top = Obstacle.Center + FVector(FMath::Cos(Angle2) * Obstacle.Extents.X,
						FMath::Sin(Angle2) * Obstacle.Extents.X, Obstacle.Extents.Z);
					FVector P1Bottom = Obstacle.Center + FVector(FMath::Cos(Angle1) * Obstacle.Extents.X,
						FMath::Sin(Angle1) * Obstacle.Extents.X, -Obstacle.Extents.Z);
					FVector P2Bottom = Obstacle.Center + FVector(FMath::Cos(Angle2) * Obstacle.Extents.X,
						FMath::Sin(Angle2) * Obstacle.Extents.X, -Obstacle.Extents.Z);

					DrawDebugLine(GetWorld(), P1Top, P2Top, Color, false, -1.0f, 0, 2.0f);
					DrawDebugLine(GetWorld(), P1Bottom, P2Bottom, Color, false, -1.0f, 0, 2.0f);
					DrawDebugLine(GetWorld(), P1Top, P1Bottom, Color, false, -1.0f, 0, 2.0f);
				}
			}
			break;

		default:
			break;
		}
	}
}
