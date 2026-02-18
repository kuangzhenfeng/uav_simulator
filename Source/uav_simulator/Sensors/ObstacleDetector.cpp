// Copyright Epic Games, Inc. All Rights Reserved.

#include "ObstacleDetector.h"
#include "DrawDebugHelpers.h"
#include "uav_simulator/Planning/ObstacleManager.h"
#include "uav_simulator/Debug/UAVLogConfig.h"

UObstacleDetector::UObstacleDetector()
{
	PrimaryComponentTick.bCanEverTick = true;
	ScanAccumulatedTime = 0.0f;
}

void UObstacleDetector::BeginPlay()
{
	Super::BeginPlay();

	// 自动查找同一 Actor 上的 ObstacleManager
	if (!ObstacleManagerRef)
	{
		ObstacleManagerRef = GetOwner()->FindComponentByClass<UObstacleManager>();
	}

	UE_LOG(LogUAVSensor, Log, TEXT("[ObstacleDetector] Initialized: Range=%.0fcm, HFOV=%.0f, VFOV=%.0f, HRes=%.0f, VRes=%.0f, Freq=%.0fHz"),
		ScanRange, HorizontalFOV, VerticalFOV, HorizontalResolution, VerticalResolution, ScanFrequency);
}

void UObstacleDetector::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bEnabled)
	{
		return;
	}

	// 按频率控制扫描
	ScanAccumulatedTime += DeltaTime;
	float ScanInterval = 1.0f / FMath::Max(ScanFrequency, 1.0f);

	if (ScanAccumulatedTime >= ScanInterval)
	{
		ScanAccumulatedTime -= ScanInterval;
		PerformScan();
	}

	if (bShowDebugTraces || bShowDetectedObstacles || bShowPointCloud)
	{
		DrawDebugInfo();
	}
}

void UObstacleDetector::UpdateSensor(const FUAVState& TrueState, float DeltaTime)
{
	// 传感器更新由 TickComponent 驱动，此处留空
}

TArray<FDetectedObstacle> UObstacleDetector::PerformScan()
{
	// 执行射线扫描
	CachedHitResults = PerformRaycastScan();
	TArray<FHitResult>& HitResults = CachedHitResults;

	// 将命中点聚类为障碍物
	DetectedObstacles = ClusterHitResults(HitResults);

	// 自动注册未知障碍物，并刷新已注册障碍物的时间戳
	if (bAutoRegisterObstacles && ObstacleManagerRef)
	{
		for (FDetectedObstacle& Obstacle : DetectedObstacles)
		{
			if (Obstacle.RegisteredObstacleID == -1)
			{
				RegisterDetectedObstacle(Obstacle);
			}
			else if (Obstacle.RegisteredObstacleID > 0)
			{
				ObstacleManagerRef->RefreshPerceivedObstacle(Obstacle.RegisteredObstacleID);
			}
		}
	}

	// 累积点云缓存
	if (bShowPointCloud)
	{
		const float CurrentTime = GetWorld()->GetTimeSeconds();
		for (const FHitResult& Hit : CachedHitResults)
		{
			PointCloudCache.Emplace(Hit.ImpactPoint, CurrentTime);
		}
		PointCloudCache.RemoveAll([CurrentTime, this](const TPair<FVector, float>& P) {
			return (CurrentTime - P.Value) > PointCloudLifetime;
		});
	}

	return DetectedObstacles;
}

TArray<FHitResult> UObstacleDetector::PerformRaycastScan() const
{
	TArray<FHitResult> AllHits;

	UWorld* World = GetWorld();
	if (!World)
	{
		return AllHits;
	}

	FVector ScanOrigin = GetOwner()->GetActorLocation();
	FRotator ScanRotation = GetOwner()->GetActorRotation();

	// 计算扫描射线数量
	float HalfHFOV = HorizontalFOV * 0.5f;
	float HalfVFOV = VerticalFOV * 0.5f;

	// 射线检测参数
	FCollisionQueryParams QueryParams;
	QueryParams.AddIgnoredActor(GetOwner());
	QueryParams.bTraceComplex = false;
	QueryParams.bReturnPhysicalMaterial = false;

	// 水平和垂直扫描
	for (float Yaw = -HalfHFOV; Yaw <= HalfHFOV; Yaw += HorizontalResolution)
	{
		for (float Pitch = -HalfVFOV; Pitch <= HalfVFOV; Pitch += VerticalResolution)
		{
			// 计算射线方向（相对于 Actor 朝向）
			FRotator RayRotation = ScanRotation + FRotator(Pitch, Yaw, 0.0f);
			FVector RayDirection = RayRotation.Vector();
			FVector RayEnd = ScanOrigin + RayDirection * ScanRange;

			FHitResult HitResult;
			bool bHit = World->LineTraceSingleByChannel(
				HitResult,
				ScanOrigin,
				RayEnd,
				TraceChannel,
				QueryParams
			);

			if (bHit)
			{
				AllHits.Add(HitResult);
			}
		}
	}

	return AllHits;
}

TArray<FDetectedObstacle> UObstacleDetector::ClusterHitResults(const TArray<FHitResult>& HitResults) const
{
	TArray<FDetectedObstacle> Clusters;

	for (const FHitResult& Hit : HitResults)
	{
		AActor* HitActor = Hit.GetActor();
		if (!HitActor)
		{
			continue;
		}

		// 检查是否已经归入某个聚类（按 Actor 分组）
		bool bFoundCluster = false;
		for (FDetectedObstacle& Cluster : Clusters)
		{
			if (Cluster.DetectedActor.Get() == HitActor)
			{
				// 同一 Actor 的命中点，更新包围盒
				bFoundCluster = true;
				break;
			}
		}

		if (!bFoundCluster)
		{
			// 地面过滤
			if (bFilterGroundActors)
			{
				FVector Origin, BoxExtent;
				HitActor->GetActorBounds(false, Origin, BoxExtent);
				float ActorTop = Origin.Z + BoxExtent.Z;
				FVector ScanOrigin = GetOwner()->GetActorLocation();
				if (ActorTop < ScanOrigin.Z)
				{
					continue;
				}
				if (HitActor->Tags.Contains(FName("Floor")) || HitActor->Tags.Contains(FName("Ground")))
				{
					continue;
				}
			}

			// 新障碍物
			FDetectedObstacle NewObstacle;
			NewObstacle.DetectedActor = HitActor;
			NewObstacle.Distance = Hit.Distance;

			// 从 Actor 获取边界信息
			FVector Origin, BoxExtent;
			HitActor->GetActorBounds(false, Origin, BoxExtent);
			NewObstacle.Center = Origin;
			NewObstacle.EstimatedExtents = BoxExtent;

			// 根据尺寸比例估算障碍物类型
			float XYRatio = BoxExtent.X / FMath::Max(BoxExtent.Y, 1.0f);
			float ZRatio = BoxExtent.Z / FMath::Max(BoxExtent.X, 1.0f);

			if (FMath::Abs(XYRatio - 1.0f) < 0.3f && FMath::Abs(ZRatio - 1.0f) < 0.3f)
			{
				NewObstacle.EstimatedType = EObstacleType::Sphere;
			}
			else if (FMath::Abs(XYRatio - 1.0f) < 0.3f && ZRatio > 1.5f)
			{
				NewObstacle.EstimatedType = EObstacleType::Cylinder;
			}
			else
			{
				NewObstacle.EstimatedType = EObstacleType::Box;
			}

			// 检查是否已在 ObstacleManager 中注册，记录真实 ID
			if (ObstacleManagerRef)
			{
				for (const FObstacleInfo& Obs : ObstacleManagerRef->GetAllObstacles())
				{
					if (Obs.LinkedActor.Get() == HitActor)
					{
						NewObstacle.RegisteredObstacleID = Obs.ObstacleID;
						break;
					}
				}
			}

			Clusters.Add(NewObstacle);
		}
	}

	return Clusters;
}

bool UObstacleDetector::IsActorAlreadyRegistered(AActor* Actor)
{
	if (!Actor || !ObstacleManagerRef)
	{
		return false;
	}

	// 检查 ObstacleManager 中是否已有此 Actor
	const TArray<FObstacleInfo>& Obstacles = ObstacleManagerRef->GetAllObstacles();
	for (const FObstacleInfo& Obstacle : Obstacles)
	{
		if (Obstacle.LinkedActor.Get() == Actor)
		{
			return true;
		}
	}

	// ObstacleManager 中已不存在，清除本地记录以允许重新注册
	RegisteredActors.Remove(TWeakObjectPtr<AActor>(Actor));
	return false;
}

void UObstacleDetector::RegisterDetectedObstacle(FDetectedObstacle& Obstacle)
{
	if (!ObstacleManagerRef)
	{
		return;
	}

	AActor* Actor = Obstacle.DetectedActor.Get();
	if (!Actor)
	{
		return;
	}

	// 再次检查避免重复注册
	if (IsActorAlreadyRegistered(Actor))
	{
		return;
	}

	// 注册到 ObstacleManager
	int32 ID = ObstacleManagerRef->RegisterPerceivedObstacleFromActor(Actor, Obstacle.EstimatedType, DetectionSafetyMargin);
	Obstacle.RegisteredObstacleID = ID;

	// 记录到本地集合
	RegisteredActors.Add(TWeakObjectPtr<AActor>(Actor));

	UE_LOG(LogUAVSensor, Warning, TEXT("[ObstacleDetector] New obstacle detected and registered: ID=%d, Actor=%s, Type=%d, Distance=%.1fcm"),
		ID, *Actor->GetName(), (int32)Obstacle.EstimatedType, Obstacle.Distance);

	// 广播事件
	OnNewObstacleDetected.Broadcast(Obstacle);
}

void UObstacleDetector::DrawDebugInfo() const
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	FVector ScanOrigin = GetOwner()->GetActorLocation();

	if (bShowDebugTraces)
	{
		for (const FHitResult& Hit : CachedHitResults)
		{
			DrawDebugLine(World, ScanOrigin, Hit.ImpactPoint, FColor::Red, false, -1.0f, 0, 1.0f);
			DrawDebugPoint(World, Hit.ImpactPoint, 5.0f, FColor::Red, false, -1.0f);
		}
	}

	if (bShowPointCloud)
	{
		for (const TPair<FVector, float>& Point : PointCloudCache)
		{
			float t = FMath::Clamp(FVector::Dist(ScanOrigin, Point.Key) / ScanRange, 0.0f, 1.0f);
			// Jet colormap: 近=蓝 → 青 → 绿 → 黄 → 远=红
			float R = FMath::Clamp(1.5f - FMath::Abs(4.0f * t - 3.0f), 0.0f, 1.0f);
			float G = FMath::Clamp(1.5f - FMath::Abs(4.0f * t - 2.0f), 0.0f, 1.0f);
			float B = FMath::Clamp(1.5f - FMath::Abs(4.0f * t - 1.0f), 0.0f, 1.0f);
			FColor Color((uint8)(R * 255), (uint8)(G * 255), (uint8)(B * 255));
			DrawDebugPoint(World, Point.Key, 4.0f, Color, false, -1.0f);
		}
	}

	if (bShowDetectedObstacles)
	{
		for (const FDetectedObstacle& Obstacle : DetectedObstacles)
		{
			// 已知障碍物用绿色，未知用黄色
			FColor Color = (Obstacle.RegisteredObstacleID > 0) ? FColor::Green : FColor::Yellow;

			switch (Obstacle.EstimatedType)
			{
			case EObstacleType::Sphere:
				DrawDebugSphere(World, Obstacle.Center, Obstacle.EstimatedExtents.X, 12, Color, false, -1.0f, 0, 2.0f);
				break;

			case EObstacleType::Box:
				DrawDebugBox(World, Obstacle.Center, Obstacle.EstimatedExtents, Color, false, -1.0f, 0, 2.0f);
				break;

			case EObstacleType::Cylinder:
				DrawDebugCylinder(World,
					Obstacle.Center - FVector(0, 0, Obstacle.EstimatedExtents.Z),
					Obstacle.Center + FVector(0, 0, Obstacle.EstimatedExtents.Z),
					Obstacle.EstimatedExtents.X, 12, Color, false, -1.0f, 0, 2.0f);
				break;

			default:
				DrawDebugSphere(World, Obstacle.Center, Obstacle.EstimatedExtents.GetMax(), 12, Color, false, -1.0f, 0, 2.0f);
				break;
			}

			// 绘制从传感器到障碍物的连线
			DrawDebugLine(World, ScanOrigin, Obstacle.Center, FColor(Color.R, Color.G, Color.B, 64), false, -1.0f, 0, 1.0f);
		}
	}
}
