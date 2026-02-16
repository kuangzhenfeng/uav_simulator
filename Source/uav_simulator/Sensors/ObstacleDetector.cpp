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

	if (bShowDebugTraces || bShowDetectedObstacles)
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
	TArray<FHitResult> HitResults = PerformRaycastScan();

	// 将命中点聚类为障碍物
	DetectedObstacles = ClusterHitResults(HitResults);

	// 自动注册未知障碍物
	if (bAutoRegisterObstacles && ObstacleManagerRef)
	{
		for (FDetectedObstacle& Obstacle : DetectedObstacles)
		{
			if (Obstacle.RegisteredObstacleID == -1)
			{
				RegisterDetectedObstacle(Obstacle);
			}
		}
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

			// 检查是否已在 ObstacleManager 中注册
			if (IsActorAlreadyRegistered(HitActor))
			{
				// 已知障碍物，标记已注册（不需要再注册）
				NewObstacle.RegisteredObstacleID = 0; // 非 -1 表示已知
			}

			Clusters.Add(NewObstacle);
		}
	}

	return Clusters;
}

bool UObstacleDetector::IsActorAlreadyRegistered(AActor* Actor) const
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

	// 检查本地已注册集合
	return RegisteredActors.Contains(TWeakObjectPtr<AActor>(Actor));
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
		// 绘制扫描范围球
		DrawDebugSphere(World, ScanOrigin, ScanRange, 24, FColor::Cyan, false, -1.0f, 0, 1.0f);
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
