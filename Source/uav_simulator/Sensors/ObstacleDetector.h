// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SensorBase.h"
#include "../Core/UAVTypes.h"
#include "ObstacleDetector.generated.h"

class UObstacleManager;

/**
 * 感知检测结果
 */
USTRUCT(BlueprintType)
struct FDetectedObstacle
{
	GENERATED_BODY()

	// 检测到的障碍物中心位置（世界坐标系）
	UPROPERTY(BlueprintReadOnly, Category = "Detection")
	FVector Center;

	// 估算的障碍物尺寸
	UPROPERTY(BlueprintReadOnly, Category = "Detection")
	FVector EstimatedExtents;

	// 估算的障碍物类型
	UPROPERTY(BlueprintReadOnly, Category = "Detection")
	EObstacleType EstimatedType;

	// 距离 (cm)
	UPROPERTY(BlueprintReadOnly, Category = "Detection")
	float Distance;

	// 关联的 Actor（如果有）
	UPROPERTY(BlueprintReadOnly, Category = "Detection")
	TWeakObjectPtr<AActor> DetectedActor;

	// 在 ObstacleManager 中注册的 ID（-1 表示未注册）
	UPROPERTY(BlueprintReadOnly, Category = "Detection")
	int32 RegisteredObstacleID;

	FDetectedObstacle()
		: Center(FVector::ZeroVector)
		, EstimatedExtents(FVector(100.0f))
		, EstimatedType(EObstacleType::Sphere)
		, Distance(0.0f)
		, RegisteredObstacleID(-1)
	{}
};

// 检测到新障碍物事件
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnNewObstacleDetected, const FDetectedObstacle&, Obstacle);

/**
 * 障碍物感知传感器
 * 基于 UE5 射线检测（Raycast）模拟雷达/激光雷达扫描
 * - 可配置扫描范围、角度分辨率、更新频率
 * - 区分已知障碍物（ObstacleManager 预注册）和未知障碍物（实时感知）
 * - 检测到未知障碍物时自动注册到 ObstacleManager
 */
UCLASS(ClassGroup=(Sensors), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UObstacleDetector : public USensorBase
{
	GENERATED_BODY()

public:
	UObstacleDetector();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 重写传感器更新
	virtual void UpdateSensor(const FUAVState& TrueState, float DeltaTime) override;

	/**
	 * 执行一次完整扫描
	 * @return 检测到的障碍物列表
	 */
	UFUNCTION(BlueprintCallable, Category = "Obstacle Detection")
	TArray<FDetectedObstacle> PerformScan();

	/**
	 * 获取当前检测到的障碍物列表
	 */
	UFUNCTION(BlueprintCallable, Category = "Obstacle Detection")
	const TArray<FDetectedObstacle>& GetDetectedObstacles() const { return DetectedObstacles; }

	/**
	 * 设置关联的 ObstacleManager（用于自动注册检测到的障碍物）
	 */
	UFUNCTION(BlueprintCallable, Category = "Obstacle Detection")
	void SetObstacleManager(UObstacleManager* InManager) { ObstacleManagerRef = InManager; }

	// 检测到新障碍物事件
	UPROPERTY(BlueprintAssignable, Category = "Obstacle Detection")
	FOnNewObstacleDetected OnNewObstacleDetected;

protected:
	// ---- 扫描参数 ----

	// 最大扫描距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scan Parameters")
	float ScanRange = 20000.0f;

	// 水平扫描角度 (度，以前方为中心的总角度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scan Parameters", meta = (ClampMin = "10.0", ClampMax = "360.0"))
	float HorizontalFOV = 360.0f;

	// 垂直扫描角度 (度，以水平面为中心的总角度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scan Parameters", meta = (ClampMin = "10.0", ClampMax = "180.0"))
	float VerticalFOV = 180.0f;

	// 水平角度分辨率 (度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scan Parameters", meta = (ClampMin = "1.0", ClampMax = "45.0"))
	float HorizontalResolution = 5.0f;

	// 垂直角度分辨率 (度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scan Parameters", meta = (ClampMin = "1.0", ClampMax = "45.0"))
	float VerticalResolution = 5.0f;

	// 扫描更新频率 (Hz)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scan Parameters", meta = (ClampMin = "1.0", ClampMax = "30.0"))
	float ScanFrequency = 10.0f;

	// ---- 检测参数 ----

	// 射线检测通道
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Detection Parameters")
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

	// 同一障碍物的聚类距离阈值 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Detection Parameters")
	float ClusteringThreshold = 200.0f;

	// 是否自动注册检测到的未知障碍物到 ObstacleManager
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Detection Parameters")
	bool bAutoRegisterObstacles = true;

	// 安全边距（注册到 ObstacleManager 时使用）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Detection Parameters")
	float DetectionSafetyMargin = 50.0f;

	// 是否过滤地面 Actor（顶部低于 UAV 或带有 Floor/Ground Tag）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Detection Parameters")
	bool bFilterGroundActors = true;

	// ---- 调试 ----

	// 是否显示调试射线
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bShowDebugTraces = false;

	// 是否显示检测到的障碍物
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bShowDetectedObstacles = false;

	// 是否显示扫描点云
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bShowPointCloud = false;

	// 点云有效时间 (秒)，与障碍物有效时间一致
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta = (EditCondition = "bShowPointCloud", ClampMin = "0.1"))
	float PointCloudLifetime = 5.0f;

private:
	// 关联的 ObstacleManager
	UPROPERTY()
	TObjectPtr<UObstacleManager> ObstacleManagerRef;

	// 当前检测到的障碍物列表
	TArray<FDetectedObstacle> DetectedObstacles;

	// 缓存的射线命中结果（用于调试射线）
	TArray<FHitResult> CachedHitResults;

	// 累积的点云缓存（位置 + 时间戳）
	TArray<TPair<FVector, float>> PointCloudCache;

	// 扫描计时器
	float ScanAccumulatedTime;

	// 已注册的 Actor 集合（避免重复注册）
	TSet<TWeakObjectPtr<AActor>> RegisteredActors;

	// 执行射线扫描，返回命中点列表
	TArray<FHitResult> PerformRaycastScan() const;

	// 将命中点聚类为障碍物
	TArray<FDetectedObstacle> ClusterHitResults(const TArray<FHitResult>& HitResults) const;

	// 检查 Actor 是否已在 ObstacleManager 中注册
	bool IsActorAlreadyRegistered(AActor* Actor) const;

	// 将检测到的障碍物注册到 ObstacleManager
	void RegisterDetectedObstacle(FDetectedObstacle& Obstacle);

	// 绘制调试信息
	void DrawDebugInfo() const;
};
