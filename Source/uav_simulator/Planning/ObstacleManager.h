// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "../Core/UAVTypes.h"
#include "ObstacleManager.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnObstacleDetected, const FObstacleInfo&, Obstacle);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnCollisionWarning, const FVector&, Position, float, Distance);

/**
 * 障碍物管理器
 * 负责障碍物的注册、查询、碰撞检测和动态跟踪
 */
UCLASS(ClassGroup=(Planning), meta=(BlueprintSpawnableComponent))
class UAV_SIMULATOR_API UObstacleManager : public UActorComponent
{
	GENERATED_BODY()

public:
	UObstacleManager();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/**
	 * 注册障碍物
	 * @param Obstacle 障碍物信息
	 * @return 障碍物ID
	 */
	UFUNCTION(BlueprintCallable, Category = "Obstacle Management")
	int32 RegisterObstacle(const FObstacleInfo& Obstacle);

	/**
	 * 从Actor注册障碍物
	 * @param Actor 障碍物Actor
	 * @param Type 障碍物类型
	 * @param SafetyMargin 安全边距
	 * @return 障碍物ID
	 */
	UFUNCTION(BlueprintCallable, Category = "Obstacle Management")
	int32 RegisterObstacleFromActor(AActor* Actor, EObstacleType Type = EObstacleType::Sphere, float SafetyMargin = 50.0f);

	/**
	 * 移除障碍物
	 * @param ObstacleID 障碍物ID
	 * @return 是否成功移除
	 */
	UFUNCTION(BlueprintCallable, Category = "Obstacle Management")
	bool RemoveObstacle(int32 ObstacleID);

	/**
	 * 清除所有障碍物
	 */
	UFUNCTION(BlueprintCallable, Category = "Obstacle Management")
	void ClearAllObstacles();

	/**
	 * 获取所有障碍物
	 */
	UFUNCTION(BlueprintCallable, Category = "Obstacle Management")
	const TArray<FObstacleInfo>& GetAllObstacles() const { return Obstacles; }

	/**
	 * 获取指定ID的障碍物
	 * @param ObstacleID 障碍物ID
	 * @param OutObstacle 输出障碍物信息
	 * @return 是否找到
	 */
	UFUNCTION(BlueprintCallable, Category = "Obstacle Management")
	bool GetObstacle(int32 ObstacleID, FObstacleInfo& OutObstacle) const;

	/**
	 * 检查点是否与任何障碍物碰撞
	 * @param Point 检查点
	 * @param Radius 检查半径
	 * @return 是否碰撞
	 */
	UFUNCTION(BlueprintCallable, Category = "Collision Detection")
	bool CheckCollision(const FVector& Point, float Radius = 0.0f) const;

	/**
	 * 检查线段是否与任何障碍物碰撞
	 * @param Start 起点
	 * @param End 终点
	 * @param Radius 检查半径
	 * @return 是否碰撞
	 */
	UFUNCTION(BlueprintCallable, Category = "Collision Detection")
	bool CheckLineCollision(const FVector& Start, const FVector& End, float Radius = 0.0f) const;

	/**
	 * 获取到最近障碍物的距离
	 * @param Point 检查点
	 * @param OutNearestObstacle 最近障碍物信息
	 * @return 距离（负值表示在障碍物内部）
	 */
	UFUNCTION(BlueprintCallable, Category = "Collision Detection")
	float GetDistanceToNearestObstacle(const FVector& Point, FObstacleInfo& OutNearestObstacle) const;

	/**
	 * 获取指定范围内的所有障碍物
	 * @param Center 中心点
	 * @param Radius 搜索半径
	 * @return 范围内的障碍物数组
	 */
	UFUNCTION(BlueprintCallable, Category = "Collision Detection")
	TArray<FObstacleInfo> GetObstaclesInRange(const FVector& Center, float Radius) const;

	/**
	 * 自动扫描场景中的障碍物
	 * @param ScanCenter 扫描中心
	 * @param ScanRadius 扫描半径
	 * @param ActorClasses 要检测的Actor类
	 */
	UFUNCTION(BlueprintCallable, Category = "Obstacle Management")
	void ScanForObstacles(const FVector& ScanCenter, float ScanRadius, const TArray<TSubclassOf<AActor>>& ActorClasses);

	/**
	 * 注册感知检测到的障碍物（标记为 bIsPerceived）
	 * @param Obstacle 障碍物信息
	 * @return 障碍物ID
	 */
	UFUNCTION(BlueprintCallable, Category = "Perceived Obstacles")
	int32 RegisterPerceivedObstacle(const FObstacleInfo& Obstacle);

	/**
	 * 从 Actor 注册感知障碍物
	 * @param Actor 障碍物Actor
	 * @param Type 障碍物类型
	 * @param SafetyMargin 安全边距
	 * @return 障碍物ID
	 */
	UFUNCTION(BlueprintCallable, Category = "Perceived Obstacles")
	int32 RegisterPerceivedObstacleFromActor(AActor* Actor, EObstacleType Type = EObstacleType::Sphere, float SafetyMargin = 50.0f);

	/**
	 * 刷新感知障碍物的时间戳（表示仍然被检测到）
	 * @param ObstacleID 障碍物ID
	 */
	UFUNCTION(BlueprintCallable, Category = "Perceived Obstacles")
	void RefreshPerceivedObstacle(int32 ObstacleID);

	/**
	 * 移除超时未被重新感知的障碍物
	 * @param MaxAge 最大存活时间（秒），超过此时间未刷新的感知障碍物将被移除
	 * @return 移除的障碍物数量
	 */
	UFUNCTION(BlueprintCallable, Category = "Perceived Obstacles")
	int32 RemoveStalePerceivedObstacles(float MaxAge = 5.0f);

	/**
	 * 获取所有感知障碍物
	 */
	UFUNCTION(BlueprintCallable, Category = "Perceived Obstacles")
	TArray<FObstacleInfo> GetPerceivedObstacles() const;

	/**
	 * 获取所有预注册障碍物（非感知）
	 */
	UFUNCTION(BlueprintCallable, Category = "Perceived Obstacles")
	TArray<FObstacleInfo> GetPreregisteredObstacles() const;

	// 碰撞警告事件
	UPROPERTY(BlueprintAssignable, Category = "Obstacle Management")
	FOnCollisionWarning OnCollisionWarning;

	// 障碍物检测事件
	UPROPERTY(BlueprintAssignable, Category = "Obstacle Management")
	FOnObstacleDetected OnObstacleDetected;

protected:
	// 障碍物列表
	UPROPERTY(BlueprintReadOnly, Category = "Obstacle Management")
	TArray<FObstacleInfo> Obstacles;

	// 默认安全边距 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacle Settings")
	float DefaultSafetyMargin = 50.0f;

	// 是否自动更新动态障碍物
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacle Settings")
	bool bAutoUpdateDynamicObstacles = true;

	// 是否自动清理过期的感知障碍物
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacle Settings")
	bool bAutoRemoveStalePerceived = true;

	// 感知障碍物最大存活时间 (秒)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacle Settings", meta = (EditCondition = "bAutoRemoveStalePerceived"))
	float PerceivedObstacleMaxAge = 5.0f;

	// 碰撞警告距离 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Obstacle Settings")
	float CollisionWarningDistance = 200.0f;

	// 是否显示调试信息
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bShowDebug = false;

private:
	// 下一个障碍物ID
	int32 NextObstacleID;

	// 更新动态障碍物位置
	void UpdateDynamicObstacles();

	// 计算点到障碍物的距离
	float CalculateDistanceToObstacle(const FVector& Point, const FObstacleInfo& Obstacle) const;

	// 检查点是否在障碍物内
	bool IsPointInObstacle(const FVector& Point, const FObstacleInfo& Obstacle, float Radius) const;

	// 绘制障碍物调试信息
	void DrawDebugObstacles() const;
};
