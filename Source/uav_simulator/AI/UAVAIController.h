// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "UAVAIController.generated.h"

class UBehaviorTreeComponent;
class UBlackboardComponent;
class UBehaviorTree;
class UWaypointsData;

/**
 * UAV的AI控制器
 * 负责运行行为树并管理黑板数据
 */
UCLASS()
class UAV_SIMULATOR_API AUAVAIController : public AAIController
{
	GENERATED_BODY()

public:
	AUAVAIController();

	// 开始运行行为树
	UFUNCTION(BlueprintCallable, Category = "UAV AI")
	void StartBehaviorTree();

	// 停止行为树
	UFUNCTION(BlueprintCallable, Category = "UAV AI")
	void StopBehaviorTree();

	// 设置目标位置到黑板
	UFUNCTION(BlueprintCallable, Category = "UAV AI")
	void SetTargetLocation(FVector Location);

	// 设置目标Actor到黑板
	UFUNCTION(BlueprintCallable, Category = "UAV AI")
	void SetTargetActor(AActor* Target);

	// 获取行为树组件
	UFUNCTION(BlueprintCallable, Category = "UAV AI")
	UBehaviorTreeComponent* GetBehaviorTreeComponent() const { return BehaviorTreeComp; }

	// 获取黑板组件
	UFUNCTION(BlueprintCallable, Category = "UAV AI")
	UBlackboardComponent* GetBlackboardComp() const { return BlackboardComp; }

	// 设置航点数组到黑板
	UFUNCTION(BlueprintCallable, Category = "UAV AI")
	void SetWaypoints(const TArray<FVector>& Waypoints);

	// 添加单个航点
	UFUNCTION(BlueprintCallable, Category = "UAV AI")
	void AddWaypoint(const FVector& Waypoint);

	// 清空航点
	UFUNCTION(BlueprintCallable, Category = "UAV AI")
	void ClearWaypoints();

	// 获取当前航点数据
	UFUNCTION(BlueprintCallable, Category = "UAV AI")
	UWaypointsData* GetWaypointsData() const;

protected:
	virtual void OnPossess(APawn* InPawn) override;
	virtual void OnUnPossess() override;
	virtual void BeginPlay() override;

	// 行为树组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "AI")
	TObjectPtr<UBehaviorTreeComponent> BehaviorTreeComp;

	// 黑板组件
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "AI")
	TObjectPtr<UBlackboardComponent> BlackboardComp;

	// 要运行的行为树资产
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "AI")
	TObjectPtr<UBehaviorTree> BehaviorTreeAsset;

	// 是否在Possess时自动启动行为树
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "AI")
	bool bAutoStartBehaviorTree = true;

public:
	// 黑板键名称
	static const FName TargetLocationKey;
	static const FName TargetActorKey;
	static const FName HomeLocationKey;
	static const FName CurrentStateKey;
	static const FName WaypointsKey;

private:
	// 缓存的航点数据对象
	UPROPERTY()
	TObjectPtr<UWaypointsData> CachedWaypointsData;
};
