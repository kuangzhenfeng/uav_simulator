// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UAVProductTypes.h"
#include "UAVTypes.h"
#include "UAVWreckActor.generated.h"

class UBoxComponent;
class UPrimitiveComponent;
class USphereComponent;

// 炸机残骸初始化参数，描述从飞控仿真交接到 Chaos 刚体的完整状态
USTRUCT(BlueprintType)
struct FUAVWreckInitParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV|Wreck")
	FUAVState InitialState;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV|Wreck")
	FUAVModelSpec ModelSpec;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV|Wreck")
	float PayloadMassKg = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV|Wreck")
	float CollisionRadiusCm = 100.0f;
};

/**
 * 独立 UAV 残骸 Actor。
 * 正常飞行由 AUAVPawn 的自定义动力学负责，炸机后由该 Actor 交给 Chaos 刚体物理接管。
 */
UCLASS()
class UAV_SIMULATOR_API AUAVWreckActor : public AActor
{
	GENERATED_BODY()

public:
	AUAVWreckActor();

	virtual void Tick(float DeltaTime) override;

	// 初始化残骸几何、质量和初始物理状态
	UFUNCTION(BlueprintCallable, Category = "UAV|Wreck")
	void InitializeWreck(const FUAVWreckInitParams& Params);

	// 获取残骸当前状态；读取前主动同步 Chaos 刚体，避免依赖 Actor Tick 的缓存状态
	UFUNCTION(BlueprintCallable, Category = "UAV|Wreck")
	FUAVState GetWreckState() const;

#if WITH_DEV_AUTOMATION_TESTS
	UPrimitiveComponent* GetBodyComponentForTest() const;
#endif

private:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV|Wreck", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UBoxComponent> BodyComponent;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV|Wreck", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USphereComponent> CoreGuardComponent;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV|Wreck", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UBoxComponent> ArmXComponent;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV|Wreck", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UBoxComponent> ArmYComponent;

	UPROPERTY(BlueprintReadOnly, Category = "UAV|Wreck", meta = (AllowPrivateAccess = "true"))
	mutable FUAVState WreckState;

	void ConfigureGeometry(const FUAVWreckInitParams& Params);
	void ConfigurePhysics(const FUAVWreckInitParams& Params);
	void ConfigurePrimitiveForWreck(UPrimitiveComponent* Component) const;
	void SyncStateFromPhysics() const;
	FVector ToWorldAngularVelocity(const FUAVState& State) const;
	FVector ToBodyAngularVelocity(const FVector& WorldAngularVelocity) const;
};
