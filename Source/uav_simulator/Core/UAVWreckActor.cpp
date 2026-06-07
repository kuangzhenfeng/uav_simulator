// Copyright Epic Games, Inc. All Rights Reserved.

#include "UAVWreckActor.h"

#include "Components/BoxComponent.h"
#include "Components/SphereComponent.h"

AUAVWreckActor::AUAVWreckActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	BodyComponent = CreateDefaultSubobject<UBoxComponent>(TEXT("WreckBody"));
	BodyComponent->InitBoxExtent(FVector(35.0f, 25.0f, 12.0f));
	RootComponent = BodyComponent;

	CoreGuardComponent = CreateDefaultSubobject<USphereComponent>(TEXT("WreckCoreGuard"));
	CoreGuardComponent->InitSphereRadius(30.0f);
	CoreGuardComponent->SetupAttachment(BodyComponent);

	ArmXComponent = CreateDefaultSubobject<UBoxComponent>(TEXT("WreckArmX"));
	ArmXComponent->InitBoxExtent(FVector(90.0f, 6.0f, 6.0f));
	ArmXComponent->SetupAttachment(BodyComponent);

	ArmYComponent = CreateDefaultSubobject<UBoxComponent>(TEXT("WreckArmY"));
	ArmYComponent->InitBoxExtent(FVector(6.0f, 90.0f, 6.0f));
	ArmYComponent->SetupAttachment(BodyComponent);

	ConfigurePrimitiveForWreck(BodyComponent);
	ConfigurePrimitiveForWreck(CoreGuardComponent);
	ConfigurePrimitiveForWreck(ArmXComponent);
	ConfigurePrimitiveForWreck(ArmYComponent);

	BodyComponent->SetSimulatePhysics(false);
	CoreGuardComponent->SetSimulatePhysics(false);
	ArmXComponent->SetSimulatePhysics(false);
	ArmYComponent->SetSimulatePhysics(false);
}

void AUAVWreckActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	SyncStateFromPhysics();
}

void AUAVWreckActor::InitializeWreck(const FUAVWreckInitParams& Params)
{
	WreckState = Params.InitialState;
	SetActorLocationAndRotation(WreckState.Position, WreckState.Rotation, false, nullptr, ETeleportType::TeleportPhysics);

	ConfigureGeometry(Params);
	ConfigurePhysics(Params);

	BodyComponent->SetSimulatePhysics(true);
	BodyComponent->SetEnableGravity(true);

	const FAttachmentTransformRules WeldRules(EAttachmentRule::KeepRelative, true);
	CoreGuardComponent->AttachToComponent(BodyComponent, WeldRules);
	ArmXComponent->AttachToComponent(BodyComponent, WeldRules);
	ArmYComponent->AttachToComponent(BodyComponent, WeldRules);
	CoreGuardComponent->SetSimulatePhysics(false);
	ArmXComponent->SetSimulatePhysics(false);
	ArmYComponent->SetSimulatePhysics(false);
	BodyComponent->WakeAllRigidBodies();
	BodyComponent->SetPhysicsLinearVelocity(WreckState.Velocity, false);
	BodyComponent->SetPhysicsAngularVelocityInRadians(ToWorldAngularVelocity(WreckState), false);
	SyncStateFromPhysics();
}

void AUAVWreckActor::ConfigureGeometry(const FUAVWreckInitParams& Params)
{
	const float CollisionRadiusCm = FMath::Max(Params.CollisionRadiusCm, 50.0f);
	const float ArmSpanCm = FMath::Max(Params.ModelSpec.ArmLength * 100.0f * 2.0f, CollisionRadiusCm);
	const float BodyHalfLengthCm = FMath::Clamp(CollisionRadiusCm * 0.28f, 18.0f, 80.0f);
	const float BodyHalfWidthCm = FMath::Clamp(CollisionRadiusCm * 0.20f, 14.0f, 60.0f);
	const float BodyHalfHeightCm = FMath::Clamp(CollisionRadiusCm * 0.10f, 8.0f, 35.0f);
	const float ArmHalfLengthCm = FMath::Max(ArmSpanCm * 0.5f, BodyHalfLengthCm);
	const float ArmHalfThicknessCm = FMath::Clamp(CollisionRadiusCm * 0.035f, 4.0f, 12.0f);
	const float CoreRadiusCm = FMath::Clamp(CollisionRadiusCm * 0.22f, 15.0f, 60.0f);

	BodyComponent->SetBoxExtent(FVector(BodyHalfLengthCm, BodyHalfWidthCm, BodyHalfHeightCm), true);
	CoreGuardComponent->SetSphereRadius(CoreRadiusCm, true);
	ArmXComponent->SetBoxExtent(FVector(ArmHalfLengthCm, ArmHalfThicknessCm, ArmHalfThicknessCm), true);
	ArmYComponent->SetBoxExtent(FVector(ArmHalfThicknessCm, ArmHalfLengthCm, ArmHalfThicknessCm), true);
}

void AUAVWreckActor::ConfigurePhysics(const FUAVWreckInitParams& Params)
{
	const float TotalMassKg = FMath::Max(Params.ModelSpec.Mass + Params.PayloadMassKg, 0.1f);
	BodyComponent->SetMassOverrideInKg(NAME_None, TotalMassKg, true);

	// Chaos 会按碰撞几何自动推导惯量；这里用机型惯量比例修正，让残骸翻滚接近真实 UAV。
	BodyComponent->BodyInstance.InertiaTensorScale = FVector(
		FMath::Max(Params.ModelSpec.MomentOfInertia.X, 0.01f),
		FMath::Max(Params.ModelSpec.MomentOfInertia.Y, 0.01f),
		FMath::Max(Params.ModelSpec.MomentOfInertia.Z, 0.01f));
}

void AUAVWreckActor::ConfigurePrimitiveForWreck(UPrimitiveComponent* Component) const
{
	if (!Component)
	{
		return;
	}

	Component->SetMobility(EComponentMobility::Movable);
	Component->SetCollisionProfileName(TEXT("UAVWreck"));
	Component->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Component->SetNotifyRigidBodyCollision(true);
	Component->SetEnableGravity(true);
	Component->SetUseCCD(true);
	Component->SetLinearDamping(0.35f);
	Component->SetAngularDamping(0.55f);
}

FUAVState AUAVWreckActor::GetWreckState() const
{
	SyncStateFromPhysics();
	return WreckState;
}

void AUAVWreckActor::SyncStateFromPhysics() const
{
	if (!BodyComponent || !BodyComponent->IsSimulatingPhysics())
	{
		return;
	}

	WreckState.Position = BodyComponent->GetComponentLocation();
	WreckState.Rotation = BodyComponent->GetComponentRotation();
	WreckState.Velocity = BodyComponent->GetPhysicsLinearVelocity();
	WreckState.AngularVelocity = ToBodyAngularVelocity(BodyComponent->GetPhysicsAngularVelocityInRadians());
}

FVector AUAVWreckActor::ToWorldAngularVelocity(const FUAVState& State) const
{
	return State.Rotation.RotateVector(State.AngularVelocity);
}

FVector AUAVWreckActor::ToBodyAngularVelocity(const FVector& WorldAngularVelocity) const
{
	return WreckState.Rotation.UnrotateVector(WorldAngularVelocity);
}

#if WITH_DEV_AUTOMATION_TESTS
UPrimitiveComponent* AUAVWreckActor::GetBodyComponentForTest() const
{
	return BodyComponent;
}
#endif
