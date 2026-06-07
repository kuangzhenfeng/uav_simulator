// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Core/UAVPawn.h"
#include "../../Core/UAVProductManager.h"
#include "../../Core/UAVWreckActor.h"
#include "Components/BoxComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/Engine.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	UWorld* CreateUAVCrashTestWorld(const TCHAR* WorldName)
	{
		if (!GEngine)
		{
			return nullptr;
		}

		static int32 WorldCounter = 0;
		const FName UniqueWorldName(*FString::Printf(TEXT("%s_%d"), WorldName, ++WorldCounter));
		UWorld::InitializationValues InitValues;
		InitValues.AllowAudioPlayback(false)
			.CreatePhysicsScene(true)
			.RequiresHitProxies(false)
			.CreateNavigation(false)
			.CreateAISystem(false)
			.ShouldSimulatePhysics(true)
			.SetTransactional(false);
		UWorld* World = UWorld::CreateWorld(
			EWorldType::Game,
			false,
			UniqueWorldName,
			GetTransientPackage(),
			false,
			ERHIFeatureLevel::Num,
			&InitValues);
		if (!World)
		{
			return nullptr;
		}

		FWorldContext& WorldContext = GEngine->CreateNewWorldContext(EWorldType::Game);
		UGameInstance* GameInstance = NewObject<UGameInstance>(GEngine);
		World->AddToRoot();
		World->SetGameInstance(GameInstance);
		WorldContext.OwningGameInstance = GameInstance;
		WorldContext.SetCurrentWorld(World);
		GameInstance->Init();

		const FURL URL;
		World->SetGameMode(URL);
		World->InitializeActorsForPlay(URL);
		if (AWorldSettings* WorldSettings = World->GetWorldSettings())
		{
			WorldSettings->bGlobalGravitySet = true;
			WorldSettings->GlobalGravityZ = -980.0f;
		}
		World->BeginPlay();
		return World;
	}

	void DestroyUAVCrashTestWorld(UWorld* World)
	{
		if (!World || !GEngine)
		{
			return;
		}

		if (World->HasBegunPlay())
		{
			World->BeginTearingDown();
			World->EndPlay(EEndPlayReason::Quit);
		}
		GEngine->DestroyWorldContext(World);
		if (World->GetGameInstance())
		{
			World->GetGameInstance()->Shutdown();
		}
		World->DestroyWorld(false);
		World->RemoveFromRoot();
	}

	FUAVWreckInitParams MakeDefaultWreckParams()
	{
		const FUAVModelSpec Spec = FUAVProductManager::GetModelSpec(EUAVModelID::Agri_AG20);
		FUAVState State;
		State.Position = FVector(0.0f, 0.0f, 500.0f);
		State.Rotation = FRotator(5.0f, 20.0f, -8.0f);
		State.Velocity = FVector(100.0f, 0.0f, -250.0f);
		State.AngularVelocity = FVector(0.2f, -0.1f, 0.4f);

		FUAVWreckInitParams Params;
		Params.InitialState = State;
		Params.ModelSpec = Spec;
		Params.PayloadMassKg = 2.0f;
		Params.CollisionRadiusCm = 160.0f;
		return Params;
	}

	AUAVPawn* SpawnCrashTestPawn(UWorld* World, const FVector& Location)
	{
		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		return World ? World->SpawnActor<AUAVPawn>(AUAVPawn::StaticClass(), Location, FRotator::ZeroRotator, SpawnParams) : nullptr;
	}

	AActor* SpawnBlockingGround(UWorld* World)
	{
		if (!World)
		{
			return nullptr;
		}

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		AActor* GroundActor = World->SpawnActor<AActor>(AActor::StaticClass(), FVector(0.0f, 0.0f, -25.0f), FRotator::ZeroRotator, SpawnParams);
		if (!GroundActor)
		{
			return nullptr;
		}

		UBoxComponent* GroundComponent = NewObject<UBoxComponent>(GroundActor, TEXT("GroundCollision"));
		GroundComponent->InitBoxExtent(FVector(10000.0f, 10000.0f, 25.0f));
		GroundComponent->SetCollisionProfileName(TEXT("BlockAll"));
		GroundComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		GroundActor->SetRootComponent(GroundComponent);
		GroundComponent->RegisterComponent();
		GroundActor->SetActorLocation(FVector(0.0f, 0.0f, -25.0f), false, nullptr, ETeleportType::TeleportPhysics);
		return GroundActor;
	}

	void TickWorldForSeconds(UWorld* World, float DurationSeconds, float StepSeconds)
	{
		for (float Elapsed = 0.0f; Elapsed < DurationSeconds; Elapsed += StepSeconds)
		{
			World->Tick(LEVELTICK_All, StepSeconds);
			GFrameCounter++;
		}
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVWreckActorInitializesPhysicsTest,
	"UAVSimulator.Core.UAVWreckActor.InitializesPhysics",
	UAV_TEST_FLAGS)

bool FUAVWreckActorInitializesPhysicsTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateUAVCrashTestWorld(TEXT("UAVWreckActorInitializesPhysics"));
	TestNotNull(TEXT("World should be created"), World);
	if (!World)
	{
		return false;
	}

	AUAVWreckActor* Wreck = World->SpawnActor<AUAVWreckActor>();
	TestNotNull(TEXT("Wreck actor should spawn"), Wreck);
	if (!Wreck)
	{
		DestroyUAVCrashTestWorld(World);
		return false;
	}

	const FUAVWreckInitParams Params = MakeDefaultWreckParams();
	Wreck->InitializeWreck(Params);

	UPrimitiveComponent* Body = Wreck->GetBodyComponentForTest();
	TestNotNull(TEXT("Wreck should expose body component"), Body);
	TestTrue(TEXT("Wreck body should simulate physics"), Body && Body->IsSimulatingPhysics());
	TestEqual(TEXT("Wreck body should use UAVWreck collision profile"), Body ? Body->GetCollisionProfileName() : NAME_None, FName(TEXT("UAVWreck")));
	TestTrue(TEXT("Wreck body should use CCD"), Body && Body->BodyInstance.bUseCCD);
	UAV_TEST_VECTOR_EQUAL(Wreck->GetWreckState().Position, Params.InitialState.Position, 1.0f);
	UAV_TEST_ROTATOR_EQUAL(Wreck->GetWreckState().Rotation, Params.InitialState.Rotation, 0.5f);

	DestroyUAVCrashTestWorld(World);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVPawnSpawnsWreckOnCrashTest,
	"UAVSimulator.Core.UAVPawn.SpawnsWreckOnCrash",
	UAV_TEST_FLAGS)

bool FUAVPawnSpawnsWreckOnCrashTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateUAVCrashTestWorld(TEXT("UAVPawnSpawnsWreckOnCrash"));
	TestNotNull(TEXT("World should be created"), World);
	if (!World)
	{
		return false;
	}

	AUAVPawn* Pawn = SpawnCrashTestPawn(World, FVector(0.0f, 0.0f, 300.0f));
	TestNotNull(TEXT("Pawn should spawn"), Pawn);
	if (!Pawn)
	{
		DestroyUAVCrashTestWorld(World);
		return false;
	}

	FUAVState CrashState;
	CrashState.Position = FVector(0.0f, 0.0f, 300.0f);
	CrashState.Rotation = FRotator(0.0f, 30.0f, 0.0f);
	CrashState.Velocity = FVector(0.0f, 0.0f, -400.0f);
	CrashState.AngularVelocity = FVector(0.0f, 0.0f, 0.6f);
	Pawn->SetUAVStateForTest(CrashState);
	Pawn->RecordSafeCrashPoseForTest();
	Pawn->TestTriggerCrash();

	AUAVWreckActor* Wreck = Pawn->GetActiveWreckActorForTest();
	TestNotNull(TEXT("Pawn should spawn active wreck actor"), Wreck);
	TestTrue(TEXT("Pawn should be crashed"), Pawn->IsCrashed());
	TestTrue(TEXT("Wreck body should simulate"), Wreck && Wreck->GetBodyComponentForTest() && Wreck->GetBodyComponentForTest()->IsSimulatingPhysics());
	const FVector WreckVelocity = Wreck ? Wreck->GetWreckState().Velocity : FVector::ZeroVector;
	UAV_TEST_VECTOR_EQUAL(WreckVelocity, CrashState.Velocity, 1.0f);

	DestroyUAVCrashTestWorld(World);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVPawnCrashHandoffUsesSafePoseTest,
	"UAVSimulator.Core.UAVPawn.CrashHandoffUsesSafePose",
	UAV_TEST_FLAGS)

bool FUAVPawnCrashHandoffUsesSafePoseTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateUAVCrashTestWorld(TEXT("UAVPawnCrashHandoffUsesSafePose"));
	TestNotNull(TEXT("World should be created"), World);
	if (!World)
	{
		return false;
	}

	AUAVPawn* Pawn = SpawnCrashTestPawn(World, FVector(0.0f, 0.0f, 300.0f));
	TestNotNull(TEXT("Pawn should spawn"), Pawn);
	if (!Pawn)
	{
		DestroyUAVCrashTestWorld(World);
		return false;
	}

	FUAVState SafeState;
	SafeState.Position = FVector(0.0f, 0.0f, 300.0f);
	SafeState.Rotation = FRotator::ZeroRotator;
	SafeState.Velocity = FVector(0.0f, 0.0f, -600.0f);
	Pawn->SetUAVStateForTest(SafeState);
	Pawn->RecordSafeCrashPoseForTest();

	FUAVState PenetratingState = SafeState;
	PenetratingState.Position = FVector(0.0f, 0.0f, -50.0f);
	Pawn->SetUAVStateForTest(PenetratingState);
	Pawn->TestTriggerCrash();

	AUAVWreckActor* Wreck = Pawn->GetActiveWreckActorForTest();
	TestNotNull(TEXT("Wreck should exist"), Wreck);
	TestTrue(TEXT("Handoff should use last safe Z instead of penetrated Z"), Wreck && Wreck->GetWreckState().Position.Z >= 250.0f);

	DestroyUAVCrashTestWorld(World);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVWreckActorBlocksGroundDuringPhysicsTickTest,
	"UAVSimulator.Core.UAVWreckActor.BlocksGroundDuringPhysicsTick",
	UAV_TEST_FLAGS)

bool FUAVWreckActorBlocksGroundDuringPhysicsTickTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateUAVCrashTestWorld(TEXT("UAVWreckActorBlocksGroundDuringPhysicsTick"));
	TestNotNull(TEXT("World should be created"), World);
	if (!World)
	{
		return false;
	}

	AActor* GroundActor = SpawnBlockingGround(World);
	TestNotNull(TEXT("Ground should spawn"), GroundActor);
	if (!GroundActor)
	{
		DestroyUAVCrashTestWorld(World);
		return false;
	}

	AUAVWreckActor* Wreck = World->SpawnActor<AUAVWreckActor>();
	TestNotNull(TEXT("Wreck should spawn"), Wreck);
	if (!Wreck)
	{
		DestroyUAVCrashTestWorld(World);
		return false;
	}

	FUAVWreckInitParams Params = MakeDefaultWreckParams();
	Params.InitialState.Position = FVector(0.0f, 0.0f, 600.0f);
	Params.InitialState.Velocity = FVector(0.0f, 0.0f, -100.0f);
	Params.CollisionRadiusCm = 120.0f;
	Wreck->InitializeWreck(Params);

	const float InitialZ = Wreck->GetWreckState().Position.Z;
	TickWorldForSeconds(World, 0.75f, 1.0f / 60.0f);
	const float MidZ = Wreck->GetWreckState().Position.Z;
	TickWorldForSeconds(World, 3.0f, 1.0f / 60.0f);
	const float FinalZ = Wreck->GetWreckState().Position.Z;
	UPrimitiveComponent* Body = Wreck->GetBodyComponentForTest();

	TestTrue(FString::Printf(TEXT("Wreck should fall under Chaos gravity: InitialZ=%.1f MidZ=%.1f FinalZ=%.1f VelZ=%.1f BodyZ=%.1f BodyVelZ=%.1f"),
		InitialZ, MidZ, FinalZ, Wreck->GetWreckState().Velocity.Z,
		Body ? Body->GetComponentLocation().Z : -99999.0f,
		Body ? Body->GetPhysicsLinearVelocity().Z : -99999.0f), MidZ < InitialZ - 10.0f);
	TestTrue(FString::Printf(TEXT("Wreck should remain above ground after collision solving: FinalZ=%.1f"), FinalZ), FinalZ > -5.0f);

	DestroyUAVCrashTestWorld(World);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUAVPawnStateFollowsWreckAfterPhysicsTickTest,
	"UAVSimulator.Core.UAVPawn.StateFollowsWreckAfterPhysicsTick",
	UAV_TEST_FLAGS)

bool FUAVPawnStateFollowsWreckAfterPhysicsTickTest::RunTest(const FString& Parameters)
{
	UWorld* World = CreateUAVCrashTestWorld(TEXT("UAVPawnStateFollowsWreckAfterPhysicsTick"));
	TestNotNull(TEXT("World should be created"), World);
	if (!World)
	{
		return false;
	}

	SpawnBlockingGround(World);

	AUAVPawn* Pawn = SpawnCrashTestPawn(World, FVector(0.0f, 0.0f, 600.0f));
	TestNotNull(TEXT("Pawn should spawn"), Pawn);
	if (!Pawn)
	{
		DestroyUAVCrashTestWorld(World);
		return false;
	}

	FUAVState CrashState;
	CrashState.Position = FVector(0.0f, 0.0f, 600.0f);
	CrashState.Rotation = FRotator::ZeroRotator;
	CrashState.Velocity = FVector(0.0f, 0.0f, -200.0f);
	Pawn->SetUAVStateForTest(CrashState);
	Pawn->RecordSafeCrashPoseForTest();
	Pawn->TestTriggerCrash();

	const float InitialPawnZ = Pawn->GetUAVState().Position.Z;
	TickWorldForSeconds(World, 1.0f, 1.0f / 60.0f);
	Pawn->Tick(1.0f / 60.0f);

	const FUAVState PawnState = Pawn->GetUAVState();
	const AUAVWreckActor* Wreck = Pawn->GetActiveWreckActorForTest();
	TestNotNull(TEXT("Wreck should exist"), Wreck);
	TestTrue(FString::Printf(TEXT("Pawn state should move down with wreck: InitialZ=%.1f PawnZ=%.1f WreckZ=%.1f WreckVelZ=%.1f"),
		InitialPawnZ, PawnState.Position.Z, Wreck ? Wreck->GetWreckState().Position.Z : 0.0f, Wreck ? Wreck->GetWreckState().Velocity.Z : 0.0f),
		PawnState.Position.Z < InitialPawnZ - 10.0f);
	const FVector WreckPosition = Wreck ? Wreck->GetWreckState().Position : FVector::ZeroVector;
	const FRotator WreckRotation = Wreck ? Wreck->GetWreckState().Rotation : FRotator::ZeroRotator;
	UAV_TEST_VECTOR_EQUAL(PawnState.Position, WreckPosition, 2.0f);
	UAV_TEST_ROTATOR_EQUAL(PawnState.Rotation, WreckRotation, 1.0f);

	DestroyUAVCrashTestWorld(World);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
