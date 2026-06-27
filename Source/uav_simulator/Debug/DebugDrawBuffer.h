#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "DebugDrawBuffer.generated.h"

UENUM(BlueprintType)
enum class EDebugPrimType : uint8
{
	Sphere UMETA(DisplayName = "Sphere"),
	Line UMETA(DisplayName = "Line"),
	Arrow UMETA(DisplayName = "Arrow"),
	Box UMETA(DisplayName = "Box"),
	Point UMETA(DisplayName = "Point"),
	Text UMETA(DisplayName = "Text"),
};

USTRUCT(BlueprintType)
struct FBufferedPrimitive
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	EDebugPrimType Type = EDebugPrimType::Sphere;

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	TArray<FVector> Points;

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	float Radius = 0.0f;

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	float Thickness = 1.0f;

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	float ArrowSize = 0.0f;

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	FQuat Rotation = FQuat::Identity;

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	FString Text;

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	FColor Color = FColor::White;

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	float Duration = -1.0f;

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	int32 AgentID = -1;

	UPROPERTY(VisibleAnywhere, Category = "Debug")
	FString Layer;
};

/**
 * 缓冲式 DrawDebug 代理: 每个 Add* 方法同时调用原生 DrawDebug*(保持编辑器内可视化)
 * 并把原语推入帧缓冲, 供 TelemetryRecorder 序列化到 ndjson.
 */
UCLASS(ClassGroup = (Debug), meta = (DisplayName = "Debug Draw Buffer"))
class UAV_SIMULATOR_API UDebugDrawBuffer : public UGameInstanceSubsystem
{
	GENERATED_BODY()

public:
	static UDebugDrawBuffer* Get(const UObject* WorldContext);

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	void AddSphere(UWorld* World, const FVector& Pos, float Radius, const FColor& Color,
		float Duration = -1.0f, int32 AgentID = -1, const FString& Layer = TEXT("default"));

	void AddLine(UWorld* World, const FVector& A, const FVector& B, const FColor& Color,
		float Thickness = 1.0f, float Duration = -1.0f, int32 AgentID = -1, const FString& Layer = TEXT("default"));

	void AddArrow(UWorld* World, const FVector& A, const FVector& B, float ArrowSize,
		const FColor& Color, float Thickness = 1.0f, float Duration = -1.0f,
		int32 AgentID = -1, const FString& Layer = TEXT("default"));

	void AddBox(UWorld* World, const FVector& Center, const FVector& Extent, const FQuat& Rotation,
		const FColor& Color, float Duration = -1.0f, int32 AgentID = -1, const FString& Layer = TEXT("default"));

	void AddPoint(UWorld* World, const FVector& Pos, float Size, const FColor& Color,
		float Duration = -1.0f, int32 AgentID = -1, const FString& Layer = TEXT("default"));

	void AddText(UWorld* World, const FVector& Pos, const FString& Text, const FColor& Color,
		float Duration = -1.0f, int32 AgentID = -1, const FString& Layer = TEXT("default"));

	TArray<FBufferedPrimitive> FlushAndReset();

	bool IsEmpty() const { return FramePrims.Num() == 0; }

private:
	TArray<FBufferedPrimitive> FramePrims;
};
