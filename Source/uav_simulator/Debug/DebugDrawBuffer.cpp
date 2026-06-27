#include "DebugDrawBuffer.h"
#include "Engine/Engine.h"
#include "DrawDebugHelpers.h"

UDebugDrawBuffer* UDebugDrawBuffer::Get(const UObject* WorldContext)
{
	if (!WorldContext) return nullptr;
	if (UWorld* World = WorldContext->GetWorld())
	{
		if (UGameInstance* GI = World->GetGameInstance())
		{
			return GI->GetSubsystem<UDebugDrawBuffer>();
		}
	}
	return nullptr;
}

void UDebugDrawBuffer::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
}

void UDebugDrawBuffer::Deinitialize()
{
	FramePrims.Reset();
	Super::Deinitialize();
}

void UDebugDrawBuffer::AddSphere(UWorld* World, const FVector& Pos, float Radius,
	const FColor& Color, float Duration, int32 AgentID, const FString& Layer)
{
	if (World)
	{
		DrawDebugSphere(World, Pos, Radius, 12, Color, false, Duration, 0, 2.0f);
	}

	FBufferedPrimitive& P = FramePrims.AddDefaulted_GetRef();
	P.Type = EDebugPrimType::Sphere;
	P.Points.Add(Pos);
	P.Radius = Radius;
	P.Color = Color;
	P.Duration = Duration;
	P.AgentID = AgentID;
	P.Layer = Layer;
}

void UDebugDrawBuffer::AddLine(UWorld* World, const FVector& A, const FVector& B,
	const FColor& Color, float Thickness, float Duration, int32 AgentID, const FString& Layer)
{
	if (World)
	{
		DrawDebugLine(World, A, B, Color, false, Duration, 0, Thickness);
	}

	FBufferedPrimitive& P = FramePrims.AddDefaulted_GetRef();
	P.Type = EDebugPrimType::Line;
	P.Points.Add(A);
	P.Points.Add(B);
	P.Thickness = Thickness;
	P.Color = Color;
	P.Duration = Duration;
	P.AgentID = AgentID;
	P.Layer = Layer;
}

void UDebugDrawBuffer::AddArrow(UWorld* World, const FVector& A, const FVector& B,
	float ArrowSize, const FColor& Color, float Thickness, float Duration,
	int32 AgentID, const FString& Layer)
{
	if (World)
	{
		DrawDebugDirectionalArrow(World, A, B, ArrowSize, Color, false, Duration, 0, Thickness);
	}

	FBufferedPrimitive& P = FramePrims.AddDefaulted_GetRef();
	P.Type = EDebugPrimType::Arrow;
	P.Points.Add(A);
	P.Points.Add(B);
	P.ArrowSize = ArrowSize;
	P.Thickness = Thickness;
	P.Color = Color;
	P.Duration = Duration;
	P.AgentID = AgentID;
	P.Layer = Layer;
}

void UDebugDrawBuffer::AddBox(UWorld* World, const FVector& Center, const FVector& Extent,
	const FQuat& Rot, const FColor& Color, float Duration, int32 AgentID, const FString& Layer)
{
	if (World)
	{
		DrawDebugBox(World, Center, Extent, Rot, Color, false, Duration, 0, 2.0f);
	}

	FBufferedPrimitive& P = FramePrims.AddDefaulted_GetRef();
	P.Type = EDebugPrimType::Box;
	P.Points.Add(Center);
	P.Radius = Extent.X;
	P.Thickness = Extent.Y;
	P.ArrowSize = Extent.Z;
	P.Rotation = Rot;
	P.Color = Color;
	P.Duration = Duration;
	P.AgentID = AgentID;
	P.Layer = Layer;
}

void UDebugDrawBuffer::AddPoint(UWorld* World, const FVector& Pos, float Size,
	const FColor& Color, float Duration, int32 AgentID, const FString& Layer)
{
	if (World)
	{
		DrawDebugPoint(World, Pos, Size, Color, false, Duration);
	}

	FBufferedPrimitive& P = FramePrims.AddDefaulted_GetRef();
	P.Type = EDebugPrimType::Point;
	P.Points.Add(Pos);
	P.Thickness = Size;
	P.Color = Color;
	P.Duration = Duration;
	P.AgentID = AgentID;
	P.Layer = Layer;
}

void UDebugDrawBuffer::AddText(UWorld* World, const FVector& Pos, const FString& Text,
	const FColor& Color, float Duration, int32 AgentID, const FString& Layer)
{
	if (World)
	{
		DrawDebugString(World, Pos, Text, nullptr, Color, Duration);
	}

	FBufferedPrimitive& P = FramePrims.AddDefaulted_GetRef();
	P.Type = EDebugPrimType::Text;
	P.Points.Add(Pos);
	P.Text = Text;
	P.Color = Color;
	P.Duration = Duration;
	P.AgentID = AgentID;
	P.Layer = Layer;
}

TArray<FBufferedPrimitive> UDebugDrawBuffer::FlushAndReset()
{
	TArray<FBufferedPrimitive> Out = MoveTemp(FramePrims);
	FramePrims.Reset();
	return Out;
}
