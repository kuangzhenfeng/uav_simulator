#pragma once

#include "CoreMinimal.h"
#include "../MultiAgent/AgricultureTypes.h"

class ACooperationGameMode;

enum class ECooperationObject : uint8 { None, Agent, Airport, Plot };

struct FCooperationAgentView
{
    FAgricultureAgentState State;
    FVector Position = FVector::ZeroVector;
    float Yaw = 0;
    float SpeedMetres = 0;
    TArray<FVector> Route;
};

struct FCooperationPlotView
{
    int32 ID = INDEX_NONE;
    TArray<int32> AgentIDs;
    FString AgentLabel() const
    {
        TArray<FString> Labels;for(int32 AgentID:AgentIDs) Labels.Add(FString::Printf(TEXT("%d"),AgentID));
        return Labels.IsEmpty() ? TEXT("—") : FString::Join(Labels,TEXT(", "));
    }
    FBox Bounds = FBox(ForceInit);
    double Area = 0;
    double Covered = 0;
    double AppliedLitres = 0;
    bool bCompleted = false;
    bool bFailed = false;
    TArray<FBox> Coverage;
    float Progress() const { return Area > 0 ? FMath::Clamp(float(Covered / Area), 0.f, 1.f) : 0.f; }
};

/** 界面和三维覆盖共用只读快照，不从描述文本解析运行状态。 */
struct FCooperationViewData
{
    TArray<FCooperationAgentView> Agents;
    TArray<FSupplyAirportState> Airports;
    TArray<FCooperationPlotView> Plots;
    TArray<FString> Events;
    uint64 Revision = 0;
    ECooperationObject Selection = ECooperationObject::None;
    int32 SelectedID = INDEX_NONE;
    int32 Preset = 0;
    int32 Alerts = 0;
    int32 CompletedPlots = 0;
    float Seconds = 0;
    float Speed = 1;
    float TankCapacity = 50;
    float BatteryReserve = .15f;
    float MinSeparationMetres = -1;
    float SafetyMetres = 0;
    double Covered = 0;
    double Area = 0;
    bool bPaused = true;
    bool bFinished = false;
    bool bEnabled = false;
    FString Reason;
    FBox MapBounds = FBox(ForceInit);

    void Capture(const ACooperationGameMode& Manager);
    void AddEvent(const FString& Message);
    static FString PhaseName(EAgriculturePhase Phase);
    static FString AirportStatus(const FSupplyAirportState& Airport);
    static FLinearColor AgentColor(int32 ID);
    float Progress() const { return Area > 0 ? FMath::Clamp(float(Covered / Area), 0.f, 1.f) : 0.f; }
};
