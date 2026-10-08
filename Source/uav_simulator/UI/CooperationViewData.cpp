#include "CooperationViewData.h"
#include "../MultiAgent/CooperationGameMode.h"
#include "../MultiAgent/AgricultureCoordinator.h"
#include "../Core/UAVPawn.h"
#include "../Planning/TrajectoryTracker.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/WorldSettings.h"

FString FCooperationViewData::PhaseName(EAgriculturePhase Phase)
{
    static const TCHAR* Names[] = {TEXT("待命"), TEXT("起飞"), TEXT("转场"), TEXT("喷洒"), TEXT("返航"), TEXT("等待"),
        TEXT("进近"), TEXT("降落"), TEXT("补给"), TEXT("恢复起飞"), TEXT("完成"), TEXT("故障")};
    return Names[int32(Phase)];
}

FString FCooperationViewData::AirportStatus(const FSupplyAirportState& Airport)
{
    TArray<FString> Problems;
    if (!Airport.Config.bEnabled) Problems.Add(TEXT("停用"));
    if (Airport.Config.WaterLitres <= 0) Problems.Add(TEXT("缺水"));
    if (Airport.Config.ConcentrateLitres <= 0) Problems.Add(TEXT("原液不足"));
    if (Airport.WasteLitres >= Airport.Config.WasteCapacityLitres) Problems.Add(TEXT("废液罐已满"));
    if (!Problems.IsEmpty()) return FString::Join(Problems, TEXT(" / "));
    return Airport.OccupantID == INDEX_NONE ? TEXT("可用") : TEXT("占用");
}

FLinearColor FCooperationViewData::AgentColor(int32 ID)
{
    const FLinearColor Colors[] = {FLinearColor(.15f,.8f,1), FLinearColor(1,.6f,.15f),
        FLinearColor(.5f,.9f,.3f), FLinearColor(.9f,.4f,1)};
    return Colors[FMath::Abs(ID % 4)];
}

void FCooperationViewData::AddEvent(const FString& Message)
{
    Events.Insert(FString::Printf(TEXT("%02d:%02d  %s"), int32(Seconds)/60, int32(Seconds)%60, *Message), 0);
    if (Events.Num() > 12) Events.SetNum(12);
}

void FCooperationViewData::Capture(const ACooperationGameMode& Manager)
{
    const auto* Agriculture = Manager.GetAgriculture();
    if (!Agriculture) return;
    ++Revision;
    const float NewSeconds = Agriculture->GetElapsedSeconds();
    const bool bReset = NewSeconds < Seconds || Manager.GetPresetIndex() != Preset;
    TMap<int32, EAgriculturePhase> OldPhases;
    if (!bReset) for (const auto& Agent : Agents) OldPhases.Add(Agent.State.AgentID, Agent.State.Phase);
    else { Events.Empty(); Selection = ECooperationObject::None; SelectedID = INDEX_NONE; Reason.Empty(); }
    Seconds = NewSeconds;
    Preset = Manager.GetPresetIndex();
    bEnabled = Agriculture->IsEnabled();
    bPaused = UGameplayStatics::IsGamePaused(&Manager);
    bFinished = Agriculture->ReadyToFinish();
    Speed = Manager.GetWorld()->GetWorldSettings()->GetEffectiveTimeDilation();
    TankCapacity = Agriculture->GetConfig().TankCapacityLitres;
    BatteryReserve = Agriculture->GetConfig().BatteryReserveFraction;
    SafetyMetres = Manager.DefaultCBFQPConfig.DSafe / 100;
    Alerts = 0; CompletedPlots = 0; Covered = 0; Area = 0;
    Agents.Reset(); Plots.Reset(); Airports = Agriculture->GetAirports();
    MapBounds = FBox(ForceInit);
    const auto Fleet = Manager.GetScenarioFleet();
    for (const auto& State : Agriculture->GetAgents())
    {
        FCooperationAgentView View;
        View.State = State;
        for (const auto* Pawn : Fleet) if (IsValid(Pawn) && Pawn->GetAgentID() == State.AgentID)
        {
            View.Position = Pawn->GetUAVState().Position;
            View.Yaw = Pawn->GetUAVState().Rotation.Yaw;
            View.SpeedMetres = Pawn->GetUAVState().Velocity.Size() / 100;
            if (const auto* Tracker = Pawn->GetTrajectoryTracker(); Tracker && Tracker->IsTracking())
            {
                // 均匀采样剩余轨迹，限制绘制成本，保留真实规划曲线而非目标直线。
                const float Start = Tracker->GetCurrentTime();
                const float End = Tracker->GetTrajectory().TotalDuration;
                for (int32 I = 0; I <= 128 && End > Start; ++I)
                    View.Route.Add(Tracker->GetDesiredState(FMath::Lerp(Start, End, I / 128.f)).Position);
            }
            break;
        }
        if (State.Phase == EAgriculturePhase::Failed || State.Battery <= BatteryReserve) ++Alerts;
        if (const auto* Previous = OldPhases.Find(State.AgentID); Previous && *Previous != State.Phase)
            AddEvent(FString::Printf(TEXT("UAV %d：%s → %s"), State.AgentID, *PhaseName(*Previous), *PhaseName(State.Phase)));
        MapBounds += View.Position;
        Agents.Add(MoveTemp(View));
    }
    MinSeparationMetres = -1;
    for (int32 I = 0; I < Agents.Num(); ++I) for (int32 J = I+1; J < Agents.Num(); ++J)
    {
        const float Distance = FVector::Dist(Agents[I].Position, Agents[J].Position) / 100;
        MinSeparationMetres = MinSeparationMetres < 0 ? Distance : FMath::Min(MinSeparationMetres, Distance);
    }
    if (MinSeparationMetres >= 0 && MinSeparationMetres < SafetyMetres) ++Alerts;
    for (const auto& Airport : Airports)
    {
        MapBounds += Airport.Config.DockPosition;
        if (AirportStatus(Airport) != TEXT("可用") && AirportStatus(Airport) != TEXT("占用")) ++Alerts;
    }
    for (const auto& State : Agriculture->GetPlots())
    {
        FCooperationPlotView View;
        View.ID = State.Config.Task.TaskID; View.AgentID = State.AgentID;
        View.Bounds = FBox(State.Config.Boundary);
        View.Area = View.Bounds.GetSize().X * View.Bounds.GetSize().Y / 10000;
        View.Covered = State.CoveredSquareMetres; View.AppliedLitres = State.AppliedLitres;
        View.bCompleted = State.bCompleted; View.bFailed = State.bFailed;
        // 与 RecordSpraySegment 使用同一有效宽度，断点进度只延伸已覆盖部分。
        const int32 StripCount = State.StripPoints.Num() / 2;
        const float Width = StripCount > 0 ? View.Bounds.GetSize().Y / StripCount : 0;
        for (int32 I = 0; I+1 < State.StripPoints.Num(); I += 2)
        {
            const FVector Start = State.StripPoints[I];
            const FVector End = State.StripPoints[I+1];
            const float Length = FVector::Dist(Start, End);
            const float Completed = I+1 < State.NextPoint ? Length : I+1 == State.NextPoint ? State.StripCoveredCm : 0;
            if (Completed <= 0 || Width <= 0) continue;
            const FVector Tip = Start + (End-Start).GetSafeNormal() * FMath::Min(Completed, Length);
            const double Z = View.Bounds.Min.Z + State.Config.CanopyHeightCm + 25;
            View.Coverage.Add(FBox(FVector(FMath::Min(Start.X,Tip.X), Start.Y-Width/2, Z),
                FVector(FMath::Max(Start.X,Tip.X), Start.Y+Width/2, Z+2)));
        }
        Area += View.Area; Covered += View.Covered;
        CompletedPlots += View.bCompleted ? 1 : 0;
        Alerts += View.bFailed ? 1 : 0;
        MapBounds += View.Bounds;
        Plots.Add(MoveTemp(View));
    }
    const FString NewReason = Agriculture->GetLastReason();
    if (Reason != NewReason) { Reason = NewReason; if (!Reason.IsEmpty()) AddEvent(Reason); }
    if (!MapBounds.IsValid) MapBounds = FBox(FVector(-20000,-15000,0), FVector(20000,15000,0));
    MapBounds = MapBounds.ExpandBy(1500);
}
