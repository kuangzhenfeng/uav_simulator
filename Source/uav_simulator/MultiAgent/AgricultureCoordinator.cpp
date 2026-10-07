#include "AgricultureCoordinator.h"
#include "AgentManager.h"
#include "TaskAllocator.h"
#include "TaskMonitor.h"
#include "../Core/UAVPawn.h"
#include "../Mission/MissionComponent.h"
#include "../Scenario/ScenarioTypes.h"
#include "../Planning/AStarPathPlanner.h"
#include "../Planning/ObstacleManager.h"
#include "../Planning/TrajectoryTracker.h"
#include "DrawDebugHelpers.h"
#include "Json.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "EngineUtils.h"

DEFINE_LOG_CATEGORY_STATIC(LogAgriculture, Log, All);

UAgricultureCoordinator::UAgricultureCoordinator()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UAgricultureCoordinator::Reset()
{
    bCollectSupplyRequests=false;PendingSupplyRequests.Empty();
    Agents.Empty(); Airports.Empty(); Plots.Empty(); PreviousPositions.Empty();StatusMaterials.Empty();VisualSeconds=0;
    Config = FAgricultureConfig(); ElapsedSeconds = 0; LogSeconds = 0; LastReason.Empty();
}

bool UAgricultureCoordinator::BuildStrips(const FAgriculturePlot& P, float Height, TArray<FVector>& Out)
{
    Out.Empty();
    if(P.Boundary.Num()!=4 || P.StripSpacingCm<=0 || P.ApplicationLitresPerHa<=0) return false;
    FBox Bounds(P.Boundary);
    if(Bounds.GetSize().X<1 || Bounds.GetSize().Y<1) return false;
    // 首版只接受轴对齐矩形，避免将任意多边形包围盒误当作有效地块。
    for(const FVector& V:P.Boundary)
        if((!FMath::IsNearlyEqual(V.X,Bounds.Min.X) && !FMath::IsNearlyEqual(V.X,Bounds.Max.X)) ||
           (!FMath::IsNearlyEqual(V.Y,Bounds.Min.Y) && !FMath::IsNearlyEqual(V.Y,Bounds.Max.Y))) return false;
    const int32 Count=FMath::CeilToInt(Bounds.GetSize().Y/P.StripSpacingCm);
    for(int32 I=0;I<Count;++I)
    {
        const float Y=Bounds.Min.Y+(I+0.5f)*Bounds.GetSize().Y/Count;
        const float X0=I%2 ? Bounds.Max.X : Bounds.Min.X;
        const float X1=I%2 ? Bounds.Min.X : Bounds.Max.X;
        Out.Add(FVector(X0,Y,Height)); Out.Add(FVector(X1,Y,Height));
    }
    return !Out.IsEmpty();
}

FVector UAgricultureCoordinator::FindEmergencyLandingSite(const FVector& Position,const TArray<FAgriculturePlotState>& Fields,float ClearanceCm)
{
    const auto Clear=[&](const FVector& Point)
    {
        for(const auto& Field:Fields)
        {
            const FBox Bounds(Field.Config.Boundary);
            if(Bounds.IsValid && Bounds.ComputeSquaredDistanceToPoint(FVector(Point.X,Point.Y,0))<FMath::Square(ClearanceCm)-.1f) return false;
        }
        return true;
    };
    if(Clear(Position)) return Position;
    FVector Best=Position;double Distance=DBL_MAX;
    for(const auto& Field:Fields)
    {
        const FBox Bounds(Field.Config.Boundary);if(!Bounds.IsValid) continue;
        const float X=FMath::Clamp(Position.X,Bounds.Min.X,Bounds.Max.X);
        const float Y=FMath::Clamp(Position.Y,Bounds.Min.Y,Bounds.Max.Y);
        const FVector Candidates[]={FVector(Bounds.Min.X-ClearanceCm,Y,Position.Z),FVector(Bounds.Max.X+ClearanceCm,Y,Position.Z),
            FVector(X,Bounds.Min.Y-ClearanceCm,Position.Z),FVector(X,Bounds.Max.Y+ClearanceCm,Position.Z)};
        for(const FVector& Candidate:Candidates) if(Clear(Candidate) && FVector::DistSquared(Position,Candidate)<Distance)
        {Best=Candidate;Distance=FVector::DistSquared(Position,Candidate);}
    }
    return Best;
}

int32 UAgricultureCoordinator::ChooseNearestAirport(const TArray<FSupplyAirportState>& Stations,
    const TArray<float>& Distances,float FlightSeconds,float Speed,float Reserve,const TArray<float>& TravelSeconds,const TArray<float>& WaitingSeconds)
{
    int32 Best=INDEX_NONE; float BestDistance=MAX_FLT;
    if(Speed<=0) return Best;
    for(int32 I=0;I<Stations.Num() && I<Distances.Num();++I)
    {
        const auto& S=Stations[I];
        if(!S.Config.bEnabled || S.Config.WaterLitres<=0 || S.Config.ConcentrateLitres<=0 ||
           S.WasteLitres>=S.Config.WasteCapacityLitres || !FMath::IsFinite(Distances[I]) || Distances[I]<0) continue;
        const float Wait=WaitingSeconds.IsValidIndex(I) ? WaitingSeconds[I] : (S.OccupantID==INDEX_NONE ? 0 : S.Config.ChargeSeconds)+S.Queue.Num()*S.Config.ChargeSeconds;
        const float Travel=TravelSeconds.IsValidIndex(I) ? TravelSeconds[I] : Distances[I]/Speed;
        if(!FMath::IsFinite(Travel) || !FMath::IsFinite(Wait) || Travel+Wait+Reserve>FlightSeconds) continue;
        if(Distances[I]<BestDistance || (FMath::IsNearlyEqual(Distances[I],BestDistance) &&
            (Best==INDEX_NONE || S.Config.AirportID<Stations[Best].Config.AirportID)))
        { Best=I; BestDistance=Distances[I]; }
    }
    return Best;
}

void UAgricultureCoordinator::Initialize(UScenario* S,AMultiAgentGameMode* GM)
{
    Reset(); Manager=GM;
    if(!S || !GM) return;
    Config=S->Agriculture;
    if(!Config.bEnabled) return;
    for(const auto& C:S->SupplyAirports) { FSupplyAirportState A; A.Config=C; Airports.Add(A); }
    for(const auto& C:S->Plots)
    {
        FAgriculturePlotState P; P.Config=C;
        P.bFailed=!BuildStrips(C,C.CanopyHeightCm+Config.HeightAboveCanopyCm,P.StripPoints);
        Plots.Add(P);
    }
    for(AUAVPawn* P:GM->GetScenarioFleet()) if(P)
    {
        P->SetExternallyDriven(true);
        FAgricultureAgentState A; A.AgentID=P->GetAgentID(); A.Battery=Config.InitialBatteryFraction;
        A.LiquidLitres=Config.InitialLiquidLitres; A.Recipe=TEXT("CropA");
        for(auto& Station:Airports)
            if(FVector::Dist(P->GetUAVState().Position,Station.Config.DockPosition)<1)
            {
                P->ConfigureLandingSurface(Station.Config.DockPosition.Z);
                if(P->ParkOnSurface()) {A.AirportID=Station.Config.AirportID;Station.OccupantID=A.AgentID;}
                break;
            }
        Agents.Add(A);
        PreviousPositions.Add(A.AgentID,P->GetUAVState().Position);
        P->UpdatePayloadMass(A.LiquidLitres);
    }
    LastReason=TEXT("农业场景装配完成");
    UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] Initialized agents=%d airports=%d plots=%d"),Agents.Num(),Airports.Num(),Plots.Num());
}

bool UAgricultureCoordinator::RecordSpraySegment(FAgriculturePlotState& P,float& Liquid,
    const FVector& Previous,const FVector& Position,float Band)
{
    if(P.NextPoint%2!=1 || !P.StripPoints.IsValidIndex(P.NextPoint)) return false;
    const FVector Start=P.StripPoints[P.NextPoint-1],End=P.StripPoints[P.NextPoint];
    const FVector Dir=(End-Start).GetSafeNormal();const float Length=FVector::Dist(Start,End);
    const float Before=FVector::DotProduct(Previous-Start,Dir),After=FVector::DotProduct(Position-Start,Dir);
    // 条带入口前仅转场，不计覆盖；保留轨迹供高度瞬态收敛。
    if(Before<=0 && After<=0) return true;
    if(FVector::Dist(Previous,Start+Dir*Before)>Band || FVector::Dist(Position,Start+Dir*After)>Band ||
        (P.StripCoveredCm<Length-1 && Before>P.StripCoveredCm+1 && After>P.StripCoveredCm+1)) return false;
    const float Advance=FMath::Max(0.0f,FMath::Min(FMath::Clamp(After,0.0f,Length)-P.StripCoveredCm,FMath::Max(0.0f,After-Before)));
    const FBox Bounds(P.Config.Boundary);const float Width=Bounds.GetSize().Y/(P.StripPoints.Num()/2);
    const float LitresPerCm=Width/10000*P.Config.ApplicationLitresPerHa/10000;
    const float Applied=FMath::Min(Advance,FMath::Max(0.0f,Liquid)/LitresPerCm);
    P.StripCoveredCm+=Applied;P.CoveredSquareMetres+=Applied*Width/10000;
    P.AppliedLitres+=Applied*LitresPerCm;Liquid=FMath::Max(0.0f,Liquid-Applied*LitresPerCm);
    return true;
}

float UAgricultureCoordinator::TransferLiquid(FAgricultureAgentState& A,FSupplyAirportState& S,
    float Request,float Capacity,float Fraction)
{
    if(Request<=0 || Fraction<=0 || Fraction>=1 || !S.Config.bEnabled) return 0;
    float Fill=FMath::Min(Request,FMath::Max(0.0f,Capacity-A.LiquidLitres));
    Fill=FMath::Min(Fill,S.Config.WaterLitres/(1-Fraction));
    Fill=FMath::Min(Fill,S.Config.ConcentrateLitres/Fraction);
    Fill=FMath::Max(0.0f,Fill);
    A.LiquidLitres+=Fill;S.Config.WaterLitres-=Fill*(1-Fraction);S.Config.ConcentrateLitres-=Fill*Fraction;
    return Fill;
}

AUAVPawn* UAgricultureCoordinator::Pawn(int32 ID) const
{
    if(Manager) for(AUAVPawn* P:Manager->GetScenarioFleet()) if(P && P->GetAgentID()==ID) return P;
    return nullptr;
}
FAgriculturePlotState* UAgricultureCoordinator::Plot(int32 ID)
{ return Plots.FindByPredicate([ID](const auto& P){return P.Config.Task.TaskID==ID;}); }
FSupplyAirportState* UAgricultureCoordinator::Airport(int32 ID)
{ return Airports.FindByPredicate([ID](const auto& A){return A.Config.AirportID==ID;}); }

void UAgricultureCoordinator::ChangePhase(FAgricultureAgentState& A,EAgriculturePhase Phase)
{
    if(A.Phase==Phase) return;
    UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] Agent=%d Phase=%s Plot=%d Airport=%d Battery=%.3f Liquid=%.2f"),
        A.AgentID,*UEnum::GetValueAsString(Phase),A.PlotID,A.AirportID,A.Battery,A.LiquidLitres);
    A.Phase=Phase; A.PhaseSeconds=0; A.DockStableSeconds=0; A.bRouteActive=false;A.RouteRetries=0;A.TransitStage=0;
    if(Phase==EAgriculturePhase::Servicing) A.bCleaned=false;
}

bool UAgricultureCoordinator::FlyTo(FAgricultureAgentState& A,const FVector& Target,float Speed)
{
    AUAVPawn* P=Pawn(A.AgentID);
    if(!P) return false;
    if(A.bRouteActive && A.Target.Equals(Target,1))
    {
        if(!P->GetTrajectoryTracker()->IsTimedOut()) return true;
        if(++A.RouteRetries>2) {Fail(A,TEXT("Trajectory tracking timeout"));return false;}
        A.bRouteActive=false;LastReason=TEXT("轨迹超时重新规划");
    }
    A.Target=Target;
    P->GetTrajectoryTracker()->SetCompletionCriteria(A.Phase==EAgriculturePhase::Landing ? Config.DockRadiusCm : Config.ArrivalRadiusCm,
        A.Phase==EAgriculturePhase::Landing ? Config.DockSpeedCm : 50);
    if(P->GetMissionComponent()->GetMissionState()==EMissionState::Failed)
        P->GetMissionComponent()->ResetMission();
    if(!Manager->StartRoute(P,Target,Speed,Manager->TaskExecutionAccelerationCm))
    { Fail(A,TEXT("Route infeasible")); return false; }
    A.bRouteActive=true; return true;
}

bool UAgricultureCoordinator::CanService(const FSupplyAirportState& S) const
{
    if(!S.Config.bEnabled || S.Config.WaterLitres<=0 || S.Config.ConcentrateLitres<=0 || S.WasteLitres>=S.Config.WasteCapacityLitres) return false;
    const auto* Occupant=Agents.FindByPredicate([&S](const auto& A){return A.AgentID==S.OccupantID;});
    return !Occupant || (Occupant->Phase!=EAgriculturePhase::Completed && Occupant->Phase!=EAgriculturePhase::Failed && Occupant->PlotID!=INDEX_NONE);
}

float UAgricultureCoordinator::EstimateWaitSeconds(const FSupplyAirportState& S,float ArrivalSeconds,int32 WaitingAgentID) const
{
    const float Departure=Config.CruiseTimeFactor*(FMath::Max(0.0f,Config.TransitHeightCm-float(S.Config.DockPosition.Z))/200.0f+1000.0f/150.0f)+2*Config.RouteArrivalAllowanceSeconds;
    float Remaining=0;
    if(const auto* A=Agents.FindByPredicate([&S](const auto& V){return V.AgentID==S.OccupantID;}))
    {
        if(A->Phase==EAgriculturePhase::Resuming || A->Phase==EAgriculturePhase::TakingOff)
        {
            Remaining=Departure;
            if(const auto* P=Pawn(A->AgentID))
            {
                const auto* Tracker=P->GetTrajectoryTracker();
                if(A->bRouteActive && Tracker->GetTrajectory().bIsValid)
                {
                    Remaining=FMath::Max(0.0f,Tracker->GetTrajectory().TotalDuration-Tracker->GetCurrentTime())+Config.RouteArrivalAllowanceSeconds;
                    if(A->TransitStage==0) Remaining+=Config.CruiseTimeFactor*1000.0f/150.0f+Config.RouteArrivalAllowanceSeconds;
                }

            }
        }
        else
        {
            const auto* Field=Plots.FindByPredicate([&A](const auto& V){return V.Config.Task.TaskID==A->PlotID;});
            const bool NeedsCleaning=!Field || A->Recipe!=Field->Config.Recipe || A->bCleaned;
            const bool FlushPending=NeedsCleaning && !A->bCleaned;
            const float Liquid=Field ? (FlushPending ? Config.TankCapacityLitres : Config.TankCapacityLitres-A->LiquidLitres) : 0;
            const float Prep=S.Config.MixSeconds+(NeedsCleaning ? S.Config.CleanSeconds : 0);
            const float Elapsed=A->Phase==EAgriculturePhase::Servicing ? A->PhaseSeconds : 0;
            const float LiquidTime=FMath::Max(0.0f,Prep-Elapsed)+Liquid/FMath::Max(.001f,S.Config.RefillLitresPerMinute/60);
            float Approach=A->Phase==EAgriculturePhase::Servicing ? 0 : Config.AirportApproachSeconds;
            if(const auto* P=Pawn(A->AgentID))
            {
                const auto* Tracker=P->GetTrajectoryTracker();
                const bool Active=A->bRouteActive && Tracker->GetTrajectory().bIsValid;
                const float Route=Active ? FMath::Max(0.0f,Tracker->GetTrajectory().TotalDuration-Tracker->GetCurrentTime())+Config.RouteArrivalAllowanceSeconds : 0;
                if(A->Phase==EAgriculturePhase::Returning)
                {
                    if(Active) Approach+=Route+(A->TransitStage==0 ? Config.CruiseTimeFactor*FVector::Dist2D(A->Target,S.Config.DockPosition)/Config.TransitSpeedCm+Config.RouteArrivalAllowanceSeconds : 0);
                    else Approach+=Config.CruiseTimeFactor*(FMath::Abs(Config.TransitHeightCm-float(P->GetUAVState().Position.Z))/150.0f+FVector::Dist2D(P->GetUAVState().Position,S.Config.DockPosition)/Config.TransitSpeedCm)+2*Config.RouteArrivalAllowanceSeconds;
                }
                else if(A->Phase==EAgriculturePhase::Approaching)
                {
                    const FVector Target=S.Config.DockPosition+FVector(0,0,300);
                    const float Pending=Active ? Route : Config.CruiseTimeFactor*FVector::Dist(P->GetUAVState().Position,Target)/100.0f+Config.RouteArrivalAllowanceSeconds;
                    Approach=Pending+Config.CruiseTimeFactor*330.0f/30.0f+Config.RouteArrivalAllowanceSeconds+Config.DockStableSeconds;
                }
                else if(A->Phase==EAgriculturePhase::Landing)
                {
                    const FVector Target=S.Config.DockPosition-FVector(0,0,30);
                    Approach=(Active ? Route : Config.CruiseTimeFactor*FVector::Dist(P->GetUAVState().Position,Target)/30.0f+Config.RouteArrivalAllowanceSeconds)+FMath::Max(0.0f,Config.DockStableSeconds-A->DockStableSeconds);
                }
            }
            const float ChargeTime=FMath::Min(1.0f,1-A->Battery+Approach/Config.BatteryFlightSeconds)*S.Config.ChargeSeconds;
            Remaining=Approach+FMath::Max(ChargeTime,LiquidTime)+Departure;
        }
    }
    const int32 Ahead=WaitingAgentID==INDEX_NONE ? S.Queue.Num() : FMath::Max(0,S.Queue.Find(WaitingAgentID));
    Remaining+=Ahead*(S.Config.ChargeSeconds+Config.AirportApproachSeconds+Departure);
    return FMath::Max(0.0f,Remaining-ArrivalSeconds);
}

FString UAgricultureCoordinator::ServiceStage(const FAgricultureAgentState& A) const
{
    if(A.Phase!=EAgriculturePhase::Servicing) return TEXT("");
    const auto* S=Airports.FindByPredicate([&A](const auto& V){return V.Config.AirportID==A.AirportID;});
    if(!S) return TEXT("无服务机场");
    const auto* P=Plots.FindByPredicate([&A](const auto& V){return V.Config.Task.TaskID==A.PlotID;});
    const bool Cleaning=!P || A.bCleaned || A.Recipe!=P->Config.Recipe;
    if(Cleaning && !A.bCleaned) return TEXT("清洗/充电");
    if(A.PhaseSeconds<S->Config.MixSeconds+(Cleaning ? S->Config.CleanSeconds : 0)) return TEXT("配液/充电");
    if(P && A.LiquidLitres<Config.TankCapacityLitres-.01f) return TEXT("加液/充电");
    return A.Battery<.999f ? TEXT("充电") : TEXT("服务完成");
}

void UAgricultureCoordinator::AssignPlots()
{
    TArray<int32> Order;
    for(int32 I=0;I<Plots.Num();++I)
        if(!Plots[I].bCompleted && !Plots[I].bFailed && Plots[I].AgentID==INDEX_NONE) Order.Add(I);
    Order.Sort([this](int32 L,int32 R){
        const auto& A=Plots[L].Config.Task; const auto& B=Plots[R].Config.Task;
        return A.Priority==B.Priority ? A.TaskID<B.TaskID : A.Priority>B.Priority;
    });
    for(int32 I:Order)
    {
        auto& P=Plots[I]; FAgricultureAgentState* Best=nullptr;
        TArray<FUAVCapability> Capabilities;
        for(auto& A:Agents) if((A.Phase==EAgriculturePhase::Idle || A.Phase==EAgriculturePhase::Completed) && A.PlotID==INDEX_NONE)
            if(AUAVPawn* UAV=Pawn(A.AgentID))
            {
                FUAVCapability C;C.AgentID=A.AgentID;C.MaxPayloadKg=Config.TankCapacityLitres;
                C.MaxSpeed=Config.TransitSpeedCm;C.CurrentPosition=UAV->GetUAVState().Position;
                C.RemainingFlightTime=FMath::Max(0.0f,P.Config.Task.LatestFinish-ElapsedSeconds);
                Capabilities.Add(C);
            }
        if(Capabilities.IsEmpty()) break;
        FTaskDescriptor Task=P.Config.Task;Task.TargetLocation=P.StripPoints[P.NextPoint];
        Task.EstimatedDuration=(P.StripPoints.Num()-P.NextPoint)*10;
        Task.Deadline-=ElapsedSeconds;Task.LatestFinish-=ElapsedSeconds;
        const auto Result=Manager->GetTaskAllocator()->Allocate({Task},Capabilities,Manager->TaskAllocationConfig);
        if(!Result.bIsFeasible || Result.Assignments.IsEmpty()) {P.bFailed=true;LastReason=TEXT("地块任务不可行");continue;}
        const int32 Assigned=Result.Assignments[0].AgentID;
        Best=Agents.FindByPredicate([Assigned](const auto& A){return A.AgentID==Assigned;});
        if(!Best) break;
        P.AgentID=Best->AgentID; Best->PlotID=P.Config.Task.TaskID;
        ChangePhase(*Best,Pawn(Best->AgentID)->IsParked() ? EAgriculturePhase::TakingOff : EAgriculturePhase::Transit);
    }
}

void UAgricultureCoordinator::ReleaseAirport(FAgricultureAgentState& A)
{
    if(auto* S=Airport(A.AirportID))
    { S->Queue.Remove(A.AgentID); if(S->OccupantID==A.AgentID) S->OccupantID=INDEX_NONE; }
    A.AirportID=INDEX_NONE;
}

void UAgricultureCoordinator::Fail(FAgricultureAgentState& A,const FString& Reason)
{
    LastReason=Reason;
    const AUAVPawn* FailedPawn=Pawn(A.AgentID);
    const auto* Reserved=Airport(A.AirportID);
    const bool AtBerth=FailedPawn && Reserved && (FailedPawn->IsParked() || FVector::Dist2D(FailedPawn->GetUAVState().Position,Reserved->Config.DockPosition)<500);
    if(!AtBerth) ReleaseAirport(A);
    else if(auto* S=Airport(A.AirportID)) {S->OccupantID=A.AgentID;S->Queue.Remove(A.AgentID);}
    if(auto* Field=Plot(A.PlotID))
    {
        Field->AgentID=INDEX_NONE;
        if(Field->NextPoint%2==1 && Pawn(A.AgentID)) Field->ResumePosition=Field->StripPoints[Field->NextPoint-1]+(Field->StripPoints[Field->NextPoint]-Field->StripPoints[Field->NextPoint-1]).GetSafeNormal()*Field->StripCoveredCm;
    }
    A.PlotID=INDEX_NONE;
    ChangePhase(A,EAgriculturePhase::Failed);
    if(AUAVPawn* P=Pawn(A.AgentID))
    {
        P->StopTrajectoryTracking(); P->SetTargetPosition(P->GetUAVState().Position);
        if(!P->IsCrashed() && !P->IsParked())
        {
            // 先在安全高度消除水平速度，再使用受控下降，避免长下降曲线携带水平惯性。
            const FVector Velocity(P->GetUAVState().Velocity.X,P->GetUAVState().Velocity.Y,0);
            A.Target=P->GetUAVState().Position+Velocity*Velocity.Size()/300;
            A.bRouteActive=Manager->StartRoute(P,A.Target,FMath::Max(150.0f,float(Velocity.Size())),150);
            if(!A.bRouteActive)
                UE_LOG(LogAgriculture,Warning,TEXT("[Agriculture] Emergency braking route unavailable; holding before descent Agent=%d"),A.AgentID);
        }
    }
    UE_LOG(LogAgriculture,Warning,TEXT("[Agriculture] Agent=%d Failed=%s"),A.AgentID,*Reason);
}

void UAgricultureCoordinator::RefreshSupplyRoutes(FAgricultureAgentState& A,bool Force)
{
    if(!Force && A.SupplyForecastTime>=0 && ElapsedSeconds-A.SupplyForecastTime<.5f) return;
    AUAVPawn* P=Pawn(A.AgentID);if(!P) return;
    if(!SupplyPlanner) SupplyPlanner=NewObject<UAStarPathPlanner>(this);
    TArray<FObstacleInfo> Obstacles=P->GetObstacleManager()->GetAllObstacles();
    Obstacles.RemoveAll([P](const FObstacleInfo& O){
        const AUAVPawn* Other=Cast<AUAVPawn>(O.LinkedActor.Get());
        return Other && (Other==P || (!Other->IsParked() && !Other->IsCrashed()));
    });
    SupplyPlanner->SetObstacles(Obstacles);
    A.SupplyPathLengths.Empty();A.SupplyTravelSeconds.Empty();
    const FVector Position=P->GetUAVState().Position;
    FVector Lift=Position;Lift.Z=Config.TransitHeightCm;
    const float LiftDistance=FVector::Dist(Position,Lift);
    const bool LiftClear=!SupplyPlanner->CheckLineCollision(Position,Lift,P->GetCollisionRadius());
    for(const auto& S:Airports)
    {
        FVector Goal=S.Config.DockPosition;Goal.Z=Config.TransitHeightCm;
        const auto* Tracker=P->GetTrajectoryTracker();
        const bool Retained=A.AirportID==S.Config.AirportID &&
            (A.Phase==EAgriculturePhase::Returning || A.Phase==EAgriculturePhase::Waiting) && A.bRouteActive && Tracker->GetTrajectory().bIsValid;
        const FVector Start=Retained ? A.Target : Lift;
        TArray<FVector> Path;
        if(Retained || LiftClear)
        {
            if(!SupplyPlanner->CheckLineCollision(Start,Goal,P->GetCollisionRadius())) Path={Start,Goal};
            else SupplyPlanner->PlanPath(Start,Goal,Path);
        }
        float Distance=MAX_FLT,Travel=MAX_FLT;
        if(!Path.IsEmpty())
        {
            Distance=0;for(int32 I=1;I<Path.Num();++I) Distance+=FVector::Dist(Path[I-1],Path[I]);
            const float CurrentLeg=Retained ? FMath::Max(0.0f,Tracker->GetTrajectory().TotalDuration-Tracker->GetCurrentTime())+Config.RouteArrivalAllowanceSeconds :
                Config.CruiseTimeFactor*LiftDistance/150.0f+(LiftDistance>Config.ArrivalRadiusCm ? Config.RouteArrivalAllowanceSeconds : 0);
            Travel=CurrentLeg+Config.CruiseTimeFactor*Distance/Config.TransitSpeedCm+(Distance>1 ? Config.RouteArrivalAllowanceSeconds : 0)+Config.AirportApproachSeconds+1;
            Distance+=Retained ? FVector::Dist(Position,Start) : LiftDistance;
        }
        A.SupplyPathLengths.Add(Distance);A.SupplyTravelSeconds.Add(Travel);
    }
    A.SupplyForecastTime=ElapsedSeconds;
}

bool UAgricultureCoordinator::RedirectSupplyReservation(FAgricultureAgentState& Request,int32 ExcludedAirportID)
{
    const float Reserve=Config.BatteryReserveFraction*Config.BatteryFlightSeconds+Config.SupplyTimeBufferSeconds;
    for(int32 I=0;I<Airports.Num();++I)
    {
        auto& Station=Airports[I];if(Station.Config.AirportID==ExcludedAirportID) continue;
        for(auto& Other:Agents)
        {
            if(Other.AgentID==Request.AgentID || Other.AirportID!=Station.Config.AirportID ||
                (Other.Phase!=EAgriculturePhase::Returning && Other.Phase!=EAgriculturePhase::Waiting)) continue;
            const auto* OtherPawn=Pawn(Other.AgentID);
            if(!OtherPawn || OtherPawn->IsParked() || OtherPawn->IsGroundContact()) continue;
            // 只改变尚未进近的预约；接地、服务和离场的泊位不能被抢占。
            FSupplyAirportState Released=Station;
            Released.Queue.Remove(Other.AgentID);
            if(Released.OccupantID==Other.AgentID) Released.OccupantID=INDEX_NONE;
            if(!CanService(Released)) continue;
            const float Travel=Request.SupplyTravelSeconds[I];
            const float Wait=EstimateWaitSeconds(Released,FMath::Max(0.0f,Travel-Config.AirportApproachSeconds));
            if(ChooseNearestAirport({Released},{Request.SupplyPathLengths[I]},Request.Battery*Config.BatteryFlightSeconds,
                Config.TransitSpeedCm,Reserve,{Travel},{Wait})==INDEX_NONE) continue;
            RefreshSupplyRoutes(Other,true);
            TArray<FSupplyAirportState> Alternatives=Airports;TArray<float> Waiting;
            for(int32 J=0;J<Alternatives.Num();++J)
            {
                if(J==I || !CanService(Alternatives[J])) Alternatives[J].Config.bEnabled=false;
                Waiting.Add(EstimateWaitSeconds(Alternatives[J],FMath::Max(0.0f,Other.SupplyTravelSeconds[J]-Config.AirportApproachSeconds),Other.AgentID));
            }
            const int32 Alternate=ChooseNearestAirport(Alternatives,Other.SupplyPathLengths,Other.Battery*Config.BatteryFlightSeconds,
                Config.TransitSpeedCm,Reserve,Other.SupplyTravelSeconds,Waiting);
            if(Alternate==INDEX_NONE) continue;
            ReleaseAirport(Other);Other.AirportID=Airports[Alternate].Config.AirportID;Other.RequestTime=ElapsedSeconds;
            Airports[Alternate].Queue.AddUnique(Other.AgentID);ChangePhase(Other,EAgriculturePhase::Returning);
            UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] SupplyReassignment RequestAgent=%d RedirectedAgent=%d ReleasedAirport=%d AlternateAirport=%d"),
                Request.AgentID,Other.AgentID,Station.Config.AirportID,Other.AirportID);
            LastReason=TEXT("重匹配未进近预约，保护唯一可达机场");return true;
        }
    }
    return false;
}

void UAgricultureCoordinator::RequestSupply(FAgricultureAgentState& A,int32 ExcludedAirportID)
{
    AUAVPawn* P=Pawn(A.AgentID); if(!P || A.Phase==EAgriculturePhase::Failed) return;
    if(bCollectSupplyRequests) {PendingSupplyRequests.Add(A.AgentID,ExcludedAirportID);return;}
    if(auto* Field=Plot(A.PlotID))
        if(Field->NextPoint%2==1) Field->ResumePosition=Field->StripPoints[Field->NextPoint-1]+(Field->StripPoints[Field->NextPoint]-Field->StripPoints[Field->NextPoint-1]).GetSafeNormal()*Field->StripCoveredCm;
    ReleaseAirport(A);
    RefreshSupplyRoutes(A,true);
    const auto& Distances=A.SupplyPathLengths;
    const auto& TravelTimes=A.SupplyTravelSeconds;
    TArray<float> WaitingTimes;
    for(int32 I=0;I<Airports.Num();++I)
    {
        const float Wait=EstimateWaitSeconds(Airports[I],FMath::Max(0.0f,TravelTimes[I]-Config.AirportApproachSeconds));
        WaitingTimes.Add(Wait);
        UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] SupplyCandidate Agent=%d Airport=%d PathCm=%.0f Travel=%.1f Wait=%.1f Reserve=%.1f Buffer=%.1f Available=%.1f Healthy=%d"),
            A.AgentID,Airports[I].Config.AirportID,Distances[I],TravelTimes[I],Wait,
            Config.BatteryReserveFraction*Config.BatteryFlightSeconds,Config.SupplyTimeBufferSeconds,A.Battery*Config.BatteryFlightSeconds,CanService(Airports[I]) ? 1 : 0);
    }
    TArray<FSupplyAirportState> Candidates=Airports;
    for(auto& S:Candidates) if(S.Config.AirportID==ExcludedAirportID || !CanService(S)) S.Config.bEnabled=false;
    const int32 Index=ChooseNearestAirport(Candidates,Distances,A.Battery*Config.BatteryFlightSeconds,
        Config.TransitSpeedCm,Config.BatteryReserveFraction*Config.BatteryFlightSeconds+Config.SupplyTimeBufferSeconds,TravelTimes,WaitingTimes);
    if(Index==INDEX_NONE)
    {
        if(RedirectSupplyReservation(A,ExcludedAirportID)) {RequestSupply(A,ExcludedAirportID);return;}
        Fail(A,TEXT("No reachable supply airport"));return;
    }
    A.AirportID=Airports[Index].Config.AirportID; A.RequestTime=ElapsedSeconds;
    Airports[Index].Queue.AddUnique(A.AgentID);
    ChangePhase(A,EAgriculturePhase::Returning);
    LastReason=TEXT("预约最近可达补给机场");
}

bool UAgricultureCoordinator::IsFlying(int32 ID) const
{
    const auto* A=Agents.FindByPredicate([ID](const auto& S){return S.AgentID==ID;});
    return !A || A->Phase!=EAgriculturePhase::Servicing;
}
int32 UAgricultureCoordinator::CompletedCount() const
{ int32 N=0;for(const auto& P:Plots) if(P.bCompleted) ++N;return N; }
bool UAgricultureCoordinator::HasFailed() const
{ for(const auto& P:Plots) if(P.bFailed) return true;return false; }
bool UAgricultureCoordinator::ReadyToFinish() const
{
    if(Plots.IsEmpty() || CompletedCount()!=Plots.Num()) return false;
    for(const auto& A:Agents)
    {
        if(A.Phase!=EAgriculturePhase::Completed && A.Phase!=EAgriculturePhase::Failed) return false;
        if(A.Phase==EAgriculturePhase::Failed) if(const auto* P=Pawn(A.AgentID)) if(!P->IsParked() && !P->IsCrashed()) return false;
    }
    return true;
}

bool UAgricultureCoordinator::CleanResidue(FAgricultureAgentState& A,FSupplyAirportState& S,FName Recipe)
{
    if(A.bCleaned) return A.Recipe==Recipe;
    const float Waste=A.LiquidLitres+5;
    if(S.Config.WaterLitres<5 || S.WasteLitres+Waste>S.Config.WasteCapacityLitres) return false;
    S.Config.WaterLitres-=5;S.WasteLitres+=Waste;A.LiquidLitres=0;A.Recipe=Recipe;A.bCleaned=true;
    return true;
}

void UAgricultureCoordinator::UpdateAirport(FSupplyAirportState& S,float Dt)
{
    S.Queue.Sort([this](int32 L,int32 R){
        const auto* A=Agents.FindByPredicate([L](const auto& V){return V.AgentID==L;});
        const auto* B=Agents.FindByPredicate([R](const auto& V){return V.AgentID==R;});
        if(!A || !B) return L<R;
        const bool LowA=A->Battery<Config.BatteryReserveFraction*2;
        const bool LowB=B->Battery<Config.BatteryReserveFraction*2;
        if(LowA!=LowB) return LowA;
        return A->RequestTime==B->RequestTime ? L<R : A->RequestTime<B->RequestTime;
    });
    if(S.Config.bEnabled && S.OccupantID==INDEX_NONE && !S.Queue.IsEmpty())
    { S.OccupantID=S.Queue[0]; S.Queue.RemoveAt(0); }
    for(auto& A:Agents) if(A.AirportID==S.Config.AirportID)
    {
        if(A.Phase==EAgriculturePhase::Failed) continue;
        if(!S.Config.bEnabled || S.Config.WaterLitres<=0 || S.Config.ConcentrateLitres<=0 ||
            S.WasteLitres>=S.Config.WasteCapacityLitres)
        {
            if(auto* P=Pawn(A.AgentID)) P->ReleaseFromSurface();
            RequestSupply(A);LastReason=FString::Printf(TEXT("机场 %d 停用或资源不足，改降"),S.Config.AirportID);continue;
        }
        if(A.Phase!=EAgriculturePhase::Servicing || S.OccupantID!=A.AgentID) continue;
        AUAVPawn* P=Pawn(A.AgentID);
        if(!P || !P->IsParked() || !P->IsGroundContact()) {Fail(A,TEXT("Dock contact lost"));continue;}
        auto* Field=Plot(A.PlotID);
        const FName Recipe=Field ? Field->Config.Recipe : A.Recipe;
        const float Fraction=Field ? Field->Config.ConcentrateFraction : 0.02f;
        A.Battery=FMath::Min(1.0f,A.Battery+Dt/FMath::Max(1.0f,S.Config.ChargeSeconds));
        const bool NeedsCleaning=A.Recipe!=Recipe || A.bCleaned || !Field;
        if((A.Recipe!=Recipe || !Field) && !A.bCleaned)
        {
            if(A.PhaseSeconds<S.Config.CleanSeconds) continue;
            if(!CleanResidue(A,S,Recipe)) {S.Config.bEnabled=false;continue;}
        }
        const float ReadyTime=S.Config.MixSeconds+(NeedsCleaning ? S.Config.CleanSeconds : 0);
        if(A.PhaseSeconds<ReadyTime) continue;
        if(Field) TransferLiquid(A,S,S.Config.RefillLitresPerMinute/60*Dt,Config.TankCapacityLitres,Fraction);
        P->UpdatePayloadMass(A.LiquidLitres);
        if(A.Battery>=0.999f && (!Field || A.LiquidLitres>=Config.TankCapacityLitres-0.01f))
        {
            if(!Field) {S.Queue.Remove(A.AgentID);ChangePhase(A,EAgriculturePhase::Completed);}
            else {P->ReleaseFromSurface();ChangePhase(A,EAgriculturePhase::Resuming);}
        }
    }
}

void UAgricultureCoordinator::UpdateAgent(FAgricultureAgentState& A,float Dt)
{
    AUAVPawn* P=Pawn(A.AgentID); if(!P) return;
    if(P->IsCrashed() && A.Phase!=EAgriculturePhase::Failed) {Fail(A,TEXT("Aircraft collision"));return;}
    if(A.Phase==EAgriculturePhase::Failed)
    {
        A.PhaseSeconds+=Dt;
        if(!P->IsParked() && !P->IsCrashed()) A.Battery=FMath::Max(0.0f,A.Battery-Dt/FMath::Max(1.0f,Config.BatteryFlightSeconds));
        if(P->IsGroundContact() && P->GetUAVState().Velocity.Size()<=Config.DockSpeedCm) {P->ParkOnSurface();return;}
        if(P->IsCrashed()) return;
        if(A.RouteRetries>0 && A.PhaseSeconds<5) return;
        // 制动路线不可行时，先等待位置控制消除速度，再从真实停稳位置下降。
        if(!A.bRouteActive && A.TransitStage==0 && P->GetUAVState().Velocity.Size()<5)
            A.Target=P->GetUAVState().Position;
        if(FVector::Dist(P->GetUAVState().Position,A.Target)<=30 && P->GetUAVState().Velocity.Size()<5)
        {
            P->ConfigureLandingSurface(150);
            FVector Target=P->GetUAVState().Position;
            int32 NextStage=A.TransitStage;float Speed=30;
            if(A.TransitStage==0)
            {
                Target=FindEmergencyLandingSite(Target,Plots,Manager->DefaultCBFQPConfig.DSafe+2*P->GetCollisionRadius()+Config.ArrivalRadiusCm);
                if(FVector::Dist2D(Target,P->GetUAVState().Position)>30) {NextStage=1;Speed=150;}
                else if(Target.Z>480) {Target.Z=450;NextStage=2;Speed=60;}
                else {Target.Z=120;NextStage=3;}
            }
            else if(A.TransitStage==1 && Target.Z>480) {Target.Z=450;NextStage=2;Speed=60;}
            else if(A.TransitStage<3) {Target.Z=120;NextStage=3;}
            else return;
            P->GetTrajectoryTracker()->SetCompletionCriteria(30,5);
            if(Manager->StartRoute(P,Target,Speed,Manager->TaskExecutionAccelerationCm))
            {A.Target=Target;A.bRouteActive=true;A.TransitStage=NextStage;A.RouteRetries=0;}
            else
            {P->StopTrajectoryTracking();P->SetTargetPosition(P->GetUAVState().Position);A.Target=P->GetUAVState().Position;A.bRouteActive=false;++A.RouteRetries;A.PhaseSeconds=0;}
        }
        return;
    }
    if(A.Phase==EAgriculturePhase::Completed) return;
    A.PhaseSeconds+=Dt;
    const FVector Position=P->GetUAVState().Position;
    const FVector Previous=PreviousPositions.FindRef(A.AgentID);
    PreviousPositions.Add(A.AgentID,Position);
    if(!P->IsParked()) A.Battery=FMath::Max(0.0f,A.Battery-Dt/FMath::Max(1.0f,Config.BatteryFlightSeconds));
    auto* Field=Plot(A.PlotID);
    auto* Station=Airport(A.AirportID);
    const auto Arrived=[&](float Radius){return FVector::Dist(Position,A.Target)<=Radius && P->GetUAVState().Velocity.Size()<5;};
    if(A.Phase==EAgriculturePhase::Idle)
    {
        if(!P->IsParked()) RequestSupply(A);
        return;
    }
    if(A.Phase==EAgriculturePhase::TakingOff || A.Phase==EAgriculturePhase::Resuming)
    {
        if(P->IsParked()) P->ReleaseFromSurface();
        FVector Target=Position;Target.Z=Config.TransitHeightCm;
        if(!A.bRouteActive && !FlyTo(A,Target,200)) return;
        if(Arrived(Config.ArrivalRadiusCm))
        {
            if(Station && A.TransitStage==0)
            {
                FVector Exit=Station->Config.DockPosition;
                Exit.X+=Exit.X<0 ? 1000 : -1000;Exit.Z=Config.TransitHeightCm;
                A.TransitStage=1;A.bRouteActive=false;if(!FlyTo(A,Exit,150)) return;
                return;
            }
            ReleaseAirport(A);
            if(Field && (A.LiquidLitres<1 || A.Battery<0.4f || A.Recipe!=Field->Config.Recipe)) RequestSupply(A);
            else ChangePhase(A,EAgriculturePhase::Transit);
        }
        return;
    }
    if(Field && (A.Phase==EAgriculturePhase::Transit || A.Phase==EAgriculturePhase::Spraying))
    {
        RefreshSupplyRoutes(A);
        float ReturnSeconds=MAX_FLT;
        for(int32 I=0;I<Airports.Num();++I) if(CanService(Airports[I]))
        {
            const float Travel=A.SupplyTravelSeconds[I];
            ReturnSeconds=FMath::Min(ReturnSeconds,Travel+EstimateWaitSeconds(Airports[I],FMath::Max(0.0f,Travel-Config.AirportApproachSeconds)));
        }
        float WorkSeconds=0;
        if(A.Phase==EAgriculturePhase::Spraying)
        {
            const float Length=FVector::Dist(Field->StripPoints[Field->NextPoint-1],Field->StripPoints[Field->NextPoint]);
            WorkSeconds=Config.CruiseTimeFactor*FMath::Max(0.0f,Length-Field->StripCoveredCm)/Config.SpraySpeedCm;
        }
        else
        {
            const FVector Target=Field->ResumePosition.IsNearlyZero() ? Field->StripPoints[Field->NextPoint] : Field->ResumePosition;
            const float Speed=A.TransitStage==1 ? 100 : FMath::Min(Config.TransitSpeedCm,400.0f);
            WorkSeconds=Config.CruiseTimeFactor*FVector::Dist(Position,Target)/Speed;
        }
        const float ReturnFraction=(ReturnSeconds+WorkSeconds+Config.SupplyTimeBufferSeconds)/Config.BatteryFlightSeconds+Config.BatteryReserveFraction;
        if(A.Recipe!=Field->Config.Recipe || A.LiquidLitres<1 || A.Battery<ReturnFraction)
        {
            const bool RecipeChanged=A.Recipe!=Field->Config.Recipe;const bool LiquidLow=A.LiquidLitres<1;RequestSupply(A);
            if(A.Phase!=EAgriculturePhase::Failed) LastReason=RecipeChanged ? TEXT("任务配方变化，返航清洗配液") : LiquidLow ? TEXT("药液不足，返航补给并保留断点") : TEXT("作业转场与返航电量储备触发补给");
            return;
        }
    }
    if(A.Phase==EAgriculturePhase::Transit)
    {
        if(!Field) {ChangePhase(A,EAgriculturePhase::Idle);return;}
        FVector Target=Field->ResumePosition.IsNearlyZero() ? Field->StripPoints[Field->NextPoint] : Field->ResumePosition;
        if(Field->NextPoint%2==0)
            Target-=(Field->StripPoints[Field->NextPoint+1]-Field->StripPoints[Field->NextPoint]).GetSafeNormal()*Config.ArrivalRadiusCm*1.5f;
        if(A.TransitStage==0 && ((A.bRouteActive && FMath::IsNearlyEqual(float(A.Target.Z),Config.TransitHeightCm,1.0f)) || FVector::Dist2D(Position,Target)>2000 || Position.Z>Target.Z+200))
        {
            FVector Cruise=Target;Cruise.Z=Config.TransitHeightCm;
            if(!FlyTo(A,Cruise,FMath::Min(Config.TransitSpeedCm,400.0f))) return;
            if(Arrived(Config.ArrivalRadiusCm)) {A.TransitStage=1;A.bRouteActive=false;}
            return;
        }
        if(!FlyTo(A,Target,A.TransitStage==1 ? 100 : Config.SpraySpeedCm)) return;
        if(Arrived(FMath::Min(Config.ArrivalRadiusCm,30.0f)))
        {
            if(Field->NextPoint%2==0) ++Field->NextPoint;
            Field->ResumePosition=FVector::ZeroVector;
            ChangePhase(A,EAgriculturePhase::Spraying);
        }
        return;
    }
    if(A.Phase==EAgriculturePhase::Spraying && Field)
    {
        const FVector End=Field->StripPoints[Field->NextPoint];
        const FVector Start=Field->StripPoints[Field->NextPoint-1];
        const FVector Direction=(End-Start).GetSafeNormal();
        const float Length=FVector::Dist(Start,End);
        if(!RecordSpraySegment(*Field,A.LiquidLitres,Previous,Position,Config.ArrivalRadiusCm))
        {
            Field->ResumePosition=Start+Direction*FMath::Max(-Config.ArrivalRadiusCm*1.5f,Field->StripCoveredCm-Config.ArrivalRadiusCm*1.5f);
            LastReason=TEXT("回飞未施用的连续断点");ChangePhase(A,EAgriculturePhase::Transit);return;
        }
        P->UpdatePayloadMass(A.LiquidLitres);
        if(!FlyTo(A,End+Direction*Config.ArrivalRadiusCm*1.5f,Config.SpraySpeedCm)) return;
        DrawDebugLine(GetWorld(),Start,Position,FColor::Green,false,Dt*2,0,30);
        if(Arrived(Config.ArrivalRadiusCm) && Field->StripCoveredCm>=Length-1)
        {
            Field->StripCoveredCm=0;
            ++Field->NextPoint;
            if(Field->NextPoint>=Field->StripPoints.Num())
            {
                Field->bCompleted=true;Field->AgentID=INDEX_NONE;A.PlotID=INDEX_NONE;
                UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] Plot=%d Completed Area=%.2f Liquid=%.2f"),Field->Config.Task.TaskID,Field->CoveredSquareMetres,Field->AppliedLitres);
                ChangePhase(A,EAgriculturePhase::Idle);
            }
            else ChangePhase(A,EAgriculturePhase::Transit);
        }
        return;
    }
    if(A.Phase==EAgriculturePhase::Returning || A.Phase==EAgriculturePhase::Waiting)
    {
        if(!Station || (Station->OccupantID!=A.AgentID && !CanService(*Station))) {RequestSupply(A);return;}
        if(Station->OccupantID!=A.AgentID)
        {
            RefreshSupplyRoutes(A);
            const float Travel=A.SupplyTravelSeconds[Airports.IndexOfByPredicate([Station](const auto& S){return S.Config.AirportID==Station->Config.AirportID;})];
            if(A.Battery*Config.BatteryFlightSeconds < EstimateWaitSeconds(*Station,FMath::Max(0.0f,Travel-Config.AirportApproachSeconds),A.AgentID)+Travel+Config.BatteryReserveFraction*Config.BatteryFlightSeconds)
            {RequestSupply(A,Station->Config.AirportID);if(A.Phase!=EAgriculturePhase::Failed) LastReason=TEXT("等待电量储备不足，重新选择可达机场");return;}
        }
        if(A.TransitStage==0 && (A.bRouteActive || Position.Z<Config.TransitHeightCm-Config.ArrivalRadiusCm))
        {
            FVector Lift=Position;Lift.Z=Config.TransitHeightCm;
            if(A.bRouteActive) Lift=A.Target;
            if(!FlyTo(A,Lift,150)) return;
            if(Arrived(Config.ArrivalRadiusCm)) {A.TransitStage=1;A.bRouteActive=false;}
            return;
        }
        A.TransitStage=1;
        FVector Target=Station->Config.DockPosition;Target.Z=Config.TransitHeightCm;
        if(Station->OccupantID!=A.AgentID)
        {
            const int32 Slot=Station->Queue.Find(A.AgentID);
            if(!Station->Config.WaitingPoints.IsValidIndex(Slot)) {RequestSupply(A);return;}
            Target=Station->Config.WaitingPoints[Slot];
            ChangePhase(A,EAgriculturePhase::Waiting);
        }
        if(!FlyTo(A,Target,Config.TransitSpeedCm)) return;
        if(Station->OccupantID==A.AgentID && Arrived(Config.ArrivalRadiusCm)) ChangePhase(A,EAgriculturePhase::Approaching);
        return;
    }
    if(A.Phase==EAgriculturePhase::Approaching)
    {
        if(!Station) {RequestSupply(A);return;}
        FVector Target=Station->Config.DockPosition;Target.Z+=300;
        if(!FlyTo(A,Target,100)) return;
        if(Arrived(Config.ArrivalRadiusCm))
        {P->ConfigureLandingSurface(Station->Config.DockPosition.Z);ChangePhase(A,EAgriculturePhase::Landing);}
        return;
    }
    if(A.Phase==EAgriculturePhase::Landing)
    {
        if(!Station) {RequestSupply(A);return;}
        // 目标稍低于支承面，由接触约束产生实际落地。
        if(!FlyTo(A,Station->Config.DockPosition-FVector(0,0,30),30)) return;
        const bool Docked=FVector::Dist2D(Position,Station->Config.DockPosition)<=Config.DockRadiusCm &&
            P->IsGroundContact() && P->GetUAVState().Velocity.Size()<=Config.DockSpeedCm;
        A.DockStableSeconds=Docked ? A.DockStableSeconds+Dt : 0;
        if(A.DockStableSeconds>=Config.DockStableSeconds && P->ParkOnSurface()) ChangePhase(A,EAgriculturePhase::Servicing);
    }
}

void UAgricultureCoordinator::ResolveSupplyRequests()
{
    // 同一步请求先按可达机场的稀缺程度处理，每次预约后重新评估余下请求。
    while(!PendingSupplyRequests.IsEmpty())
    {
        FAgricultureAgentState* Best=nullptr;int32 BestCount=MAX_int32;
        for(auto& A:Agents)
        {
            const int32* Excluded=PendingSupplyRequests.Find(A.AgentID);if(!Excluded) continue;
            RefreshSupplyRoutes(A,true);
            int32 Count=0;
            for(int32 I=0;I<Airports.Num();++I)
            {
                const auto& S=Airports[I];if(S.Config.AirportID==*Excluded || !CanService(S)) continue;
                const float Travel=A.SupplyTravelSeconds[I];
                const float Wait=EstimateWaitSeconds(S,FMath::Max(0.0f,Travel-Config.AirportApproachSeconds),A.AgentID);
                if(FMath::IsFinite(Travel) && Travel+Wait+Config.BatteryReserveFraction*Config.BatteryFlightSeconds+Config.SupplyTimeBufferSeconds<=A.Battery*Config.BatteryFlightSeconds) ++Count;
            }
            if(!Best || Count<BestCount || (Count==BestCount && (A.Battery<Best->Battery || (A.Battery==Best->Battery && A.AgentID<Best->AgentID))))
            {Best=&A;BestCount=Count;}
        }
        if(!Best) {PendingSupplyRequests.Empty();break;}
        const int32 Excluded=PendingSupplyRequests.FindChecked(Best->AgentID);
        PendingSupplyRequests.Remove(Best->AgentID);RequestSupply(*Best,Excluded);
    }
}

void UAgricultureCoordinator::Update(float Dt)
{
    if(!Config.bEnabled || !Manager || Dt<=0) return;
    ElapsedSeconds+=Dt;
    AssignPlots();
    for(auto& S:Airports) UpdateAirport(S,Dt);
    bCollectSupplyRequests=true;
    for(auto& A:Agents) UpdateAgent(A,Dt);
    bCollectSupplyRequests=false;ResolveSupplyRequests();
    if(!Agents.ContainsByPredicate([](const auto& A){return A.Phase!=EAgriculturePhase::Failed;}))
        for(auto& P:Plots) if(!P.bCompleted) P.bFailed=true;
    for(auto& P:Plots)
    {
        const FBox Bounds(P.Config.Boundary);
        const float Area=Bounds.GetSize().X*Bounds.GetSize().Y/10000;
        if(!P.bCompleted && ElapsedSeconds>FMath::Min(P.Config.Task.Deadline,P.Config.Task.LatestFinish))
        {P.bFailed=true;LastReason=TEXT("地块作业超时");}
        Manager->GetTaskMonitor()->ReportExecution(P.Config.Task.TaskID,P.AgentID,
            P.bCompleted ? ETaskStatus::Completed : P.bFailed ? ETaskStatus::Failed : P.AgentID==INDEX_NONE ? ETaskStatus::Pending : ETaskStatus::InProgress,
            Area>0 ? P.CoveredSquareMetres/Area : 0);
    }
    LogSeconds+=Dt;
    VisualSeconds+=Dt;
    if(VisualSeconds>=1)
    {
        VisualSeconds=0;
        for(const auto& S:Airports)
        {
            auto* MID=StatusMaterials.Find(S.Config.AirportID);
            if(!MID)
            {
                const FName Tag(*FString::Printf(TEXT("SupplyAirport%d"),S.Config.AirportID));
                for(TActorIterator<AStaticMeshActor> It(GetWorld());It;++It)
                    if(It->ActorHasTag(Tag)) StatusMaterials.Add(S.Config.AirportID,It->GetStaticMeshComponent()->CreateDynamicMaterialInstance(0));
                MID=StatusMaterials.Find(S.Config.AirportID);
            }
            if(MID && *MID)
                (*MID)->SetVectorParameterValue(TEXT("Tint"),!S.Config.bEnabled || S.Config.WaterLitres<=0 || S.Config.ConcentrateLitres<=0 || S.WasteLitres>=S.Config.WasteCapacityLitres ? FLinearColor::Red :
                    S.OccupantID!=INDEX_NONE ? FLinearColor::Yellow : FLinearColor::Green);
        }
        for(const auto& P:Plots)
        {
            for(int32 I=0;I+1<P.NextPoint && I+1<P.StripPoints.Num();I+=2)
                DrawDebugLine(GetWorld(),P.StripPoints[I]-FVector(0,0,Config.HeightAboveCanopyCm-20),P.StripPoints[I+1]-FVector(0,0,Config.HeightAboveCanopyCm-20),FColor::Green,false,1.2f,0,60);
            if(P.NextPoint%2==1 && P.StripPoints.IsValidIndex(P.NextPoint))
            {
                const FVector Start=P.StripPoints[P.NextPoint-1];
                const FVector End=Start+(P.StripPoints[P.NextPoint]-Start).GetSafeNormal()*P.StripCoveredCm;
                DrawDebugLine(GetWorld(),Start-FVector(0,0,Config.HeightAboveCanopyCm-20),End-FVector(0,0,Config.HeightAboveCanopyCm-20),FColor::Green,false,1.2f,0,60);
            }
        }
    }
    if(LogSeconds>=5)
    {
        LogSeconds=0;
        UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] Progress=%d/%d Time=%.1f"),CompletedCount(),Plots.Num(),ElapsedSeconds);
    }
}

void UAgricultureCoordinator::Command(int32 Command,int32 ID)
{
    if(Command==0)
    {
        if(auto* A=Agents.FindByPredicate([ID](const auto& V){return V.AgentID==ID;}))
        {
            if(A->Phase==EAgriculturePhase::Failed || A->Phase==EAgriculturePhase::Completed || A->Phase==EAgriculturePhase::Servicing ||
                A->Phase==EAgriculturePhase::Returning || A->Phase==EAgriculturePhase::Waiting || A->Phase==EAgriculturePhase::Approaching || A->Phase==EAgriculturePhase::Landing) return;
            RequestSupply(*A);
        }
    }
    else if(Command==1 || Command==2)
    {if(auto* S=Airport(ID)) {if(Command==1) S->Config.bEnabled=!S->Config.bEnabled;else S->Config.WaterLitres=0;}}
    else if(Command==3)
    {if(auto* A=Agents.FindByPredicate([ID](const auto& V){return V.AgentID==ID;})) if(A->Phase!=EAgriculturePhase::Failed) Fail(*A,TEXT("Injected aircraft failure"));}
    else if(Command==4 && !Plots.IsEmpty())
    {
        FAgriculturePlotState P;P.Config=Plots[0].Config;int32 Next=0;
        for(const auto& V:Plots) Next=FMath::Max(Next,V.Config.Task.TaskID+1);
        P.Config.Task.TaskID=Next;P.Config.Task.Priority=ETaskPriority::Critical;
        P.Config.Boundary={FVector(-2000,11000,0),FVector(2000,11000,0),FVector(2000,13000,0),FVector(-2000,13000,0)};
        P.bFailed=!BuildStrips(P.Config,P.Config.CanopyHeightCm+Config.HeightAboveCanopyCm,P.StripPoints);
        Plots.Add(P);
        // 已完成覆盖保留；中断普通地块后让紧急任务参与下一次分配。
        for(auto& A:Agents) if(auto* Field=Plot(A.PlotID))
        {
            if(A.AirportID!=INDEX_NONE) continue;
            if(A.Phase==EAgriculturePhase::Spraying) Field->ResumePosition=Field->StripPoints[Field->NextPoint-1]+(Field->StripPoints[Field->NextPoint]-Field->StripPoints[Field->NextPoint-1]).GetSafeNormal()*Field->StripCoveredCm;
            Field->AgentID=INDEX_NONE;A.PlotID=INDEX_NONE;
            if(A.AirportID==INDEX_NONE) {Pawn(A.AgentID)->StopTrajectoryTracking();ChangePhase(A,EAgriculturePhase::Idle);}
        }
        LastReason=TEXT("紧急地块插入，保留已执行覆盖");
    }
}

FString UAgricultureCoordinator::Describe() const
{
    FString S=FString::Printf(TEXT("农业作业 %d / %d\n"),CompletedCount(),Plots.Num());
    static const TCHAR* Names[]={TEXT("待命"),TEXT("起飞"),TEXT("转场"),TEXT("喷洒"),TEXT("返航"),TEXT("等待"),TEXT("进近"),TEXT("降落"),TEXT("补给"),TEXT("起飞恢复"),TEXT("完成"),TEXT("故障")};
    for(const auto& A:Agents)
    {
        S+=FString::Printf(TEXT("UAV %d %s\n电量 %.0f%% 药液 %.1f L 地块 %d 机场 %d\n"),A.AgentID,
            Names[int32(A.Phase)],A.Battery*100,A.LiquidLitres,A.PlotID,A.AirportID);
        if(A.Phase==EAgriculturePhase::Servicing)
            if(const auto* Station=Airports.FindByPredicate([&A](const auto& V){return V.Config.AirportID==A.AirportID;}))
                S+=FString::Printf(TEXT("服务 %.1fs / 充电 %.0f%% %s\n"),A.PhaseSeconds,A.Battery*100,
                    *ServiceStage(A));
    }
    for(const auto& A:Airports) S+=FString::Printf(TEXT("机场 %d %s 占用 %d 排队 %d\n水 %.0f L 原液 %.1f L 废液 %.1f L\n"),A.Config.AirportID,
        A.Config.bEnabled ? TEXT("可用") : TEXT("停用"),A.OccupantID,A.Queue.Num(),A.Config.WaterLitres,A.Config.ConcentrateLitres,A.WasteLitres);
    for(const auto& P:Plots) S+=FString::Printf(TEXT("地块 %d → UAV %d 覆盖 %.0f m² %s\n"),P.Config.Task.TaskID,P.AgentID,P.CoveredSquareMetres,
        P.bCompleted ? TEXT("完成") : P.bFailed ? TEXT("不可行") : P.AgentID==INDEX_NONE ? TEXT("待分配") : TEXT("执行中"));
    return S+TEXT("最近调度：")+LastReason;
}

FString UAgricultureCoordinator::GetTelemetryJson() const
{
    TSharedRef<FJsonObject> Root=MakeShared<FJsonObject>();
    Root->SetStringField(TEXT("type"),TEXT("agriculture"));Root->SetNumberField(TEXT("t"),ElapsedSeconds);
    Root->SetNumberField(TEXT("completed"),CompletedCount());Root->SetNumberField(TEXT("total"),Plots.Num());
    Root->SetStringField(TEXT("reason"),LastReason);
    TArray<TSharedPtr<FJsonValue>> Values;
    for(const auto& A:Agents)
    {
        auto V=MakeShared<FJsonObject>();V->SetNumberField(TEXT("agentId"),A.AgentID);V->SetNumberField(TEXT("plotId"),A.PlotID);
        V->SetNumberField(TEXT("airportId"),A.AirportID);V->SetNumberField(TEXT("battery"),A.Battery);V->SetNumberField(TEXT("liquidLitres"),A.LiquidLitres);
        V->SetNumberField(TEXT("serviceSeconds"),A.PhaseSeconds);V->SetBoolField(TEXT("cleaned"),A.bCleaned);
        V->SetStringField(TEXT("serviceStage"),ServiceStage(A));
        V->SetStringField(TEXT("phase"),UEnum::GetValueAsString(A.Phase));Values.Add(MakeShared<FJsonValueObject>(V));
    }
    Root->SetArrayField(TEXT("agents"),Values);
    Values.Empty();
    for(const auto& A:Airports)
    {
        auto V=MakeShared<FJsonObject>();V->SetNumberField(TEXT("airportId"),A.Config.AirportID);
        V->SetNumberField(TEXT("occupantId"),A.OccupantID);V->SetBoolField(TEXT("enabled"),A.Config.bEnabled);
        V->SetNumberField(TEXT("waterLitres"),A.Config.WaterLitres);V->SetNumberField(TEXT("concentrateLitres"),A.Config.ConcentrateLitres);
        V->SetNumberField(TEXT("wasteLitres"),A.WasteLitres);
        TArray<TSharedPtr<FJsonValue>> Queue;for(int32 ID:A.Queue) Queue.Add(MakeShared<FJsonValueNumber>(ID));
        V->SetArrayField(TEXT("queue"),Queue);Values.Add(MakeShared<FJsonValueObject>(V));
    }
    Root->SetArrayField(TEXT("airports"),Values);Values.Empty();
    for(const auto& P:Plots)
    {
        auto V=MakeShared<FJsonObject>();V->SetNumberField(TEXT("taskId"),P.Config.Task.TaskID);V->SetNumberField(TEXT("agentId"),P.AgentID);
        V->SetNumberField(TEXT("nextPoint"),P.NextPoint);V->SetNumberField(TEXT("stripCoveredCm"),P.StripCoveredCm);
        V->SetNumberField(TEXT("coveredSquareMetres"),P.CoveredSquareMetres);V->SetNumberField(TEXT("appliedLitres"),P.AppliedLitres);
        V->SetBoolField(TEXT("completed"),P.bCompleted);V->SetBoolField(TEXT("failed"),P.bFailed);Values.Add(MakeShared<FJsonValueObject>(V));
    }
    Root->SetArrayField(TEXT("plots"),Values);
    FString Json;FJsonSerializer::Serialize(Root,TJsonWriterFactory<TCHAR,TCondensedJsonPrintPolicy<TCHAR>>::Create(&Json));return Json;
}

