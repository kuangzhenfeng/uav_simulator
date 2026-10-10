#include "AgricultureCoordinator.h"
#include "../Utility/Filter.h"
#include "AgentManager.h"
#include "TaskAllocator.h"
#include "TaskMonitor.h"
#include "../Core/UAVPawn.h"
#include "../Mission/MissionComponent.h"
#include "../Scenario/ScenarioTypes.h"
#include "../Planning/AStarPathPlanner.h"
#include "../Planning/ObstacleManager.h"
#include "../Planning/TrajectoryTracker.h"
#include "../Planning/CoveragePlanner.h"
#include "../Planning/TrajectoryOptimizer.h"
#include "DrawDebugHelpers.h"
#include "Json.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "EngineUtils.h"

DEFINE_LOG_CATEGORY(LogAgriculture);

UAgricultureCoordinator::UAgricultureCoordinator()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UAgricultureCoordinator::Reset()
{
    bCollectSupplyRequests=false;PendingSupplyRequests.Empty();SupplySlots.Empty();
    Agents.Empty(); Airports.Empty(); Plots.Empty(); Sections.Empty(); PreviousPositions.Empty();StatusMaterials.Empty();VisualSeconds=0;SupplyPlanTime=-1;AssignmentPlanTime=-1;
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

FVector UAgricultureCoordinator::SupplyDepartureTarget(const FVector& Position,const FVector& Velocity,float Height,float Acceleration)
{
    const float Duration=1.9f*Velocity.Size()/FMath::Max(1.0f,Acceleration);
    FVector Target=Position+Velocity*(Duration*.5f);Target.Z=Height;return Target;
}

bool UAgricultureCoordinator::BuildSupplyDeparture(const FVector& Position,const FVector& Velocity,float Height,float Acceleration,FTrajectory& Out)
{
    Out=FTrajectory();
    if(Acceleration<=0) return false;
    auto* Optimizer=NewObject<UTrajectoryOptimizer>();
    FVector Stop=Position;
    // 按当前三维速度制动至零，再从静止状态爬升；两段保持位置、速度、加速度连续。
    if(Velocity.Size()>1)
    {
        const float Duration=1.9f*Velocity.Size()/Acceleration;
        Stop=Position+Velocity*(Duration*.5f);
        Optimizer->SetStartVelocity(Velocity);
        Out=Optimizer->OptimizeTrajectoryWithTiming({Position,Stop},{Duration});
        if(!Out.bIsValid) return false;
    }
    const FVector Lift(Stop.X,Stop.Y,Height);
    if(!Stop.Equals(Lift,1))
    {
        Optimizer->SetStartVelocity(FVector::ZeroVector);
        const auto Climb=Optimizer->OptimizeTrajectory({Stop,Lift},150,Acceleration);
        if(!Climb.bIsValid) return false;
        if(Out.Points.IsEmpty()) Out=Climb;
        else
        {
            for(int32 I=1;I<Climb.Points.Num();++I)
            {auto Point=Climb.Points[I];Point.TimeStamp+=Out.TotalDuration;Out.Points.Add(Point);}
            Out.TotalDuration+=Climb.TotalDuration;
        }
    }
    return Out.bIsValid && Out.Points.Num()>=2;
}

bool UAgricultureCoordinator::StartSupplyDeparture(FAgricultureAgentState& A)
{
    auto* P=Pawn(A.AgentID);if(!P) return false;
    FTrajectory Route;
    if(!BuildSupplyDeparture(P->GetUAVState().Position,P->GetUAVState().Velocity,Config.TransitHeightCm,Manager->TaskExecutionAccelerationCm,Route) ||
       !Manager->IsRouteClear(P,Route))
    {Fail(A,TEXT("Supply departure infeasible"));return false;}
    A.Target=Route.Points.Last().Position;A.bRouteActive=true;
    P->GetTrajectoryTracker()->SetCompletionCriteria(Config.ArrivalRadiusCm,50);
    P->SetTrajectory(Route);P->StartTrajectoryTracking();return true;
}

TArray<FAgriculturePlotState> UAgricultureCoordinator::PartitionPlot(const FAgriculturePlotState& P,int32 MaxSections)
{
    TArray<FAgriculturePlotState> Result;
    const int32 Strips=P.StripPoints.Num()/2;
    if(P.bFailed || Strips==0 || MaxSections<=0) return Result;
    const int32 Count=FMath::Min(MaxSections,FMath::Max(1,Strips/2));
    const FBox Bounds(P.Config.Boundary);
    const double Width=Bounds.GetSize().Y/Strips;
    // 连续条带分区保留原航向，边界与有效喷幅一致，覆盖面积互不重叠。
    for(int32 I=0;I<Count;++I)
    {
        const int32 Begin=I*Strips/Count,End=(I+1)*Strips/Count;
        FAgriculturePlotState Part;Part.Config=P.Config;
        const double Y0=Bounds.Min.Y+Begin*Width,Y1=Bounds.Min.Y+End*Width;
        Part.Config.Boundary={FVector(Bounds.Min.X,Y0,Bounds.Min.Z),FVector(Bounds.Max.X,Y0,Bounds.Min.Z),
            FVector(Bounds.Max.X,Y1,Bounds.Min.Z),FVector(Bounds.Min.X,Y1,Bounds.Min.Z)};
        Part.StripPoints.Append(P.StripPoints.GetData()+Begin*2,(End-Begin)*2);
        Result.Add(MoveTemp(Part));
    }
    return Result;
}

FAgriculturePlotState* UAgricultureCoordinator::Section(const FAgricultureAgentState& A)
{
    if(!Sections.IsValidIndex(A.SectionID)) return nullptr;
    auto& S=Sections[A.SectionID];
    return S.Config.Task.TaskID==A.PlotID && S.AgentID==A.AgentID ? &S : nullptr;
}

void UAgricultureCoordinator::RefreshPlots()
{
    for(auto& P:Plots)
    {
        P.CoveredSquareMetres=0;P.AppliedLitres=0;P.AgentIDs.Empty();
        bool Complete=true,Found=false;
        for(const auto& S:Sections) if(S.Config.Task.TaskID==P.Config.Task.TaskID)
        {
            Found=true;Complete&=S.bCompleted;P.bFailed|=S.bFailed;
            P.CoveredSquareMetres+=S.CoveredSquareMetres;P.AppliedLitres+=S.AppliedLitres;
            if(S.AgentID!=INDEX_NONE) P.AgentIDs.AddUnique(S.AgentID);
        }
        P.AgentIDs.Sort();
        if(!P.bCompleted && Found && Complete)
            UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] Plot=%d Completed Area=%.2f Liquid=%.2f"),P.Config.Task.TaskID,P.CoveredSquareMetres,P.AppliedLitres);
        P.bCompleted=Found && Complete;
    }
}

FVector UAgricultureCoordinator::FindEmergencyLandingSite(const FVector& Position,const TArray<FAgriculturePlotState>& Fields,float ClearanceCm,
    const TArray<FObstacleInfo>& Obstacles,float CollisionRadius)
{
    auto* Planner=NewObject<UAStarPathPlanner>();
    Planner->SetObstacles(Obstacles);
    const auto Clear=[&](const FVector& Point)
    {
        for(const auto& Field:Fields)
        {
            const FBox Bounds(Field.Config.Boundary);
            if(Bounds.IsValid && Bounds.ComputeSquaredDistanceToPoint(FVector(Point.X,Point.Y,0))<FMath::Square(ClearanceCm)-.1f) return false;
        }
        // 使用与执行路线相同的安全壳检查完整下降柱，而非仅检查落点或农田边界。
        return !Planner->CheckLineCollision(Point,FVector(Point.X,Point.Y,120),CollisionRadius);
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
    for(const auto& Obstacle:Obstacles)
    {
        const FVector Padding=Obstacle.Extents+FVector(Obstacle.SafetyMargin+ClearanceCm);
        const float X=FMath::Clamp(Position.X,Obstacle.Center.X-Padding.X,Obstacle.Center.X+Padding.X);
        const float Y=FMath::Clamp(Position.Y,Obstacle.Center.Y-Padding.Y,Obstacle.Center.Y+Padding.Y);
        const FVector Candidates[]={FVector(Obstacle.Center.X-Padding.X,Y,Position.Z),FVector(Obstacle.Center.X+Padding.X,Y,Position.Z),
            FVector(X,Obstacle.Center.Y-Padding.Y,Position.Z),FVector(X,Obstacle.Center.Y+Padding.Y,Position.Z)};
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
        if(!S.Config.bEnabled || !FMath::IsFinite(Distances[I]) || Distances[I]<0) continue;
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
    if(!FMath::IsFinite(Config.LiquidDensityKgPerLitre) || Config.LiquidDensityKgPerLitre<=0 ||
        !FMath::IsFinite(Config.BatteryFlightSeconds) || Config.BatteryFlightSeconds<=0)
    {
        Config.bEnabled=false;LastReason=TEXT("药液密度和空载续航必须为有限正值");
        UE_LOG(LogAgriculture,Error,TEXT("[Agriculture] Invalid liquid density or reference endurance"));return;
    }
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
        const auto Spec=FUAVProductManager::GetModelSpec(P->GetModelID());
        A.EmptyMassKg=Spec.Mass;A.PayloadLimitKg=Spec.MaxPayloadKg;
        A.LiquidLitres=FMath::Clamp(Config.InitialLiquidLitres,0.f,FMath::Min(Config.TankCapacityLitres,Spec.MaxPayloadKg/FMath::Max(.001f,Config.LiquidDensityKgPerLitre))); A.Recipe=TEXT("CropA");
        for(auto& Station:Airports)
            if(FVector::Dist(P->GetUAVState().Position,Station.Config.DockPosition)<1)
            {
                P->ConfigureLandingSurface(Station.Config.DockPosition.Z);
                if(P->ParkOnSurface()) {A.AirportID=Station.Config.AirportID;Station.OccupantID=A.AgentID;}
                break;
            }
        Agents.Add(A);
        PreviousPositions.Add(A.AgentID,P->GetUAVState().Position);
        P->UpdatePayloadMass(A.LiquidLitres*Config.LiquidDensityKgPerLitre);
    }
    for(const auto& P:Plots) Sections.Append(PartitionPlot(P,Agents.Num()));
    RefreshPlots();
    LastReason=TEXT("农业场景装配完成");
    UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] Initialized agents=%d airports=%d plots=%d"),Agents.Num(),Airports.Num(),Plots.Num());
    for(const auto& Field:Plots)
        UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] PlotSchedule Plot=%d Priority=%d Deadline=%.2f LatestFinish=%.2f Area=%.2f"),
            Field.Config.Task.TaskID,int32(Field.Config.Task.Priority),Field.Config.Task.Deadline,Field.Config.Task.LatestFinish,
            FBox(Field.Config.Boundary).GetSize().X*FBox(Field.Config.Boundary).GetSize().Y/10000);
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

float UAgricultureCoordinator::DrainLiquid(FAgricultureAgentState& A,FSupplyAirportState& S,float Amount,float Target)
{
    if(!S.Config.bEnabled || Amount<=0 || Target<0) return 0;
    const float Drained=FMath::Min(Amount,FMath::Min(FMath::Max(0.f,A.LiquidLitres-Target),
        FMath::Max(0.f,S.Config.WasteCapacityLitres-S.WasteLitres)));
    A.LiquidLitres-=Drained;S.WasteLitres+=Drained;return Drained;
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
FSupplyAirportState* UAgricultureCoordinator::Airport(int32 ID)
{ return Airports.FindByPredicate([ID](const auto& A){return A.Config.AirportID==ID;}); }

void UAgricultureCoordinator::ChangePhase(FAgricultureAgentState& A,EAgriculturePhase Phase)
{
    if(A.Phase==Phase) return;
    UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] Agent=%d Phase=%s Plot=%d Airport=%d Battery=%.3f Liquid=%.2f"),
        A.AgentID,*UEnum::GetValueAsString(Phase),A.PlotID,A.AirportID,A.Battery,A.LiquidLitres);
    A.Phase=Phase; A.PhaseSeconds=0; A.DockStableSeconds=0; A.bRouteActive=false;A.RouteRetries=0;A.TransitStage=0;
    A.bWorkRouteActive=false;A.WorkStripEndTimes.Empty();A.WorkStartTime=0;
    A.PlannedSupplyAirportID=INDEX_NONE;A.PlannedSupplyDelaySeconds=0;SupplyPlanTime=-1;
    A.SupplyEntryTravelSeconds.Empty();
    if(Phase==EAgriculturePhase::Idle || Phase==EAgriculturePhase::Failed) AssignmentPlanTime=-1;
    if(Phase==EAgriculturePhase::Servicing) {A.bCleaned=false;A.ServiceLiquidTarget=-1;A.ServiceBatteryTarget=1;A.ServiceAirportTargetID=INDEX_NONE;A.ServiceDepartureNotBefore=0;A.ServicePlanTime=-1;A.bServiceCleanBeforeTransfer=false;}
}

bool UAgricultureCoordinator::FlyTo(FAgricultureAgentState& A,const FVector& Target,float Speed,const FVector& EndVelocity)
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
        A.Phase==EAgriculturePhase::Landing ? Config.DockSpeedCm : FMath::Max(50.0f,float(EndVelocity.Size())+50));
    if(P->GetMissionComponent()->GetMissionState()==EMissionState::Failed)
        P->GetMissionComponent()->ResetMission();
    Speed=FMath::Max(Speed,float(P->GetUAVState().Velocity.Size())+1);
    if(!Manager->StartRoute(P,Target,Speed,Manager->TaskExecutionAccelerationCm,EndVelocity))
    { Fail(A,TEXT("Route infeasible")); return false; }
    A.bRouteActive=true; return true;
}

bool UAgricultureCoordinator::StartWorkRoute(FAgricultureAgentState& A,FAgriculturePlotState& Field)
{
    auto* P=Pawn(A.AgentID);
    if(!P) return false;
    const int32 FirstEnd=Field.NextPoint%2==0 ? Field.NextPoint+1 : Field.NextPoint;
    if(!Field.StripPoints.IsValidIndex(FirstEnd)) return false;
    const FVector Direction=(Field.StripPoints[FirstEnd]-Field.StripPoints[FirstEnd-1]).GetSafeNormal();
    const float Headland=FCoveragePlanner::HeadlandDistance(Config.SpraySpeedCm,Manager->TaskExecutionAccelerationCm);
    const FVector Entry=Field.StripPoints[FirstEnd-1]+Direction*(Field.StripCoveredCm-Headland);
    TArray<FVector> Targets;
    const FVector Position=P->GetUAVState().Position;
    if(FVector::Dist2D(Position,Entry)>2000 || Position.Z>Entry.Z+200)
    {
        // 在当前航向前方完成爬升，避免长距离转场的平滑曲线贴地跨越障碍。
        FVector Departure=Position+P->GetUAVState().Velocity.GetSafeNormal2D()*Headland;
        Departure.Z=Config.TransitHeightCm;
        Targets.Add(Departure);
        FVector Cruise=Entry;Cruise.Z=Config.TransitHeightCm;Targets.Add(Cruise);
    }
    Targets.Add(Entry);
    FTrajectory Approach,Work;
    TMap<int32,float> EndTimes;
    const float EntrySpeed=FMath::Min(Config.SpraySpeedCm,100.0f);
    if(!Manager->PlanRoute(P,Targets,FMath::Min(Config.TransitSpeedCm,400.0f),Manager->TaskExecutionAccelerationCm,Direction*EntrySpeed,Approach))
    {Fail(A,TEXT("Coverage approach infeasible"));return false;}
    if(!FCoveragePlanner::Build(Field.StripPoints,FirstEnd,Field.StripCoveredCm,Approach,Config.SpraySpeedCm,Manager->TaskExecutionAccelerationCm,Work,EndTimes))
    {Fail(A,TEXT("Coverage profile infeasible"));return false;}
    if(!Manager->IsRouteClear(P,Work))
    {Fail(A,TEXT("Coverage clearance infeasible"));return false;}
    Field.NextPoint=FirstEnd;Field.ResumePosition=FVector::ZeroVector;
    A.bWorkRouteActive=true;A.bRouteActive=true;A.WorkStartTime=Approach.TotalDuration;
    A.WorkStripEndTimes=MoveTemp(EndTimes);A.Target=Work.Points.Last().Position;
    P->GetTrajectoryTracker()->SetCompletionCriteria(Config.ArrivalRadiusCm,50);
    if(P->GetMissionComponent()->GetMissionState()==EMissionState::Failed) P->GetMissionComponent()->ResetMission();
    P->SetTrajectory(Work);P->StartTrajectoryTracking();
    UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] CoveragePlan Agent=%d Plot=%d FirstEnd=%d Strips=%d Duration=%.2f Approach=%.2f Samples=%d"),
        A.AgentID,A.PlotID,FirstEnd,A.WorkStripEndTimes.Num(),Work.TotalDuration,A.WorkStartTime,Work.Points.Num());
    return true;
}

bool UAgricultureCoordinator::CanService(const FSupplyAirportState& S) const
{
    if(!S.Config.bEnabled) return false;
    const auto* Occupant=Agents.FindByPredicate([&S](const auto& A){return A.AgentID==S.OccupantID;});
    return !Occupant || (Occupant->Phase!=EAgriculturePhase::Completed && Occupant->Phase!=EAgriculturePhase::Failed && Occupant->PlotID!=INDEX_NONE);
}

float UAgricultureCoordinator::EstimateWaitSeconds(const FSupplyAirportState& S,float ArrivalSeconds,int32 WaitingAgentID) const
{
    const float Departure=Config.CruiseTimeFactor*(FMath::Max(0.0f,Config.TransitHeightCm-float(S.Config.DockPosition.Z))/200.0f+1000.0f/150.0f)+2*Config.RouteArrivalAllowanceSeconds;
    float Remaining=0;
    if(const auto* A=Agents.FindByPredicate([&S,WaitingAgentID](const auto& V){return V.AgentID==S.OccupantID && V.AgentID!=WaitingAgentID;}))
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
            const auto* Field=Sections.IsValidIndex(A->SectionID) ? &Sections[A->SectionID] : nullptr;
            // 估算占用者的服务时长时不再次估算队列，避免补给预测与等待预测相互递归。
            const auto Plan=Field && A->ServiceLiquidTarget==-1 ? PlanDockService(*A,*Field,S,false) : FAgricultureServicePlan();
            const bool LocalService=Field && (A->ServiceLiquidTarget>=0 || (A->ServiceLiquidTarget==-1 && Plan.LiquidLitres>=0));
            const bool SourceCleaning=Field && (A->bServiceCleanBeforeTransfer || (A->ServiceLiquidTarget==-1 && Plan.bCleanBeforeTransfer));
            const bool NeedsCleaning=(LocalService || SourceCleaning) && (A->Recipe!=Field->Config.Recipe || A->bCleaned);
            const bool FlushPending=NeedsCleaning && !A->bCleaned;
            const float TargetLiquid=Field ? (A->ServiceLiquidTarget>=0 ? A->ServiceLiquidTarget : A->ServiceLiquidTarget==-1 ? Plan.LiquidLitres : 0) : 0;
            const float TargetBattery=Field ? (A->ServiceLiquidTarget!=-1 ? A->ServiceBatteryTarget : Plan.BatteryFraction) : 1;
            const float Liquid=Field ? FMath::Max(0.f,TargetLiquid-(FlushPending ? 0 : A->LiquidLitres)) : 0;
            const float Drain=Field && !FlushPending && (A->ServiceLiquidTarget>=0 || (A->ServiceLiquidTarget==-1 && Plan.LiquidLitres>=0)) ?
                FMath::Max(0.f,A->LiquidLitres-TargetLiquid) : 0;
            const float Prep=LocalService ? S.Config.MixSeconds+(NeedsCleaning ? S.Config.CleanSeconds : 0) : SourceCleaning ? S.Config.CleanSeconds : 0;
            const float Elapsed=A->Phase==EAgriculturePhase::Servicing ? A->PhaseSeconds : 0;
            const float LiquidTime=FMath::Max(0.0f,Prep-Elapsed)+(Liquid+Drain)/FMath::Max(.001f,S.Config.RefillLitresPerMinute/60);
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
            const float ChargeTime=FMath::Clamp(TargetBattery-A->Battery+Approach/FlightSeconds(*A),0.f,1.f)*S.Config.ChargeSeconds;
            Remaining=Approach+FMath::Max(ChargeTime,LiquidTime)+Departure;
            Remaining=FMath::Max(Remaining,float(FMath::Max(0.0,A->ServiceDepartureNotBefore-ElapsedSeconds))+Departure);
        }
    }
    const int32 QueueIndex=S.Queue.Find(WaitingAgentID);
    const int32 Ahead=QueueIndex==INDEX_NONE ? S.Queue.Num() : QueueIndex;
    const int32 StationIndex=Airports.IndexOfByPredicate([&S](const auto& V){return V.Config.AirportID==S.Config.AirportID;});
    for(int32 I=0;I<Ahead;++I)
    {
        const auto* Queued=Agents.FindByPredicate([&S,I](const auto& V){return V.AgentID==S.Queue[I];});
        if(!Queued) {Remaining+=S.Config.ChargeSeconds+Config.AirportApproachSeconds+Departure;continue;}
        const float Travel=Queued->SupplyTravelSeconds.IsValidIndex(StationIndex) && Queued->SupplyTravelSeconds[StationIndex]<MAX_FLT ?
            Queued->SupplyTravelSeconds[StationIndex] : Config.AirportApproachSeconds;
        const float ApproachStart=FMath::Max(0.f,Travel-Config.AirportApproachSeconds);
        const float Holding=FMath::Max(0.f,Remaining-ApproachStart);
        auto Arriving=*Queued;Arriving.Battery=FMath::Max(0.f,Queued->Battery-(Travel+Holding)/FlightSeconds(*Queued));
        const auto* Field=Sections.IsValidIndex(Queued->SectionID) ? &Sections[Queued->SectionID] : nullptr;
        const auto Plan=Field ? PlanDockService(Arriving,*Field,S,false) : FAgricultureServicePlan();
        const bool Cleaning=Field && (Plan.LiquidLitres>=0 || Plan.bCleanBeforeTransfer) && Arriving.Recipe!=Field->Config.Recipe;
        const float TargetBattery=Field && Plan.AirportID!=INDEX_NONE ? Plan.BatteryFraction : 1;
        const float Fill=Field && Plan.LiquidLitres>=0 ? FMath::Max(0.f,Plan.LiquidLitres-(Cleaning ? 0 : Arriving.LiquidLitres)) : 0;
        const float Drain=Field && !Cleaning && Plan.LiquidLitres>=0 ? FMath::Max(0.f,Arriving.LiquidLitres-Plan.LiquidLitres) : 0;
        const float Preparation=Field && Plan.LiquidLitres>=0 ? S.Config.MixSeconds+(Cleaning ? S.Config.CleanSeconds : 0) : Plan.bCleanBeforeTransfer ? S.Config.CleanSeconds : 0;
        const float Service=FMath::Max((TargetBattery-Arriving.Battery)*S.Config.ChargeSeconds,
            Preparation+(Fill+Drain)/FMath::Max(.001f,S.Config.RefillLitresPerMinute/60));
        // 排队飞机到达进近区后才能占用泊位，充电和加液并行；不假定每架都从零充满。
        Remaining=FMath::Max(Remaining,ApproachStart)+
            Config.AirportApproachSeconds+Service+Departure;
    }
    return FMath::Max(0.0f,Remaining-ArrivalSeconds);
}

FString UAgricultureCoordinator::ServiceStage(const FAgricultureAgentState& A) const
{
    if(A.Phase!=EAgriculturePhase::Servicing) return TEXT("");
    if(A.ServiceLiquidTarget<-1 && !A.bServiceCleanBeforeTransfer) return TEXT("充电/改降");
    if(A.ServiceLiquidTarget>=0 && A.LiquidLitres>A.ServiceLiquidTarget+.01f) return TEXT("残液减载/充电");
    const auto* S=Airports.FindByPredicate([&A](const auto& V){return V.Config.AirportID==A.AirportID;});
    if(!S) return TEXT("无服务机场");
    const auto* P=Sections.IsValidIndex(A.SectionID) ? &Sections[A.SectionID] : nullptr;
    if(!P) return A.Battery<.999f ? TEXT("充电") : TEXT("服务完成");
    const bool Cleaning=A.bCleaned || A.Recipe!=P->Config.Recipe;
    if(Cleaning && !A.bCleaned) return TEXT("清洗/充电");
    if(A.ServiceLiquidTarget<-1) return TEXT("充电/改降");
    if(A.PhaseSeconds<S->Config.MixSeconds+(Cleaning ? S->Config.CleanSeconds : 0)) return TEXT("配液/充电");
    if(P && A.LiquidLitres<(A.ServiceLiquidTarget>=0 ? A.ServiceLiquidTarget : Config.TankCapacityLitres)-.01f) return TEXT("加液/充电");
    return A.Battery<A.ServiceBatteryTarget-.001f ? TEXT("充电") : TEXT("服务完成");
}

double UAgricultureCoordinator::RemainingWorkSeconds(const FAgriculturePlotState& Part,double& Liquid) const
{
    const int32 FirstEnd=Part.NextPoint%2==0 ? Part.NextPoint+1 : Part.NextPoint;
    double Length=0,Turns=0;
    for(int32 I=FirstEnd;I<Part.StripPoints.Num();I+=2)
    {
        Length+=FMath::Max(0.0,FVector::Dist(Part.StripPoints[I-1],Part.StripPoints[I])-(I==FirstEnd ? Part.StripCoveredCm : 0));
        if(I+2<Part.StripPoints.Num()) ++Turns;
    }
    const double Width=FBox(Part.Config.Boundary).GetSize().Y/FMath::Max(1,Part.StripPoints.Num()/2);
    Liquid=Length*Width/10000*Part.Config.ApplicationLitresPerHa/10000;
    const double Headland=FCoveragePlanner::HeadlandDistance(Config.SpraySpeedCm,Manager ? Manager->TaskExecutionAccelerationCm : 150);
    return Length/FMath::Max(1.f,Config.SpraySpeedCm)+Turns*(2*Headland/FMath::Max(1.f,Config.SpraySpeedCm)+PI*Width/FMath::Max(1.f,Config.SpraySpeedCm));
}

void UAgricultureCoordinator::AssignPlots()
{
    if(AssignmentPlanTime>=0 && ElapsedSeconds-AssignmentPlanTime<.5f) return;
    AssignmentPlanTime=ElapsedSeconds;
    TArray<FAgricultureAgentState*> Available;
    for(auto& A:Agents) if((A.Phase==EAgriculturePhase::Idle || A.Phase==EAgriculturePhase::Completed) && A.PlotID==INDEX_NONE && Pawn(A.AgentID)) Available.Add(&A);
    if(Available.IsEmpty()) return;
    TArray<int32> Candidates;
    for(int32 I=0;I<Sections.Num();++I) if(!Sections[I].bCompleted && !Sections[I].bFailed && Sections[I].AgentID==INDEX_NONE) Candidates.Add(I);
    if(Candidates.IsEmpty()) return;
    Candidates.Sort([this](int32 L,int32 R){return Sections[L].Config.Task.Priority>Sections[R].Config.Task.Priority;});
    const double PlanningStarted=FPlatformTime::Seconds();
    TArray<TArray<double>> Costs;double Maximum=1;
    TArray<TArray<FAgriculturePlotState>> Orders;
    for(auto* A:Available)
    {
        TArray<double> Row;TArray<FAgriculturePlotState> OrderedRow;
        auto* Aircraft=Pawn(A->AgentID);const auto* Dock=Airport(A->AirportID);
        const bool AtOwnDock=Aircraft->IsParked() && Dock && Dock->OccupantID==A->AgentID;
        if(AtOwnDock) RefreshSupplyRoutes(*A,true);
        for(int32 I:Candidates)
        {
            const FVector Position=Pawn(A->AgentID)->GetUAVState().Position;
            double Cost=DBL_MAX;
            OrderedRow.Add(ChooseWorkOrder(*A,Position,Sections[I],&Cost));
            if(Cost<DBL_MAX && AtOwnDock && PlanDockService(*A,OrderedRow.Last(),*Dock).AirportID==INDEX_NONE) Cost=DBL_MAX;
            Row.Add(Cost);if(Cost<DBL_MAX) Maximum=FMath::Max(Maximum,Cost);
        }
        Costs.Add(MoveTemp(Row));Orders.Add(MoveTemp(OrderedRow));
    }
    // 先保留所有可执行候选，避免不可行的高优先级任务阻塞机队；匹配数量相同时优先高优先级任务。
    const double PriorityPenalty=(2*Available.Num()+1)*Maximum;
    for(auto& Row:Costs) for(int32 J=0;J<Candidates.Num();++J) if(Row[J]<DBL_MAX)
        Row[J]+=PriorityPenalty*(int32(ETaskPriority::Critical)-int32(Sections[Candidates[J]].Config.Task.Priority));
    const auto Assigned=UTaskAllocator::MatchMinimumMakespan(Costs);
    const double PlanningMilliseconds=(FPlatformTime::Seconds()-PlanningStarted)*1000;
    if(PlanningMilliseconds>50)
        UE_LOG_THROTTLE(10.0,LogAgriculture,Log,TEXT("[Agriculture] AssignmentPlanning FarmTime=%.2f Milliseconds=%.2f Available=%d Candidates=%d"),ElapsedSeconds,PlanningMilliseconds,Available.Num(),Candidates.Num());
    for(int32 I=0;I<Available.Num();++I) if(Assigned[I]!=INDEX_NONE)
    {
        auto& A=*Available[I];const int32 SectionIndex=Candidates[Assigned[I]];auto& Part=Sections[SectionIndex];
        Part=MoveTemp(Orders[I][Assigned[I]]);
        Part.AgentID=A.AgentID;A.PlotID=Part.Config.Task.TaskID;A.SectionID=SectionIndex;
        UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] SectionAssigned Plot=%d Section=%d Agent=%d EstimatedSeconds=%.2f"),A.PlotID,SectionIndex,A.AgentID,
            Costs[I][Assigned[I]]-PriorityPenalty*(int32(ETaskPriority::Critical)-int32(Part.Config.Task.Priority)));
        auto* P=Pawn(A.AgentID);auto* Station=Airport(A.AirportID);
        const auto Service=Station ? PlanDockService(A,Part,*Station) : FAgricultureServicePlan();
        const bool SupplyFirst=A.Recipe!=Part.Config.Recipe || (Station && (Service.LiquidLitres<0 ||
            FMath::Abs(A.LiquidLitres-Service.LiquidLitres)>.01f || A.Battery<Service.BatteryFraction-.001f));
        if(P->IsParked() && Station && Station->OccupantID==A.AgentID && Station->Config.bEnabled && SupplyFirst)
            ChangePhase(A,EAgriculturePhase::Servicing);
        else ChangePhase(A,P->IsParked() ? EAgriculturePhase::TakingOff : EAgriculturePhase::Transit);
    }
}

void UAgricultureCoordinator::ReleaseAirport(FAgricultureAgentState& A)
{
    if(auto* S=Airport(A.AirportID))
    { S->Queue.Remove(A.AgentID); if(S->OccupantID==A.AgentID) S->OccupantID=INDEX_NONE; }
    A.AirportID=INDEX_NONE;
}

void UAgricultureCoordinator::DeferDockedSection(FAgricultureAgentState& A)
{
    if(auto* Field=Section(A))
    {
        UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] SectionDeferred Agent=%d Plot=%d Section=%d Airport=%d Reason=NoFeasibleDockService"),
            A.AgentID,A.PlotID,A.SectionID,A.AirportID);
        Field->AgentID=INDEX_NONE;
    }
    // 不可行的是当前任务与补给组合；保留地面泊位、药液和断点，健康飞机充电后重新参与调度。
    A.PlotID=INDEX_NONE;A.SectionID=INDEX_NONE;A.ServiceLiquidTarget=-1;A.ServiceBatteryTarget=1;
    A.ServiceAirportTargetID=INDEX_NONE;A.ServiceDepartureNotBefore=0;A.ServicePlanTime=-1;
    A.bServiceCleanBeforeTransfer=false;A.PlannedSupplyAirportID=INDEX_NONE;A.PlannedSupplyDelaySeconds=0;
    AssignmentPlanTime=-1;SupplyPlanTime=-1;
    LastReason=TEXT("当前补给组合不可行，保留覆盖断点并重新分配任务");
}

void UAgricultureCoordinator::Fail(FAgricultureAgentState& A,const FString& Reason)
{
    LastReason=Reason;
    const AUAVPawn* FailedPawn=Pawn(A.AgentID);
    const auto* Reserved=Airport(A.AirportID);
    const bool AtBerth=FailedPawn && Reserved && (FailedPawn->IsParked() || FVector::Dist2D(FailedPawn->GetUAVState().Position,Reserved->Config.DockPosition)<500);
    if(!AtBerth) ReleaseAirport(A);
    else if(auto* S=Airport(A.AirportID)) {S->OccupantID=A.AgentID;S->Queue.Remove(A.AgentID);}
    if(auto* Field=Section(A))
    {
        Field->AgentID=INDEX_NONE;
        if(Field->NextPoint%2==1 && Pawn(A.AgentID)) Field->ResumePosition=Field->StripPoints[Field->NextPoint-1]+(Field->StripPoints[Field->NextPoint]-Field->StripPoints[Field->NextPoint-1]).GetSafeNormal()*Field->StripCoveredCm;
    }
    A.PlotID=INDEX_NONE;A.SectionID=INDEX_NONE;
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
    if(!Force && A.SupplyForecastTime>=0 && ElapsedSeconds-A.SupplyForecastTime<.5f &&
        (A.Phase!=EAgriculturePhase::Transit || A.SupplyEntryTravelSeconds.Num()==Airports.Num())) return;
    AUAVPawn* P=Pawn(A.AgentID);if(!P) return;
    if(!SupplyPlanner) SupplyPlanner=NewObject<UAStarPathPlanner>(this);
    TArray<FObstacleInfo> Obstacles=P->GetObstacleManager()->GetAllObstacles();
    Obstacles.RemoveAll([P](const FObstacleInfo& O){
        const AUAVPawn* Other=Cast<AUAVPawn>(O.LinkedActor.Get());
        return Other && (Other==P || (!Other->IsParked() && !Other->IsCrashed()));
    });
    SupplyPlanner->SetObstacles(Obstacles);
    A.SupplyPathLengths.Empty();A.SupplyTravelSeconds.Empty();A.SupplyEntryTravelSeconds.Empty();
    const FVector Position=P->GetUAVState().Position;
    const FVector Lift=SupplyDepartureTarget(Position,P->GetUAVState().Velocity,Config.TransitHeightCm,Manager->TaskExecutionAccelerationCm);
    const bool LiftClear=!SupplyPlanner->CheckLineCollision(Position,Lift,P->GetCollisionRadius());
    const auto* Field=A.Phase==EAgriculturePhase::Transit ? Section(A) : nullptr;
    FVector Entry=Position,EntryVelocity=FVector::ZeroVector,EntryLift=Position;
    bool EntryClear=false;
    if(Field)
    {
        const int32 End=Field->NextPoint%2==0 ? Field->NextPoint+1 : Field->NextPoint;
        if(Field->StripPoints.IsValidIndex(End))
        {
            const FVector Direction=(Field->StripPoints[End]-Field->StripPoints[End-1]).GetSafeNormal();
            Entry=Field->StripPoints[End-1]+Direction*Field->StripCoveredCm;
            EntryVelocity=Direction*Config.SpraySpeedCm;
            EntryLift=SupplyDepartureTarget(Entry,EntryVelocity,Config.TransitHeightCm,Manager->TaskExecutionAccelerationCm);
            EntryClear=!SupplyPlanner->CheckLineCollision(Entry,EntryLift,P->GetCollisionRadius());
        }
    }
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
            if(Retained)
                Travel=FMath::Max(0.0f,Tracker->GetTrajectory().TotalDuration-Tracker->GetCurrentTime())+Config.RouteArrivalAllowanceSeconds+
                    Config.CruiseTimeFactor*Distance/Config.TransitSpeedCm+(Distance>1 ? Config.RouteArrivalAllowanceSeconds : 0)+Config.AirportApproachSeconds+1;
            else Travel=float(SupplyTravelTime(Position,P->GetUAVState().Velocity,Distance,Manager->TaskExecutionAccelerationCm));
            const float BrakeTime=1.9f*P->GetUAVState().Velocity.Size()/FMath::Max(1.0f,Manager->TaskExecutionAccelerationCm);
            const FVector Stop=Position+P->GetUAVState().Velocity*(BrakeTime*.5f);
            const float LiftDistance=FVector::Dist(Position,Stop)+FMath::Abs(Config.TransitHeightCm-float(Stop.Z));
            Distance+=Retained ? FVector::Dist(Position,Start) : LiftDistance;
        }
        A.SupplyPathLengths.Add(Distance);A.SupplyTravelSeconds.Add(Travel);
        float EntryTravel=MAX_FLT;
        if(EntryClear)
        {
            TArray<FVector> EntryPath;
            if(!SupplyPlanner->CheckLineCollision(EntryLift,Goal,P->GetCollisionRadius())) EntryPath={EntryLift,Goal};
            else SupplyPlanner->PlanPath(EntryLift,Goal,EntryPath);
            if(!EntryPath.IsEmpty())
            {
                double EntryDistance=0;for(int32 I=1;I<EntryPath.Num();++I) EntryDistance+=FVector::Dist(EntryPath[I-1],EntryPath[I]);
                EntryTravel=float(SupplyTravelTime(Entry,EntryVelocity,EntryDistance,Manager->TaskExecutionAccelerationCm));
            }
        }
        A.SupplyEntryTravelSeconds.Add(EntryTravel);
    }
    A.SupplyForecastTime=ElapsedSeconds;
}

bool UAgricultureCoordinator::RedirectSupplyReservation(FAgricultureAgentState& Request,int32 ExcludedAirportID)
{
    const float Reserve=Config.BatteryReserveFraction*FlightSeconds(Request)+Config.SupplyTimeBufferSeconds;
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
            const float Wait=EstimateWaitSeconds(Released,FMath::Max(0.0f,Travel-Config.AirportApproachSeconds),Request.AgentID);
            if(ChooseNearestAirport({Released},{Request.SupplyPathLengths[I]},Request.Battery*FlightSeconds(Request),
                Config.TransitSpeedCm,Reserve,{Travel},{Wait})==INDEX_NONE) continue;
            RefreshSupplyRoutes(Other,true);
            TArray<FSupplyAirportState> Alternatives=Airports;TArray<float> Waiting;
            for(int32 J=0;J<Alternatives.Num();++J)
            {
                if(J==I || !CanService(Alternatives[J])) Alternatives[J].Config.bEnabled=false;
                Waiting.Add(EstimateWaitSeconds(Alternatives[J],FMath::Max(0.0f,Other.SupplyTravelSeconds[J]-Config.AirportApproachSeconds),Other.AgentID));
            }
            const int32 Alternate=ChooseNearestAirport(Alternatives,Other.SupplyPathLengths,Other.Battery*FlightSeconds(Other),
                Config.TransitSpeedCm,Config.BatteryReserveFraction*FlightSeconds(Other)+Config.SupplyTimeBufferSeconds,Other.SupplyTravelSeconds,Waiting);
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
    if(auto* Field=Section(A))
        if(Field->NextPoint%2==1) Field->ResumePosition=Field->StripPoints[Field->NextPoint-1]+(Field->StripPoints[Field->NextPoint]-Field->StripPoints[Field->NextPoint-1]).GetSafeNormal()*Field->StripCoveredCm;
    ReleaseAirport(A);
    RefreshSupplyRoutes(A,true);
    const auto& Distances=A.SupplyPathLengths;
    const auto& TravelTimes=A.SupplyTravelSeconds;
    TArray<float> WaitingTimes;
    for(int32 I=0;I<Airports.Num();++I)
    {
        const float Wait=EstimateWaitSeconds(Airports[I],FMath::Max(0.0f,TravelTimes[I]-Config.AirportApproachSeconds),A.AgentID);
        WaitingTimes.Add(Wait);
        UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] SupplyCandidate Agent=%d Airport=%d PathCm=%.0f Travel=%.1f Wait=%.1f Reserve=%.1f Buffer=%.1f Available=%.1f Healthy=%d"),
            A.AgentID,Airports[I].Config.AirportID,Distances[I],TravelTimes[I],Wait,
            Config.BatteryReserveFraction*FlightSeconds(A),Config.SupplyTimeBufferSeconds,A.Battery*FlightSeconds(A),CanService(Airports[I]) ? 1 : 0);
    }
    TArray<FSupplyAirportState> Candidates=Airports;
    for(auto& S:Candidates) if(S.Config.AirportID==ExcludedAirportID || !CanService(S)) S.Config.bEnabled=false;
    const auto LandingCandidates=Candidates;
    const auto* Remaining=Section(A);
    for(auto& S:Candidates)
        if(S.Config.bEnabled && Remaining && PlanService(A,*Remaining,AvailableSupplyStock(S,A.AgentID)).LiquidLitres<0) S.Config.bEnabled=false;
    auto Preferred=Candidates;
    for(auto& S:Preferred) if(S.Config.AirportID!=A.PlannedSupplyAirportID) S.Config.bEnabled=false;
    int32 Index=ChooseNearestAirport(Preferred,Distances,A.Battery*FlightSeconds(A),
        Config.TransitSpeedCm,Config.BatteryReserveFraction*FlightSeconds(A)+Config.SupplyTimeBufferSeconds,TravelTimes,WaitingTimes);
    if(Index==INDEX_NONE) Index=ChooseNearestAirport(Candidates,Distances,A.Battery*FlightSeconds(A),
        Config.TransitSpeedCm,Config.BatteryReserveFraction*FlightSeconds(A)+Config.SupplyTimeBufferSeconds,TravelTimes,WaitingTimes);
    if(Index==INDEX_NONE) Index=ChooseNearestAirport(LandingCandidates,Distances,A.Battery*FlightSeconds(A),
        Config.TransitSpeedCm,Config.BatteryReserveFraction*FlightSeconds(A)+Config.SupplyTimeBufferSeconds,TravelTimes,WaitingTimes);
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
        if(!S.Config.bEnabled)
        {
            if(auto* P=Pawn(A.AgentID)) P->ReleaseFromSurface();
            RequestSupply(A);LastReason=FString::Printf(TEXT("机场 %d 停用，改降"),S.Config.AirportID);continue;
        }
        if(A.Phase!=EAgriculturePhase::Servicing || S.OccupantID!=A.AgentID) continue;
        AUAVPawn* P=Pawn(A.AgentID);
        if(!P || !P->IsParked() || !P->IsGroundContact()) {Fail(A,TEXT("Dock contact lost"));continue;}
        auto* Field=Section(A);
        const FName Recipe=Field ? Field->Config.Recipe : A.Recipe;
        const float Fraction=Field ? Field->Config.ConcentrateFraction : 0.02f;
        A.Battery=FMath::Min(1.0f,A.Battery+Dt/FMath::Max(1.0f,S.Config.ChargeSeconds));
        if(Field && A.ServiceLiquidTarget==-1)
        {
            *Field=ChooseWorkOrder(A,S.Config.DockPosition,*Field);
            RefreshSupplyRoutes(A,true);
            const auto Plan=PlanDockService(A,*Field,S);
            if(Plan.AirportID==INDEX_NONE) {DeferDockedSection(A);continue;}
            A.ServiceLiquidTarget=Plan.LiquidLitres<0 ? -2 : Plan.LiquidLitres;A.ServiceBatteryTarget=Plan.BatteryFraction;
            A.ServiceAirportTargetID=Plan.AirportID;
            A.ServiceDepartureNotBefore=ElapsedSeconds+Plan.DepartureDelaySeconds;
            A.ServicePlanTime=ElapsedSeconds;
            A.bServiceCleanBeforeTransfer=Plan.bCleanBeforeTransfer;
            UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] ServicePlan Agent=%d Airport=%d TargetAirport=%d LiquidTarget=%.2f BatteryTarget=%.3f EstimatedSeconds=%.2f CompletionSeconds=%.2f DepartureNotBeforeSeconds=%.2f CleanBeforeTransfer=%d"),
                A.AgentID,S.Config.AirportID,A.ServiceAirportTargetID,A.ServiceLiquidTarget,A.ServiceBatteryTarget,Plan.ServiceSeconds,Plan.CompletionSeconds,A.ServiceDepartureNotBefore,int32(A.bServiceCleanBeforeTransfer));
            if(Plan.LiquidLitres>=0)
            {
                const auto Forecast=ForecastSortie(A,S.Config.DockPosition,*Field,S,Plan.LiquidLitres,Plan.BatteryFraction);
                UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] ServiceBudget Agent=%d Airport=%d FlightSeconds=%.2f ReturnSeconds=%.2f RequiredBattery=%.3f AppliedLitres=%.2f"),
                    A.AgentID,S.Config.AirportID,Forecast.FlightSeconds,Forecast.ReturnSeconds,Forecast.RequiredBattery,Forecast.AppliedLitres);
            }
        }
        const bool NeedsCleaning=Field && (A.ServiceLiquidTarget>=0 || A.bServiceCleanBeforeTransfer) && (A.Recipe!=Recipe || A.bCleaned);
        if(NeedsCleaning && !A.bCleaned)
        {
            if(A.PhaseSeconds<S.Config.CleanSeconds) continue;
            const float WaterBefore=S.Config.WaterLitres,WasteBefore=S.WasteLitres;
            if(!CleanResidue(A,S,Recipe)) {A.ServiceLiquidTarget=-1;continue;}
            ConsumeSupplyReservation(S.Config.AirportID,A.AgentID,WaterBefore-S.Config.WaterLitres,0,S.WasteLitres-WasteBefore);
            P->UpdatePayloadMass(A.LiquidLitres*Config.LiquidDensityKgPerLitre);
        }
        const float ReadyTime=Field && A.ServiceLiquidTarget>=0 ? S.Config.MixSeconds+(NeedsCleaning ? S.Config.CleanSeconds : 0) : A.bServiceCleanBeforeTransfer ? S.Config.CleanSeconds : 0;
        if(A.PhaseSeconds<ReadyTime) continue;
        const float TargetLiquid=Field ? A.ServiceLiquidTarget : A.LiquidLitres;
        const float TargetBattery=Field ? A.ServiceBatteryTarget : 1;
        if(Field && TargetLiquid<0)
        {
            const bool OldPlanReady=A.Battery>=A.ServiceBatteryTarget-.001f && ElapsedSeconds>=A.ServiceDepartureNotBefore;
            if(!OldPlanReady && A.ServicePlanTime>=0 && ElapsedSeconds-A.ServicePlanTime<.5f) continue;
            A.ServicePlanTime=ElapsedSeconds;
            const auto DeparturePlan=PlanDockService(A,*Field,S);
            if(DeparturePlan.AirportID==INDEX_NONE) {DeferDockedSection(A);continue;}
            if(DeparturePlan.LiquidLitres>=0 || DeparturePlan.AirportID==INDEX_NONE ||
                DeparturePlan.DepartureDelaySeconds>.05 || DeparturePlan.BatteryFraction>A.Battery+.001f ||
                DeparturePlan.bCleanBeforeTransfer!=A.bServiceCleanBeforeTransfer)
            {
                const bool NewCleaning=(DeparturePlan.LiquidLitres>=0 || DeparturePlan.bCleanBeforeTransfer) &&
                    A.Recipe!=Field->Config.Recipe && !A.bCleaned;
                A.ServiceLiquidTarget=DeparturePlan.LiquidLitres>=0 ? DeparturePlan.LiquidLitres : -2;
                A.ServiceBatteryTarget=DeparturePlan.BatteryFraction;A.ServiceAirportTargetID=DeparturePlan.AirportID;
                A.ServiceDepartureNotBefore=ElapsedSeconds+DeparturePlan.DepartureDelaySeconds;
                A.bServiceCleanBeforeTransfer=DeparturePlan.bCleanBeforeTransfer;
                if(NewCleaning) A.PhaseSeconds=0;
                else if(DeparturePlan.LiquidLitres>=0) A.PhaseSeconds=A.bCleaned ? S.Config.CleanSeconds : 0;
                UE_LOG_THROTTLE(2.0,LogAgriculture,Log,TEXT("[Agriculture] TransferPreflight Agent=%d SourceAirport=%d TargetAirport=%d GroundDelay=%.2f BatteryTarget=%.3f"),
                    A.AgentID,S.Config.AirportID,A.ServiceAirportTargetID,DeparturePlan.DepartureDelaySeconds,A.ServiceBatteryTarget);
                continue;
            }
            A.ServiceAirportTargetID=DeparturePlan.AirportID;
            if(A.ServiceAirportTargetID!=INDEX_NONE)
            {
                const auto* Destination=Airport(A.ServiceAirportTargetID);
                if(!Destination || !CanService(*Destination) || PlanService(A,*Field,AvailableSupplyStock(*Destination,A.AgentID),false).LiquidLitres<0)
                {A.ServiceLiquidTarget=-1;continue;}
                A.PlannedSupplyAirportID=A.ServiceAirportTargetID;
            }
            bool Alternative=A.ServiceAirportTargetID!=INDEX_NONE;
            for(const auto& Candidate:Airports)
                if(Candidate.Config.AirportID!=S.Config.AirportID && CanService(Candidate) &&
                    PlanService(A,*Field,AvailableSupplyStock(Candidate,A.AgentID),false).LiquidLitres>=0) {Alternative=true;break;}
            if(!Alternative) {DeferDockedSection(A);continue;}
            UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] SupplyRelocation Agent=%d Airport=%d Plot=%d Liquid=%.2f Battery=%.3f"),
                A.AgentID,S.Config.AirportID,A.PlotID,A.LiquidLitres,A.Battery);
            P->ReleaseFromSurface();ChangePhase(A,EAgriculturePhase::TakingOff);continue;
        }
        if(Field)
        {
            const float WaterBefore=S.Config.WaterLitres,ConcentrateBefore=S.Config.ConcentrateLitres,WasteBefore=S.WasteLitres;
            DrainLiquid(A,S,S.Config.RefillLitresPerMinute/60*Dt,TargetLiquid);
            TransferLiquid(A,S,S.Config.RefillLitresPerMinute/60*Dt,TargetLiquid,Fraction);
            ConsumeSupplyReservation(S.Config.AirportID,A.AgentID,WaterBefore-S.Config.WaterLitres,ConcentrateBefore-S.Config.ConcentrateLitres,S.WasteLitres-WasteBefore);
        }
        P->UpdatePayloadMass(A.LiquidLitres*Config.LiquidDensityKgPerLitre);
        if(A.Battery>=TargetBattery-.001f && (!Field || FMath::Abs(A.LiquidLitres-TargetLiquid)<=.01f))
        {
            if(Field)
            {
                const auto DeparturePlan=PlanDockService(A,*Field,S);
                if(DeparturePlan.LiquidLitres<0 || FMath::Abs(DeparturePlan.LiquidLitres-A.LiquidLitres)>.01f || DeparturePlan.BatteryFraction>A.Battery+.001f)
                {
                    A.ServiceLiquidTarget=DeparturePlan.LiquidLitres<0 ? -2 : DeparturePlan.LiquidLitres;
                    A.ServiceBatteryTarget=DeparturePlan.BatteryFraction;A.ServiceAirportTargetID=DeparturePlan.AirportID;
                    A.ServiceDepartureNotBefore=ElapsedSeconds+DeparturePlan.DepartureDelaySeconds;
                    A.bServiceCleanBeforeTransfer=DeparturePlan.bCleanBeforeTransfer;
                    UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] ServiceReplan Agent=%d Airport=%d TargetAirport=%d LiquidTarget=%.2f BatteryTarget=%.3f"),
                        A.AgentID,S.Config.AirportID,A.ServiceAirportTargetID,A.ServiceLiquidTarget,A.ServiceBatteryTarget);
                    continue;
                }
            }
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
        if(!P->IsParked() && !P->IsCrashed()) A.Battery=FMath::Max(0.0f,A.Battery-Dt/FMath::Max(1.0f,FlightSeconds(A)));
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
                auto Obstacles=P->GetObstacleManager()->GetAllObstacles();
                Obstacles.RemoveAll([](const FObstacleInfo& O){
                    const auto* Other=Cast<AUAVPawn>(O.LinkedActor.Get());
                    return Other && !Other->IsParked() && !Other->IsCrashed();
                });
                Target=FindEmergencyLandingSite(Target,Plots,Manager->DefaultCBFQPConfig.DSafe+2*P->GetCollisionRadius()+Config.ArrivalRadiusCm,
                    Obstacles,P->GetCollisionRadius());
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
    const float FrameStartLiquid=A.LiquidLitres,FrameStartBattery=A.Battery;
    if(!P->IsParked()) A.Battery=FMath::Max(0.0f,A.Battery-Dt/FMath::Max(1.0f,FlightSeconds(A)));
    auto* Field=Section(A);
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
            const int32 DepartureAirportID=A.AirportID;
            ReleaseAirport(A);
            if(A.ServiceLiquidTarget<-1)
            {
                A.PlannedSupplyAirportID=A.ServiceAirportTargetID;
                RequestSupply(A,DepartureAirportID);return;
            }
            if(Field && (A.LiquidLitres<1 || A.Battery<=Config.BatteryReserveFraction || A.Recipe!=Field->Config.Recipe)) RequestSupply(A);
            else ChangePhase(A,EAgriculturePhase::Transit);
        }
        return;
    }
    if(Field && (A.Phase==EAgriculturePhase::Transit || A.Phase==EAgriculturePhase::Spraying))
    {
        RefreshSupplyRoutes(A);
        float ReturnSeconds=MAX_FLT,EmergencyReturnSeconds=MAX_FLT,ReturnTravelSeconds=MAX_FLT,ReturnWaitSeconds=0;
        for(int32 I=0;I<Airports.Num();++I) if(CanService(Airports[I]))
        {
            const float Travel=A.SupplyTravelSeconds[I];
            const float Budget=Travel+EstimateWaitSeconds(Airports[I],FMath::Max(0.0f,Travel-Config.AirportApproachSeconds),A.AgentID);
            EmergencyReturnSeconds=FMath::Min(EmergencyReturnSeconds,Budget);
            if(A.PlannedSupplyAirportID!=INDEX_NONE && Airports[I].Config.AirportID!=A.PlannedSupplyAirportID) continue;
            if(Budget<ReturnSeconds) {ReturnSeconds=Budget;ReturnTravelSeconds=Travel;ReturnWaitSeconds=Budget-Travel;}
        }
        double WorkSeconds=0,NextLiquid=A.LiquidLitres,Extension=0;
        const bool Spraying=A.Phase==EAgriculturePhase::Spraying;
        if(Spraying)
        {
            const float Length=FVector::Dist(Field->StripPoints[Field->NextPoint-1],Field->StripPoints[Field->NextPoint]);
            const double Advance=FMath::Min(500.0,FMath::Max(0.0,double(Length-Field->StripCoveredCm)));
            const double Width=FBox(Field->Config.Boundary).GetSize().Y/FMath::Max(1,Field->StripPoints.Num()/2);
            NextLiquid=FMath::Max(0.0,double(A.LiquidLitres)-Advance*Width*Field->Config.ApplicationLitresPerHa/1.e8);
            WorkSeconds=Advance/FMath::Max(1.f,Config.SpraySpeedCm);
            if(A.bWorkRouteActive) WorkSeconds/=FMath::Max(.01f,P->GetTrajectoryTracker()->MinAdaptiveTimeScale);
            Extension=Config.CruiseTimeFactor*Advance/FMath::Max(1.f,FMath::Min(Config.TransitSpeedCm,400.f));
        }
        else
        {
            const int32 FirstEnd=Field->NextPoint%2==0 ? Field->NextPoint+1 : Field->NextPoint;
            const FVector Direction=(Field->StripPoints[FirstEnd]-Field->StripPoints[FirstEnd-1]).GetSafeNormal();
            const FVector Target=Field->StripPoints[FirstEnd-1]+Direction*Field->StripCoveredCm;
            const float EntrySeconds=2*FCoveragePlanner::HeadlandDistance(Config.SpraySpeedCm,Manager->TaskExecutionAccelerationCm)/
                FMath::Max(1.f,FMath::Min(Config.SpraySpeedCm,100.f)+Config.SpraySpeedCm);
            WorkSeconds=A.bWorkRouteActive ? FMath::Max(0.f,A.WorkStartTime+EntrySeconds-P->GetTrajectoryTracker()->GetCurrentTime()) :
                Config.CruiseTimeFactor*FVector::Dist(Position,Target)/FMath::Max(1.f,FMath::Min(Config.TransitSpeedCm,400.f));
            if(A.bWorkRouteActive) WorkSeconds/=FMath::Max(.01f,P->GetTrajectoryTracker()->MinAdaptiveTimeScale);
            // 转场完成后的返场从条带入口计算，当前位置的应急返场独立校验。
            ReturnSeconds=MAX_FLT;
            for(int32 I=0;I<Airports.Num();++I) if(CanService(Airports[I]))
                {
                    const auto& AirportState=Airports[I];
                    if(A.PlannedSupplyAirportID!=INDEX_NONE && AirportState.Config.AirportID!=A.PlannedSupplyAirportID) continue;
                    const double Travel=A.SupplyEntryTravelSeconds[I];
                    const double Wait=EstimateWaitSeconds(AirportState,float(FMath::Max(0.0,WorkSeconds+Travel-Config.AirportApproachSeconds)),A.AgentID);
                    if(Travel+Wait<ReturnSeconds)
                    {ReturnSeconds=float(Travel+Wait);ReturnTravelSeconds=float(Travel);ReturnWaitSeconds=float(Wait);}
                }
        }
        const double WorkEnergy=SegmentEnergyFraction(A.EmptyMassKg,Config.LiquidDensityKgPerLitre,A.LiquidLitres,NextLiquid,WorkSeconds,Config.BatteryFlightSeconds);
        const double ReturnEnergy=SegmentEnergyFraction(A.EmptyMassKg,Config.LiquidDensityKgPerLitre,NextLiquid,NextLiquid,
            ReturnSeconds+Extension+Config.SupplyTimeBufferSeconds+Config.RouteArrivalAllowanceSeconds,Config.BatteryFlightSeconds);
        const double EmergencyEnergy=Spraying ? 0 : SegmentEnergyFraction(A.EmptyMassKg,Config.LiquidDensityKgPerLitre,
            A.LiquidLitres,A.LiquidLitres,EmergencyReturnSeconds+Config.SupplyTimeBufferSeconds+Config.RouteArrivalAllowanceSeconds,Config.BatteryFlightSeconds);
        const double ReturnFraction=FMath::Max(WorkEnergy+ReturnEnergy,EmergencyEnergy)+Config.BatteryReserveFraction;
        if(A.Recipe!=Field->Config.Recipe || A.LiquidLitres<1 || A.Battery<ReturnFraction)
        {
            UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] SupplyTrigger Agent=%d Phase=%d Battery=%.3f Required=%.3f Liquid=%.2f WorkSeconds=%.2f ReturnSeconds=%.2f Extension=%.2f PlannedAirport=%d"),
                A.AgentID,int32(A.Phase),A.Battery,ReturnFraction,A.LiquidLitres,WorkSeconds,ReturnSeconds,Extension,A.PlannedSupplyAirportID);
            const int32 End=Field->NextPoint%2==0 ? Field->NextPoint+1 : Field->NextPoint;
            const FVector Entry=Field->StripPoints[End-1]+(Field->StripPoints[End]-Field->StripPoints[End-1]).GetSafeNormal()*Field->StripCoveredCm;
            UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] SupplyBudget Agent=%d Section=%d Position=%s Entry=%s ReturnTravelSeconds=%.2f ReturnWaitSeconds=%.2f EmergencyReturnSeconds=%.2f WorkEnergy=%.4f ReturnEnergy=%.4f EmergencyEnergy=%.4f"),
                A.AgentID,A.SectionID,*Position.ToString(),*Entry.ToString(),ReturnTravelSeconds,ReturnWaitSeconds,EmergencyReturnSeconds,WorkEnergy,ReturnEnergy,EmergencyEnergy);
            const bool RecipeChanged=A.Recipe!=Field->Config.Recipe;const bool LiquidLow=A.LiquidLitres<1;RequestSupply(A);
            if(A.Phase!=EAgriculturePhase::Failed) LastReason=RecipeChanged ? TEXT("任务配方变化，返航清洗配液") : LiquidLow ? TEXT("药液不足，返航补给并保留断点") : TEXT("作业转场与返航电量储备触发补给");
            return;
        }
    }
    if(A.Phase==EAgriculturePhase::Transit)
    {
        if(!Field) {ChangePhase(A,EAgriculturePhase::Idle);return;}
        if(!A.bWorkRouteActive && !StartWorkRoute(A,*Field)) return;
        // 阶段只描述作业状态，不重新启动或清空整条轨迹。
        if(P->GetTrajectoryTracker()->GetCurrentTime()>=A.WorkStartTime)
        {A.Phase=EAgriculturePhase::Spraying;A.PhaseSeconds=0;}
        return;
    }
    if(A.Phase==EAgriculturePhase::Spraying && Field)
    {
        const FVector End=Field->StripPoints[Field->NextPoint];
        const FVector Start=Field->StripPoints[Field->NextPoint-1];
        const FVector Direction=(End-Start).GetSafeNormal();
        const float Length=FVector::Dist(Start,End);
        if(P->GetTrajectoryTracker()->IsTimedOut())
        {
            Field->ResumePosition=Start+Direction*Field->StripCoveredCm;
            ChangePhase(A,EAgriculturePhase::Transit);return;
        }
        if(!RecordSpraySegment(*Field,A.LiquidLitres,Previous,Position,Config.ArrivalRadiusCm))
        {
            Field->ResumePosition=Start+Direction*FMath::Max(-Config.ArrivalRadiusCm*1.5f,Field->StripCoveredCm-Config.ArrivalRadiusCm*1.5f);
            LastReason=TEXT("回飞未施用的连续断点");ChangePhase(A,EAgriculturePhase::Transit);return;
        }
        if(!P->IsParked()) A.Battery=FMath::Max(0.f,FrameStartBattery-float(SegmentEnergyFraction(A.EmptyMassKg,
            Config.LiquidDensityKgPerLitre,FrameStartLiquid,A.LiquidLitres,Dt,Config.BatteryFlightSeconds)));
        P->UpdatePayloadMass(A.LiquidLitres*Config.LiquidDensityKgPerLitre);
        DrawDebugLine(GetWorld(),Start,Position,FColor::Green,false,Dt*2,0,30);
        const bool HasNext=Field->StripPoints.IsValidIndex(Field->NextPoint+2);
        // 实机越过条带边界后再切覆盖游标，避免地头两行的同一纵向平面被误判为漏喷。
        const float ExitMargin=FMath::Min(Config.ArrivalRadiusCm*1.5f,
            FCoveragePlanner::HeadlandDistance(Config.SpraySpeedCm,Manager->TaskExecutionAccelerationCm)*0.5f);
        const bool ExitedStrip=FVector::DotProduct(Position-End,Direction)>=ExitMargin;
        if(Field->StripCoveredCm>=Length-1 && (!HasNext || ExitedStrip))
        {
            Field->StripCoveredCm=0;
            Field->NextPoint+=2;
            if(Field->NextPoint>=Field->StripPoints.Num())
            {
                UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] SectionCompleted Plot=%d Section=%d Agent=%d Area=%.2f Liquid=%.2f"),Field->Config.Task.TaskID,A.SectionID,A.AgentID,Field->CoveredSquareMetres,Field->AppliedLitres);
                Field->bCompleted=true;Field->AgentID=INDEX_NONE;A.PlotID=INDEX_NONE;A.SectionID=INDEX_NONE;
                ChangePhase(A,EAgriculturePhase::Idle);
            }
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
            if(A.Battery*FlightSeconds(A) < EstimateWaitSeconds(*Station,FMath::Max(0.0f,Travel-Config.AirportApproachSeconds),A.AgentID)+Travel+Config.BatteryReserveFraction*FlightSeconds(A))
            {RequestSupply(A,Station->Config.AirportID);if(A.Phase!=EAgriculturePhase::Failed) LastReason=TEXT("等待电量储备不足，重新选择可达机场");return;}
        }
        if(A.TransitStage==0 && (A.bRouteActive || P->GetUAVState().Velocity.Size()>=5 ||
            FMath::Abs(Position.Z-Config.TransitHeightCm)>Config.ArrivalRadiusCm))
        {
            if(!A.bRouteActive && !StartSupplyDeparture(A)) return;
            if(P->GetTrajectoryTracker()->IsTimedOut()) {A.bRouteActive=false;return;}
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
            A.TransitStage=1;
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

void UAgricultureCoordinator::PlanSupplyAssignments()
{
    if(SupplyPlanTime>=0 && ElapsedSeconds-SupplyPlanTime<.5f) return;
    SupplyPlanTime=ElapsedSeconds;
    TArray<FAgricultureAgentState*> Workers;
    for(auto& A:Agents)
    {
        A.PlannedSupplyAirportID=INDEX_NONE;A.PlannedSupplyDelaySeconds=0;
        if((A.Phase!=EAgriculturePhase::Transit && A.Phase!=EAgriculturePhase::Spraying) || !Section(A) || A.AirportID!=INDEX_NONE) continue;
        RefreshSupplyRoutes(A);Workers.Add(&A);
    }
    // 先安排续航余量最小的飞机；同一机场可被多架飞机预约不同时间段。
    Workers.Sort([this](const FAgricultureAgentState& L,const FAgricultureAgentState& R)
    {
        const float Left=(L.Battery-Config.BatteryReserveFraction)*FlightSeconds(L);
        const float Right=(R.Battery-Config.BatteryReserveFraction)*FlightSeconds(R);
        return Left==Right ? L.AgentID<R.AgentID : Left<Right;
    });
    auto SupplyStocks=Airports;
    TArray<double> Available;
    for(const auto& Station:Airports) Available.Add(ElapsedSeconds+EstimateWaitSeconds(Station,0,INDEX_NONE));
    SupplySlots.Empty();SupplySlots.SetNum(Airports.Num());auto& Slots=SupplySlots;
    const auto DepartureSeconds=[this](const FSupplyAirportState& S)
    {return Config.CruiseTimeFactor*(FMath::Max(0.0,double(Config.TransitHeightCm)-S.Config.DockPosition.Z)/200+1000.0/150)+2*Config.RouteArrivalAllowanceSeconds;};
    const auto ReserveService=[&](int32 I,const FAgricultureAgentState& A,const FAgriculturePlotState& Field,float Target,double Begin,double End)
    {
        auto& Stock=SupplyStocks[I];FAgricultureSupplySlot Slot{A.AgentID,Begin,End};
        Slot.WaterLitres=Stock.Config.WaterLitres;Slot.ConcentrateLitres=Stock.Config.ConcentrateLitres;Slot.WasteLitres=-Stock.WasteLitres;
        ReserveSupplyStock(Stock,A,Field,Target);
        Slot.WaterLitres-=Stock.Config.WaterLitres;Slot.ConcentrateLitres-=Stock.Config.ConcentrateLitres;Slot.WasteLitres+=Stock.WasteLitres;
        Slots[I].Add(Slot);
    };
    TMap<int32,FAgricultureServicePlan> DockPlans;
    for(const auto& A:Agents)
    {
        if(A.Phase!=EAgriculturePhase::Servicing) continue;
        if(A.ServiceLiquidTarget<-1)
        {
            if(A.bServiceCleanBeforeTransfer && !A.bCleaned)
            {
                const auto* Field=Section(A);
                const int32 I=Airports.IndexOfByPredicate([&A](const auto& S){return S.Config.AirportID==A.AirportID;});
                if(Field && I!=INDEX_NONE)
                    ReserveService(I,A,*Field,0,ElapsedSeconds,FMath::Max(A.ServiceDepartureNotBefore,
                        double(ElapsedSeconds)+FMath::Max(0.f,Airports[I].Config.CleanSeconds-A.PhaseSeconds))+DepartureSeconds(Airports[I]));
            }
            continue;
        }
        const auto* Field=Sections.IsValidIndex(A.SectionID) ? &Sections[A.SectionID] : nullptr;
        const int32 I=Airports.IndexOfByPredicate([&A](const auto& S){return S.Config.AirportID==A.AirportID;});
        if(!Field || I==INDEX_NONE) continue;
        FAgricultureServicePlan Service;
        if(A.ServiceLiquidTarget>=0) {Service.LiquidLitres=A.ServiceLiquidTarget;Service.BatteryFraction=A.ServiceBatteryTarget;}
        else Service=PlanService(A,*Field,SupplyStocks[I],false);
        if(Service.LiquidLitres<0) continue;
        const bool Cleaning=A.Recipe!=Field->Config.Recipe;
        const double Prep=FMath::Max(0.0,double(Airports[I].Config.MixSeconds+(Cleaning ? Airports[I].Config.CleanSeconds : 0))-A.PhaseSeconds);
        const double Fill=FMath::Max(0.f,Service.LiquidLitres-(Cleaning ? 0 : A.LiquidLitres))/FMath::Max(.001f,Airports[I].Config.RefillLitresPerMinute/60);
        const double Drain=Cleaning ? 0 : FMath::Max(0.f,A.LiquidLitres-Service.LiquidLitres)/FMath::Max(.001f,Airports[I].Config.RefillLitresPerMinute/60);
        Service.ServiceSeconds=FMath::Max(Prep+Fill+Drain,double(FMath::Max(0.f,Service.BatteryFraction-A.Battery)*Airports[I].Config.ChargeSeconds));
        DockPlans.Add(A.AgentID,Service);
        ReserveService(I,A,*Field,Service.LiquidLitres,ElapsedSeconds,ElapsedSeconds+Service.ServiceSeconds+DepartureSeconds(Airports[I]));
    }
    const auto BookSortie=[&](FAgricultureAgentState Future,const FAgriculturePlotState& Field,int32 I,double Offset)
    {
        const auto& Station=Airports[I];auto& Stock=SupplyStocks[I];
        const auto Forecast=ForecastSortie(Future,Station.Config.DockPosition,Field,Station,Future.LiquidLitres,Future.Battery,false);
        if(!Forecast.bFeasible) return;
        Future.LiquidLitres=FMath::Max(0.f,Future.LiquidLitres-float(Forecast.AppliedLitres));
        Future.Battery=FMath::Max(0.f,Future.Battery-float(Forecast.EnergyFraction));
        auto Remaining=Field;AdvanceForecastProgress(Remaining,Forecast.AppliedLitres);
        const auto Next=Forecast.bCompleted ? FAgricultureServicePlan() : PlanService(Future,Remaining,Stock,false);
        const double WaterBefore=Stock.Config.WaterLitres,ConcentrateBefore=Stock.Config.ConcentrateLitres,WasteBefore=Stock.WasteLitres;
        double NextService=(1-Future.Battery)*Station.Config.ChargeSeconds;
        if(!Forecast.bCompleted && Next.LiquidLitres>=0)
        {NextService=Next.ServiceSeconds;ReserveSupplyStock(Stock,Future,Remaining,Next.LiquidLitres);}
        const double Begin=FMath::Max(double(ElapsedSeconds),ElapsedSeconds+Offset+Forecast.EntryFlightSeconds+Forecast.EntryReturnSeconds-
            Config.AirportApproachSeconds-Config.SupplyTimeBufferSeconds);
        const double End=ElapsedSeconds+Offset+Forecast.FlightSeconds+Forecast.ReturnSeconds+NextService+DepartureSeconds(Station);
        if(End>Begin) Slots[I].Add({Future.AgentID,Begin,End,WaterBefore-Stock.Config.WaterLitres,
            ConcentrateBefore-Stock.Config.ConcentrateLitres,Stock.WasteLitres-WasteBefore});
    };
    // 已在补给和起飞的航次也承诺未来返场窗口，避免后分配的飞机插队耗尽其返场储备。
    for(auto& A:Agents)
    {
        if(A.Phase!=EAgriculturePhase::Servicing && A.Phase!=EAgriculturePhase::TakingOff && A.Phase!=EAgriculturePhase::Resuming) continue;
        const auto* Field=Sections.IsValidIndex(A.SectionID) ? &Sections[A.SectionID] : nullptr;
        const int32 I=Airports.IndexOfByPredicate([&A](const auto& S){return S.Config.AirportID==A.AirportID;});
        if(!Field || I==INDEX_NONE) continue;
        const auto& Station=Airports[I];
        if(A.ServiceLiquidTarget<-1)
        {
            const int32 Destination=Airports.IndexOfByPredicate([&A](const auto& S){return S.Config.AirportID==A.ServiceAirportTargetID;});
            if(Destination==INDEX_NONE || Destination==I) continue;
            if(A.Phase!=EAgriculturePhase::Servicing) RefreshSupplyRoutes(A);
            const auto& Target=Airports[Destination];auto& Stock=SupplyStocks[Destination];
            const FVector Lift=SupplyDepartureTarget(Station.Config.DockPosition,FVector::ZeroVector,Config.TransitHeightCm,150);
            FVector Goal=Target.Config.DockPosition;Goal.Z=Config.TransitHeightCm;
            const double Travel=(A.SupplyTravelSeconds.IsValidIndex(Destination) ? A.SupplyTravelSeconds[Destination] :
                SupplyTravelTime(Station.Config.DockPosition,FVector::ZeroVector,FVector::Dist(Lift,Goal),150))+
                (A.Phase==EAgriculturePhase::Servicing || A.TransitStage==0 ? Config.CruiseTimeFactor*1000.0/150+Config.RouteArrivalAllowanceSeconds : 0);
            if(!FMath::IsFinite(Travel) || Travel>=MAX_FLT) continue;
            const double CleaningRemaining=A.bServiceCleanBeforeTransfer && !A.bCleaned ?
                FMath::Max(0.f,Station.Config.CleanSeconds-A.PhaseSeconds) : 0;
            const double Ready=A.Phase==EAgriculturePhase::Servicing ?
                FMath::Max(A.ServiceDepartureNotBefore,double(ElapsedSeconds)+FMath::Max(CleaningRemaining,double(FMath::Max(0.f,A.ServiceBatteryTarget-A.Battery)*Station.Config.ChargeSeconds))) : ElapsedSeconds;
            const float DepartureBattery=A.Phase==EAgriculturePhase::Servicing ? FMath::Max(A.Battery,A.ServiceBatteryTarget) : A.Battery;
            auto Arriving=A;
            if(A.bServiceCleanBeforeTransfer && A.Recipe!=Field->Config.Recipe) {Arriving.LiquidLitres=0;Arriving.Recipe=Field->Config.Recipe;}
            Arriving.Battery=FMath::Max(0.f,DepartureBattery-float(Travel/FlightSeconds(Arriving)));
            const auto Service=PlanService(Arriving,*Field,Stock,false);
            if(Service.LiquidLitres<0) continue;
            const double Begin=FMath::Max(double(ElapsedSeconds),Ready+Travel-Config.AirportApproachSeconds);
            const double End=Ready+Travel+Service.ServiceSeconds+DepartureSeconds(Target);
            ReserveService(Destination,Arriving,*Field,Service.LiquidLitres,Begin,End);
            auto Loaded=Arriving;Loaded.LiquidLitres=Service.LiquidLitres;Loaded.Battery=FMath::Max(Arriving.Battery,Service.BatteryFraction);
            Loaded.Recipe=Field->Config.Recipe;
            BookSortie(Loaded,*Field,Destination,Ready+Travel+Service.ServiceSeconds-ElapsedSeconds);
            continue;
        }
        FAgricultureServicePlan Service;
        if(A.Phase!=EAgriculturePhase::Servicing)
        {Service.LiquidLitres=A.LiquidLitres;Service.BatteryFraction=FMath::Min(1.f,A.Battery+A.PhaseSeconds/FlightSeconds(A));}
        else if(const auto* Planned=DockPlans.Find(A.AgentID)) Service=*Planned;
        if(Service.LiquidLitres<0) continue;
        auto Future=A;Future.LiquidLitres=Service.LiquidLitres;Future.Battery=FMath::Max(A.Battery,Service.BatteryFraction);
        Future.Recipe=Field->Config.Recipe;
        double Offset=-A.PhaseSeconds;
        if(A.Phase==EAgriculturePhase::Servicing)
            Offset=Service.ServiceSeconds;
        BookSortie(Future,*Field,I,Offset);
    }
    for(auto* A:Workers)
    {
        int32 Best=INDEX_NONE;double BestFinish=DBL_MAX,BestWait=0,BestAvailable=0,BestBegin=0;
        FSupplyAirportState BestStock;FAgricultureSupplySlot BestSlot;
        FString CommitmentBudget=TEXT("StationMissing");
        const auto* Field=Section(*A);const auto* P=Pawn(A->AgentID);if(!Field || !P) continue;
        // 起飞载荷和电量以服务机场返场为约束；承诺仍可履行时不能重新按远期成本改站。
        const int32 Committed=Airports.IndexOfByPredicate([A](const auto& S){return S.Config.AirportID==A->ServiceAirportTargetID;});
        for(int32 CandidatePass=0;CandidatePass<2 && Best==INDEX_NONE;++CandidatePass)
        for(int32 I=0;I<Airports.Num();++I)
        {
            if((CandidatePass==0 && I!=Committed) || (CandidatePass==1 && I==Committed)) continue;
            const auto& Station=Airports[I];if(!CanService(Station))
            {if(I==Committed) CommitmentBudget=TEXT("StationUnavailable");continue;}
            const auto Forecast=ForecastSortie(*A,P->GetUAVState().Position,*Field,Station,A->LiquidLitres,A->Battery);
            const double Travel=Forecast.bFeasible ? Forecast.ReturnSeconds : A->SupplyTravelSeconds[I];
            const double Arrival=ElapsedSeconds+(Forecast.bFeasible ? Forecast.FlightSeconds : 0)+Travel;
            double Wait=FMath::Max(0.0,Available[I]-(Arrival-Config.AirportApproachSeconds));
            const double Liquid=FMath::Max(0.0,double(A->LiquidLitres)-Forecast.AppliedLitres);
            auto Future=*A;auto Remaining=*Field;auto Stock=SupplyStocks[I];
            Future.LiquidLitres=float(Liquid);
            const float ArrivalBattery=FMath::Max(0.f,A->Battery-float(Forecast.bFeasible ? Forecast.EnergyFraction : Travel/FlightSeconds(*A)));
            if(Forecast.bFeasible) AdvanceForecastProgress(Remaining,Forecast.AppliedLitres);
            const double Departure=Config.CruiseTimeFactor*(FMath::Max(0.0,double(Config.TransitHeightCm)-Station.Config.DockPosition.Z)/200+1000.0/150)+2*Config.RouteArrivalAllowanceSeconds;
            FAgricultureServicePlan Service;double Duration=0;bool SlotFeasible=false;
            for(int32 Pass=0;Pass<=Slots[I].Num()+1;++Pass)
            {
                Future.Battery=FMath::Max(0.f,ArrivalBattery-float(SegmentEnergyFraction(A->EmptyMassKg,Config.LiquidDensityKgPerLitre,Liquid,Liquid,Wait,Config.BatteryFlightSeconds)));
                Service=Forecast.bCompleted ? FAgricultureServicePlan() : PlanService(Future,Remaining,Stock,false);
                if(!Forecast.bCompleted && Service.LiquidLitres<0) break;
                Duration=Forecast.bCompleted ? double((1-Future.Battery)*Station.Config.ChargeSeconds) : Service.ServiceSeconds;
                const double ExtraWait=FindSupplySlotWait(Arrival+Wait-Config.AirportApproachSeconds,
                    Config.AirportApproachSeconds+Duration+Departure,Slots[I],A->AgentID);
                if(ExtraWait<1.e-6) {SlotFeasible=true;break;}
                Wait+=ExtraWait;
            }
            const double Required=Forecast.bFeasible ? Forecast.RequiredBattery+
                SegmentEnergyFraction(A->EmptyMassKg,Config.LiquidDensityKgPerLitre,Liquid,Liquid,Wait,Config.BatteryFlightSeconds) :
                (Travel+Wait+Config.SupplyTimeBufferSeconds)/FlightSeconds(*A)+Config.BatteryReserveFraction;
            if(I==Committed)
                CommitmentBudget=FString::Printf(TEXT("ForecastFeasible=%d Completed=%d Applied=%.3f Travel=%.2f Wait=%.2f Required=%.5f Battery=%.5f SlotFeasible=%d ServiceLiquid=%.2f StockWater=%.2f StockConcentrate=%.2f StockWaste=%.2f"),
                    int32(Forecast.bFeasible),int32(Forecast.bCompleted),Forecast.AppliedLitres,Travel,Wait,Required,A->Battery,
                    int32(SlotFeasible),Service.LiquidLitres,SupplyStocks[I].Config.WaterLitres,SupplyStocks[I].Config.ConcentrateLitres,SupplyStocks[I].WasteLitres);
            if(!SlotFeasible || !FMath::IsFinite(Required) || Required>A->Battery) continue;
            if(!Forecast.bCompleted)
                ReserveSupplyStock(Stock,Future,Remaining,Service.LiquidLitres);
            // 补给完成得早不代表作业完成得早，评分必须包含补给后的剩余航次。
            const double Finish=Arrival+Wait+(Forecast.bCompleted ? Duration : Service.CompletionSeconds);
            if(Finish<BestFinish)
            {
                Best=I;BestFinish=Finish;BestWait=Wait;BestStock=Stock;
                BestAvailable=Arrival+Wait+Duration+Departure;
                BestBegin=FMath::Max(double(ElapsedSeconds),Forecast.bFeasible ?
                    ElapsedSeconds+Forecast.EntryFlightSeconds+Forecast.EntryReturnSeconds-Config.AirportApproachSeconds-Config.SupplyTimeBufferSeconds :
                    Arrival+Wait-Config.AirportApproachSeconds);
                BestSlot={A->AgentID,BestBegin,BestAvailable,SupplyStocks[I].Config.WaterLitres-Stock.Config.WaterLitres,
                    SupplyStocks[I].Config.ConcentrateLitres-Stock.Config.ConcentrateLitres,Stock.WasteLitres-SupplyStocks[I].WasteLitres};
            }
        }
        if(Best!=INDEX_NONE)
        {
            if(A->ServiceAirportTargetID!=INDEX_NONE && A->ServiceAirportTargetID!=Airports[Best].Config.AirportID)
                UE_LOG(LogAgriculture,Log,TEXT("[Agriculture] SortieReturnReplanned Agent=%d PreviousAirport=%d Airport=%d Reason=CommitmentUnavailable %s"),
                    A->AgentID,A->ServiceAirportTargetID,Airports[Best].Config.AirportID,*CommitmentBudget);
            A->ServiceAirportTargetID=Airports[Best].Config.AirportID;
            A->PlannedSupplyAirportID=Airports[Best].Config.AirportID;
            A->PlannedSupplyDelaySeconds=float(BestWait);Slots[Best].Add(BestSlot);SupplyStocks[Best]=BestStock;
        }
        // 预测无法排入未来时隙时保留作业，由逐段安全检查决定实际返供时机。
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
                if(FMath::IsFinite(Travel) && Travel+Wait+Config.BatteryReserveFraction*FlightSeconds(A)+Config.SupplyTimeBufferSeconds<=A.Battery*FlightSeconds(A)) ++Count;
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
    PlanSupplyAssignments();
    for(auto& A:Agents) UpdateAgent(A,Dt);
    bCollectSupplyRequests=false;ResolveSupplyRequests();
    RefreshPlots();
    if(!Agents.ContainsByPredicate([](const auto& A){return A.Phase!=EAgriculturePhase::Failed;}))
        for(auto& P:Plots) if(!P.bCompleted) P.bFailed=true;
    for(auto& P:Plots)
    {
        const FBox Bounds(P.Config.Boundary);
        const float Area=Bounds.GetSize().X*Bounds.GetSize().Y/10000;
        if(!P.bCompleted && ElapsedSeconds>FMath::Min(P.Config.Task.Deadline,P.Config.Task.LatestFinish))
        {P.bFailed=true;for(auto& S:Sections) if(S.Config.Task.TaskID==P.Config.Task.TaskID) S.bFailed=true;LastReason=TEXT("地块作业超时");}
        Manager->GetTaskMonitor()->ReportExecution(P.Config.Task.TaskID,(P.AgentIDs.Num()==1 ? P.AgentIDs[0] : INDEX_NONE),
            P.bCompleted ? ETaskStatus::Completed : P.bFailed ? ETaskStatus::Failed : P.AgentIDs.IsEmpty() ? ETaskStatus::Pending : ETaskStatus::InProgress,
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
        for(const auto& P:Sections)
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
    AssignmentPlanTime=-1;SupplyPlanTime=-1;
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
        Plots.Add(P);Sections.Append(PartitionPlot(P,Agents.Num()));
        // 已完成覆盖保留；中断普通地块后让紧急任务参与下一次分配。
        for(auto& A:Agents) if(auto* Field=Section(A))
        {
            if(A.AirportID!=INDEX_NONE) continue;
            if(A.Phase==EAgriculturePhase::Spraying) Field->ResumePosition=Field->StripPoints[Field->NextPoint-1]+(Field->StripPoints[Field->NextPoint]-Field->StripPoints[Field->NextPoint-1]).GetSafeNormal()*Field->StripCoveredCm;
            Field->AgentID=INDEX_NONE;A.PlotID=INDEX_NONE;A.SectionID=INDEX_NONE;
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
    for(const auto& P:Plots) S+=FString::Printf(TEXT("地块 %d → %d 架 UAV 覆盖 %.0f m² %s\n"),P.Config.Task.TaskID,P.AgentIDs.Num(),P.CoveredSquareMetres,
        P.bCompleted ? TEXT("完成") : P.bFailed ? TEXT("不可行") : P.AgentIDs.IsEmpty() ? TEXT("待分配") : TEXT("执行中"));
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
        V->SetNumberField(TEXT("sectionId"),A.SectionID);
        V->SetBoolField(TEXT("cleanBeforeTransfer"),A.bServiceCleanBeforeTransfer);
        V->SetNumberField(TEXT("airportId"),A.AirportID);V->SetNumberField(TEXT("battery"),A.Battery);V->SetNumberField(TEXT("liquidLitres"),A.LiquidLitres);
        V->SetNumberField(TEXT("massKg"),A.EmptyMassKg+A.LiquidLitres*Config.LiquidDensityKgPerLitre);
        V->SetNumberField(TEXT("powerRatio"),PayloadPowerRatio(A.EmptyMassKg,A.LiquidLitres,Config.LiquidDensityKgPerLitre));
        V->SetNumberField(TEXT("serviceLiquidTarget"),A.ServiceLiquidTarget);V->SetNumberField(TEXT("serviceBatteryTarget"),A.ServiceBatteryTarget);
        V->SetNumberField(TEXT("serviceDepartureNotBefore"),A.ServiceDepartureNotBefore);
        V->SetNumberField(TEXT("serviceAirportTargetId"),A.ServiceAirportTargetID);
        V->SetNumberField(TEXT("plannedSupplyAirportId"),A.PlannedSupplyAirportID);V->SetNumberField(TEXT("plannedSupplyDelaySeconds"),A.PlannedSupplyDelaySeconds);
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
    for(int32 I=0;I<SupplySlots.Num() && I<Airports.Num();++I) for(const auto& Slot:SupplySlots[I])
    {
        auto V=MakeShared<FJsonObject>();V->SetNumberField(TEXT("airportId"),Airports[I].Config.AirportID);
        V->SetNumberField(TEXT("agentId"),Slot.AgentID);V->SetNumberField(TEXT("beginSeconds"),Slot.BeginSeconds);
        V->SetNumberField(TEXT("endSeconds"),Slot.EndSeconds);Values.Add(MakeShared<FJsonValueObject>(V));
        V->SetNumberField(TEXT("reservedWaterLitres"),Slot.WaterLitres);V->SetNumberField(TEXT("reservedConcentrateLitres"),Slot.ConcentrateLitres);
        V->SetNumberField(TEXT("reservedWasteLitres"),Slot.WasteLitres);
    }
    Root->SetArrayField(TEXT("futureSupplySlots"),Values);Values.Empty();
    for(const auto& P:Plots)
    {
        auto V=MakeShared<FJsonObject>();V->SetNumberField(TEXT("taskId"),P.Config.Task.TaskID);TArray<TSharedPtr<FJsonValue>> Assigned;
        for(int32 ID:P.AgentIDs) Assigned.Add(MakeShared<FJsonValueNumber>(ID));
        V->SetArrayField(TEXT("agentIds"),Assigned);
        V->SetNumberField(TEXT("coveredSquareMetres"),P.CoveredSquareMetres);V->SetNumberField(TEXT("appliedLitres"),P.AppliedLitres);
        V->SetBoolField(TEXT("completed"),P.bCompleted);V->SetBoolField(TEXT("failed"),P.bFailed);Values.Add(MakeShared<FJsonValueObject>(V));
    }
    Root->SetArrayField(TEXT("plots"),Values);Values.Empty();
    for(int32 I=0;I<Sections.Num();++I)
    {
        const auto& S=Sections[I];auto V=MakeShared<FJsonObject>();
        V->SetNumberField(TEXT("sectionId"),I);V->SetNumberField(TEXT("plotId"),S.Config.Task.TaskID);
        V->SetNumberField(TEXT("agentId"),S.AgentID);V->SetNumberField(TEXT("nextPoint"),S.NextPoint);
        V->SetNumberField(TEXT("stripCoveredCm"),S.StripCoveredCm);V->SetNumberField(TEXT("coveredSquareMetres"),S.CoveredSquareMetres);
        V->SetBoolField(TEXT("completed"),S.bCompleted);V->SetBoolField(TEXT("failed"),S.bFailed);
        Values.Add(MakeShared<FJsonValueObject>(V));
    }
    Root->SetArrayField(TEXT("sections"),Values);
    FString Json;FJsonSerializer::Serialize(Root,TJsonWriterFactory<TCHAR,TCondensedJsonPrintPolicy<TCHAR>>::Create(&Json));return Json;
}
