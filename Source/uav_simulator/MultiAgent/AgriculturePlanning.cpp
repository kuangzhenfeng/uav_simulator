#include "AgricultureCoordinator.h"
#include "AgentManager.h"
#include "../Core/UAVPawn.h"
#include "../Planning/CoveragePlanner.h"
#include "../Planning/TrajectoryTracker.h"
#include "../Utility/Filter.h"


float UAgricultureCoordinator::PayloadPowerRatio(float EmptyMassKg,float LiquidLitres,float DensityKgPerLitre)
{
    // 固定桨盘面积下，动量理论的悬停诱导功率正比于总质量的 3/2 次方。
    return FMath::Pow(1.f+FMath::Max(0.f,LiquidLitres)*FMath::Max(.001f,DensityKgPerLitre)/FMath::Max(.001f,EmptyMassKg),1.5f);
}

double UAgricultureCoordinator::SegmentEnergyFraction(float EmptyMassKg,float DensityKgPerLitre,
    double StartLitres,double EndLitres,double Seconds,double ReferenceSeconds)
{
    if(Seconds<=0) return 0;
    const double Scale=FMath::Max(.001f,DensityKgPerLitre)/FMath::Max(.001f,EmptyMassKg);
    const double Start=1+FMath::Max(0.0,StartLitres)*Scale,End=1+FMath::Max(0.0,EndLitres)*Scale;
    // 喷洒段药量线性下降时解析积分，避免固定满载上界掩盖提前减重收益。
    const double Mean=FMath::Abs(Start-End)<1.e-8 ? FMath::Pow((Start+End)*.5,1.5) :
        (FMath::Pow(Start,2.5)-FMath::Pow(End,2.5))/(2.5*(Start-End));
    return Seconds*Mean/FMath::Max(1.0,ReferenceSeconds);
}

float UAgricultureCoordinator::FlightSeconds(const FAgricultureAgentState& A) const
{
    return FMath::Max(1.f,Config.BatteryFlightSeconds)/PayloadPowerRatio(A.EmptyMassKg,A.LiquidLitres,Config.LiquidDensityKgPerLitre);
}

double UAgricultureCoordinator::SupplyTravelTime(const FVector& Position,const FVector& Velocity,double PathDistance,float Acceleration) const
{
    const double BrakeTime=1.9*Velocity.Size()/FMath::Max(1.f,Acceleration);
    const FVector Stop=Position+Velocity*(BrakeTime*.5);
    const double LiftDistance=FVector::Dist(Position,Stop)+FMath::Abs(double(Config.TransitHeightCm)-Stop.Z);
    return Config.CruiseTimeFactor*LiftDistance/150+
        (LiftDistance>Config.ArrivalRadiusCm ? Config.RouteArrivalAllowanceSeconds : 0)+
        Config.CruiseTimeFactor*PathDistance/FMath::Max(1.f,Config.TransitSpeedCm)+
        (PathDistance>1 ? Config.RouteArrivalAllowanceSeconds : 0)+Config.AirportApproachSeconds+1;
}

FAgricultureSortieForecast UAgricultureCoordinator::ForecastSortie(const FAgricultureAgentState& A,
    const FVector& Position,const FAgriculturePlotState& Field,const FSupplyAirportState& Station,float Load,float Battery,bool IncludeQueue,double StationWaitSnapshotSeconds) const
{
    FAgricultureSortieForecast Out;Out.EndPosition=Position;
    const int32 FirstEnd=Field.NextPoint%2==0 ? Field.NextPoint+1 : Field.NextPoint;
    if(!Field.StripPoints.IsValidIndex(FirstEnd) || !Station.Config.bEnabled || Load<1 || Battery<=Config.BatteryReserveFraction) return Out;
    const double Width=FBox(Field.Config.Boundary).GetSize().Y/FMath::Max(1,Field.StripPoints.Num()/2);
    const double LitresPerCm=Width*Field.Config.ApplicationLitresPerHa/1.e8;
    if(LitresPerCm<=0) return Out;
    const double Cruise=Config.CruiseTimeFactor/FMath::Max(1.f,FMath::Min(Config.TransitSpeedCm,400.f));
    const float Acceleration=Manager ? Manager->TaskExecutionAccelerationCm : 150.f;
    const auto* Aircraft=Pawn(A.AgentID);
    const auto* Tracker=Aircraft ? Aircraft->GetTrajectoryTracker() : GetDefault<UTrajectoryTracker>();
    const double WorkTimeScale=FMath::Max(.01f,Tracker->MinAdaptiveTimeScale);
    const double Headland=FCoveragePlanner::HeadlandDistance(Config.SpraySpeedCm,Acceleration);
    FVector ReturnVelocity=FVector::ZeroVector;
    double Liquid=Load,Elapsed=0;
    // 机场时间线只取一次快照，未来到达时扣除已流逝时间；队列悬停也消耗载荷对应的电量。
    const double StationAvailable=IncludeQueue ? (StationWaitSnapshotSeconds>=0 ? StationWaitSnapshotSeconds : EstimateWaitSeconds(Station,0,A.AgentID)) : 0;
    const double ReferenceSeconds=FMath::Max(1.0,double(Config.BatteryFlightSeconds));
    const double MassScale=FMath::Max(.001f,Config.LiquidDensityKgPerLitre)/FMath::Max(.001f,A.EmptyMassKg);
    double PowerLoads[2]={-1,-1},PowerRatios[2]={0,0};int32 NextPowerSlot=0;
    const auto Energy=[&](double Start,double End,double Time)
    {
        if(Time<=0) return 0.0;
        if(Start!=End) return SegmentEnergyFraction(A.EmptyMassKg,Config.LiquidDensityKgPerLitre,Start,End,Time,Config.BatteryFlightSeconds);
        // 每个空间步反复检查当前与下一载荷的返场预算，复用相同载荷的功率计算。
        const double Load=FMath::Max(0.0,Start);
        for(int32 I=0;I<2;++I) if(PowerLoads[I]==Load) return Time*PowerRatios[I]/ReferenceSeconds;
        const int32 Slot=NextPowerSlot;NextPowerSlot=1-NextPowerSlot;
        PowerLoads[Slot]=Load;PowerRatios[Slot]=FMath::Pow(1+Load*MassScale,1.5);
        return Time*PowerRatios[Slot]/ReferenceSeconds;
    };
    const auto ReturnTime=[&](const FVector& Point)
    {
        const FVector Lift=SupplyDepartureTarget(Point,ReturnVelocity,Config.TransitHeightCm,Acceleration);
        FVector Goal=Station.Config.DockPosition;Goal.Z=Config.TransitHeightCm;
        const double Travel=SupplyTravelTime(Point,ReturnVelocity,FVector::Dist(Lift,Goal),Acceleration);
        return Travel+FMath::Max(0.0,StationAvailable-FMath::Max(0.0,Elapsed+Travel-Config.AirportApproachSeconds));
    };
    const auto Required=[&](const FVector& Point,double Remaining,double Used)
    {return Used+Energy(Remaining,Remaining,ReturnTime(Point)+Config.SupplyTimeBufferSeconds+Config.RouteArrivalAllowanceSeconds)+Config.BatteryReserveFraction;};
    const FVector FirstDirection=(Field.StripPoints[FirstEnd]-Field.StripPoints[FirstEnd-1]).GetSafeNormal();
    const FVector Entry=Field.StripPoints[FirstEnd-1]+FirstDirection*Field.StripCoveredCm;
    const FVector HeadlandEntry=Entry-FirstDirection*Headland;
    FVector Departure=Position;
    const bool AtDock=Position.Equals(Station.Config.DockPosition,1);
    if(AtDock)
    {
        Departure=Station.Config.DockPosition;
        Departure.X+=Departure.X<0 ? 1000 : -1000;Departure.Z=Config.TransitHeightCm;
    }
    double Approach=0;
    if(AtDock)
        Approach+=Config.CruiseTimeFactor*(FMath::Max(0.0,double(Config.TransitHeightCm)-Position.Z)/200+1000.0/150)+2*Config.RouteArrivalAllowanceSeconds;
    if(FVector::Dist2D(Departure,HeadlandEntry)>2000 || Departure.Z>HeadlandEntry.Z+200)
    {
        FVector CruiseEntry=HeadlandEntry;CruiseEntry.Z=Config.TransitHeightCm;
        FVector CruiseDeparture=Departure;CruiseDeparture.Z=Config.TransitHeightCm;
        Approach+=(FVector::Dist(Departure,CruiseDeparture)+FVector::Dist(CruiseDeparture,CruiseEntry)+FVector::Dist(CruiseEntry,HeadlandEntry))*Cruise;
    }
    else Approach+=FVector::Dist(Departure,HeadlandEntry)*Cruise;
    Approach+=Config.RouteArrivalAllowanceSeconds+2*Headland/FMath::Max(1.f,Config.SpraySpeedCm+FMath::Min(Config.SpraySpeedCm,100.f));
    // 进入条带和地头转弯也由追踪器推进，须与执行侧采用同一最低时间推进比例。
    Approach/=WorkTimeScale;
    ReturnVelocity=FirstDirection*Config.SpraySpeedCm;
    double Used=Energy(Liquid,Liquid,Approach);Elapsed=Approach;
    Out.EntryFlightSeconds=Approach;Out.EntryReturnSeconds=ReturnTime(Entry);
    double PeakRequired=Required(Entry,Liquid,Used);
    if(PeakRequired>Battery+1.e-6) return Out;
    FVector Cursor=Entry;
    for(int32 End=FirstEnd;End<Field.StripPoints.Num();End+=2)
    {
        const FVector Direction=(Field.StripPoints[End]-Field.StripPoints[End-1]).GetSafeNormal();
        ReturnVelocity=Direction*Config.SpraySpeedCm;
        const double Length=FVector::Dist(Field.StripPoints[End-1],Field.StripPoints[End]);
        double Covered=End==FirstEnd ? Field.StripCoveredCm : 0;
        if(End!=FirstEnd)
        {
            const double Turn=(2*Headland+PI*FVector::Dist(Cursor,Field.StripPoints[End-1]))/FMath::Max(1.f,Config.SpraySpeedCm)/WorkTimeScale;
            const double NextUsed=Used+Energy(Liquid,Liquid,Turn);
            if(Required(Field.StripPoints[End-1],Liquid,NextUsed)>Battery+1.e-6) break;
            PeakRequired=FMath::Max(PeakRequired,Required(Field.StripPoints[End-1],Liquid,NextUsed));
            Used=NextUsed;Elapsed+=Turn;Cursor=Field.StripPoints[End-1];
        }
        while(Covered<Length-1.e-4)
        {
            // 固定空间步长同时检查中途返场可达性，药液只在有效条带上消耗。
            const double Advance=FMath::Min(500.0,FMath::Min(Length-Covered,FMath::Max(0.0,Liquid-1)/LitresPerCm));
            if(Advance<1.e-4) break;
            const double NextLiquid=Liquid-Advance*LitresPerCm,Time=Advance/FMath::Max(1.f,Config.SpraySpeedCm)/WorkTimeScale;
            const FVector Next=Field.StripPoints[End-1]+Direction*(Covered+Advance);
            const double NextUsed=Used+Energy(Liquid,NextLiquid,Time);
            const double ForwardRequired=NextUsed+Energy(NextLiquid,NextLiquid,ReturnTime(Cursor)+Advance*Cruise+
                Config.SupplyTimeBufferSeconds+Config.RouteArrivalAllowanceSeconds)+Config.BatteryReserveFraction;
            const double NextRequired=FMath::Max(Required(Next,NextLiquid,NextUsed),ForwardRequired);
            if(NextRequired>Battery+1.e-6) break;
            PeakRequired=FMath::Max(PeakRequired,NextRequired);
            Covered+=Advance;Out.AppliedLitres+=Advance*LitresPerCm;Liquid=NextLiquid;
            Used=NextUsed;Elapsed+=Time;Cursor=Next;
            Out.bFeasible=true;Out.EndPosition=Cursor;Out.FlightSeconds=Elapsed;
            Out.ReturnSeconds=ReturnTime(Cursor);Out.RequiredBattery=PeakRequired;
            Out.EnergyFraction=Used+Energy(Liquid,Liquid,Out.ReturnSeconds);
        }
        if(Covered<Length-1.e-4) break;
        Out.bCompleted=End+2>=Field.StripPoints.Num();
    }
    return Out;
}

double UAgricultureCoordinator::FindSupplySlotWait(double Start,double Duration,const TArray<FAgricultureSupplySlot>& Slots,int32 AgentID)
{
    double Cursor=Start;
    for(int32 Pass=0;Pass<=Slots.Num();++Pass)
    {
        const double Previous=Cursor;
        for(const auto& Slot:Slots)
            if(Slot.AgentID!=AgentID && Cursor<Slot.EndSeconds && Cursor+Duration>Slot.BeginSeconds)
                Cursor=Slot.EndSeconds;
        if(Cursor==Previous) break;
    }
    return Cursor-Start;
}

void UAgricultureCoordinator::ReserveSupplyStock(FSupplyAirportState& Station,const FAgricultureAgentState& A,
    const FAgriculturePlotState& Field,float Target)
{
    const bool Cleaning=A.Recipe!=Field.Config.Recipe;
    const double Fraction=FMath::Clamp(double(Field.Config.ConcentrateFraction),0.0,1.0);
    const double Fill=FMath::Max(0.0,double(Target)-(Cleaning ? 0 : A.LiquidLitres));
    Station.Config.WaterLitres-=float(Fill*(1-Fraction)+(Cleaning ? 5 : 0));
    Station.Config.ConcentrateLitres-=float(Fill*Fraction);
    if(Cleaning) Station.WasteLitres+=A.LiquidLitres+5;
    else Station.WasteLitres+=FMath::Max(0.f,A.LiquidLitres-Target);
}

FSupplyAirportState UAgricultureCoordinator::AvailableSupplyStock(const FSupplyAirportState& Station,int32 AgentID) const
{
    auto Available=Station;
    const int32 I=Airports.IndexOfByPredicate([&Station](const auto& S){return S.Config.AirportID==Station.Config.AirportID;});
    if(SupplySlots.IsValidIndex(I)) for(const auto& Slot:SupplySlots[I])
        if(Slot.AgentID!=AgentID && Slot.EndSeconds>ElapsedSeconds)
        {
            Available.Config.WaterLitres-=float(Slot.WaterLitres);Available.Config.ConcentrateLitres-=float(Slot.ConcentrateLitres);
            Available.WasteLitres+=float(Slot.WasteLitres);
        }
    return Available;
}

void UAgricultureCoordinator::ConsumeSupplyReservation(int32 AirportID,int32 AgentID,double Water,double Concentrate,double Waste)
{
    const int32 I=Airports.IndexOfByPredicate([AirportID](const auto& S){return S.Config.AirportID==AirportID;});
    if(!SupplySlots.IsValidIndex(I)) return;
    for(auto& Slot:SupplySlots[I]) if(Slot.AgentID==AgentID && Slot.BeginSeconds<=ElapsedSeconds)
    {
        const double UsedWater=FMath::Min(Water,Slot.WaterLitres),UsedConcentrate=FMath::Min(Concentrate,Slot.ConcentrateLitres),UsedWaste=FMath::Min(Waste,Slot.WasteLitres);
        Slot.WaterLitres-=UsedWater;Slot.ConcentrateLitres-=UsedConcentrate;Slot.WasteLitres-=UsedWaste;
        Water-=UsedWater;Concentrate-=UsedConcentrate;Waste-=UsedWaste;
    }
}

FAgricultureServicePlan UAgricultureCoordinator::PlanService(const FAgricultureAgentState& A,
    const FAgriculturePlotState& Field,const FSupplyAirportState& Station,bool IncludeQueue) const
{
    FAgricultureServicePlan Best;
    double Demand=0;RemainingWorkSeconds(Field,Demand);
    const bool Cleaning=A.Recipe!=Field.Config.Recipe;
    const double Existing=Cleaning ? 0 : A.LiquidLitres;
    const double Capacity=FMath::Min(double(Config.TankCapacityLitres),double(A.PayloadLimitKg)/FMath::Max(.001f,Config.LiquidDensityKgPerLitre));
    const double Fraction=FMath::Clamp(double(Field.Config.ConcentrateFraction),0.0,1.0);
    const double Water=Station.Config.WaterLitres-(Cleaning ? 5 : 0);
    if(Cleaning && (Water<0 || Station.WasteLitres+A.LiquidLitres+5>Station.Config.WasteCapacityLitres)) return Best;
    const double Stock=FMath::Min(Fraction<1 ? FMath::Max(0.0,Water)/(1-Fraction) : DBL_MAX,
        Fraction>0 ? double(Station.Config.ConcentrateLitres)/Fraction : DBL_MAX);
    const double Maximum=FMath::Min(Capacity,FMath::Min(FMath::Max(Existing,Demand+1),Existing+Stock));
    if(Maximum<1) return Best;
    // 服务候选只改变本机装载和电量；同次搜索内共用队列快照，避免重复求解前机服务方案。
    const double StationWait=IncludeQueue ? EstimateWaitSeconds(Station,0,A.AgentID) : 0;
    // 优先保留已配药液；只有现有载荷没有可行航次时，才搜索受废液容量约束的减载。
    for(int32 Pass=0;Pass<2 && Best.LiquidLitres<0;++Pass)
    {
        const double Minimum=Pass==0 ? FMath::Max(1.0,Existing) : 1.0;
        const double Upper=Pass==0 ? Maximum : FMath::Min(Maximum,Existing-.001);
        if(Upper<Minimum) continue;
        for(int32 I=0;I<=16;++I)
        {
            const float Load=float(FMath::Lerp(Minimum,Upper,I/16.0));
            const double Drain=FMath::Max(0.0,Existing-Load);
            if(Station.WasteLitres+Drain>Station.Config.WasteCapacityLitres) continue;
            float PreviousBattery=-1;
            for(int32 J=0;J<=4;++J)
            {
                const float Battery=FMath::Max(A.Battery,FMath::Lerp(Config.BatteryReserveFraction,1.f,J/4.f));
                if(FMath::IsNearlyEqual(Battery,PreviousBattery)) continue;
                PreviousBattery=Battery;
                const auto Forecast=ForecastSortie(A,Station.Config.DockPosition,Field,Station,Load,Battery,IncludeQueue,StationWait);
                if(!Forecast.bFeasible || Forecast.AppliedLitres<=0) continue;
                const float TargetBattery=FMath::Clamp(FMath::Max(A.Battery,float(Forecast.RequiredBattery)+.005f),0.f,1.f);
                const double Prep=Station.Config.MixSeconds+(Cleaning ? Station.Config.CleanSeconds : 0);
                const double Fill=FMath::Max(0.0,double(Load)-Existing)/FMath::Max(.001f,Station.Config.RefillLitresPerMinute/60);
                const double Charge=FMath::Max(0.f,TargetBattery-A.Battery)*Station.Config.ChargeSeconds;
                const double Service=FMath::Max(Prep+Fill+Drain/FMath::Max(.001f,Station.Config.RefillLitresPerMinute/60),Charge);
                const double Cycles=FMath::Max(1.0,FMath::CeilToDouble(Demand/Forecast.AppliedLitres-1.e-6));
                const double Recharge=Forecast.EnergyFraction*Station.Config.ChargeSeconds;
                const double Refill=Forecast.AppliedLitres/FMath::Max(.001f,Station.Config.RefillLitresPerMinute/60)+Station.Config.MixSeconds;
                const double Completion=Service+Forecast.FlightSeconds+Forecast.ReturnSeconds+
                    (Cycles-1)*(Forecast.FlightSeconds+Forecast.ReturnSeconds+FMath::Max(Recharge,Refill));
                if(Completion<Best.CompletionSeconds-1.e-6 ||
                    (FMath::IsNearlyEqual(Completion,Best.CompletionSeconds,1.e-6) && Load<Best.LiquidLitres))
                {Best.AirportID=Station.Config.AirportID;Best.LiquidLitres=Load;Best.BatteryFraction=TargetBattery;Best.ServiceSeconds=Service;Best.CompletionSeconds=Completion;}
                // 作业已完成或达到本次装载的可喷洒上限时，更高电量会产生相同航次与充电目标。
                if(Forecast.bCompleted || Forecast.AppliedLitres>=double(Load)-1-1.e-6) break;
            }
        }
    }
    return Best;
}

double UAgricultureCoordinator::SupplyTransferTravelSeconds(const FAgricultureAgentState& A,
    const FSupplyAirportState& Source,const FSupplyAirportState& Destination) const
{
    const FVector Lift=SupplyDepartureTarget(Source.Config.DockPosition,FVector::ZeroVector,Config.TransitHeightCm,150);
    FVector Goal=Destination.Config.DockPosition;Goal.Z=Config.TransitHeightCm;
    const int32 I=Airports.IndexOfByPredicate([&Destination](const auto& S){return S.Config.AirportID==Destination.Config.AirportID;});
    const bool AtOwnDock=(A.Phase==EAgriculturePhase::Servicing || A.Phase==EAgriculturePhase::Idle || A.Phase==EAgriculturePhase::Completed) &&
        A.AirportID==Source.Config.AirportID;
    return (AtOwnDock && A.SupplyTravelSeconds.IsValidIndex(I) ? A.SupplyTravelSeconds[I] :
        SupplyTravelTime(Source.Config.DockPosition,FVector::ZeroVector,FVector::Dist(Lift,Goal),150))+
        Config.CruiseTimeFactor*1000.0/150+Config.RouteArrivalAllowanceSeconds;
}

FAgricultureServicePlan UAgricultureCoordinator::PlanDockService(const FAgricultureAgentState& A,
    const FAgriculturePlotState& Field,const FSupplyAirportState& Station,bool IncludeQueue) const
{
    auto Best=PlanService(A,Field,AvailableSupplyStock(Station,A.AgentID),IncludeQueue);
    if(!Station.Config.bEnabled) return Best;
    for(int32 I=0;I<Airports.Num();++I)
    {
        const auto& Candidate=Airports[I];
        if(Candidate.Config.AirportID==Station.Config.AirportID || !CanService(Candidate)) continue;
        for(int32 CleanFirst=0;CleanFirst<2;++CleanFirst)
        {
            auto Departing=A;double Preparation=0;
            if(CleanFirst)
            {
                if(A.Recipe==Field.Config.Recipe) continue;
                auto SourceStock=AvailableSupplyStock(Station,A.AgentID);
                Departing.bCleaned=false;
                if(!CleanResidue(Departing,SourceStock,Field.Config.Recipe)) continue;
                Preparation=Station.Config.CleanSeconds;
            }
            const double Travel=SupplyTransferTravelSeconds(Departing,Station,Candidate);
            if(!FMath::IsFinite(Travel) || Travel>=MAX_FLT) continue;
            const double Energy=SegmentEnergyFraction(A.EmptyMassKg,Config.LiquidDensityKgPerLitre,Departing.LiquidLitres,Departing.LiquidLitres,
                Travel,Config.BatteryFlightSeconds);
            const double Required=Energy+Config.BatteryReserveFraction+
                SegmentEnergyFraction(A.EmptyMassKg,Config.LiquidDensityKgPerLitre,Departing.LiquidLitres,Departing.LiquidLitres,
                    Config.SupplyTimeBufferSeconds+Config.RouteArrivalAllowanceSeconds,Config.BatteryFlightSeconds)+.005;
            if(Required>1) continue;
            const float ChargeTarget=float(FMath::Max(double(A.Battery),Required));
            auto Future=Departing;Future.Battery=ChargeTarget-float(Energy);
            const auto Next=PlanService(Future,Field,AvailableSupplyStock(Candidate,A.AgentID),false);
            if(Next.LiquidLitres<0) continue;
            const double Service=FMath::Max(Preparation,double(ChargeTarget-A.Battery)*Station.Config.ChargeSeconds);
            double GroundDelay=IncludeQueue ? EstimateWaitSeconds(Candidate,float(FMath::Max(0.0,Service+Travel-Config.AirportApproachSeconds)),A.AgentID) : 0;
            const double Departure=Config.CruiseTimeFactor*(FMath::Max(0.0,double(Config.TransitHeightCm)-Candidate.Config.DockPosition.Z)/200+1000.0/150)+2*Config.RouteArrivalAllowanceSeconds;
            if(IncludeQueue && SupplySlots.IsValidIndex(I))
                GroundDelay+=FindSupplySlotWait(ElapsedSeconds+Service+GroundDelay+Travel-Config.AirportApproachSeconds,
                    Config.AirportApproachSeconds+Next.ServiceSeconds+Departure,SupplySlots[I],A.AgentID);
            const double Completion=Service+GroundDelay+Travel+Next.CompletionSeconds;
            if(Completion<Best.CompletionSeconds-1.e-6)
            {
                Best.AirportID=Candidate.Config.AirportID;Best.LiquidLitres=-2;
                Best.BatteryFraction=ChargeTarget;Best.ServiceSeconds=Service;Best.CompletionSeconds=Completion;
                Best.DepartureDelaySeconds=Service+GroundDelay;Best.bCleanBeforeTransfer=CleanFirst!=0;
            }
        }
    }
    return Best;
}

FAgriculturePlotState UAgricultureCoordinator::ChooseWorkOrder(const FAgricultureAgentState& A,
    const FVector& Position,const FAgriculturePlotState& Field,double* OutCost) const
{
    FAgriculturePlotState Best=Field;double BestCost=DBL_MAX;
    const int32 FirstEnd=Field.NextPoint%2==0 ? Field.NextPoint+1 : Field.NextPoint;
    if(!Field.StripPoints.IsValidIndex(FirstEnd))
    {if(OutCost) *OutCost=DBL_MAX;return Field;}
    const double Length=FVector::Dist(Field.StripPoints[FirstEnd-1],Field.StripPoints[FirstEnd]);
    if(Field.StripCoveredCm>=Length-1)
    {if(OutCost) *OutCost=EstimateSectionCost(A,Position,Field);return Field;}
    TArray<FVector> Remaining;
    for(int32 I=FirstEnd-1;I<Field.StripPoints.Num();++I) Remaining.Add(Field.StripPoints[I]);
    // 只保留当前条带尚未喷洒的线段，反向进入时不重复施药，也不改变已有覆盖记录。
    Remaining[0]+=(Remaining[1]-Remaining[0]).GetSafeNormal()*Field.StripCoveredCm;
    for(int32 Variant=0;Variant<4;++Variant)
    {
        FAgriculturePlotState Candidate=Field;Candidate.StripPoints.Empty();
        for(int32 I=0;I<FirstEnd-1;++I) Candidate.StripPoints.Add(Field.StripPoints[I]);
        Candidate.NextPoint=FirstEnd;Candidate.StripCoveredCm=0;Candidate.ResumePosition=FVector::ZeroVector;
        const int32 Count=Remaining.Num()/2;
        for(int32 I=0;I<Count;++I)
        {
            const int32 Strip=(Variant&1) ? Count-1-I : I;
            const int32 Flip=(Variant&2) ? 1 : 0;
            Candidate.StripPoints.Add(Remaining[Strip*2+Flip]);
            Candidate.StripPoints.Add(Remaining[Strip*2+1-Flip]);
        }
        const double Cost=EstimateSectionCost(A,Position,Candidate);
        if(Cost<BestCost) {BestCost=Cost;Best=MoveTemp(Candidate);}
    }
    if(OutCost) *OutCost=BestCost;
    return Best;
}

double UAgricultureCoordinator::EstimateSectionCost(const FAgricultureAgentState& A,const FVector& Position,
    const FAgriculturePlotState& Part) const
{
    if(Part.Config.Task.RequiredCapabilities!=0 || Part.Config.Task.RequiredPayload>A.PayloadLimitKg) return DBL_MAX;
    const int32 FirstEnd=Part.NextPoint%2==0 ? Part.NextPoint+1 : Part.NextPoint;
    if(!Part.StripPoints.IsValidIndex(FirstEnd)) return DBL_MAX;
    const double Deadline=FMath::Min(double(Part.Config.Task.Deadline),double(Part.Config.Task.LatestFinish));
    const double Limit=Deadline-ElapsedSeconds;
    if(Limit<0) return DBL_MAX;
    double Best=DBL_MAX,MinimumRejectedCost=DBL_MAX;
    uint32 RejectionMask=0;int32 UsableAirports=0;
    const double Cruise=Config.CruiseTimeFactor/FMath::Max(1.f,FMath::Min(Config.TransitSpeedCm,400.f));
    for(const auto& Station:Airports)
    {
        const bool AtOwnBerth=Station.OccupantID==A.AgentID && A.AirportID==Station.Config.AirportID;
        if((!CanService(Station) && !AtOwnBerth) || !Station.Config.bEnabled) continue;
        ++UsableAirports;
        for(int32 SupplyFirst=0;SupplyFirst<4;++SupplyFirst)
        {
            FAgricultureAgentState State=A;FAgriculturePlotState Remaining=Part;
            FSupplyAirportState Supplies=AvailableSupplyStock(Station,A.AgentID);
            FVector Cursor=Position;
            double Cost=FMath::Max(0.0,double(Part.Config.Task.EarliestStart)-ElapsedSeconds);
            bool Complete=false,NeedsService=SupplyFirst!=0,TransferArrived=false;
            if(SupplyFirst>=2)
            {
                const auto* Source=Airports.FindByPredicate([&A](const auto& S){return S.Config.AirportID==A.AirportID && S.OccupantID==A.AgentID;});
                if(!Source || !Source->Config.bEnabled || Source->Config.AirportID==Station.Config.AirportID ||
                    !Position.Equals(Source->Config.DockPosition,1)) continue;
                double SourcePreparation=0;
                if(SupplyFirst==3)
                {
                    if(A.Recipe==Part.Config.Recipe) continue;
                    auto SourceStock=AvailableSupplyStock(*Source,A.AgentID);
                    State.bCleaned=false;
                    if(!CleanResidue(State,SourceStock,Part.Config.Recipe)) {RejectionMask|=4;continue;}
                    SourcePreparation=Source->Config.CleanSeconds;
                }
                const double Travel=SupplyTransferTravelSeconds(State,*Source,Station);
                if(!FMath::IsFinite(Travel) || Travel>=MAX_FLT) continue;
                const double Energy=SegmentEnergyFraction(A.EmptyMassKg,Config.LiquidDensityKgPerLitre,State.LiquidLitres,State.LiquidLitres,Travel,Config.BatteryFlightSeconds);
                const double Required=Energy+Config.BatteryReserveFraction+.005+
                    SegmentEnergyFraction(A.EmptyMassKg,Config.LiquidDensityKgPerLitre,State.LiquidLitres,State.LiquidLitres,
                        Config.SupplyTimeBufferSeconds+Config.RouteArrivalAllowanceSeconds,Config.BatteryFlightSeconds);
                if(Required>1) {RejectionMask|=2;continue;}
                const double ChargeTarget=FMath::Max(double(A.Battery),Required);
                const double Charge=FMath::Max(SourcePreparation,(ChargeTarget-A.Battery)*Source->Config.ChargeSeconds);
                // 尚未起飞时在源站充电并等待，按选择的清洗方案保留或降低载荷，不将等待算成悬停耗电。
                const double GroundWait=EstimateWaitSeconds(Station,float(Cost+Charge+Travel-Config.AirportApproachSeconds),A.AgentID);
                Cost+=Charge+GroundWait+Travel;
                State.Battery=float(ChargeTarget-Energy);Cursor=Station.Config.DockPosition;TransferArrived=true;
            }
            for(int32 Cycle=0;Cycle<128;++Cycle)
            {
                if(Cost>Limit || Cost>=Best)
                {if(Cost>Limit){RejectionMask|=16;MinimumRejectedCost=FMath::Min(MinimumRejectedCost,Cost);}break;}
                auto Forecast=ForecastSortie(State,Cursor,Remaining,Supplies,State.LiquidLitres,State.Battery);
                if(NeedsService || State.Recipe!=Remaining.Config.Recipe || !Forecast.bFeasible)
                {
                    const bool AtDock=Cursor.Equals(Supplies.Config.DockPosition,1);
                    const double Travel=AtDock ? 0 : FVector::Dist(Cursor,Supplies.Config.DockPosition)*Cruise+Config.AirportApproachSeconds;
                    const double Wait=(AtOwnBerth || TransferArrived) && Cycle==0 ? 0 : EstimateWaitSeconds(Station,float(Cost+Travel),A.AgentID);
                    const double Energy=SegmentEnergyFraction(State.EmptyMassKg,Config.LiquidDensityKgPerLitre,
                        State.LiquidLitres,State.LiquidLitres,Travel+Wait+Config.SupplyTimeBufferSeconds,Config.BatteryFlightSeconds);
                    if(!AtDock && State.Battery-Energy<Config.BatteryReserveFraction) {RejectionMask|=2;break;}
                    State.Battery=FMath::Max(0.f,State.Battery-float(Travel+Wait)/FlightSeconds(State));
                    Cost+=Travel+Wait;
                    if(Cost>Limit || Cost>=Best)
                {if(Cost>Limit){RejectionMask|=16;MinimumRejectedCost=FMath::Min(MinimumRejectedCost,Cost);}break;}
                    const auto Service=PlanService(State,Remaining,Supplies);
                    if(Service.LiquidLitres<0) {RejectionMask|=4;break;}
                    const bool Cleaning=State.Recipe!=Remaining.Config.Recipe;
                    const double Fraction=FMath::Clamp(double(Remaining.Config.ConcentrateFraction),0.0,1.0);
                    const double Fill=FMath::Max(0.0,double(Service.LiquidLitres)-(Cleaning ? 0 : State.LiquidLitres));
                    Supplies.Config.WaterLitres-=float(Fill*(1-Fraction)+(Cleaning ? 5 : 0));
                    Supplies.Config.ConcentrateLitres-=float(Fill*Fraction);
                    if(Cleaning) Supplies.WasteLitres+=State.LiquidLitres+5;
                    else Supplies.WasteLitres+=FMath::Max(0.f,State.LiquidLitres-Service.LiquidLitres);
                    Cost+=Service.ServiceSeconds;
                    if(Cost>Limit || Cost>=Best)
                {if(Cost>Limit){RejectionMask|=16;MinimumRejectedCost=FMath::Min(MinimumRejectedCost,Cost);}break;}
                    State.LiquidLitres=Service.LiquidLitres;State.Battery=FMath::Max(State.Battery,Service.BatteryFraction);
                    State.Recipe=Remaining.Config.Recipe;Cursor=Supplies.Config.DockPosition;
                    Forecast=ForecastSortie(State,Cursor,Remaining,Supplies,State.LiquidLitres,State.Battery);
                }
                if(!Forecast.bFeasible || Forecast.AppliedLitres<1.e-6) {RejectionMask|=8;break;}
                Cost+=Forecast.FlightSeconds+Forecast.ReturnSeconds;
                if(Cost>Limit || Cost>=Best)
                {if(Cost>Limit){RejectionMask|=16;MinimumRejectedCost=FMath::Min(MinimumRejectedCost,Cost);}break;}
                State.LiquidLitres=FMath::Max(0.f,State.LiquidLitres-float(Forecast.AppliedLitres));
                State.Battery=FMath::Max(0.f,State.Battery-float(Forecast.EnergyFraction));
                Cursor=Supplies.Config.DockPosition;
                if(Forecast.bCompleted) {Complete=true;break;}
                AdvanceForecastProgress(Remaining,Forecast.AppliedLitres);
                NeedsService=true;
            }
            if(Complete) Best=FMath::Min(Best,Cost);
        }
    }
    if(Best==DBL_MAX)
        UE_LOG_THROTTLE(10.0,LogAgriculture,Log,TEXT("[Agriculture] SectionCostRejected Agent=%d Plot=%d FarmTime=%.2f Limit=%.2f UsableAirports=%d Reasons=%u FirstOverBudget=%.2f Liquid=%.2f Battery=%.3f"),
            A.AgentID,Part.Config.Task.TaskID,ElapsedSeconds,Limit,UsableAirports,RejectionMask,MinimumRejectedCost==DBL_MAX ? -1 : MinimumRejectedCost,A.LiquidLitres,A.Battery);
    return Best<DBL_MAX && ElapsedSeconds+Best<=Deadline ? Best : DBL_MAX;
}

void UAgricultureCoordinator::AdvanceForecastProgress(FAgriculturePlotState& Field,double AppliedLitres)
{
    const double Width=FBox(Field.Config.Boundary).GetSize().Y/FMath::Max(1,Field.StripPoints.Num()/2);
    if(Width<=0 || Field.Config.ApplicationLitresPerHa<=0) return;
    double Advance=AppliedLitres*1.e8/(Width*Field.Config.ApplicationLitresPerHa);
    Field.NextPoint=Field.NextPoint%2==0 ? Field.NextPoint+1 : Field.NextPoint;
    while(Field.StripPoints.IsValidIndex(Field.NextPoint) && Advance>1.e-4)
    {
        const double Length=FVector::Dist(Field.StripPoints[Field.NextPoint-1],Field.StripPoints[Field.NextPoint]);
        const double Step=FMath::Min(Advance,FMath::Max(0.0,Length-Field.StripCoveredCm));
        Field.StripCoveredCm+=float(Step);Advance-=Step;
        if(Field.StripCoveredCm>=Length-1.e-3) {Field.NextPoint+=2;Field.StripCoveredCm=0;}
        else break;
    }
    Field.AppliedLitres+=AppliedLitres;
    Field.CoveredSquareMetres+=AppliedLitres*10000/Field.Config.ApplicationLitresPerHa;
}
