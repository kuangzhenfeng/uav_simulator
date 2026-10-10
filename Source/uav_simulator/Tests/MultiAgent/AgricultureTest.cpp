#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../MultiAgent/AgricultureCoordinator.h"
#include "../../MultiAgent/TaskMonitor.h"
#include "../../MultiAgent/TaskAllocator.h"
#include "../../Control/AttitudeController.h"
#include "../../Planning/AStarPathPlanner.h"
#include "../../Planning/TrajectoryOptimizer.h"
#include "../../Planning/TrajectoryTracker.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureStripTest,"UAVSimulator.MultiAgent.Agriculture.Strips",UAV_TEST_FLAGS)
bool FAgricultureStripTest::RunTest(const FString&)
{
    FAgriculturePlot P;P.Boundary={FVector(0,0,0),FVector(6000,0,0),FVector(6000,6000,0),FVector(0,6000,0)};
    TArray<FVector> Points;
    TestTrue(TEXT("Rectangle accepted"),UAgricultureCoordinator::BuildStrips(P,400,Points));
    TestEqual(TEXT("Ten strips have twenty endpoints"),Points.Num(),20);
    TestEqual(TEXT("Spray altitude"),Points[0].Z,400.0);
    TestEqual(TEXT("First forward strip"),Points[1].X,6000.0);
    TestEqual(TEXT("Second reverse strip"),Points[3].X,0.0);
    P.Boundary[2].X=3000;
    TestFalse(TEXT("Unsupported polygon fails explicitly"),UAgricultureCoordinator::BuildStrips(P,400,Points));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgriculturePartitionTest,"UAVSimulator.MultiAgent.Agriculture.PartitionCoverage",UAV_TEST_FLAGS)
bool FAgriculturePartitionTest::RunTest(const FString&)
{
    FAgriculturePlotState P;P.Config.Task.TaskID=42;
    P.Config.Boundary={FVector(0,0,0),FVector(6000,0,0),FVector(6000,6100,0),FVector(0,6100,0)};
    TestTrue(TEXT("Build uneven-width field"),UAgricultureCoordinator::BuildStrips(P.Config,400,P.StripPoints));
    auto Parts=UAgricultureCoordinator::PartitionPlot(P,4);
    TestEqual(TEXT("Four concurrent sections"),Parts.Num(),4);
    TArray<FVector> Joined;double Covered=0,Applied=0;
    for(auto& Part:Parts)
    {
        TestEqual(TEXT("Parent identity retained"),Part.Config.Task.TaskID,42);
        Joined.Append(Part.StripPoints);
        float Liquid=50;
        for(int32 I=1;I<Part.StripPoints.Num();I+=2)
        {
            Part.NextPoint=I;Part.StripCoveredCm=0;
            const FVector Start=Part.StripPoints[I-1],End=Part.StripPoints[I],Mid=(Start+End)/2;
            TestTrue(TEXT("First half recorded"),UAgricultureCoordinator::RecordSpraySegment(Part,Liquid,Start,Mid,100));
            const double Before=Part.CoveredSquareMetres;
            TestTrue(TEXT("Repeated segment accepted"),UAgricultureCoordinator::RecordSpraySegment(Part,Liquid,Start,Mid,100));
            TestEqual(TEXT("No double coverage on resume"),Part.CoveredSquareMetres,Before);
            TestTrue(TEXT("Second half recorded"),UAgricultureCoordinator::RecordSpraySegment(Part,Liquid,Mid,End,100));
        }
        Covered+=Part.CoveredSquareMetres;Applied+=Part.AppliedLitres;
    }
    TestTrue(TEXT("Every original strip owned exactly once"),Joined==P.StripPoints);
    TestTrue(TEXT("All sections cover exact parent area"),FMath::IsNearlyEqual(Covered,3660.0,.01));
    TestTrue(TEXT("Application volume matches parent area"),FMath::IsNearlyEqual(Applied,54.9,.01));
    TestTrue(TEXT("Empty fleet does not partition"),UAgricultureCoordinator::PartitionPlot(P,0).IsEmpty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureSectionStateTest,"UAVSimulator.MultiAgent.Agriculture.SectionOwnership",UAV_TEST_FLAGS)
bool FAgricultureSectionStateTest::RunTest(const FString&)
{
    auto* C=NewObject<UAgricultureCoordinator>();
    FAgriculturePlotState P;P.Config.Task.TaskID=7;
    P.Config.Boundary={FVector(0,0,0),FVector(6000,0,0),FVector(6000,6000,0),FVector(0,6000,0)};
    UAgricultureCoordinator::BuildStrips(P.Config,400,P.StripPoints);
    C->Plots.Add(P);C->Sections=UAgricultureCoordinator::PartitionPlot(P,2);
    for(int32 I=0;I<2;++I)
    {
        FAgricultureAgentState A;A.AgentID=I;A.PlotID=7;A.SectionID=I;A.Phase=EAgriculturePhase::Spraying;
        C->Agents.Add(A);C->Sections[I].AgentID=I;C->Sections[I].NextPoint=1;
        C->Sections[I].StripCoveredCm=100+I;C->Sections[I].CoveredSquareMetres=6+I;
    }
    C->RefreshPlots();
    TestEqual(TEXT("Two agents own the same field"),C->Plots[0].AgentIDs.Num(),2);
    TestEqual(TEXT("Coverage aggregates sections"),C->Plots[0].CoveredSquareMetres,13.0);
    C->Agents[0].Phase=EAgriculturePhase::Returning;
    TestTrue(TEXT("Supply keeps section ownership"),C->Section(C->Agents[0])==&C->Sections[0]);
    C->Fail(C->Agents[0],TEXT("Injected test failure"));C->RefreshPlots();
    TestEqual(TEXT("Fault releases only its section"),C->Sections[0].AgentID,INDEX_NONE);
    TestEqual(TEXT("Other section stays assigned"),C->Sections[1].AgentID,1);
    TestEqual(TEXT("Fault preserves coverage cursor"),C->Sections[0].StripCoveredCm,100.f);
    TestEqual(TEXT("Fault preserves aggregate progress"),C->Plots[0].CoveredSquareMetres,13.0);
    auto& Docked=C->Agents[0];Docked.Phase=EAgriculturePhase::Servicing;Docked.PlotID=7;Docked.SectionID=0;Docked.AirportID=10;
    Docked.Battery=.8f;Docked.LiquidLitres=3;C->Sections[0].AgentID=Docked.AgentID;
    FSupplyAirportState Dock;Dock.Config.AirportID=10;Dock.OccupantID=Docked.AgentID;C->Airports.Add(Dock);
    C->DeferDockedSection(Docked);C->RefreshPlots();
    TestTrue(TEXT("Unavailable service releases work without failing a healthy docked aircraft"),
        Docked.Phase==EAgriculturePhase::Servicing && Docked.PlotID==INDEX_NONE && Docked.SectionID==INDEX_NONE);
    TestTrue(TEXT("Deferred work remains incomplete and preserves its checkpoint"),
        C->Sections[0].AgentID==INDEX_NONE && !C->Sections[0].bCompleted && !C->Sections[0].bFailed &&
        C->Sections[0].StripCoveredCm==100 && C->Plots[0].CoveredSquareMetres==13);
    TestTrue(TEXT("Deferring work retains the physical berth, charge and liquid"),
        Docked.AirportID==10 && C->Airports[0].OccupantID==Docked.AgentID && Docked.Battery==.8f && Docked.LiquidLitres==3);
    TestEqual(TEXT("Deferred work preserves other agent ownership"),C->Sections[1].AgentID,1);
    C->Sections[0].AgentID=2;C->Agents[0].AgentID=2;C->Agents[0].PlotID=7;C->Agents[0].SectionID=0;
    TestTrue(TEXT("Replacement owns existing checkpoint"),C->Section(C->Agents[0])==&C->Sections[0]);
    C->Sections[0].bCompleted=true;C->RefreshPlots();
    TestFalse(TEXT("One section cannot finish parent"),C->Plots[0].bCompleted);
    C->Sections[1].bCompleted=true;C->RefreshPlots();
    TestTrue(TEXT("All sections finish parent"),C->Plots[0].bCompleted);
    C->Reset();TestTrue(TEXT("Reset clears all section reservations"),C->GetSections().IsEmpty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureDepartureTest,"UAVSimulator.MultiAgent.Agriculture.DescendingSupplyDeparture",UAV_TEST_FLAGS)
bool FAgricultureDepartureTest::RunTest(const FString&)
{
    const FVector Position(-6593,-4098.3,989.1),Velocity(176.4,-31.4,-243.2);
    FTrajectory Route;
    TestTrue(TEXT("Return departure builds"),UAgricultureCoordinator::BuildSupplyDeparture(Position,Velocity,1500,150,Route));
    TestTrue(TEXT("Descending return transition is dynamically feasible"),Route.bIsValid);
    for(const auto& Point:Route.Points)
    {
        TestTrue(TEXT("Transition retains safe height"),Point.Position.Z>500);
        TestTrue(TEXT("Transition respects acceleration"),Point.Acceleration.Size()<=151);
    }
    const FVector CruisePosition(17000,10500,1500),CruiseVelocity(300,0,0);
    TestTrue(TEXT("Cruise-height return still builds a braking segment"),
        UAgricultureCoordinator::BuildSupplyDeparture(CruisePosition,CruiseVelocity,1500,150,Route));
    TestTrue(TEXT("Cruise-height departure removes velocity before airport turn"),Route.Points.Last().Velocity.Size()<1);
    TestTrue(TEXT("Cruise-height braking stays on incoming flight axis"),Route.Points.Last().Position.Y==CruisePosition.Y);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureEfficiencyTest,"UAVSimulator.MultiAgent.Agriculture.EfficiencyCosts",UAV_TEST_FLAGS)
bool FAgricultureEfficiencyTest::RunTest(const FString&)
{
    auto* C=NewObject<UAgricultureCoordinator>();FSupplyAirportState Station;Station.Config.AirportID=1;
    Station.Config.DockPosition=FVector(0,0,0);C->Airports.Add(Station);
    FAgriculturePlotState Near;Near.Config.Task.Deadline=3600;Near.Config.Task.LatestFinish=3600;
    Near.Config.Boundary={FVector(1000,0,0),FVector(3000,0,0),FVector(3000,1200,0),FVector(1000,1200,0)};
    UAgricultureCoordinator::BuildStrips(Near.Config,400,Near.StripPoints);
    FAgricultureAgentState Agent;Agent.AgentID=0;Agent.Battery=1;Agent.LiquidLitres=12;Agent.Recipe=Near.Config.Recipe;
    const double Compatible=C->EstimateSectionCost(Agent,FVector::ZeroVector,Near);
    const double Distant=C->EstimateSectionCost(Agent,FVector(-10000,0,0),Near);
    TestTrue(FString::Printf(TEXT("Transfer distance changes assignment cost (near=%.6g, distant=%.6g)"),Compatible,Distant),Compatible<Distant);
    Agent.Recipe=TEXT("Different");const double Changed=C->EstimateSectionCost(Agent,FVector::ZeroVector,Near);
    TestTrue(TEXT("Cleaning and supply are included in efficiency cost"),Changed>Compatible);
    Agent.Recipe=Near.Config.Recipe;Agent.LiquidLitres=0;
    TestTrue(TEXT("Empty tank includes supply cost"),C->EstimateSectionCost(Agent,FVector::ZeroVector,Near)>Compatible);
    double BeforeLiquid=0,AfterLiquid=0;const double Before=C->RemainingWorkSeconds(Near,BeforeLiquid);
    Near.NextPoint=1;Near.StripCoveredCm=1000;
    TestTrue(TEXT("Partial work reduces estimated duration"),C->RemainingWorkSeconds(Near,AfterLiquid)<Before);
    TestTrue(TEXT("Partial work reduces refill requirement"),AfterLiquid<BeforeLiquid);
    Agent.LiquidLitres=0;
    const float Target=C->PlanService(Agent,Near,Station).LiquidLitres;
    TestTrue(TEXT("Short section does not require a full tank"),Target>1 && Target<10);
    TestEqual(TEXT("Service never lowers existing charge"),C->PlanService(Agent,Near,Station).BatteryFraction,Agent.Battery);
    Agent.Battery=.2f;
    const auto Service=C->PlanService(Agent,Near,Station);
    TestTrue(TEXT("Short sortie can depart before full charge"),Service.BatteryFraction<.95f);
    const auto Forecast=C->ForecastSortie(Agent,Station.Config.DockPosition,Near,Station,Service.LiquidLitres,Service.BatteryFraction);
    TestTrue(TEXT("Service targets produce an energy-feasible sortie"),Forecast.bFeasible);
    TestTrue(TEXT("Forecast includes safe return and reserve"),Forecast.RequiredBattery<=Service.BatteryFraction+1.e-6);
    const auto LimitedForecast=C->ForecastSortie(Agent,Station.Config.DockPosition,Near,Station,2,.9f);
    const auto ChargedForecast=C->ForecastSortie(Agent,Station.Config.DockPosition,Near,Station,2,1);
    TestTrue(FString::Printf(TEXT("Extra charge cannot improve a liquid-limited sortie (complete=%d/%d applied=%.9g/%.9g time=%.9g/%.9g reserve=%.9g/%.9g)"),
        LimitedForecast.bCompleted,ChargedForecast.bCompleted,LimitedForecast.AppliedLitres,ChargedForecast.AppliedLitres,
        LimitedForecast.FlightSeconds,ChargedForecast.FlightSeconds,LimitedForecast.RequiredBattery,ChargedForecast.RequiredBattery),
        !LimitedForecast.bCompleted && FMath::IsNearlyEqual(LimitedForecast.AppliedLitres,1.0,1.e-6) &&
        FMath::IsNearlyEqual(LimitedForecast.AppliedLitres,ChargedForecast.AppliedLitres,1.e-6) &&
        FMath::IsNearlyEqual(LimitedForecast.FlightSeconds,ChargedForecast.FlightSeconds,1.e-6) &&
        FMath::IsNearlyEqual(LimitedForecast.RequiredBattery,ChargedForecast.RequiredBattery,1.e-6));
    FSupplyAirportState Busy=Station;Busy.Queue.Add(99);
    TestEqual(TEXT("Future requester accounts for every queued aircraft"),
        C->EstimateWaitSeconds(Busy,0,Agent.AgentID),C->EstimateWaitSeconds(Busy,0,INDEX_NONE));
    const auto FreeSortie=C->ForecastSortie(Agent,Station.Config.DockPosition,Near,Station,2,1);
    auto* TrackerDefaults=GetMutableDefault<UTrajectoryTracker>();
    const float OriginalTimeScale=TrackerDefaults->MinAdaptiveTimeScale;
    TrackerDefaults->MinAdaptiveTimeScale=OriginalTimeScale*.75f;
    const auto SlowerSortie=C->ForecastSortie(Agent,Station.Config.DockPosition,Near,Station,2,1);
    TrackerDefaults->MinAdaptiveTimeScale=OriginalTimeScale;
    TestTrue(TEXT("Adaptive tracking slowdown increases entry travel time and departure reserve"),
        FreeSortie.bFeasible && SlowerSortie.bFeasible && SlowerSortie.EntryFlightSeconds>FreeSortie.EntryFlightSeconds &&
        SlowerSortie.RequiredBattery>FreeSortie.RequiredBattery);
    const auto QueuedSortie=C->ForecastSortie(Agent,Station.Config.DockPosition,Near,Busy,2,1);
    TestTrue(TEXT("Return queue increases sortie energy and required departure charge"),
        FreeSortie.bFeasible && QueuedSortie.bFeasible && QueuedSortie.EnergyFraction>FreeSortie.EnergyFraction &&
        QueuedSortie.RequiredBattery>FreeSortie.RequiredBattery);
    FAgricultureAgentState Occupant=Agent;Occupant.AgentID=99;Occupant.SectionID=0;
    Occupant.Phase=EAgriculturePhase::Servicing;Occupant.ServiceLiquidTarget=-1;
    C->Agents.Add(Occupant);C->Sections.Add(Near);Busy.OccupantID=99;Busy.Queue.Empty();
    TestTrue(TEXT("Unplanned occupant service can be estimated without recursive queue planning"),
        FMath::IsFinite(C->EstimateWaitSeconds(Busy,0,Agent.AgentID)));
    Busy.OccupantID=INDEX_NONE;Busy.Queue.Add(99);
    C->Agents[0].Battery=1;C->Agents[0].SupplyTravelSeconds={80};
    const float ChargedQueue=C->EstimateWaitSeconds(Busy,0,Agent.AgentID);
    C->Agents[0].Battery=.2f;
    TestTrue(TEXT("Queue service time reflects arrival charge rather than a fixed full charge"),
        C->EstimateWaitSeconds(Busy,0,Agent.AgentID)>ChargedQueue);
    TestTrue(TEXT("Charged queued aircraft does not occupy a full recharge interval"),
        ChargedQueue<Station.Config.ChargeSeconds);
    C->Agents[0].Battery=1;
    auto Following=C->Agents[0];Following.AgentID=100;Following.Battery=.6f;C->Agents.Add(Following);
    Busy.Queue={100};const float FollowingAlone=C->EstimateWaitSeconds(Busy,0,Agent.AgentID);
    Busy.Queue={99,100};const float SerialQueue=C->EstimateWaitSeconds(Busy,0,Agent.AgentID);
    TestTrue(TEXT("Serial holding energy increases the following aircraft recharge time"),
        SerialQueue-ChargedQueue>FollowingAlone);
    const TArray<FAgricultureSupplySlot> Reserved={{99,100,200},{100,240,300}};
    TestEqual(TEXT("Supply slots preserve usable gaps before future returns"),
        C->FindSupplySlotWait(50,40,Reserved,Agent.AgentID),0.0);
    TestEqual(TEXT("Conflicting approach waits until the promised return clears"),
        C->FindSupplySlotWait(80,30,Reserved,Agent.AgentID),120.0);
    TestEqual(TEXT("Service window shifts across chained reservations"),
        C->FindSupplySlotWait(80,60,Reserved,Agent.AgentID),220.0);
    TestEqual(TEXT("Aircraft does not wait behind its own return promise"),
        C->FindSupplySlotWait(100,20,Reserved,99),0.0);
    C->Agents.Empty();C->Sections.Empty();
    const double LowReturn=C->SupplyTravelTime(FVector(3000,0,400),FVector::ZeroVector,3000,150);
    const double HighReturn=C->SupplyTravelTime(FVector(3000,0,1500),FVector::ZeroVector,3000,150);
    TestTrue(TEXT("Return budget includes climb from spray height"),LowReturn>HighReturn);
    TestTrue(TEXT("Return budget includes braking before climb"),
        C->SupplyTravelTime(FVector(3000,0,400),FVector(300,0,0),3000,150)>LowReturn);
    const double FrozenEnergy=UAgricultureCoordinator::SegmentEnergyFraction(Agent.EmptyMassKg,1,Service.LiquidLitres,Service.LiquidLitres,
        Forecast.FlightSeconds+Forecast.ReturnSeconds,600);
    TestTrue(TEXT("Forecast accounts for spray-induced mass reduction"),Forecast.EnergyFraction<FrozenEnergy);
    auto PredictedProgress=Near;
    C->AdvanceForecastProgress(PredictedProgress,Forecast.AppliedLitres);
    TestTrue(TEXT("Forecast progress consumes only future coverage"),PredictedProgress.AppliedLitres>Near.AppliedLitres && PredictedProgress.NextPoint>=Near.NextPoint);
    TestTrue(TEXT("Forecast does not mutate actual coverage"),Near.CoveredSquareMetres==0 && Near.NextPoint==1);
    Near.CoveredSquareMetres=6;
    const auto Preserved=C->ChooseWorkOrder(Agent,FVector(3000,1200,400),Near);
    double PreservedDemand=0;C->RemainingWorkSeconds(Preserved,PreservedDemand);
    TestTrue(TEXT("Remaining work reorder preserves covered area and unsprayed demand"),
        Preserved.CoveredSquareMetres==Near.CoveredSquareMetres && FMath::IsNearlyEqual(PreservedDemand,AfterLiquid,1.e-5));
    auto Partial=Near;Partial.Config.Boundary={FVector(0,0,0),FVector(10000,0,0),FVector(10000,600,0),FVector(0,600,0)};
    Partial.StripPoints={FVector(0,300,400),FVector(10000,300,400)};
    Partial.NextPoint=1;Partial.StripCoveredCm=4000;Partial.CoveredSquareMetres=240;Partial.AppliedLitres=3.6;
    auto OrderAgent=Agent;OrderAgent.Battery=1;OrderAgent.LiquidLitres=6.4f;OrderAgent.PayloadLimitKg=50;
    const FVector SavedDock=C->Airports[0].Config.DockPosition;
    C->Airports[0].Config.DockPosition=FVector(10000,300,150);
    auto Reversed=C->ChooseWorkOrder(OrderAgent,C->Airports[0].Config.DockPosition,Partial);
    C->Airports[0].Config.DockPosition=SavedDock;
    TestTrue(TEXT("Partial strip can start at its nearer unsprayed end"),
        Reversed.StripPoints[0].Equals(Partial.StripPoints[1],.01) && Reversed.StripPoints[1].Equals(FVector(4000,300,400),.01));
    float RemainingMixture=6.4f;
    UAgricultureCoordinator::RecordSpraySegment(Reversed,RemainingMixture,Reversed.StripPoints[0],Reversed.StripPoints[1],100);
    TestTrue(TEXT("Reversed remainder completes prescribed coverage without reapplying the prefix"),
        FMath::IsNearlyEqual(Reversed.CoveredSquareMetres,600.0,1.e-3) && FMath::IsNearlyEqual(Reversed.AppliedLitres,9.0,1.e-3));
    Agent.PayloadLimitKg=2;
    TestTrue(TEXT("Refill respects rated payload"),C->PlanService(Agent,Near,Station).LiquidLitres<=2);
    FSupplyAirportState Remote=Station;Remote.Config.DockPosition=FVector(1000000,0,0);
    TestTrue(TEXT("Impossible sortie cannot request departure load"),C->PlanService(Agent,Near,Remote).LiquidLitres<0);
    FAgricultureAgentState Stranded=Agent;Stranded.PayloadLimitKg=50;Stranded.LiquidLitres=6.4f;Stranded.Battery=1;
    Remote.Config.DockPosition=FVector(-30000,0,0);
    auto Heavy=Stranded;Heavy.LiquidLitres=20;
    auto UnloadDepot=Remote;UnloadDepot.Config.DockPosition=FVector(-20000,0,0);
    TestFalse(TEXT("Existing load cannot fly a remote work sortie"),C->ForecastSortie(Heavy,UnloadDepot.Config.DockPosition,Near,UnloadDepot,20,1,false).bFeasible);
    const auto Reduced=C->PlanService(Heavy,Near,UnloadDepot);
    TestTrue(TEXT("Controlled unloading restores a feasible remote sortie"),Reduced.LiquidLitres>=1 && Reduced.LiquidLitres<20);
    auto FullWaste=UnloadDepot;FullWaste.Config.WasteCapacityLitres=FullWaste.WasteLitres;
    TestTrue(TEXT("Full waste tank prevents unloading fallback"),C->PlanService(Heavy,Near,FullWaste).LiquidLitres<0);
    auto ReservedDrain=UnloadDepot;C->ReserveSupplyStock(ReservedDrain,Heavy,Near,Reduced.LiquidLitres);
    TestTrue(TEXT("Unloading reserves the recovered mixture volume"),FMath::IsNearlyEqual(ReservedDrain.WasteLitres-UnloadDepot.WasteLitres,20-Reduced.LiquidLitres,.001f));
    Remote.Config.WasteCapacityLitres=Remote.WasteLitres;
    TestTrue(TEXT("Closer depot can resume the same checkpoint"),C->PlanService(Stranded,Near,Station).LiquidLitres>=0);
    Remote.Config.AirportID=2;
    const auto Relocate=C->PlanDockService(Stranded,Near,Remote);
    TestTrue(TEXT("Dock service preserves checkpoint and selects a reachable work depot"),
        Relocate.LiquidLitres==-2 && Relocate.AirportID==Station.Config.AirportID && Relocate.BatteryFraction<=1);
    TestEqual(TEXT("Reposition planning does not fill at remote depot"),Stranded.LiquidLitres,6.4f);
    FAgricultureAgentState BusyOccupant;BusyOccupant.AgentID=99;BusyOccupant.PlotID=0;BusyOccupant.SectionID=0;
    BusyOccupant.Phase=EAgriculturePhase::Servicing;BusyOccupant.Recipe=Near.Config.Recipe;
    BusyOccupant.Battery=.1f;BusyOccupant.ServiceLiquidTarget=1;BusyOccupant.ServiceBatteryTarget=1;
    C->Agents.Add(BusyOccupant);C->Sections.Add(Near);
    const float OriginalChargeSeconds=C->Airports[0].Config.ChargeSeconds;
    C->Airports[0].Config.ChargeSeconds=600;C->Airports[0].OccupantID=99;
    const auto UpdatedRelocate=C->PlanDockService(Stranded,Near,Remote);
    TestTrue(TEXT("Transfer preflight refresh keeps newly increased destination wait on the ground"),
        UpdatedRelocate.LiquidLitres==-2 && UpdatedRelocate.DepartureDelaySeconds>Relocate.DepartureDelaySeconds+100);
    C->Airports[0].Config.ChargeSeconds=OriginalChargeSeconds;C->Airports[0].OccupantID=INDEX_NONE;
    C->Agents.Empty();C->Sections.Empty();
    auto QueuedTransfer=Stranded;QueuedTransfer.AgentID=101;QueuedTransfer.Battery=.5f;QueuedTransfer.SectionID=0;
    C->Agents.Add(QueuedTransfer);C->Sections.Add(Near);Remote.Queue={QueuedTransfer.AgentID};
    auto ArrivingTransfer=QueuedTransfer;
    ArrivingTransfer.Battery=FMath::Max(0.f,QueuedTransfer.Battery-C->Config.AirportApproachSeconds/C->FlightSeconds(QueuedTransfer));
    const auto TransferPlan=C->PlanDockService(ArrivingTransfer,Near,Remote,false);
    TestTrue(TEXT("Queued aircraft can prepare a transfer when local work is infeasible"),
        TransferPlan.LiquidLitres==-2 && TransferPlan.BatteryFraction<1);
    const double ExitSeconds=C->Config.CruiseTimeFactor*(FMath::Max(0.0,double(C->Config.TransitHeightCm)-Remote.Config.DockPosition.Z)/200+1000.0/150)+2*C->Config.RouteArrivalAllowanceSeconds;
    const double ExpectedQueue=C->Config.AirportApproachSeconds+
        double(TransferPlan.BatteryFraction-ArrivingTransfer.Battery)*Remote.Config.ChargeSeconds+ExitSeconds;
    TestTrue(TEXT("Queue service uses the same transfer charge target as dock planning"),
        FMath::IsNearlyEqual(double(C->EstimateWaitSeconds(Remote,0,Agent.AgentID)),ExpectedQueue,.01));
    Remote.Queue.Empty();C->Agents.Empty();C->Sections.Empty();
    C->SupplySlots.SetNum(1);C->SupplySlots[0].Add({99,0,1000});
    const auto ScheduledRelocate=C->PlanDockService(Stranded,Near,Remote);
    TestTrue(TEXT("Ground transfer respects another sortie return window"),
        ScheduledRelocate.AirportID==Station.Config.AirportID && ScheduledRelocate.DepartureDelaySeconds>Relocate.DepartureDelaySeconds);
    TestEqual(TEXT("Ground holding adds time without demanding hover charge"),
        ScheduledRelocate.BatteryFraction,Relocate.BatteryFraction);
    C->SupplySlots[0][0].WaterLitres=10;C->SupplySlots[0][0].ConcentrateLitres=1;C->SupplySlots[0][0].WasteLitres=2;
    const auto RemainingStock=C->AvailableSupplyStock(Station,Agent.AgentID);
    TestEqual(TEXT("Existing promises reserve water for other aircraft"),RemainingStock.Config.WaterLitres,Station.Config.WaterLitres-10);
    TestEqual(TEXT("Existing promises reserve concentrate for other aircraft"),RemainingStock.Config.ConcentrateLitres,Station.Config.ConcentrateLitres-1);
    TestEqual(TEXT("Existing promises reserve cleaning waste capacity"),RemainingStock.WasteLitres,Station.WasteLitres+2);
    TestEqual(TEXT("Aircraft can use its own promised supply stock"),C->AvailableSupplyStock(Station,99).Config.WaterLitres,Station.Config.WaterLitres);
    C->SupplySlots[0].Add({99,100,1000,2,.2,1});
    const auto BeforeConsumption=C->AvailableSupplyStock(Station,Agent.AgentID);
    auto ActualStock=Station;ActualStock.Config.WaterLitres-=3;ActualStock.Config.ConcentrateLitres-=.3f;ActualStock.WasteLitres+=.5f;
    C->ConsumeSupplyReservation(Station.Config.AirportID,99,3,.3,.5);
    const auto AfterConsumption=C->AvailableSupplyStock(ActualStock,Agent.AgentID);
    TestEqual(TEXT("Actual transfer and remaining reservation count consumed water once"),
        BeforeConsumption.Config.WaterLitres,AfterConsumption.Config.WaterLitres);
    TestTrue(TEXT("Actual service preserves unrelated future reservations"),C->SupplySlots[0][1].WaterLitres==2);
    auto ReservedStock=Station;auto Filling=Agent;Filling.Recipe=Near.Config.Recipe;Filling.LiquidLitres=1;
    C->ReserveSupplyStock(ReservedStock,Filling,Near,3);
    TestTrue(TEXT("Virtual service reserves only additional mixture and leaves actual stock untouched"),
        FMath::IsNearlyEqual(ReservedStock.Config.WaterLitres,Station.Config.WaterLitres-1.96f,.001f) &&
        FMath::IsNearlyEqual(ReservedStock.Config.ConcentrateLitres,Station.Config.ConcentrateLitres-.04f,.001f) &&
        Station.Config.WaterLitres==2000);
    C->SupplySlots.Empty();
    TestTrue(TEXT("Supply repositioning keeps remaining task schedulable"),
        C->EstimateSectionCost(Stranded,Remote.Config.DockPosition,Near)<DBL_MAX);
    auto ChargingSource=Remote;ChargingSource.OccupantID=Stranded.AgentID;
    auto LowCharge=Stranded;LowCharge.Battery=.2f;LowCharge.AirportID=ChargingSource.Config.AirportID;LowCharge.Phase=EAgriculturePhase::Servicing;
    C->Airports.Add(ChargingSource);
    TestTrue(TEXT("Dock charging before a supply transfer keeps a low-charge task schedulable"),
        C->EstimateSectionCost(LowCharge,ChargingSource.Config.DockPosition,Near)<DBL_MAX);
    TestEqual(TEXT("Cost prediction does not charge the real aircraft"),LowCharge.Battery,.2f);
    C->Airports.Pop();
    auto WashSource=Remote;WashSource.Config.WasteCapacityLitres=500;WashSource.OccupantID=Stranded.AgentID;
    auto Mismatched=Stranded;Mismatched.Recipe=TEXT("Different");Mismatched.LiquidLitres=8;Mismatched.Battery=.8f;
    Mismatched.AirportID=WashSource.Config.AirportID;Mismatched.Phase=EAgriculturePhase::Servicing;
    C->Airports[0].Config.WasteCapacityLitres=12;
    C->Airports.Add(WashSource);
    TestTrue(TEXT("Small destination waste tank cannot clean the incoming residual mixture"),C->PlanService(Mismatched,Near,C->Airports[0],false).LiquidLitres<0);
    const auto WashTransfer=C->PlanDockService(Mismatched,Near,WashSource,false);
    TestTrue(TEXT("Source cleaning enables a lighter transfer to the nearby work depot"),
        WashTransfer.LiquidLitres==-2 && WashTransfer.bCleanBeforeTransfer && WashTransfer.AirportID==Station.Config.AirportID);
    TestTrue(TEXT("Source cleaning and transfer preparation are budgeted together"),WashTransfer.ServiceSeconds>=WashSource.Config.CleanSeconds);
    TestEqual(TEXT("Planning does not discharge actual residual mixture"),Mismatched.LiquidLitres,8.f);
    TestEqual(TEXT("Planning does not consume actual source waste capacity"),WashSource.WasteLitres,0.f);
    auto NoWash=WashSource;NoWash.Config.WasteCapacityLitres=12;
    TestTrue(TEXT("Transfer cleaning is rejected if neither station can recover the mixture"),C->PlanDockService(Mismatched,Near,NoWash,false).LiquidLitres<0 && !C->PlanDockService(Mismatched,Near,NoWash,false).bCleanBeforeTransfer);
    Mismatched.Battery=.2f;
    TestTrue(TEXT("Task cost includes source cleaning before a low-charge transfer"),C->EstimateSectionCost(Mismatched,WashSource.Config.DockPosition,Near)<DBL_MAX);
    C->Airports.Pop();C->Airports[0].Config.WasteCapacityLitres=Station.Config.WasteCapacityLitres;
    auto SameRecipe=Stranded;auto WasteFull=Station;WasteFull.WasteLitres=WasteFull.Config.WasteCapacityLitres;
    TestTrue(TEXT("Full waste tank permits compatible refill and charge"),C->PlanService(SameRecipe,Near,WasteFull).LiquidLitres>=SameRecipe.LiquidLitres);
    SameRecipe.Recipe=TEXT("Different");
    TestTrue(TEXT("Full waste tank rejects a recipe switch requiring cleaning"),C->PlanService(SameRecipe,Near,WasteFull).LiquidLitres<0);
    Agent.PayloadLimitKg=50;Agent.LiquidLitres=12;
    const float LightEndurance=C->FlightSeconds(Agent);Agent.LiquidLitres=50;
    TestTrue(TEXT("Loaded return budget is reduced"),C->FlightSeconds(Agent)<LightEndurance);
    C->Airports[0].Config.bEnabled=false;
    TestEqual(TEXT("No healthy supply airport is not schedulable"),C->EstimateSectionCost(Agent,FVector::ZeroVector,Near),DBL_MAX);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureAirportTest,"UAVSimulator.MultiAgent.Agriculture.NearestReachable",UAV_TEST_FLAGS)
bool FAgricultureAirportTest::RunTest(const FString&)
{
    TArray<FSupplyAirportState> Stations;Stations.SetNum(3);
    for(int32 I=0;I<3;++I) Stations[I].Config.AirportID=I;
    TestEqual(TEXT("Nearest station selected"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},300,100,30),1);
    Stations[1].Config.bEnabled=false;
    TestEqual(TEXT("Disabled station skipped"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},300,100,30),2);
    Stations[2].Config.WaterLitres=0;
    Stations[2].Config.ConcentrateLitres=0;Stations[2].WasteLitres=Stations[2].Config.WasteCapacityLitres;
    TestEqual(TEXT("Liquid stocks and full waste storage do not prevent safe landing or charging"),
        UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},300,100,30),2);
    Stations[2].Config.bEnabled=false;
    TestEqual(TEXT("Reserve must remain available"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},40,100,30),INDEX_NONE);
    Stations[0].OccupantID=1;
    TestEqual(TEXT("Waiting energy accounted"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},200,100,30),INDEX_NONE);
    Stations[0].OccupantID=INDEX_NONE;
    TestEqual(TEXT("Actual travel budget includes approach before reserve"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},150,100,30,{130,0,0},{0,0,0}),INDEX_NONE);
    TestEqual(TEXT("Conservative service forecast can release occupied berth before arrival"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},300,100,30,{130,0,0},{0,0,0}),0);
    Stations[2].Config.WaterLitres=10;
    Stations[2].Config.bEnabled=true;
    TestEqual(TEXT("Equal distances ordered by ID"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{2000,1000,2000},300,100,30),0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureResetTest,"UAVSimulator.MultiAgent.Agriculture.Reset",UAV_TEST_FLAGS)
bool FAgricultureResetTest::RunTest(const FString&)
{
    UAgricultureCoordinator* C=NewObject<UAgricultureCoordinator>();
    C->Reset();C->Reset();C->Update(0);C->Command(0,99);
    TestEqual(TEXT("No old plots"),C->PlotCount(),0);
    TestEqual(TEXT("No old agents"),C->GetAgents().Num(),0);
    TestEqual(TEXT("No old reservations"),C->GetAirports().Num(),0);
    TestFalse(TEXT("Reset disabled"),C->IsEnabled());
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureResourceTest,"UAVSimulator.MultiAgent.Agriculture.ResourceConservation",UAV_TEST_FLAGS)
bool FAgricultureResourceTest::RunTest(const FString&)
{
    FAgricultureAgentState A;FSupplyAirportState S;
    S.Config.WaterLitres=98;S.Config.ConcentrateLitres=2;
    TestEqual(TEXT("Actual transfer"),UAgricultureCoordinator::TransferLiquid(A,S,10,50,.02f),10.0f);
    TestTrue(TEXT("Water conserved"),FMath::IsNearlyEqual(S.Config.WaterLitres,88.2f));
    TestTrue(TEXT("Concentrate conserved"),FMath::IsNearlyEqual(S.Config.ConcentrateLitres,1.8f));
    UAgricultureCoordinator::TransferLiquid(A,S,100,50,.02f);
    TestEqual(TEXT("Tank cannot overflow"),A.LiquidLitres,50.0f);
    S.Config.ConcentrateLitres=0;A.LiquidLitres=0;
    TestEqual(TEXT("No original chemical cannot create mixture"),UAgricultureCoordinator::TransferLiquid(A,S,10,50,.02f),0.0f);
    S.Config.ConcentrateLitres=10;
    TestEqual(TEXT("Pause has zero transfer"),UAgricultureCoordinator::TransferLiquid(A,S,0,50,.02f),0.0f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureProgressTest,"UAVSimulator.MultiAgent.Agriculture.ExecutionProgress",UAV_TEST_FLAGS)
bool FAgricultureProgressTest::RunTest(const FString&)
{
    UTaskMonitor* M=NewObject<UTaskMonitor>();
    M->ReportExecution(1,0,ETaskStatus::InProgress,.4f);
    TestEqual(TEXT("Partial coverage incomplete"),M->GetCompletedTaskCount(),0);
    M->ReportExecution(1,2,ETaskStatus::InProgress,.6f);
    TestEqual(TEXT("Fault handover progress retained"),M->GetTaskProgress(1),.6f);
    M->ReportExecution(1,2,ETaskStatus::Completed,1);
    M->ReportExecution(1,0,ETaskStatus::Assigned,0);
    TestEqual(TEXT("Completed task cannot run twice"),M->GetCompletedTaskCount(),1);
    TestEqual(TEXT("Terminal progress retained"),M->GetTaskStatus(1),ETaskStatus::Completed);
    M->Reset();TestEqual(TEXT("Reset clears external progress"),M->GetTotalTaskCount(),0);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCollectiveAllocationTest,"UAVSimulator.Control.CollectiveAllocation",UAV_TEST_FLAGS)
bool FCollectiveAllocationTest::RunTest(const FString&)
{
    for(float C:{0.0f,.1f,.294f,.7f,1.0f})
    {
        const FMotorOutput M=UAttitudeController::AllocateCollective(C,{-.4f,0,.4f,0});
        float Sum=0;for(float T:M.Thrusts) {Sum+=T;TestTrue(TEXT("Motor limits"),T>=0 && T<=1);}
        TestTrue(TEXT("Saturation preserves collective thrust"),FMath::IsNearlyEqual(Sum/4,C));
    }
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureContinuousCoverageTest,"UAVSimulator.MultiAgent.Agriculture.ContinuousCoverage",UAV_TEST_FLAGS)
bool FAgricultureContinuousCoverageTest::RunTest(const FString&)
{
    FAgriculturePlotState P;P.Config.Boundary={FVector(0,0,0),FVector(6000,0,0),FVector(6000,6000,0),FVector(0,6000,0)};
    UAgricultureCoordinator::BuildStrips(P.Config,400,P.StripPoints);P.NextPoint=1;
    const FVector Start=P.StripPoints[0];float Liquid=10;
    TestTrue(TEXT("Lead-in before plot does not abort transit"),UAgricultureCoordinator::RecordSpraySegment(P,Liquid,Start+FVector(-150,0,150),Start+FVector(-120,0,140),100));
    TestTrue(TEXT("Lead-in cannot apply outside plot"),FMath::IsNearlyZero(P.CoveredSquareMetres) && FMath::IsNearlyEqual(Liquid,10.0f));
    TestTrue(TEXT("Actual continuous movement sprays"),UAgricultureCoordinator::RecordSpraySegment(P,Liquid,Start,Start+FVector(1000,0,0),100));
    TestTrue(TEXT("Application follows covered area"),FMath::IsNearlyEqual(P.StripCoveredCm,1000.0f) && FMath::IsNearlyEqual(Liquid,9.1f));
    TestFalse(TEXT("Off-strip movement cannot spray"),UAgricultureCoordinator::RecordSpraySegment(P,Liquid,Start+FVector(1000,200,0),Start+FVector(5000,200,0),100));
    TestFalse(TEXT("Re-entry cannot retrospectively cover a gap"),UAgricultureCoordinator::RecordSpraySegment(P,Liquid,Start+FVector(5000,0,0),Start+FVector(5500,0,0),100));
    TestTrue(TEXT("Coverage gap preserved"),FMath::IsNearlyEqual(P.StripCoveredCm,1000.0f));
    Liquid=.2f;UAgricultureCoordinator::RecordSpraySegment(P,Liquid,Start+FVector(1000,0,0),Start+FVector(2000,0,0),100);
    TestTrue(TEXT("Finite liquid limits actual coverage"),FMath::IsNearlyEqual(P.StripCoveredCm,1222.2222f,.01f) && FMath::IsNearlyZero(Liquid));
    P.StripCoveredCm=6000;
    TestTrue(TEXT("Completed strip can brake beyond boundary without re-entry"),UAgricultureCoordinator::RecordSpraySegment(P,Liquid,Start+FVector(6050,0,0),Start+FVector(6150,0,0),100));
    TestTrue(TEXT("No application outside boundary"),FMath::IsNearlyZero(Liquid));
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureBerthQueueTest,"UAVSimulator.MultiAgent.Agriculture.BerthQueue",UAV_TEST_FLAGS)
bool FAgricultureBerthQueueTest::RunTest(const FString&)
{
    auto* C=NewObject<UAgricultureCoordinator>();C->Config.BatteryReserveFraction=.15f;
    for(int32 I=0;I<3;++I)
    {FAgricultureAgentState A;A.AgentID=I;A.Battery=I==2 ? .2f : .8f;A.RequestTime=2-I;C->Agents.Add(A);}
    FSupplyAirportState S;S.Queue={0,1,2};C->UpdateAirport(S,.02f);
    TestEqual(TEXT("Low battery precedes earlier requests"),S.OccupantID,2);
    TestEqual(TEXT("Remaining requests retain FIFO order"),S.Queue[0],1);
    C->UpdateAirport(S,.02f);TestEqual(TEXT("Occupied berth cannot be assigned twice"),S.OccupantID,2);
    TestEqual(TEXT("Occupant removed from reservation queue"),S.Queue.Num(),2);
    auto Empty=S;Empty.Queue.Empty();S.Queue={1};
    TestEqual(TEXT("Waiting aircraft does not count its own future service"),C->EstimateWaitSeconds(S,30,1),C->EstimateWaitSeconds(Empty,30,INDEX_NONE));
    C->Agents[2].Phase=EAgriculturePhase::Failed;
    TestFalse(TEXT("Failed physical occupant blocks another reservation"),C->CanService(S));
    C->Agents[2].Phase=EAgriculturePhase::Completed;
    TestFalse(TEXT("Completed parked occupant retains berth exclusion"),C->CanService(S));
    C->Agents[2].Phase=EAgriculturePhase::Servicing;
    TestFalse(TEXT("Final parking service cannot promise future berth release"),C->CanService(S));
    C->Agents[2].PlotID=0;
    TestTrue(TEXT("Active service permits a future queued reservation"),C->CanService(S));
    auto FullWaste=S;FullWaste.WasteLitres=FullWaste.Config.WasteCapacityLitres;
    TestTrue(TEXT("Waste storage capacity does not disable charging or compatible service"),C->CanService(FullWaste));
    FullWaste.Config.WaterLitres=0;FullWaste.Config.ConcentrateLitres=0;
    TestTrue(TEXT("Empty liquid stocks do not disable charge-only service"),C->CanService(FullWaste));
    FullWaste.Config.bEnabled=false;
    TestFalse(TEXT("Disabled airport still rejects every service"),C->CanService(FullWaste));
    C->Agents[2].Battery=.999f;C->Agents[2].LiquidLitres=50;C->Agents[2].PhaseSeconds=200;C->Agents[2].bCleaned=true;
    S.Queue.Empty();
    TestEqual(TEXT("Aircraft already owning a berth does not wait behind its own service"),C->EstimateWaitSeconds(S,0,2),0.f);
    const float BeforeDeparture=C->EstimateWaitSeconds(S,0,INDEX_NONE);
    C->Agents[2].Phase=EAgriculturePhase::Resuming;
    TestTrue(TEXT("Service completion does not add approach time to berth wait"),C->EstimateWaitSeconds(S,0,INDEX_NONE)<=BeforeDeparture);
    TestTrue(TEXT("Departure wait contains only takeoff and protection exit"),C->EstimateWaitSeconds(S,0,INDEX_NONE)<50);
    TestTrue(TEXT("Queued approach starts after the berth is released"),C->EstimateWaitSeconds(S,10,INDEX_NONE)+10+C->Config.AirportApproachSeconds>=C->EstimateWaitSeconds(S,0,INDEX_NONE)+C->Config.AirportApproachSeconds-.01f);
    C->Agents[2].Phase=EAgriculturePhase::TakingOff;C->Agents[2].ServiceLiquidTarget=-2;S.Queue={1};
    C->UpdateAirport(S,.02f);
    TestEqual(TEXT("Transfer departure retains the source berth while clearing its protection area"),S.OccupantID,2);
    TestEqual(TEXT("Inbound aircraft remains queued during transfer departure"),S.Queue.Num(),1);
    C->Airports.Add(S);C->Reset();TestTrue(TEXT("Reset releases all reservations"),C->Airports.IsEmpty());
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureRecipeCleaningTest,"UAVSimulator.MultiAgent.Agriculture.RecipeCleaning",UAV_TEST_FLAGS)
bool FAgricultureRecipeCleaningTest::RunTest(const FString&)
{
    FAgricultureAgentState A;A.LiquidLitres=8;A.Recipe=TEXT("CropA");
    FSupplyAirportState S;S.Config.WaterLitres=4;
    TestFalse(TEXT("Insufficient water cannot switch recipe"),UAgricultureCoordinator::CleanResidue(A,S,TEXT("CropB")));
    TestEqual(TEXT("Rejected cleaning preserves residual mixture"),A.LiquidLitres,8.0f);
    TestTrue(TEXT("Rejected cleaning preserves recipe"),A.Recipe==TEXT("CropA"));
    S.Config.WaterLitres=20;S.Config.WasteCapacityLitres=10;
    TestFalse(TEXT("Waste capacity constrains cleaning"),UAgricultureCoordinator::CleanResidue(A,S,TEXT("CropB")));
    S.Config.WasteCapacityLitres=30;
    TestTrue(TEXT("Cleaning permits recipe change"),UAgricultureCoordinator::CleanResidue(A,S,TEXT("CropB")));
    TestEqual(TEXT("Cleaning water consumed"),S.Config.WaterLitres,15.0f);
    TestEqual(TEXT("All residual liquid and wash water recovered"),S.WasteLitres,13.0f);
    UAgricultureCoordinator::CleanResidue(A,S,TEXT("CropB"));
    TestEqual(TEXT("Completed cleaning cannot exchange resources twice"),S.WasteLitres,13.0f);
    A.LiquidLitres=8;S.WasteLitres=13;S.Config.WasteCapacityLitres=18;
    TestEqual(TEXT("Unloading obeys pump throughput"),UAgricultureCoordinator::DrainLiquid(A,S,2,3),2.f);
    TestEqual(TEXT("Unloading stops at the requested payload"),UAgricultureCoordinator::DrainLiquid(A,S,20,3),3.f);
    TestEqual(TEXT("Aircraft plus recovered waste conserves liquid"),A.LiquidLitres+S.WasteLitres,21.f);
    TestEqual(TEXT("Full waste tank rejects further unloading"),UAgricultureCoordinator::DrainLiquid(A,S,20,1),0.f);
    TestEqual(TEXT("Unloading does not recover fresh water"),S.Config.WaterLitres,15.f);
    TestTrue(TEXT("Unloading preserves the mixture recipe"),A.Recipe==TEXT("CropB"));
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureEmergencySiteTest,"UAVSimulator.MultiAgent.Agriculture.EmergencyLandingSite",UAV_TEST_FLAGS)
bool FAgricultureEmergencySiteTest::RunTest(const FString&)
{
    FAgriculturePlotState Field;Field.Config.Boundary={FVector(0,0,0),FVector(1000,0,0),FVector(1000,1000,0),FVector(0,1000,0)};
    FAgriculturePlotState Neighbour=Field;
    for(FVector& V:Neighbour.Config.Boundary) V.X+=1600;
    const TArray<FAgriculturePlotState> Fields={Field,Neighbour};
    const FVector Site=UAgricultureCoordinator::FindEmergencyLandingSite(FVector(950,500,400),Fields,600);
    for(const auto& P:Fields)
        TestTrue(TEXT("Parked emergency aircraft leaves every work area reachable"),FBox(P.Config.Boundary).ComputeSquaredDistanceToPoint(FVector(Site.X,Site.Y,0))>=360000-.1);
    TestEqual(TEXT("Site planning does not lower aircraft before safe lateral transit"),Site.Z,400.0);
    const FVector Outside(500,-700,400);
    TestEqual(TEXT("Already clear location needs no lateral diversion"),UAgricultureCoordinator::FindEmergencyLandingSite(Outside,Fields,600),Outside);
    TestEqual(TEXT("No fields retain actual location"),UAgricultureCoordinator::FindEmergencyLandingSite(Outside,{},600),Outside);
    FObstacleInfo Building;Building.Center=FVector(-15500,-9600,445);Building.Extents=FVector(250,250,425);Building.SafetyMargin=200;
    const FVector Blocked(-14943.978,-9548.711,1500);
    const FVector Safe=UAgricultureCoordinator::FindEmergencyLandingSite(Blocked,{},950,{Building},150);
    auto* Planner=NewObject<UAStarPathPlanner>();Planner->SetObstacles({Building});
    TestFalse(TEXT("Emergency descent clears building safety envelope"),Planner->CheckLineCollision(Safe,FVector(Safe.X,Safe.Y,120),150));
    TestTrue(TEXT("Unsafe descent requires a lateral diversion"),FVector::Dist2D(Blocked,Safe)>30);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgriculturePayloadEnergyTest,"UAVSimulator.MultiAgent.Agriculture.PayloadEnergy",UAV_TEST_FLAGS)
bool FAgriculturePayloadEnergyTest::RunTest(const FString&)
{
    TestEqual(TEXT("Empty mass defines reference endurance"),UAgricultureCoordinator::PayloadPowerRatio(22,0,1),1.f);
    TestTrue(TEXT("Double mass requires 2sqrt2 induced power"),FMath::IsNearlyEqual(UAgricultureCoordinator::PayloadPowerRatio(22,22,1),FMath::Pow(2.f,1.5f)));
    TestTrue(TEXT("More liquid reduces endurance"),UAgricultureCoordinator::PayloadPowerRatio(22,50,1)>UAgricultureCoordinator::PayloadPowerRatio(22,12,1));
    TestEqual(TEXT("Density converts volume to payload mass"),UAgricultureCoordinator::PayloadPowerRatio(22,10,2),UAgricultureCoordinator::PayloadPowerRatio(22,20,1));
    TestEqual(TEXT("No negative payload energy"),UAgricultureCoordinator::PayloadPowerRatio(22,-10,1),1.f);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureEnergyIntegralTest,"UAVSimulator.MultiAgent.Agriculture.EnergyIntegral",UAV_TEST_FLAGS)
bool FAgricultureEnergyIntegralTest::RunTest(const FString&)
{
    const double Exact=UAgricultureCoordinator::SegmentEnergyFraction(22,1,20,0,100,600);
    double Numerical=0;
    for(int32 I=0;I<10000;++I)
    {
        const double Liquid=20*(1-(I+.5)/10000);
        Numerical+=FMath::Pow(1+Liquid/22,1.5)*100/10000/600;
    }
    TestTrue(TEXT("Analytic energy agrees with independent quadrature"),FMath::Abs(Exact-Numerical)<1.e-7);
    const double Split=UAgricultureCoordinator::SegmentEnergyFraction(22,1,20,10,50,600)+
        UAgricultureCoordinator::SegmentEnergyFraction(22,1,10,0,50,600);
    TestTrue(TEXT("Energy does not depend on segment subdivision"),FMath::Abs(Exact-Split)<1.e-10);
    const double HeavyTransfer=UAgricultureCoordinator::SegmentEnergyFraction(22,1,20,20,100,600);
    const double LightTransfer=UAgricultureCoordinator::SegmentEnergyFraction(22,1,0,0,100,600);
    TestTrue(TEXT("Applying liquid before equal-duration transfer saves energy"),Exact+LightTransfer<HeavyTransfer+Exact);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureMakespanTest,"UAVSimulator.MultiAgent.Agriculture.MakespanMatching",UAV_TEST_FLAGS)
bool FAgricultureMakespanTest::RunTest(const FString&)
{
    const TArray<TArray<double>> Costs={{10,22},{22,30}};
    const auto Total=UTaskAllocator::MatchMinimumCost(Costs);
    const auto Finish=UTaskAllocator::MatchMinimumMakespan(Costs);
    TestTrue(TEXT("Total-cost optimum leaves a late aircraft"),Total[0]==0 && Total[1]==1);
    TestTrue(TEXT("Fleet completion optimum balances finish times"),Finish[0]==1 && Finish[1]==0);
    const auto Partial=UTaskAllocator::MatchMinimumMakespan({{DBL_MAX,12},{DBL_MAX,8},{3,DBL_MAX}});
    TestTrue(TEXT("Partial matching retains maximum count and smallest latest finish"),Partial[0]==INDEX_NONE && Partial[1]==1 && Partial[2]==0);
    const auto Fallback=UTaskAllocator::MatchMinimumMakespan({{DBL_MAX,100,250},{DBL_MAX,120,240}});
    TestTrue(TEXT("Infeasible highest priority retains lower priority fleet work"),Fallback[0]==1 && Fallback[1]==2);
    TestTrue(TEXT("Empty fleet is accepted"),UTaskAllocator::MatchMinimumMakespan({}).IsEmpty());
    return true;
}
#endif
