#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../MultiAgent/AgricultureCoordinator.h"
#include "../../MultiAgent/TaskMonitor.h"
#include "../../Control/AttitudeController.h"
#include "../../Planning/AStarPathPlanner.h"

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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAgricultureAirportTest,"UAVSimulator.MultiAgent.Agriculture.NearestReachable",UAV_TEST_FLAGS)
bool FAgricultureAirportTest::RunTest(const FString&)
{
    TArray<FSupplyAirportState> Stations;Stations.SetNum(3);
    for(int32 I=0;I<3;++I) Stations[I].Config.AirportID=I;
    TestEqual(TEXT("Nearest station selected"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},300,100,30),1);
    Stations[1].Config.bEnabled=false;
    TestEqual(TEXT("Disabled station skipped"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},300,100,30),2);
    Stations[2].Config.WaterLitres=0;
    TestEqual(TEXT("Dry station skipped"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},300,100,30),0);
    TestEqual(TEXT("Reserve must remain available"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},40,100,30),INDEX_NONE);
    Stations[0].OccupantID=1;
    TestEqual(TEXT("Waiting energy accounted"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},200,100,30),INDEX_NONE);
    Stations[0].OccupantID=INDEX_NONE;
    TestEqual(TEXT("Actual travel budget includes approach before reserve"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},150,100,30,{130,0,0},{0,0,0}),INDEX_NONE);
    TestEqual(TEXT("Conservative service forecast can release occupied berth before arrival"),UAgricultureCoordinator::ChooseNearestAirport(Stations,{3000,1000,2000},300,100,30,{130,0,0},{0,0,0}),0);
    Stations[2].Config.WaterLitres=10;
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
    TestEqual(TEXT("Waiting aircraft does not count its own future service"),C->EstimateWaitSeconds(S,30,1),C->EstimateWaitSeconds(Empty,30));
    C->Agents[2].Phase=EAgriculturePhase::Failed;
    TestFalse(TEXT("Failed physical occupant blocks another reservation"),C->CanService(S));
    C->Agents[2].Phase=EAgriculturePhase::Completed;
    TestFalse(TEXT("Completed parked occupant retains berth exclusion"),C->CanService(S));
    C->Agents[2].Phase=EAgriculturePhase::Servicing;
    TestFalse(TEXT("Final parking service cannot promise future berth release"),C->CanService(S));
    C->Agents[2].PlotID=0;
    TestTrue(TEXT("Active service permits a future queued reservation"),C->CanService(S));
    C->Agents[2].Battery=.999f;C->Agents[2].LiquidLitres=50;C->Agents[2].PhaseSeconds=200;C->Agents[2].bCleaned=true;
    S.Queue.Empty();
    const float BeforeDeparture=C->EstimateWaitSeconds(S,0);
    C->Agents[2].Phase=EAgriculturePhase::Resuming;
    TestTrue(TEXT("Service completion does not add approach time to berth wait"),C->EstimateWaitSeconds(S,0)<=BeforeDeparture);
    TestTrue(TEXT("Departure wait contains only takeoff and protection exit"),C->EstimateWaitSeconds(S,0)<50);
    TestTrue(TEXT("Queued approach starts after the berth is released"),C->EstimateWaitSeconds(S,10)+10+C->Config.AirportApproachSeconds>=C->EstimateWaitSeconds(S,0)+C->Config.AirportApproachSeconds-.01f);
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
#endif
