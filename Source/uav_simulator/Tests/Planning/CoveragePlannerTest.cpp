#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../Planning/CoveragePlanner.h"
#include "../../Planning/TrajectoryOptimizer.h"
#include "../../Planning/AStarPathPlanner.h"
#include "../../MultiAgent/AgricultureCoordinator.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoverageMovingResumeTest,
    "UAVSimulator.Planning.Coverage.MovingResume",UAV_TEST_FLAGS)
bool FCoverageMovingResumeTest::RunTest(const FString&)
{
    FAgriculturePlot Plot;Plot.Boundary={FVector(-13000,-7500,0),FVector(-7000,-7500,0),FVector(-7000,-1500,0),FVector(-13000,-1500,0)};
    TArray<FVector> Strips;
    if(!UAgricultureCoordinator::BuildStrips(Plot,400,Strips)) return false;
    const FVector Direction=(Strips[3]-Strips[2]).GetSafeNormal();
    const FVector Entry=Strips[2]+Direction*(1778.515f-FCoveragePlanner::HeadlandDistance(300,150));
    auto* Optimizer=NewObject<UTrajectoryOptimizer>();
    Optimizer->SetStartVelocity(FVector(-301.67,0.51,0.02));
    Optimizer->SetStartAcceleration(FVector(1,0,0));
    Optimizer->SetEndVelocity(Direction*100);
    FVector Cruise=Entry;Cruise.Z=1500;
    const FTrajectory Approach=Optimizer->OptimizeTrajectory({FVector(7000,-1800,400),FVector(6400,-1800,1500),Cruise,Entry},400,150);
    TestTrue(TEXT("Moving multi-axis approach is feasible"),Approach.bIsValid);
    auto* Planner=NewObject<UAStarPathPlanner>();
    FObstacleInfo Road;Road.Center=FVector(5000,0,4);Road.Extents=FVector(350,14500,4);Road.SafetyMargin=200;
    Planner->SetObstacles({Road});
    bool Clear=true;
    for(int32 I=1;I<Approach.Points.Num();++I)
        Clear &= !Planner->CheckLineCollision(Approach.Points[I-1].Position,Approach.Points[I].Position,150);
    TestTrue(TEXT("Departure climb clears the road safety envelope"),Clear);
    FTrajectory Work;TMap<int32,float> Times;
    TestTrue(TEXT("Moving approach joins a partial-strip coverage profile"),
        FCoveragePlanner::Build(Strips,3,1778.515f,Approach,300,150,Work,Times));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoverageCompleteRouteTest,
    "UAVSimulator.Planning.Coverage.CompleteRoute",UAV_TEST_FLAGS)
bool FCoverageCompleteRouteTest::RunTest(const FString&)
{
    FAgriculturePlot Plot;
    Plot.Boundary={FVector(0,0,0),FVector(6000,0,0),FVector(6000,6000,0),FVector(0,6000,0)};
    TArray<FVector> Strips;
    if(!UAgricultureCoordinator::BuildStrips(Plot,400,Strips)) return false;
    auto* Optimizer=NewObject<UTrajectoryOptimizer>();
    Optimizer->SetEndVelocity(FVector(100,0,0));
    const float Headland=FCoveragePlanner::HeadlandDistance(300,150);
    const FVector Entry=Strips[0]-FVector(Headland,0,0);
    const FTrajectory Approach=Optimizer->OptimizeTrajectory({Entry-FVector(1000,0,0),Entry},300,150);
    FTrajectory Work;TMap<int32,float> EndTimes;
    TestTrue(TEXT("Whole field is planned before execution"),FCoveragePlanner::Build(Strips,1,0,Approach,300,150,Work,EndTimes));
    if(!Work.bIsValid) return false;
    TestEqual(TEXT("Every strip has a coverage milestone"),EndTimes.Num(),10);
    TestTrue(TEXT("Only route end stops"),Work.Points.Last().Velocity.Size()<1);
    float MinimumSpeed=300;float LastTime=-1;bool Limits=true;
    for(const auto& Point:Work.Points)
    {
        if(Limits && !(Point.TimeStamp>LastTime && Point.Position.Z>399 && Point.Position.Z<401 &&
            Point.Velocity.Size()<=301 && Point.Acceleration.Size()<=151))
        {
            AddError(FString::Printf(TEXT("Invalid route sample: time=%.6f previous=%.6f altitude=%.3f speed=%.3f acceleration=%.3f"),
                Point.TimeStamp,LastTime,Point.Position.Z,Point.Velocity.Size(),Point.Acceleration.Size()));
            Limits=false;
        }
        LastTime=Point.TimeStamp;
        if(Point.TimeStamp>Approach.TotalDuration && Point.TimeStamp<EndTimes.FindChecked(19))
            MinimumSpeed=FMath::Min(MinimumSpeed,float(Point.Velocity.Size()));
    }
    TestTrue(TEXT("All turns remain moving"),MinimumSpeed>50);
    TestTrue(TEXT("Time, altitude and dynamics limits hold"),Limits);
    TestTrue(TEXT("Coverage cruise is within 10 percent of distance divided by speed"),
        EndTimes.FindChecked(1)-Approach.TotalDuration<25);
    FAgriculturePlotState Field;Field.Config=Plot;Field.StripPoints=Strips;Field.NextPoint=1;
    float Liquid=100;FVector Previous=Entry;bool ContinuousCoverage=true;
    for(const auto& Point:Work.Points)
    {
        if(Point.TimeStamp<Approach.TotalDuration) continue;
        if(!UAgricultureCoordinator::RecordSpraySegment(Field,Liquid,Previous,Point.Position,100))
        {ContinuousCoverage=false;break;}
        const FVector Direction=(Strips[Field.NextPoint]-Strips[Field.NextPoint-1]).GetSafeNormal();
        const float Length=FVector::Dist(Strips[Field.NextPoint],Strips[Field.NextPoint-1]);
        if(Field.StripCoveredCm>=Length-1 && FVector::DotProduct(Point.Position-Strips[Field.NextPoint],Direction)>=150 && Field.NextPoint+2<Strips.Num())
        {Field.NextPoint+=2;Field.StripCoveredCm=0;}
        Previous=Point.Position;
    }
    TestTrue(TEXT("Complete moving route never mistakes headland turns for missing coverage"),ContinuousCoverage);
    TestTrue(TEXT("All 3600 square metres are covered"),FMath::Abs(Field.CoveredSquareMetres-3600)<1);
    TestTrue(TEXT("Application follows covered area"),FMath::Abs(Field.AppliedLitres-54)<0.01);
    const int32 ResumeEnd=5;const float Covered=2300;
    const FVector Direction=(Strips[ResumeEnd]-Strips[ResumeEnd-1]).GetSafeNormal();
    const FVector ResumeEntry=Strips[ResumeEnd-1]+Direction*(Covered-Headland);
    Optimizer->SetEndVelocity(Direction*100);
    const FTrajectory ResumeApproach=Optimizer->OptimizeTrajectory({ResumeEntry-Direction*1000,ResumeEntry},300,150);
    TestTrue(TEXT("Remaining route can be rebuilt at exact coverage breakpoint"),
        FCoveragePlanner::Build(Strips,ResumeEnd,Covered,ResumeApproach,300,150,Work,EndTimes));
    TestEqual(TEXT("Completed strips are omitted on resume"),EndTimes.Num(),8);
    TestFalse(TEXT("Invalid strip cursor is rejected"),FCoveragePlanner::Build(Strips,4,0,Approach,300,150,Work,EndTimes));
    const TArray<FVector> OffsetStrips={FVector(10003,6000,400),FVector(7001.847,6000,400),
        FVector(10003.799,6600,400),FVector(13000,6600,400)};
    const FVector OffsetEntry=OffsetStrips[0]+FVector(Headland,0,0);
    Optimizer->SetEndVelocity(FVector(-100,0,0));
    const FTrajectory OffsetApproach=Optimizer->OptimizeTrajectory({OffsetEntry+FVector(1000,0,0),OffsetEntry},300,150);
    TestTrue(TEXT("Reversed partial strip joins a longitudinally offset remainder"),
        FCoveragePlanner::Build(OffsetStrips,1,0,OffsetApproach,300,150,Work,EndTimes));
    bool OffsetLimits=Work.bIsValid;float OffsetPreviousTime=-1;
    for(const auto& Point:Work.Points)
    {
        OffsetLimits &= Point.TimeStamp>OffsetPreviousTime && Point.Velocity.Size()<=301 && Point.Acceleration.Size()<=151;
        OffsetPreviousTime=Point.TimeStamp;
    }
    TestTrue(TEXT("Offset headland keeps continuous time and physical speed/acceleration limits"),OffsetLimits);
    TestEqual(TEXT("Offset continuation retains both unsprayed coverage milestones"),EndTimes.Num(),2);
    return true;
}
#endif
