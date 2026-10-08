#include "CoveragePlanner.h"
#include "TrajectoryOptimizer.h"
#include "../Debug/UAVLogConfig.h"

float FCoveragePlanner::HeadlandDistance(float Speed,float Acceleration)
{
    return FMath::Max(150.0f,Speed*Speed/FMath::Max(Acceleration,1.0f));
}

bool FCoveragePlanner::Build(const TArray<FVector>& Points,int32 FirstEnd,float CoveredCm,
    const FTrajectory& Approach,float Speed,float Acceleration,
    FTrajectory& Out,TMap<int32,float>& EndTimes)
{
    Out=FTrajectory();EndTimes.Empty();
    if(FirstEnd<1 || FirstEnd%2!=1 || Points.Num()%2!=0 || !Points.IsValidIndex(FirstEnd) ||
        !Approach.bIsValid || Approach.Points.Num()<2 || Speed<=0 || Acceleration<=0) return false;
    FTrajectory Result=Approach;
    TMap<int32,float> Times;
    auto* Optimizer=NewObject<UTrajectoryOptimizer>();
    const float Headland=HeadlandDistance(Speed,Acceleration);
    const auto Append=[&](const FVector& Target,const FVector& EndVelocity,bool Straight)
    {
        const auto Last=Result.Points.Last();
        const float Distance=FVector::Dist(Last.Position,Target);
        if(Distance<1)
        {
            const bool Compatible=Last.Velocity.Equals(EndVelocity,1);
            if(!Compatible) UE_LOG(LogUAVPlanning,Warning,TEXT("[Coverage] Coincident endpoint with incompatible velocity Distance=%.6f StartSpeed=%.3f EndSpeed=%.3f"),Distance,Last.Velocity.Size(),EndVelocity.Size());
            return Compatible;
        }
        Optimizer->SetStartVelocity(Last.Velocity);
        Optimizer->SetStartAcceleration(Last.Acceleration);
        Optimizer->SetEndVelocity(EndVelocity);
        const TArray<FVector> Path={Last.Position,Target};
        const FTrajectory Segment=Straight ? Optimizer->OptimizeTrajectoryWithTiming(Path,
            {2*Distance/FMath::Max(1.0f,float(Last.Velocity.Size()+EndVelocity.Size()))}) :
            Optimizer->OptimizeTrajectory(Path,Speed,Acceleration);
        if(!Segment.bIsValid || Segment.Points.Num()<2)
        {UE_LOG(LogUAVPlanning,Warning,TEXT("[Coverage] Invalid segment Start=%s End=%s Straight=%d"),*Last.Position.ToString(),*Target.ToString(),Straight);return false;}
        for(int32 I=1;I<Segment.Points.Num();++I)
        {
            auto Point=Segment.Points[I];
            if(Point.Position.ContainsNaN() || Point.Velocity.Size()>Speed+1 || Point.Acceleration.Size()>Acceleration+1)
            {
                UE_LOG(LogUAVPlanning,Warning,TEXT("[Coverage] Profile limit Start=%s End=%s Straight=%d Time=%.6f Speed=%.3f Acceleration=%.3f"),
                    *Last.Position.ToString(),*Target.ToString(),Straight,Point.TimeStamp,Point.Velocity.Size(),Point.Acceleration.Size());
                return false;
            }
            Point.TimeStamp+=Result.TotalDuration;
            // 累计时间的浮点舍入可能合并极短的末尾采样，保留精确边界状态并避免重复时间戳。
            if(Point.TimeStamp<=Result.Points.Last().TimeStamp) Result.Points.Last()=Point;
            else Result.Points.Add(Point);
        }
        Result.TotalDuration+=Segment.TotalDuration;
        return true;
    };
    for(int32 End=FirstEnd;End<Points.Num();End+=2)
    {
        const FVector Direction=(Points[End]-Points[End-1]).GetSafeNormal();
        const FVector Start=Points[End-1]+Direction*(End==FirstEnd ? FMath::Clamp(CoveredCm,0.0f,float(FVector::Dist(Points[End-1],Points[End]))) : 0);
        if(!Append(Start,Direction*Speed,true) || !Append(Points[End],Direction*Speed,true)) return false;
        Times.Add(End,Result.TotalDuration);
        const bool HasNext=Points.IsValidIndex(End+2);
        const float TurnSpeed=HasNext ? FMath::Min(Speed,0.6f*FMath::Sqrt(Acceleration*FVector::Dist(Points[End],Points[End+1])*0.5f)) : 0;
        if(!Append(Points[End]+Direction*Headland,Direction*TurnSpeed,true)) return false;
        if(HasNext)
        {
            const FVector NextDirection=(Points[End+2]-Points[End+1]).GetSafeNormal();
            if(!Append(Points[End+1]-NextDirection*Headland,NextDirection*TurnSpeed,false)) return false;
        }
    }
    Result.bIsValid=true;
    Out=MoveTemp(Result);EndTimes=MoveTemp(Times);
    return true;
}
