#include "CooperationGameMode.h"
#include "AgricultureCoordinator.h"
#include "../UI/CooperationPanel.h"
#include "../Core/UAVPawn.h"
#include "../AI/UAVAIController.h"
#include "../Scenario/ScenarioTypes.h"
#include "../Scenario/ScenarioEvaluator.h"
#include "../Mission/MissionComponent.h"
#include "../Planning/TrajectoryOptimizer.h"
#include "TaskMonitor.h"
#include "FormationComponent.h"
#include "../Planning/TrajectoryTracker.h"
#include "Kismet/GameplayStatics.h"
#include "Camera/CameraActor.h"
#include "EngineUtils.h"
#include "DrawDebugHelpers.h"
#include "Misc/App.h"

ACooperationGameMode::ACooperationGameMode(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    PrimaryActorTick.bTickEvenWhenPaused = true;
    Agriculture=CreateDefaultSubobject<UAgricultureCoordinator>(TEXT("AgricultureCoordinator"));
}

void ACooperationGameMode::BeginPlay()
{
    if (!Presets.IsEmpty() && DefaultScenario.IsNull()) DefaultScenario = Presets[0];
    Super::BeginPlay();
    Agriculture->Initialize(ActiveScenario,this);
    for (int32 I=0;I<Presets.Num();++I) if (Presets[I].Get()==ActiveScenario) PresetIndex=I;
    ScenarioStatus=TEXT("就绪");
    if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
    {
        if (!FApp::IsUnattended())
        {
            Panel = CreateWidget<UCooperationPanel>(PC);
            Panel->Manager = this;
            Panel->AddToViewport(20);
            PC->PrimaryActorTick.bTickEvenWhenPaused=true;
            PC->bShowMouseCursor = true;
            PC->SetInputMode(FInputModeGameAndUI());
            DemoCommand(13);
            UGameplayStatics::SetGamePaused(this,true);
        }
    }
}

void ACooperationGameMode::SelectPreset(int32 Index)
{
    if (!Presets.IsValidIndex(Index)) return;
    UScenario* S = Presets[Index].LoadSynchronous();
    if (!S) { ScenarioStatus = TEXT("预设资产加载失败"); return; }
    UGameplayStatics::SetGamePaused(this,false);
    PresetIndex=Index;
    MinSeparation=MAX_FLT; FormationTimer=0; FormationError=0;SimulationAccumulator=0;DemoElapsedSeconds=0;FiredDemoEvents.Empty();
    ScenarioStatus=TEXT("就绪");
    ReloadScenario(S);
    Agriculture->Initialize(ActiveScenario,this);
    DemoCommand(13);
    UGameplayStatics::SetGamePaused(this,true);
}

void ACooperationGameMode::DemoCommand(int32 Command)
{
    if(Command==0) {UGameplayStatics::SetGamePaused(this,false);ScenarioStatus=TEXT("运行中");}
    else if(Command==1) {UGameplayStatics::SetGamePaused(this,true);ScenarioStatus=TEXT("已暂停");}
    else if(Command==2) SelectPreset(PresetIndex);
    else if(Command>=3 && Command<=6)
    {
        if(Agriculture->IsEnabled()) {AgricultureCommand(0,Command-3);return;}
        if(PresetIndex!=0) return;
        FFormationConfig C=GetFormationConfig(); C.Type=static_cast<EFormationType>(Command-2); SetFormation(C);
    }
    else if(Command==7)
    {
        if(Agriculture->IsEnabled()) {AgricultureCommand(4,0);return;}
        if(PresetIndex<2) return;
        FTaskDescriptor T; T.Priority=ETaskPriority::Critical; T.TargetLocation=FVector(6500,0,1500);
        T.EstimatedDuration=2; T.Deadline=600; T.LatestFinish=600; T.Reward=20;
        InsertTask(T);
    }
    else if(Command>=8 && Command<=11) {if(Agriculture->IsEnabled()) AgricultureCommand(3,Command-8);else if(PresetIndex>=2) FailAgent(Command-8);}
    else if(Command>=12)
    {
        const FName Tag=Command==12 ? TEXT("CooperationOverview") : TEXT("CooperationTop");
        for(TActorIterator<ACameraActor> It(GetWorld());It;++It)
            if(It->ActorHasTag(Tag)) if(auto* PC=GetWorld()->GetFirstPlayerController()) PC->SetViewTargetWithBlend(*It,0.0f);
    }
}

void ACooperationGameMode::Tick(float DeltaTime)
{
    if(UGameplayStatics::IsGamePaused(this)) return;
    if(Agriculture->IsEnabled())
    {
        constexpr float Step=.02f;
        SimulationAccumulator+=FMath::Max(double(DeltaTime),0.0);
        int32 StepsThisFrame=0;
        // 保留过载帧的积压，限制单帧工作量而不丢弃仿真时间。
        while(SimulationAccumulator>=Step && StepsThisFrame++<50)
        {
            SimulationAccumulator-=Step;
            Super::Tick(Step);
            for(AUAVPawn* P:GetScenarioFleet()) if(P) P->AdvanceFlightSimulation(Step);
            Agriculture->Update(Step);
            DemoElapsedSeconds+=Step;
            if(ActiveScenario) for(int32 I=0;I<ActiveScenario->Agriculture.DemoEvents.Num();++I)
            {
                const auto& Event=ActiveScenario->Agriculture.DemoEvents[I];
                if(!FiredDemoEvents.Contains(I) && DemoElapsedSeconds>=Event.TimeSeconds)
                {FiredDemoEvents.Add(I);AgricultureCommand(Event.Command,Event.TargetID);}
            }
        }
    }
    else Super::Tick(DeltaTime);
    const auto States=GetAllAgentStates();
    float CurrentMin=MAX_FLT;
    FormationError=0;
    for(int32 I=0;I<States.Num();++I)
    {
        for(int32 J=I+1;J<States.Num();++J) CurrentMin=FMath::Min(CurrentMin,float(FVector::Dist(States[I].State.Position,States[J].State.Position)));
        FVector Target;
        if(GetFormationTarget(States[I].AgentID,Target)) FormationError=FMath::Max(FormationError,float(FVector::Dist(Target,States[I].State.Position)));
    }
    MinSeparation=FMath::Min(MinSeparation,CurrentMin);
    FormationTimer+=DeltaTime;
    if(FormationTimer>=1.0f)
    {
        FormationTimer=0;
        for(AUAVPawn* P:GetScenarioFleet())
        {
            if(!P || P->IsCrashed()) continue;
            const FColor Colors[]={FColor::Cyan,FColor::Orange,FColor::Green,FColor::Magenta};
            DrawDebugString(GetWorld(),P->GetActorLocation()+FVector(0,0,180),FString::Printf(TEXT("UAV %d"),P->GetAgentID()),nullptr,Colors[P->GetAgentID()%4],1.1f);
            if(GetFormationConfig().Type==EFormationType::None || IsLeader(P->GetAgentID())) continue;
            FVector Target;
            if(!GetFormationTarget(P->GetAgentID(),Target)) continue;
            if(auto* AI=Cast<AUAVAIController>(P->GetController())) AI->StopBehaviorTree();
            AUAVPawn* Leader=nullptr;
            for(AUAVPawn* Candidate:GetScenarioFleet()) if(Candidate && IsLeader(Candidate->GetAgentID())) Leader=Candidate;
            if(!Leader) continue;
            auto* Tracker=Leader->GetTrajectoryTracker();
            const FVector Offset=P->GetFormationComponent()->GetCurrentOffset();
            FTrajectory Reference; Reference.TotalDuration=10; Reference.bIsValid=true;
            for(int32 K=0;K<=50;++K)
            {
                const float T=K*0.2f;
                FTrajectoryPoint Point;
                if(Tracker && Tracker->IsTracking()) Point=Tracker->GetPredictionState(T);
                else {Point.Position=Leader->GetUAVState().Position;Point.Velocity=FVector::ZeroVector;}
                Point.TimeStamp=T;Point.Position+=Offset;Reference.Points.Add(Point);
            }
            P->SetTrajectory(Reference); P->StartTrajectoryTracking();
        }
    }
}

FText ACooperationGameMode::GetDemoStatus() const
{
    if(Agriculture->IsEnabled()) return FText::FromString(FString::Printf(TEXT("%s\n最小机间距 %.2f m / 阈值 %.2f m\n联合控制 %d 架\n"),
        *ScenarioStatus,MinSeparation==MAX_FLT ? 0 : MinSeparation/100,DefaultCBFQPConfig.DSafe/100,GetJointControlCount())+Agriculture->Describe());
    FString Status=ScenarioStatus;
    int32 Failed=0;
    if(auto* M=GetTaskMonitor()) for(const auto& T:GetTaskPool()) if(M->GetTaskStatus(T.TaskID)==ETaskStatus::Failed) ++Failed;
    if(!GetTaskPool().IsEmpty())
    {
        if(Failed>0 || !GetCurrentTaskAllocation().bIsFeasible) Status=TEXT("任务失败 / 请查看调度原因");
        else if(GetTaskMonitor()->GetCompletedTaskCount()==GetTaskPool().Num()) Status=TEXT("任务执行完成（画面保留）");
    }
    FString S=FString::Printf(TEXT("%s\n最小机间距 %.2f m / 安全阈值 %.2f m\n最大编队误差 %.2f m\n"),*Status,
        MinSeparation==MAX_FLT ? 0 : MinSeparation/100,DefaultCBFQPConfig.DSafe/100,FormationError/100);
    auto* Monitor=GetTaskMonitor();
    if(Monitor)
    {
        S+=FString::Printf(TEXT("任务完成 %d / %d\n"),Monitor->GetCompletedTaskCount(),GetTaskPool().Num());
        for(const auto& A:GetTaskPool())
            S+=FString::Printf(TEXT("T%d → UAV %d : %s\n"),A.TaskID,GetAssignedAgent(A.TaskID),*UEnum::GetValueAsString(Monitor->GetTaskStatus(A.TaskID)));
    }
    for (const auto& State:GetAllAgentStates())
        S+=FString::Printf(TEXT("UAV %d 当前任务 T%d\n"),State.AgentID,Monitor ? Monitor->GetActiveTaskID(State.AgentID) : -1);
    for(const auto& State:GetAllAgentStates()) if(IsAgentFailed(State.AgentID)) S+=FString::Printf(TEXT("UAV %d 已停用\n"),State.AgentID);
    S+=FString::Printf(TEXT("联合控制 %d 架 / 其余使用单机回退\n失败任务 %d\n"),GetJointControlCount(),Failed);
    S+=TEXT("最近调度：")+GetLastReplanReason();
    return FText::FromString(S);
}

void ACooperationGameMode::AgricultureCommand(int32 Command,int32 TargetID)
{
    if(!Agriculture->IsEnabled()) return;
    if(Command==5)
    {
        const FName Tag(*FString::Printf(TEXT("CooperationAirport%d"),TargetID));
        for(TActorIterator<ACameraActor> It(GetWorld());It;++It)
            if(It->ActorHasTag(Tag)) if(auto* PC=GetWorld()->GetFirstPlayerController()) PC->SetViewTargetWithBlend(*It,0.0f);
        return;
    }
    Agriculture->Command(Command,TargetID);
    if(ScenarioEvaluatorComponent) ScenarioEvaluatorComponent->InvalidateFinalResult();
}
