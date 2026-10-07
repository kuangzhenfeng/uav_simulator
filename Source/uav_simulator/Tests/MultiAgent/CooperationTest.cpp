#include "Misc/AutomationTest.h"
#include "../UAVTestCommon.h"
#include "../../MultiAgent/TaskMonitor.h"
#include "../../MultiAgent/TaskAllocator.h"
#include "../../Scenario/ScenarioEvaluator.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCooperationArrivalTest,"UAVSimulator.MultiAgent.Cooperation.ActualArrival",UAV_TEST_FLAGS)
bool FCooperationArrivalTest::RunTest(const FString&)
{
    UTaskMonitor* M=NewObject<UTaskMonitor>();
    FTaskDescriptor T; T.TaskID=1; T.TargetLocation=FVector(5000,0,1500); T.EstimatedDuration=1;
    FTaskAllocationResult A; A.bIsFeasible=true; FTaskAssignment Entry; Entry.TaskID=1;Entry.AgentID=0;Entry.EstimatedCompletionTime=2;A.Assignments.Add(Entry);
    M->Initialize(A,FTaskMonitorConfig(),{T}); M->ActivateTask(1);
    auto S=UAVTestHelpers::CreateAgentSnapshot(0,FVector(0,0,1500));
    M->Update(20,{S});
    TestEqual(TEXT("Estimated time cannot complete a distant task"),M->GetCompletedTaskCount(),0);
    S.State.Position=T.TargetLocation; S.State.Velocity=FVector::ZeroVector;
    M->Update(0.5f,{S});TestEqual(TEXT("Service must finish"),M->GetCompletedTaskCount(),0);
    M->Update(0.5f,{S});TestEqual(TEXT("Arrival plus service completes task"),M->GetCompletedTaskCount(),1);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCooperationQueueTest,"UAVSimulator.MultiAgent.Cooperation.QueuedTasks",UAV_TEST_FLAGS)
bool FCooperationQueueTest::RunTest(const FString&)
{
    UTaskMonitor* M=NewObject<UTaskMonitor>();
    FTaskAllocationResult A;A.bIsFeasible=true; TArray<FTaskDescriptor> Tasks;
    for(int32 I=0;I<2;++I)
    {
        FTaskDescriptor T;T.TaskID=I;T.TargetLocation=FVector(1000+I*1000,0,1500);T.EstimatedDuration=0;Tasks.Add(T);
        FTaskAssignment E;E.TaskID=I;E.AgentID=0;E.EstimatedCompletionTime=1;A.Assignments.Add(E);
    }
    M->Initialize(A,FTaskMonitorConfig(),Tasks);M->ActivateTask(0);
    M->Update(20,{UAVTestHelpers::CreateAgentSnapshot(0,Tasks[1].TargetLocation)});
    TestEqual(TEXT("Queued target arrival cannot complete task"),M->GetTaskStatus(1),ETaskStatus::Assigned);
    TestEqual(TEXT("Only one active task per agent"),M->GetActiveTaskID(0),0);
    M->Update(0.5f,{UAVTestHelpers::CreateAgentSnapshot(0,Tasks[0].TargetLocation)});
    TestEqual(TEXT("First task completes"),M->GetTaskStatus(0),ETaskStatus::Completed);
    M->ActivateTask(1);M->Update(0.5f,{UAVTestHelpers::CreateAgentSnapshot(0,Tasks[1].TargetLocation)});
    TestEqual(TEXT("Both tasks retained and completed"),M->GetCompletedTaskCount(),2);
    M->Initialize(FTaskAllocationResult(),FTaskMonitorConfig(),{});M->PreserveCompleted({0,1});
    TestEqual(TEXT("Replan preserves completed tasks"),M->GetCompletedTaskCount(),2);
    M->Reset();TestEqual(TEXT("Reset clears completed state"),M->GetCompletedTaskCount(),0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCooperationReallocateTest,"UAVSimulator.MultiAgent.Cooperation.FaultAndInsertion",UAV_TEST_FLAGS)
bool FCooperationReallocateTest::RunTest(const FString&)
{
    UTaskAllocator* Allocator=NewObject<UTaskAllocator>();
    FTaskDescriptor T;T.TaskID=10;T.TargetLocation=FVector(1000,0,1500);
    FUAVCapability C;C.AgentID=0;C.CurrentPosition=FVector(0,0,1500);
    FUAVCapability D=C;D.AgentID=1;
    auto Previous=Allocator->Allocate({T},{C,D},FTaskAllocationConfig());
    TestTrue(TEXT("Initial allocation feasible"),Previous.bIsFeasible);
    FTaskDescriptor Urgent=T;Urgent.TaskID=11;Urgent.Priority=ETaskPriority::Critical;
    auto Next=Allocator->Reallocate(Previous,{Urgent},{0},{},{C,D},FTaskAllocationConfig());
    TestTrue(TEXT("Reallocation feasible"),Next.bIsFeasible);
    TestEqual(TEXT("Old and inserted tasks both retained"),Next.Assignments.Num(),2);
    for(const auto& E:Next.Assignments)TestEqual(TEXT("Faulted agent excluded"),E.AgentID,1);
    T.RequiredPayload=1000;
    TestFalse(TEXT("Impossible payload rejected"),Allocator->Allocate({T},{D},FTaskAllocationConfig()).bIsFeasible);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCooperationSafetyTest,"UAVSimulator.MultiAgent.Cooperation.SeparationAcceptance",UAV_TEST_FLAGS)
bool FCooperationSafetyTest::RunTest(const FString&)
{
    UAcceptanceCriteria* C=NewObject<UAcceptanceCriteria>();
    FScenarioMetrics M;M.AgentSafeDistanceCm=500;M.MinAgentSeparationCm=499;
    TestFalse(TEXT("Unsafe separation fails"),UScenarioEvaluator::Evaluate(M,C).bPassed);
    M.MinAgentSeparationCm=501;
    TestTrue(TEXT("Safe separation passes"),UScenarioEvaluator::Evaluate(M,C).bPassed);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCooperationFailureStateTest,"UAVSimulator.MultiAgent.Cooperation.FailureAndServiceWindow",UAV_TEST_FLAGS)
bool FCooperationFailureStateTest::RunTest(const FString&)
{
    auto* M=NewObject<UTaskMonitor>(); FTaskDescriptor T;T.TaskID=1;T.EarliestStart=5;T.Deadline=8;T.EstimatedDuration=1;T.TargetLocation=FVector(0,0,1500);
    FTaskAllocationResult A;FTaskAssignment E;E.TaskID=1;E.AgentID=0;A.Assignments.Add(E);
    M->Initialize(A,FTaskMonitorConfig(),{T});M->ActivateTask(1);
    const auto S=UAVTestHelpers::CreateAgentSnapshot(0,T.TargetLocation);
    M->Update(1,{S});TestEqual(TEXT("Service cannot finish before time window"),M->GetTaskStatus(1),ETaskStatus::InProgress);
    M->Update(8,{S});TestEqual(TEXT("True deadline expires task"),M->GetTaskStatus(1),ETaskStatus::Failed);
    M->Initialize({},FTaskMonitorConfig(),{});M->PreserveFailed({1});
    TestEqual(TEXT("Failure retained across replanning"),M->GetTaskStatus(1),ETaskStatus::Failed);
    M->Reset();TestEqual(TEXT("Reset removes terminal failure"),M->GetTaskStatus(1),ETaskStatus::Pending);
    return true;
}
#endif
