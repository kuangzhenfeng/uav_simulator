#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AgricultureTypes.h"
#include "AgricultureCoordinator.generated.h"

class AMultiAgentGameMode;
class AUAVPawn;
class UScenario;
class UMaterialInstanceDynamic;
class UAStarPathPlanner;
struct FTrajectory;

/** 地块进度与共享机场的唯一运行状态源。 */
UCLASS()
class UAV_SIMULATOR_API UAgricultureCoordinator : public UActorComponent
{
    GENERATED_BODY()
public:
    UAgricultureCoordinator();
    void Initialize(UScenario* Scenario, AMultiAgentGameMode* Manager);
    void Reset();
    void Update(float DeltaTime);
    void Command(int32 Command, int32 TargetID);
    bool IsEnabled() const { return Config.bEnabled; }
    float GetElapsedSeconds() const { return ElapsedSeconds; }
    bool IsFlying(int32 AgentID) const;
    int32 CompletedCount() const;
    int32 PlotCount() const { return Plots.Num(); }
    bool HasFailed() const;
    bool ReadyToFinish() const;
    FString Describe() const;
    FString GetTelemetryJson() const;
    FString GetLastReason() const { return LastReason; }
    const TArray<FAgricultureAgentState>& GetAgents() const { return Agents; }
    const TArray<FSupplyAirportState>& GetAirports() const { return Airports; }
    const TArray<FAgriculturePlotState>& GetPlots() const { return Plots; }
    const FAgricultureConfig& GetConfig() const { return Config; }
    const TArray<FAgriculturePlotState>& GetSections() const { return Sections; }
    static TArray<FAgriculturePlotState> PartitionPlot(const FAgriculturePlotState& Plot, int32 MaxSections);
    static bool BuildSupplyDeparture(const FVector& Position,const FVector& Velocity,float Height,float Acceleration,FTrajectory& Out);
    static FVector SupplyDepartureTarget(const FVector& Position,const FVector& Velocity,float Height,float Acceleration);
    static bool BuildStrips(const FAgriculturePlot& Plot, float Height, TArray<FVector>& OutPoints);
    static FVector FindEmergencyLandingSite(const FVector& Position,const TArray<FAgriculturePlotState>& Fields,float ClearanceCm,
        const TArray<FObstacleInfo>& Obstacles={},float CollisionRadius=0);
    static int32 ChooseNearestAirport(const TArray<FSupplyAirportState>& Candidates,
        const TArray<float>& PathLengths, float AvailableFlightSeconds, float SpeedCm, float ReserveSeconds,const TArray<float>& TravelSeconds={},const TArray<float>& WaitingSeconds={});
    static bool RecordSpraySegment(FAgriculturePlotState& Plot,float& Liquid,const FVector& Previous,const FVector& Position,float BandCm);
    static bool CleanResidue(FAgricultureAgentState& Agent,FSupplyAirportState& Airport,FName Recipe);
    static float TransferLiquid(FAgricultureAgentState& Agent,FSupplyAirportState& Airport,
        float RequestedLitres,float CapacityLitres,float ConcentrateFraction);
private:
    friend class FAgricultureBerthQueueTest;
    friend class FAgricultureSectionStateTest;
    friend class FAgricultureEfficiencyTest;
    UPROPERTY(Transient) TObjectPtr<AMultiAgentGameMode> Manager;
    UPROPERTY(Transient) TObjectPtr<UAStarPathPlanner> SupplyPlanner;
    UPROPERTY(Transient) TMap<int32,TObjectPtr<UMaterialInstanceDynamic>> StatusMaterials;
    FAgricultureConfig Config;
    TArray<FAgricultureAgentState> Agents;
    TArray<FSupplyAirportState> Airports;
    TArray<FAgriculturePlotState> Plots;
    TArray<FAgriculturePlotState> Sections;
    FAgriculturePlotState* Section(const FAgricultureAgentState& Agent);
    void RefreshPlots();
    TMap<int32,FVector> PreviousPositions;
    FString LastReason;
    float ElapsedSeconds = 0;
    float LogSeconds = 0;
    float VisualSeconds = 0;
    bool bCollectSupplyRequests = false;
    TMap<int32,int32> PendingSupplyRequests;
    void ResolveSupplyRequests();
    void PlanSupplyAssignments();
    AUAVPawn* Pawn(int32 ID) const;
    FSupplyAirportState* Airport(int32 ID);
    void ChangePhase(FAgricultureAgentState& Agent, EAgriculturePhase Phase);
    bool FlyTo(FAgricultureAgentState& Agent, const FVector& Target, float Speed, const FVector& EndVelocity = FVector::ZeroVector);
    bool StartSupplyDeparture(FAgricultureAgentState& Agent);
    bool StartWorkRoute(FAgricultureAgentState& Agent,FAgriculturePlotState& Field);
    bool CanService(const FSupplyAirportState& Station) const;
    float EstimateWaitSeconds(const FSupplyAirportState& Station,float ArrivalSeconds,int32 WaitingAgentID=INDEX_NONE) const;
    FString ServiceStage(const FAgricultureAgentState& Agent) const;
    void AssignPlots();
    double EstimateSectionCost(const FAgricultureAgentState& Agent,const FVector& Position,const FAgriculturePlotState& Part) const;
    double RemainingWorkSeconds(const FAgriculturePlotState& Part,double& LiquidLitres) const;
    void RequestSupply(FAgricultureAgentState& Agent,int32 ExcludedAirportID=INDEX_NONE);
    bool RedirectSupplyReservation(FAgricultureAgentState& Agent,int32 ExcludedAirportID);
    void RefreshSupplyRoutes(FAgricultureAgentState& Agent,bool Force=false);
    void ReleaseAirport(FAgricultureAgentState& Agent);
    void UpdateAirport(FSupplyAirportState& Station, float DeltaTime);
    void UpdateAgent(FAgricultureAgentState& Agent, float DeltaTime);
    void Fail(FAgricultureAgentState& Agent, const FString& Reason);
};
