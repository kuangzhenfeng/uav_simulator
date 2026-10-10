#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AgricultureTypes.h"
#include "AgricultureCoordinator.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogAgriculture, Log, All);

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
    static double SegmentEnergyFraction(float EmptyMassKg,float DensityKgPerLitre,double StartLitres,double EndLitres,double Seconds,double ReferenceSeconds);
    static float PayloadPowerRatio(float EmptyMassKg,float LiquidLitres,float DensityKgPerLitre);
    static bool RecordSpraySegment(FAgriculturePlotState& Plot,float& Liquid,const FVector& Previous,const FVector& Position,float BandCm);
    static bool CleanResidue(FAgricultureAgentState& Agent,FSupplyAirportState& Airport,FName Recipe);
    static float DrainLiquid(FAgricultureAgentState& Agent,FSupplyAirportState& Airport,float Amount,float Target);
    static float TransferLiquid(FAgricultureAgentState& Agent,FSupplyAirportState& Airport,
        float RequestedLitres,float CapacityLitres,float ConcentrateFraction);
private:
    double SupplyTransferTravelSeconds(const FAgricultureAgentState& Agent,const FSupplyAirportState& Source,const FSupplyAirportState& Destination) const;
    double SupplyTravelTime(const FVector& Position,const FVector& Velocity,double PathDistance,float Acceleration) const;
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
    TArray<TArray<FAgricultureSupplySlot>> SupplySlots;
    FAgriculturePlotState* Section(const FAgricultureAgentState& Agent);
    void RefreshPlots();
    TMap<int32,FVector> PreviousPositions;
    FString LastReason;
    float ElapsedSeconds = 0;
    float LogSeconds = 0;
    float VisualSeconds = 0;
    float SupplyPlanTime = -1;
    float AssignmentPlanTime = -1;
    bool bCollectSupplyRequests = false;
    TMap<int32,int32> PendingSupplyRequests;
    void ResolveSupplyRequests();
    void PlanSupplyAssignments();
    static double FindSupplySlotWait(double StartSeconds,double DurationSeconds,const TArray<FAgricultureSupplySlot>& Slots,int32 AgentID);
    static void ReserveSupplyStock(FSupplyAirportState& Station,const FAgricultureAgentState& Agent,const FAgriculturePlotState& Field,float TargetLitres);
    FSupplyAirportState AvailableSupplyStock(const FSupplyAirportState& Station,int32 AgentID) const;
    void ConsumeSupplyReservation(int32 AirportID,int32 AgentID,double WaterLitres,double ConcentrateLitres,double WasteLitres);
    AUAVPawn* Pawn(int32 ID) const;
    FSupplyAirportState* Airport(int32 ID);
    void ChangePhase(FAgricultureAgentState& Agent, EAgriculturePhase Phase);
    bool FlyTo(FAgricultureAgentState& Agent, const FVector& Target, float Speed, const FVector& EndVelocity = FVector::ZeroVector);
    bool StartSupplyDeparture(FAgricultureAgentState& Agent);
    bool StartWorkRoute(FAgricultureAgentState& Agent,FAgriculturePlotState& Field);
    bool CanService(const FSupplyAirportState& Station) const;
    float EstimateWaitSeconds(const FSupplyAirportState& Station,float ArrivalSeconds,int32 WaitingAgentID) const;
    FString ServiceStage(const FAgricultureAgentState& Agent) const;
    float FlightSeconds(const FAgricultureAgentState& Agent) const;
    static void AdvanceForecastProgress(FAgriculturePlotState& Field,double AppliedLitres);
    FAgricultureSortieForecast ForecastSortie(const FAgricultureAgentState& Agent,const FVector& Position,const FAgriculturePlotState& Field,
        const FSupplyAirportState& Station,float LiquidLitres,float BatteryFraction,bool IncludeQueue=true,double StationWaitSnapshotSeconds=-1) const;
    FAgricultureServicePlan PlanService(const FAgricultureAgentState& Agent,const FAgriculturePlotState& Field,const FSupplyAirportState& Station,bool IncludeQueue=true) const;
    FAgricultureServicePlan PlanDockService(const FAgricultureAgentState& Agent,const FAgriculturePlotState& Field,const FSupplyAirportState& Station,bool IncludeQueue=true) const;
    FAgriculturePlotState ChooseWorkOrder(const FAgricultureAgentState& Agent,const FVector& Position,const FAgriculturePlotState& Field,double* OutCost=nullptr) const;
    void AssignPlots();
    double EstimateSectionCost(const FAgricultureAgentState& Agent,const FVector& Position,const FAgriculturePlotState& Part) const;
    double RemainingWorkSeconds(const FAgriculturePlotState& Part,double& LiquidLitres) const;
    void RequestSupply(FAgricultureAgentState& Agent,int32 ExcludedAirportID=INDEX_NONE);
    bool RedirectSupplyReservation(FAgricultureAgentState& Agent,int32 ExcludedAirportID);
    void RefreshSupplyRoutes(FAgricultureAgentState& Agent,bool Force=false);
    void ReleaseAirport(FAgricultureAgentState& Agent);
    void DeferDockedSection(FAgricultureAgentState& Agent);
    void UpdateAirport(FSupplyAirportState& Station, float DeltaTime);
    void UpdateAgent(FAgricultureAgentState& Agent, float DeltaTime);
    void Fail(FAgricultureAgentState& Agent, const FString& Reason);
};
