#pragma once

#include "CoreMinimal.h"
#include "TaskAllocationTypes.h"
#include "AgricultureTypes.generated.h"

/** 农业作业生命周期，补给和飞行共享同一个执行状态。 */
UENUM(BlueprintType)
enum class EAgriculturePhase : uint8
{
    Idle, TakingOff, Transit, Spraying, Returning, Waiting, Approaching,
    Landing, Servicing, Resuming, Completed, Failed
};

/** 共享机场声明；所有位置均为世界坐标 cm，库存单位为 L。 */
USTRUCT(BlueprintType)
struct FSupplyAirportConfig
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite) int32 AirportID = 0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) FVector DockPosition = FVector::ZeroVector;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) TArray<FVector> WaitingPoints;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) bool bEnabled = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float WaterLitres = 2000;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float ConcentrateLitres = 100;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float WasteCapacityLitres = 500;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float RefillLitresPerMinute = 60;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float ChargeSeconds = 210;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float MixSeconds = 10;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float CleanSeconds = 20;
};

/** 矩形地块以闭合边界声明，执行器生成平行条带。 */
USTRUCT(BlueprintType)
struct FAgriculturePlot
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite) FTaskDescriptor Task;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) TArray<FVector> Boundary;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float StripSpacingCm = 600;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float ApplicationLitresPerHa = 150;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) FName Recipe = TEXT("CropA");
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float ConcentrateFraction = 0.02f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float CanopyHeightCm = 100;
};

/** 预设注入事件复用面板与 HTTP 的统一命令入口。 */
USTRUCT(BlueprintType)
struct FAgricultureDemoEvent
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float TimeSeconds=0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) int32 Command=0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) int32 TargetID=0;
};

/** 工程模型参数，不代表厂家性能标定。 */
USTRUCT(BlueprintType)
struct FAgricultureConfig
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite) bool bEnabled = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) TArray<FAgricultureDemoEvent> DemoEvents;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float TankCapacityLitres = 50;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float InitialLiquidLitres = 12;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float InitialBatteryFraction = 0.8f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float BatteryFlightSeconds = 600; // 空载悬停基准续航，载荷通过诱导功率模型折算
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float LiquidDensityKgPerLitre = 1;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float BatteryReserveFraction = 0.15f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float TransitHeightCm = 1500;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float HeightAboveCanopyCm = 300;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float SpraySpeedCm = 300;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float TransitSpeedCm = 600;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float CruiseTimeFactor = 2.2f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float AirportApproachSeconds = 80;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float RouteArrivalAllowanceSeconds = 5;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float SupplyTimeBufferSeconds = 20;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float ArrivalRadiusCm = 100;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float DockRadiusCm = 30;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float DockSpeedCm = 20;
    UPROPERTY(EditAnywhere, BlueprintReadWrite) float DockStableSeconds = 1;
};

struct FAgricultureAgentState
{
    int32 AgentID = INDEX_NONE;
    int32 PlotID = INDEX_NONE;
    int32 SectionID = INDEX_NONE;
    int32 PlannedSupplyAirportID = INDEX_NONE;
    int32 AirportID = INDEX_NONE;
    EAgriculturePhase Phase = EAgriculturePhase::Idle;
    float EmptyMassKg = 22;
    float PayloadLimitKg = 50;
    float ServiceLiquidTarget = -1; // -1 尚未规划，-2 充电后转移至更合适的补给机场
    float ServiceBatteryTarget = 1;
    int32 ServiceAirportTargetID = INDEX_NONE;
    double ServiceDepartureNotBefore = 0;
    float ServicePlanTime = -1;
    bool bServiceCleanBeforeTransfer = false;
    float PlannedSupplyDelaySeconds = 0;
    float Battery = 1;
    float LiquidLitres = 0;
    FName Recipe;
    FVector Target = FVector::ZeroVector;
    float PhaseSeconds = 0;
    float DockStableSeconds = 0;
    double RequestTime = 0;
    bool bRouteActive = false;
    bool bCleaned = false;
    int32 RouteRetries = 0;
    int32 TransitStage = 0;
    float SupplyForecastTime = -1;
    TArray<float> SupplyPathLengths;
    TArray<float> SupplyTravelSeconds;
    TArray<float> SupplyEntryTravelSeconds;
    bool bWorkRouteActive = false;
    float WorkStartTime = 0;
    TMap<int32,float> WorkStripEndTimes;
};

struct FAgriculturePlotState
{
    FAgriculturePlot Config;
    TArray<FVector> StripPoints;
    int32 NextPoint = 0;
    int32 AgentID = INDEX_NONE;
    TArray<int32> AgentIDs;
    FVector ResumePosition = FVector::ZeroVector;
    double CoveredSquareMetres = 0;
    double AppliedLitres = 0;
    bool bCompleted = false;
    bool bFailed = false;
    float StripCoveredCm = 0;
};

struct FSupplyAirportState
{
    FSupplyAirportConfig Config;
    int32 OccupantID = INDEX_NONE;
    TArray<int32> Queue;
    float WasteLitres = 0;
};

/** 同一航次逐段预测，能量单位为额定电池容量的比例。 */
struct FAgricultureSortieForecast
{
    double FlightSeconds = 0;
    double EnergyFraction = 0;
    double AppliedLitres = 0;
    double RequiredBattery = 0;
    double ReturnSeconds = 0;
    double EntryFlightSeconds = 0;
    double EntryReturnSeconds = 0;
    FVector EndPosition = FVector::ZeroVector;
    bool bCompleted = false;
    bool bFeasible = false;
};

/** 机场占用窗口，起止时间使用农业仿真的绝对秒数。 */
struct FAgricultureSupplySlot
{
    int32 AgentID = INDEX_NONE;
    double BeginSeconds = 0;
    double EndSeconds = 0;
    double WaterLitres = 0;
    double ConcentrateLitres = 0;
    double WasteLitres = 0;
};

struct FAgricultureServicePlan
{
    int32 AirportID = INDEX_NONE;
    float LiquidLitres = -1;
    float BatteryFraction = 1;
    double ServiceSeconds = 0;
    double CompletionSeconds = DBL_MAX;
    double DepartureDelaySeconds = 0;
    bool bCleanBeforeTransfer = false;
};
