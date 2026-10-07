#pragma once

#include "AgentManager.h"
#include "CooperationGameMode.generated.h"

class UCooperationPanel;
class UAgricultureCoordinator;

/** 协同测试场的运行入口；动态内容由 Scenario 装配。 */
UCLASS()
class UAV_SIMULATOR_API ACooperationGameMode : public AMultiAgentGameMode
{
	GENERATED_BODY()
public:
	ACooperationGameMode(const FObjectInitializer& ObjectInitializer);
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cooperation")
	TArray<TSoftObjectPtr<UScenario>> Presets;
	UFUNCTION(BlueprintCallable, Category = "Cooperation")
	void SelectPreset(int32 Index);
	UFUNCTION(BlueprintCallable, Category = "Cooperation")
	void DemoCommand(int32 Command);
	FText GetDemoStatus() const;
	int32 GetPresetIndex() const { return PresetIndex; }
	float GetFormationErrorCm() const { return FormationError; }
	float GetMinSeparationCm() const { return MinSeparation; }
	UAgricultureCoordinator* GetAgriculture() const { return Agriculture; }
	void AgricultureCommand(int32 Command, int32 TargetID);
private:
	UPROPERTY(Transient)
	TObjectPtr<UAgricultureCoordinator> Agriculture;
	UPROPERTY(Transient)
	TObjectPtr<UCooperationPanel> Panel;
	int32 PresetIndex = 0;
	double SimulationAccumulator=0;
    double DemoElapsedSeconds=0;
    TSet<int32> FiredDemoEvents;
	float FormationTimer = 0.0f;
	float MinSeparation = MAX_FLT;
	float FormationError = 0.0f;
	FString ScenarioStatus;
};
