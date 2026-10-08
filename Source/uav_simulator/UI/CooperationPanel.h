#pragma once

#include "Blueprint/UserWidget.h"
#include "CooperationPanel.generated.h"

class ACooperationGameMode;
class UInstancedStaticMeshComponent;
struct FCooperationViewData;
enum class ECooperationObject : uint8;

UCLASS()
class UAV_SIMULATOR_API UCooperationPanel : public UUserWidget
{
	GENERATED_BODY()
public:
	UPROPERTY(Transient)
	TObjectPtr<ACooperationGameMode> Manager;
    void RefreshView();
    void SelectObject(ECooperationObject Type, int32 ID);
    void RunCommand(int32 Command);
    void RunAgricultureCommand(int32 Command);
    void FollowSelected();
protected:
	virtual TSharedRef<SWidget> RebuildWidget() override;
    virtual void NativeDestruct() override;
private:
    TSharedPtr<FCooperationViewData> View;
    UPROPERTY(Transient) TObjectPtr<AActor> CoverageActor;
    UPROPERTY(Transient) TObjectPtr<UInstancedStaticMeshComponent> CoverageMesh;
    TArray<FTransform> CoverageTransforms;
};
