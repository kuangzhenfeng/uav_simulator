#pragma once

#include "Blueprint/UserWidget.h"
#include "CooperationPanel.generated.h"

class ACooperationGameMode;

UCLASS()
class UAV_SIMULATOR_API UCooperationPanel : public UUserWidget
{
	GENERATED_BODY()
public:
	UPROPERTY(Transient)
	TObjectPtr<ACooperationGameMode> Manager;
protected:
	virtual TSharedRef<SWidget> RebuildWidget() override;
};
