// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Styling/SlateTypes.h"
#include "CameraSwitcherWidget.generated.h"

class UButton;
class UTextBlock;
class UVerticalBox;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnViewChanged, int32, AgentID);

/**
 * 视角切换按钮面板
 *
 * 显示"全局视角"和每台无人机的跟随按钮，支持点击切换。
 */
UCLASS()
class UAV_SIMULATOR_API UCameraSwitcherWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	UPROPERTY(BlueprintAssignable, Category = "Camera")
	FOnViewChanged OnViewChanged;

	UFUNCTION(BlueprintCallable, Category = "Camera")
	void RefreshDroneList(int32 Count);

	UFUNCTION(BlueprintCallable, Category = "Camera")
	void HighlightCurrentView(int32 AgentID);

protected:
	virtual void NativeConstruct() override;

private:
	UPROPERTY(Transient)
	TObjectPtr<UVerticalBox> ButtonList;

	int32 DroneCount = 0;
	int32 CurrentHighlightAgentID = -1;

	UPROPERTY(Transient)
	TObjectPtr<UButton> GlobalButton;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UButton>> DroneButtons;

	UButton* CreateButton(const FString& Label, int32 AgentID);
	void UpdateHighlight();
	void SetButtonHighlight(UButton* Button, bool bHighlighted);

	UFUNCTION()
	void OnGlobalClicked();

	UFUNCTION()
	void OnDrone0Clicked();
	UFUNCTION()
	void OnDrone1Clicked();
	UFUNCTION()
	void OnDrone2Clicked();
	UFUNCTION()
	void OnDrone3Clicked();
	UFUNCTION()
	void OnDrone4Clicked();
	UFUNCTION()
	void OnDrone5Clicked();
	UFUNCTION()
	void OnDrone6Clicked();
	UFUNCTION()
	void OnDrone7Clicked();
};
