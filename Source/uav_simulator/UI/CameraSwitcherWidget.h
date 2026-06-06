// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Styling/SlateTypes.h"
#include "CameraSwitcherWidget.generated.h"

class SButton;
class SVerticalBox;
class SHorizontalBox;
struct FButtonStyle;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnViewChanged, int32, AgentID);

/**
 * 视角切换按钮面板
 *
 * 横向紧凑布局：标题 + 横排按钮 [All] [#0] [#1] [#2]
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
	virtual TSharedRef<SWidget> RebuildWidget() override;

private:
	int32 DroneCount = 0;
	int32 CurrentHighlightAgentID = -1;

	// 根容器（纵向：标题 + 分隔线 + 按钮行）
	TSharedPtr<SVerticalBox> RootVBox;

	// 按钮行容器（横向）
	TSharedPtr<SHorizontalBox> ButtonRow;

	// Slate 按钮引用（用于高亮更新）
	TSharedPtr<SButton> GlobalSlateButton;
	TArray<TSharedPtr<SButton>> DroneSlateButtons;

	// 按钮样式（作为成员变量保证生命周期，SButton 持有指针引用）
	FButtonStyle NormalStyle;
	FButtonStyle HighlightedStyle;

	// 面板背景 Brush
	FSlateBrush PanelBgBrush;

	// 重建按钮行
	void BuildContent();

	// 更新按钮高亮
	void UpdateHighlight();
};
