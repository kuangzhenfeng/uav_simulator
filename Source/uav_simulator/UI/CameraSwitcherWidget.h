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

// 视角切换委托
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnViewChanged, int32, AgentID);

// 调试开关 toggle 委托（ToggleIndex 对应: 0=Vis, 1=Obs, 2=Path, 3=Dbg）
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnToggleDebug, int32, ToggleIndex);

/**
 * 视角切换 + 调试开关按钮面板
 *
 * 横向紧凑布局：
 *   Debug  |Vis|Obs|Path|Dbg|
 *   Camera |All|#0|#1|#2|
 */
UCLASS()
class UAV_SIMULATOR_API UCameraSwitcherWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	UPROPERTY(BlueprintAssignable, Category = "Camera")
	FOnViewChanged OnViewChanged;

	UPROPERTY(BlueprintAssignable, Category = "Debug")
	FOnToggleDebug OnToggleDebug;

	UFUNCTION(BlueprintCallable, Category = "Camera")
	void RefreshDroneList(int32 Count);

	UFUNCTION(BlueprintCallable, Category = "Camera")
	void HighlightCurrentView(int32 AgentID);

protected:
	virtual TSharedRef<SWidget> RebuildWidget() override;

private:
	int32 DroneCount = 0;
	int32 CurrentHighlightAgentID = -1;

	// 调试 toggle 状态（true = 开启，高亮）
	static constexpr int32 DebugToggleCount = 4;
	bool DebugToggleStates[4] = { true, true, true, true };

	// 根容器（纵向）
	TSharedPtr<SVerticalBox> RootVBox;

	// Camera 按钮行
	TSharedPtr<SHorizontalBox> CameraButtonRow;
	TSharedPtr<SButton> GlobalSlateButton;
	TArray<TSharedPtr<SButton>> DroneSlateButtons;

	// Debug 按钮行
	TSharedPtr<SHorizontalBox> DebugButtonRow;
	TArray<TSharedPtr<SButton>> DebugSlateButtons;

	// 按钮样式
	FButtonStyle NormalStyle;
	FButtonStyle HighlightedStyle;

	// 面板背景 Brush
	FSlateBrush PanelBgBrush;

	// 重建 Camera 按钮行
	void BuildContent();

	// 重建 Debug 按钮行
	void BuildDebugContent();

	// 更新 Camera 按钮高亮
	void UpdateHighlight();

	// 更新 Debug 按钮高亮
	void UpdateDebugHighlight();
};
