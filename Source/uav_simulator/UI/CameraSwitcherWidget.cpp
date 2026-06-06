// Copyright Epic Games, Inc. All Rights Reserved.

#include "CameraSwitcherWidget.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Layout/SSpacer.h"
#include "Styling/CoreStyle.h"

DEFINE_LOG_CATEGORY_STATIC(LogCameraSwitcher, Log, All);

// 面板背景色
static const FLinearColor PanelBg(0.05f, 0.05f, 0.08f, 0.85f);
// 按钮普通态
static const FLinearColor BtnNormal(0.12f, 0.12f, 0.15f, 0.6f);
static const FLinearColor BtnHovered(0.18f, 0.20f, 0.25f, 0.75f);
static const FLinearColor BtnPressed(0.08f, 0.08f, 0.10f, 0.9f);
// 选中态
static const FLinearColor BtnActive(0.0f, 0.42f, 0.72f, 0.85f);
static const FLinearColor BtnActiveHovered(0.05f, 0.52f, 0.82f, 0.9f);
static const FLinearColor BtnActivePressed(0.0f, 0.35f, 0.65f, 1.0f);
// 分隔线
static const FLinearColor SeparatorColor(0.3f, 0.35f, 0.45f, 0.4f);

// Debug 按钮标签
static const TCHAR* DebugLabels[] = { TEXT("Vis"), TEXT("Obs"), TEXT("Path"), TEXT("Dbg") };

static FSlateBrush MakeColorBrush(FLinearColor Color)
{
	FSlateBrush Brush;
	Brush.TintColor = FSlateColor(Color);
	return Brush;
}

TSharedRef<SWidget> UCameraSwitcherWidget::RebuildWidget()
{
	// 初始化按钮样式
	NormalStyle = FButtonStyle();
	NormalStyle.Normal  = MakeColorBrush(BtnNormal);
	NormalStyle.Hovered = MakeColorBrush(BtnHovered);
	NormalStyle.Pressed = MakeColorBrush(BtnPressed);
	NormalStyle.NormalPadding = FMargin(4, 2);

	HighlightedStyle = FButtonStyle();
	HighlightedStyle.Normal  = MakeColorBrush(BtnActive);
	HighlightedStyle.Hovered = MakeColorBrush(BtnActiveHovered);
	HighlightedStyle.Pressed = MakeColorBrush(BtnActivePressed);
	HighlightedStyle.NormalPadding = FMargin(4, 2);

	// 面板背景 Brush
	PanelBgBrush = MakeColorBrush(PanelBg);

	// 创建根容器
	SAssignNew(RootVBox, SVerticalBox);
	SAssignNew(DebugButtonRow, SHorizontalBox);
	SAssignNew(CameraButtonRow, SHorizontalBox);

	// === Debug 面板 ===
	BuildDebugContent();

	RootVBox->AddSlot()
		.AutoHeight()
		.Padding(2, 0, 2, 1)
		[
			SNew(STextBlock)
			.Text(FText::FromString(TEXT("Debug")))
			.Font(FSlateFontInfo(FCoreStyle::GetDefaultFontStyle("Bold", 8)))
			.ColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.65f, 0.75f)))
		];

	RootVBox->AddSlot()
		.AutoHeight()
		.Padding(0, 0, 0, 2)
		[
			DebugButtonRow.ToSharedRef()
		];

	// 分隔线
	RootVBox->AddSlot()
		.AutoHeight()
		.Padding(0, 0, 0, 2)
		[
			SNew(SSeparator)
			.Thickness(0.5f)
			.ColorAndOpacity(FSlateColor(SeparatorColor))
		];

	// === Camera 面板 ===
	BuildContent();

	RootVBox->AddSlot()
		.AutoHeight()
		.Padding(2, 0, 2, 1)
		[
			SNew(STextBlock)
			.Text(FText::FromString(TEXT("Camera")))
			.Font(FSlateFontInfo(FCoreStyle::GetDefaultFontStyle("Bold", 8)))
			.ColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.65f, 0.75f)))
		];

	RootVBox->AddSlot()
		.AutoHeight()
		.Padding(0, 0, 0, 2)
		[
			SNew(SSeparator)
			.Thickness(0.5f)
			.ColorAndOpacity(FSlateColor(SeparatorColor))
		];

	RootVBox->AddSlot()
		.AutoHeight()
		[
			CameraButtonRow.ToSharedRef()
		];

	// 用 SOverlay 包裹面板，定位到左下角
	TSharedRef<SWidget> Panel = SNew(SBorder)
		.BorderImage(&PanelBgBrush)
		.BorderBackgroundColor(FLinearColor::White)
		.Padding(FMargin(4, 3))
		[
			RootVBox.ToSharedRef()
		];

	return SNew(SOverlay)
		+ SOverlay::Slot()
		.HAlign(HAlign_Left)
		.VAlign(VAlign_Bottom)
		.Padding(FMargin(20, 0, 0, 20))
		[
			Panel
		];
}

void UCameraSwitcherWidget::BuildDebugContent()
{
	DebugSlateButtons.Empty();

	if (!DebugButtonRow.IsValid())
	{
		return;
	}

	DebugButtonRow->ClearChildren();

	for (int32 i = 0; i < DebugToggleCount; ++i)
	{
		const int32 ToggleIndex = i;
		const bool bActive = DebugToggleStates[i];

		TSharedRef<SButton> Btn = SNew(SButton)
			.ButtonStyle(bActive ? &HighlightedStyle : &NormalStyle)
			.OnClicked_Lambda([this, ToggleIndex]() -> FReply
			{
				// 切换状态
				DebugToggleStates[ToggleIndex] = !DebugToggleStates[ToggleIndex];
				UpdateDebugHighlight();
				// 通知 PlayerController
				OnToggleDebug.Broadcast(ToggleIndex);
				return FReply::Handled();
			})
			[
				SNew(STextBlock)
				.Text(FText::FromString(DebugLabels[i]))
				.Font(FSlateFontInfo(FCoreStyle::GetDefaultFontStyle("Regular", 8)))
				.ColorAndOpacity(FSlateColor(FLinearColor::White))
			];
		DebugSlateButtons.Add(Btn);
		DebugButtonRow->AddSlot()
			.AutoWidth()
			.Padding(i == 0 ? FMargin(0, 0, 1, 0) : FMargin(1, 0, 0, 0))
			[
				Btn
			];
	}
}

void UCameraSwitcherWidget::BuildContent()
{
	GlobalSlateButton.Reset();
	DroneSlateButtons.Empty();

	if (!CameraButtonRow.IsValid())
	{
		return;
	}

	CameraButtonRow->ClearChildren();

	// 全局视角按钮
	{
		const bool bHL = (CurrentHighlightAgentID == -1);
		TSharedRef<SButton> Btn = SNew(SButton)
			.ButtonStyle(bHL ? &HighlightedStyle : &NormalStyle)
			.OnClicked_Lambda([this]() -> FReply
			{
				OnViewChanged.Broadcast(-1);
				return FReply::Handled();
			})
			[
				SNew(STextBlock)
				.Text(FText::FromString(TEXT("All")))
				.Font(FSlateFontInfo(FCoreStyle::GetDefaultFontStyle("Regular", 8)))
				.ColorAndOpacity(FSlateColor(FLinearColor::White))
			];
		GlobalSlateButton = Btn;
		CameraButtonRow->AddSlot()
			.AutoWidth()
			.Padding(0, 0, 1, 0)
			[
				Btn
			];
	}

	// 无人机按钮
	for (int32 i = 0; i < DroneCount; ++i)
	{
		const int32 AgentID = i;
		const FString Label = FString::Printf(TEXT("#%d"), AgentID);
		const bool bHL = (AgentID == CurrentHighlightAgentID);

		TSharedRef<SButton> Btn = SNew(SButton)
			.ButtonStyle(bHL ? &HighlightedStyle : &NormalStyle)
			.OnClicked_Lambda([this, AgentID]() -> FReply
			{
				OnViewChanged.Broadcast(AgentID);
				return FReply::Handled();
			})
			[
				SNew(STextBlock)
				.Text(FText::FromString(Label))
				.Font(FSlateFontInfo(FCoreStyle::GetDefaultFontStyle("Regular", 8)))
				.ColorAndOpacity(FSlateColor(FLinearColor::White))
			];
		DroneSlateButtons.Add(Btn);
		CameraButtonRow->AddSlot()
			.AutoWidth()
			.Padding(1, 0, 0, 0)
			[
				Btn
			];
	}
}

void UCameraSwitcherWidget::RefreshDroneList(int32 Count)
{
	if (Count == DroneCount)
	{
		return;
	}

	DroneCount = Count;
	BuildContent();

	UE_LOG(LogCameraSwitcher, Log, TEXT("RefreshDroneList: %d drones"), DroneCount);
}

void UCameraSwitcherWidget::HighlightCurrentView(int32 AgentID)
{
	if (CurrentHighlightAgentID == AgentID)
	{
		return;
	}

	CurrentHighlightAgentID = AgentID;
	UpdateHighlight();
}

void UCameraSwitcherWidget::UpdateHighlight()
{
	if (GlobalSlateButton.IsValid())
	{
		GlobalSlateButton->SetButtonStyle(
			(CurrentHighlightAgentID == -1) ? &HighlightedStyle : &NormalStyle);
	}

	for (int32 i = 0; i < DroneSlateButtons.Num(); ++i)
	{
		if (DroneSlateButtons[i].IsValid())
		{
			DroneSlateButtons[i]->SetButtonStyle(
				(i == CurrentHighlightAgentID) ? &HighlightedStyle : &NormalStyle);
		}
	}
}

void UCameraSwitcherWidget::UpdateDebugHighlight()
{
	for (int32 i = 0; i < DebugSlateButtons.Num(); ++i)
	{
		if (DebugSlateButtons[i].IsValid())
		{
			DebugSlateButtons[i]->SetButtonStyle(
				DebugToggleStates[i] ? &HighlightedStyle : &NormalStyle);
		}
	}
}
