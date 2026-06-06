// Copyright Epic Games, Inc. All Rights Reserved.

#include "CameraSwitcherWidget.h"
#include "Components/VerticalBox.h"
#include "Components/VerticalBoxSlot.h"
#include "Components/Button.h"
#include "Components/TextBlock.h"

DEFINE_LOG_CATEGORY_STATIC(LogCameraSwitcher, Log, All);

void UCameraSwitcherWidget::NativeConstruct()
{
	Super::NativeConstruct();

	// 创建根垂直布局
	ButtonList = NewObject<UVerticalBox>(this);
	if (!ButtonList)
	{
		return;
	}

	// 创建全局视角按钮
	GlobalButton = CreateButton(TEXT("Global View"), -1);

	// 默认高亮全局
	HighlightCurrentView(-1);

	UE_LOG(LogCameraSwitcher, Log, TEXT("CameraSwitcherWidget constructed"));
}

UButton* UCameraSwitcherWidget::CreateButton(const FString& Label, int32 AgentID)
{
	UButton* Button = NewObject<UButton>(this);
	if (!Button)
	{
		return nullptr;
	}

	UTextBlock* Text = NewObject<UTextBlock>(this);
	if (Text)
	{
		Text->SetText(FText::FromString(Label));
		Text->SetColorAndOpacity(FSlateColor(FLinearColor::White));
	}

	// 设置按钮样式
	FButtonStyle Style;
	FLinearColor NormalColor(0.15f, 0.15f, 0.15f, 0.6f);
	FLinearColor HoveredColor(0.25f, 0.25f, 0.25f, 0.7f);
	FLinearColor PressedColor(0.1f, 0.1f, 0.1f, 0.8f);

	FSlateBrush NormalBrush;
	NormalBrush.TintColor = FSlateColor(NormalColor);
	Style.Normal = NormalBrush;

	FSlateBrush HoveredBrush;
	HoveredBrush.TintColor = FSlateColor(HoveredColor);
	Style.Hovered = HoveredBrush;

	FSlateBrush PressedBrush;
	PressedBrush.TintColor = FSlateColor(PressedColor);
	Style.Pressed = PressedBrush;

	Button->SetStyle(Style);

	// 添加到根容器
	if (ButtonList)
	{
		UVerticalBoxSlot* ButtonSlot = ButtonList->AddChildToVerticalBox(Button);
		if (ButtonSlot)
		{
			ButtonSlot->SetPadding(FMargin(0, 2));
			ButtonSlot->SetHorizontalAlignment(HAlign_Left);
		}
	}

	// 添加文本子组件
	if (Text)
	{
		Button->AddChild(Text);
	}

	// 绑定点击事件
	if (AgentID == -1)
	{
		Button->OnClicked.AddDynamic(this, &UCameraSwitcherWidget::OnGlobalClicked);
	}
	else if (AgentID >= 0 && AgentID < 8)
	{
		switch (AgentID)
		{
		case 0: Button->OnClicked.AddDynamic(this, &UCameraSwitcherWidget::OnDrone0Clicked); break;
		case 1: Button->OnClicked.AddDynamic(this, &UCameraSwitcherWidget::OnDrone1Clicked); break;
		case 2: Button->OnClicked.AddDynamic(this, &UCameraSwitcherWidget::OnDrone2Clicked); break;
		case 3: Button->OnClicked.AddDynamic(this, &UCameraSwitcherWidget::OnDrone3Clicked); break;
		case 4: Button->OnClicked.AddDynamic(this, &UCameraSwitcherWidget::OnDrone4Clicked); break;
		case 5: Button->OnClicked.AddDynamic(this, &UCameraSwitcherWidget::OnDrone5Clicked); break;
		case 6: Button->OnClicked.AddDynamic(this, &UCameraSwitcherWidget::OnDrone6Clicked); break;
		case 7: Button->OnClicked.AddDynamic(this, &UCameraSwitcherWidget::OnDrone7Clicked); break;
		}
		DroneButtons.Add(Button);
	}

	return Button;
}

void UCameraSwitcherWidget::OnGlobalClicked()
{
	OnViewChanged.Broadcast(-1);
}

void UCameraSwitcherWidget::OnDrone0Clicked() { OnViewChanged.Broadcast(0); }
void UCameraSwitcherWidget::OnDrone1Clicked() { OnViewChanged.Broadcast(1); }
void UCameraSwitcherWidget::OnDrone2Clicked() { OnViewChanged.Broadcast(2); }
void UCameraSwitcherWidget::OnDrone3Clicked() { OnViewChanged.Broadcast(3); }
void UCameraSwitcherWidget::OnDrone4Clicked() { OnViewChanged.Broadcast(4); }
void UCameraSwitcherWidget::OnDrone5Clicked() { OnViewChanged.Broadcast(5); }
void UCameraSwitcherWidget::OnDrone6Clicked() { OnViewChanged.Broadcast(6); }
void UCameraSwitcherWidget::OnDrone7Clicked() { OnViewChanged.Broadcast(7); }

void UCameraSwitcherWidget::RefreshDroneList(int32 Count)
{
	if (Count == DroneCount)
	{
		return;
	}

	DroneCount = Count;

	// 清除现有无人机按钮
	for (UButton* Btn : DroneButtons)
	{
		if (Btn && ButtonList)
		{
			ButtonList->RemoveChild(Btn);
		}
	}
	DroneButtons.Empty();

	// 重建无人机按钮
	for (int32 i = 0; i < Count; ++i)
	{
		FString Label = FString::Printf(TEXT("UAV #%d"), i);
		CreateButton(Label, i);
	}

	// 恢复高亮状态
	UpdateHighlight();

	UE_LOG(LogCameraSwitcher, Log, TEXT("RefreshDroneList: %d drones"), Count);
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
	SetButtonHighlight(GlobalButton, CurrentHighlightAgentID == -1);

	for (int32 i = 0; i < DroneButtons.Num(); ++i)
	{
		SetButtonHighlight(DroneButtons[i], i == CurrentHighlightAgentID);
	}
}

void UCameraSwitcherWidget::SetButtonHighlight(UButton* Button, bool bHighlighted)
{
	if (!Button)
	{
		return;
	}

	FButtonStyle Style;
	FLinearColor NormalColor, HoveredColor, PressedColor;

	if (bHighlighted)
	{
		// 高亮状态：蓝色
		NormalColor = FLinearColor(0.0f, 0.5f, 1.0f, 0.9f);
		HoveredColor = FLinearColor(0.1f, 0.6f, 1.0f, 0.95f);
		PressedColor = FLinearColor(0.0f, 0.4f, 0.9f, 1.0f);
	}
	else
	{
		// 普通状态：灰色
		NormalColor = FLinearColor(0.15f, 0.15f, 0.15f, 0.6f);
		HoveredColor = FLinearColor(0.25f, 0.25f, 0.25f, 0.7f);
		PressedColor = FLinearColor(0.1f, 0.1f, 0.1f, 0.8f);
	}

	FSlateBrush NormalBrush;
	NormalBrush.TintColor = FSlateColor(NormalColor);
	Style.Normal = NormalBrush;

	FSlateBrush HoveredBrush;
	HoveredBrush.TintColor = FSlateColor(HoveredColor);
	Style.Hovered = HoveredBrush;

	FSlateBrush PressedBrush;
	PressedBrush.TintColor = FSlateColor(PressedColor);
	Style.Pressed = PressedBrush;

	Button->SetStyle(Style);
}
