#include "CooperationPanel.h"
#include "../MultiAgent/CooperationGameMode.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Styling/CoreStyle.h"

TSharedRef<SWidget> UCooperationPanel::RebuildWidget()
{
	TSharedRef<SVerticalBox> Box = SNew(SVerticalBox);
	Box->AddSlot().AutoHeight().Padding(6)[SNew(STextBlock).Text(FText::FromString(TEXT("X SERIES / 农业协同作业场")))];
	const TCHAR* Labels[] = {TEXT("协同植保"), TEXT("最近机场补给"), TEXT("优先级调度"), TEXT("故障恢复")};
	for (int32 I=0; I<4; ++I)
		Box->AddSlot().AutoHeight().Padding(3)[SNew(SButton).Text(FText::FromString(Labels[I]))
		.OnClicked_Lambda([this,I](){ if(Manager) Manager->SelectPreset(I); return FReply::Handled(); })];
	const TCHAR* Actions[] = {TEXT("开始 / 继续"),TEXT("暂停"),TEXT("重置"),TEXT("返航 UAV 0"),TEXT("返航 UAV 1"),TEXT("返航 UAV 2"),TEXT("返航 UAV 3"),TEXT("插入紧急地块"),TEXT("故障 UAV 0"),TEXT("故障 UAV 1"),TEXT("故障 UAV 2"),TEXT("故障 UAV 3"),TEXT("全景"),TEXT("俯视")};
	for(int32 I=0; I<14; I+=2)
	{
		TSharedRef<SHorizontalBox> Row = SNew(SHorizontalBox);
		for(int32 J=I;J<FMath::Min(I+2,14);++J)
			Row->AddSlot().FillWidth(1).Padding(3)[SNew(SButton).Text(FText::FromString(Actions[J]))
			.OnClicked_Lambda([this,J](){if(Manager) Manager->DemoCommand(J); return FReply::Handled();})];
		Box->AddSlot().AutoHeight()[Row];
	}
	for(int32 I=0;I<4;++I)
	{
		TSharedRef<SHorizontalBox> Row=SNew(SHorizontalBox);
		for(int32 J=1;J<=2;++J)
			Row->AddSlot().FillWidth(1).Padding(3)[SNew(SButton).Text(FText::FromString(J==1 ? FString::Printf(TEXT("启停机场 %d"),I) : FString::Printf(TEXT("机场 %d 缺水"),I)))
			.OnClicked_Lambda([this,I,J](){if(Manager) Manager->AgricultureCommand(J,I);return FReply::Handled();})];
		Box->AddSlot().AutoHeight()[Row];
	}
	TSharedRef<SHorizontalBox> Airports=SNew(SHorizontalBox);
	for(int32 I=0;I<4;++I)
		Airports->AddSlot().FillWidth(1).Padding(2)[SNew(SButton).Text(FText::FromString(FString::Printf(TEXT("机场%d视角"),I)))
		.OnClicked_Lambda([this,I](){if(Manager) Manager->AgricultureCommand(5,I);return FReply::Handled();})];
	Box->AddSlot().AutoHeight()[Airports];
	Box->AddSlot().AutoHeight().Padding(6)[SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Regular",12)).AutoWrapText(true)
		.Text_Lambda([this](){return Manager ? Manager->GetDemoStatus() : FText::GetEmpty();})];
	return SNew(SOverlay)+SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Top).Padding(16,16)
	[SNew(SBox).WidthOverride(340).MaxDesiredHeight(900)[SNew(SBorder).Padding(8).BorderBackgroundColor(FLinearColor(0.025f,0.035f,0.06f,0.94f))[SNew(SScrollBox)+SScrollBox::Slot()[Box]]]];
}
