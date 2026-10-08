#include "CooperationPanel.h"
#include "CooperationViewData.h"
#include "SCooperationMap.h"
#include "../MultiAgent/CooperationGameMode.h"
#include "../Core/UAVPlayerController.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "Engine/World.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SExpandableArea.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SSlider.h"
#include "Widgets/Notifications/SProgressBar.h"
#include "Widgets/Text/STextBlock.h"
#include "Styling/CoreStyle.h"
#include "HAL/PlatformTime.h"

namespace
{
const FLinearColor Muted(.6f,.7f,.8f);
const FLinearColor Green(.15f,.75f,.4f);
const FLinearColor Red(1,.25f,.25f);

class SCooperationDashboard : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SCooperationDashboard) {}
        SLATE_ARGUMENT(TWeakObjectPtr<UCooperationPanel>, Owner)
        SLATE_ARGUMENT(TSharedPtr<FCooperationViewData>, Data)
    SLATE_END_ARGS()

    void Construct(const FArguments& Args)
    {
        Owner=Args._Owner;Data=Args._Data;
        TSharedRef<SVerticalBox> Left=SNew(SVerticalBox);
        TSharedRef<SHorizontalBox> Tabs=SNew(SHorizontalBox);
        const TCHAR* Names[]={TEXT("机队"),TEXT("机场"),TEXT("地块")};
        for(int32 I=0;I<3;++I) Tabs->AddSlot().FillWidth(1).Padding(2)
            [Button(Names[I],[this,I](){Tab=I;BuildCards();})];
        Left->AddSlot().AutoHeight()[Tabs];
        Left->AddSlot().FillHeight(1)[SNew(SScrollBox)+SScrollBox::Slot()[SAssignNew(Cards,SVerticalBox)]];
        Left->AddSlot().AutoHeight().Padding(0,8)[SNew(SExpandableArea).InitiallyCollapsed(true)
            .HeaderContent()[Text(TEXT("演示与测试"),11,Muted)]
            .BodyContent()[BuildDemoControls()]];
        TSharedRef<SVerticalBox> Right=SNew(SVerticalBox);
        Right->AddSlot().AutoHeight()[Button(TEXT("俯视态势图 · 展开 / 收起"),[this](){bLargeMap=!bLargeMap;})];
        Right->AddSlot().AutoHeight().Padding(0,6)
            [SNew(SBox).HeightOverride_Lambda([this](){return bLargeMap ? 380.f : 220.f;})
                [SNew(SCooperationMap).Data(Data).OnSelect([this](ECooperationObject Type,int32 ID)
                    {if(Owner.IsValid()) Owner->SelectObject(Type,ID);})]];
        Right->AddSlot().AutoHeight()[Text(TEXT("绿色面：实际覆盖   虚线：所选机规划航线"),9,Muted)];
        Right->AddSlot().AutoHeight().Padding(0,10)[Text(TEXT("对象详情"),12)];
        Right->AddSlot().AutoHeight()[SAssignNew(Details,SVerticalBox)];
        Right->AddSlot().AutoHeight().Padding(0,10)[Text(TEXT("最近状态变化"),12)];
        Right->AddSlot().AutoHeight()[DynamicText([this](){return FString::Join(Data->Events,TEXT("\n"));},10,Muted)];
        ChildSlot[SNew(SOverlay).Visibility(EVisibility::SelfHitTestInvisible)
            +SOverlay::Slot()[SNew(SCooperationMap).Data(Data)
                .WorldController(Owner.IsValid() ? Owner->GetOwningPlayer() : nullptr)
                .OnSelect([this](ECooperationObject Type,int32 ID){if(Owner.IsValid()) Owner->SelectObject(Type,ID);})]
            +SOverlay::Slot()[SNew(SVerticalBox).Visibility(EVisibility::SelfHitTestInvisible)
                +SVerticalBox::Slot().AutoHeight().Padding(12,12,12,8)[Surface(BuildHeader())]
                +SVerticalBox::Slot().FillHeight(1).Padding(12,0,12,110)
                [SNew(SHorizontalBox).Visibility(EVisibility::SelfHitTestInvisible)
                    +SHorizontalBox::Slot().AutoWidth()[SNew(SBox).WidthOverride_Lambda([this](){return PanelWidth;})[Surface(Left)]]
                    +SHorizontalBox::Slot().FillWidth(1)[SNew(SBox).Visibility(EVisibility::HitTestInvisible)]
                    +SHorizontalBox::Slot().AutoWidth()[SNew(SBox).WidthOverride_Lambda([this](){return PanelWidth;})
                        [Surface(SNew(SScrollBox)+SScrollBox::Slot()[Right])]]]]];
        BuildCards();BuildDetails();
    }

    virtual void Tick(const FGeometry& Geometry,double Now,float Delta) override
    {
        SCompoundWidget::Tick(Geometry,Now,Delta);
        PanelWidth=Geometry.GetLocalSize().X<1200 ? 270.f : 320.f;
        const double RealNow=FPlatformTime::Seconds();
        if(RealNow-LastRefresh<.2) return;
        LastRefresh=RealNow;
        if(Owner.IsValid()) Owner->RefreshView();
        FString Key=FString::FromInt(Tab)+TEXT(":")+FString::FromInt(Data->Preset);
        for(const auto& A:Data->Agents) Key+=FString::Printf(TEXT("A%d"),A.State.AgentID);
        for(const auto& A:Data->Airports) Key+=FString::Printf(TEXT("S%d"),A.Config.AirportID);
        for(const auto& P:Data->Plots) Key+=FString::Printf(TEXT("P%d"),P.ID);
        if(Key!=CardKey) {CardKey=Key;BuildCards();}
        if(Selection!=Data->Selection || SelectedID!=Data->SelectedID)
        {Selection=Data->Selection;SelectedID=Data->SelectedID;BuildDetails();}
    }

private:
    TWeakObjectPtr<UCooperationPanel> Owner;
    TSharedPtr<FCooperationViewData> Data;
    TSharedPtr<SVerticalBox> Cards,Details;
    int32 Tab=0,SelectedID=INDEX_NONE;
    ECooperationObject Selection=ECooperationObject::None;
    FString CardKey;
    double LastRefresh=0;
    float PanelWidth=320;
    bool bLargeMap=false;

    TSharedRef<SWidget> Text(const FString& Value,int32 Size=11,FLinearColor Color=FLinearColor::White)
    {return SNew(STextBlock).Text(FText::FromString(Value)).Font(FCoreStyle::GetDefaultFontStyle("Regular",Size+6)).ColorAndOpacity(Color).AutoWrapText(true);}
    TSharedRef<SWidget> DynamicText(TFunction<FString()> Value,int32 Size=11,FLinearColor Color=FLinearColor::White)
    {return SNew(STextBlock).Text_Lambda([Value](){return FText::FromString(Value());})
        .Font(FCoreStyle::GetDefaultFontStyle("Regular",Size+6)).ColorAndOpacity(Color).AutoWrapText(true);}
    TSharedRef<SWidget> Button(const FString& Label,TFunction<void()> Action,TFunction<bool()> Enabled=[](){return true;})
    {return SNew(SButton).ContentPadding(FMargin(8,5)).IsEnabled_Lambda([Enabled](){return Enabled();})
        .OnClicked_Lambda([Action](){Action();return FReply::Handled();})
        [SNew(STextBlock).Text(FText::FromString(Label)).Font(FCoreStyle::GetDefaultFontStyle("Regular",16))];}
    TSharedRef<SWidget> Surface(TSharedRef<SWidget> Content)
    {return SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
        .BorderBackgroundColor(FLinearColor(.025f,.04f,.065f,.94f)).Padding(12)[Content];}
    TSharedRef<SWidget> Progress(TFunction<float()> Value,TFunction<FLinearColor()> Color)
    {return SNew(SBox).HeightOverride(7)[SNew(SProgressBar).Percent_Lambda([Value](){return TOptional<float>(Value());})
        .FillColorAndOpacity_Lambda([Color](){return FSlateColor(Color());})];}
    const FCooperationAgentView* Agent(int32 ID) const
    {return Data->Agents.FindByPredicate([ID](const auto& A){return A.State.AgentID==ID;});}
    const FSupplyAirportState* Airport(int32 ID) const
    {return Data->Airports.FindByPredicate([ID](const auto& A){return A.Config.AirportID==ID;});}
    const FCooperationPlotView* Plot(int32 ID) const
    {return Data->Plots.FindByPredicate([ID](const auto& P){return P.ID==ID;});}
    void Command(int32 ID) {if(Owner.IsValid()) Owner->RunCommand(ID);}
    void AgricultureCommand(int32 ID) {if(Owner.IsValid()) Owner->RunAgricultureCommand(ID);}

    TSharedRef<SWidget> BuildHeader()
    {
        TSharedRef<SVerticalBox> Header=SNew(SVerticalBox);
        Header->AddSlot().AutoHeight()[DynamicText([this]()
        {
            const FString Status=!Data->bEnabled ? TEXT("农业预设未启用") : Data->bFinished ? TEXT("作业与返场完成") : Data->bPaused ? TEXT("已暂停") : TEXT("运行中");
            return FString::Printf(TEXT("农业协同作业   |   %s   |   %02d:%02d   |   覆盖 %.1f%%   |   地块 %d/%d   |   告警 %d"),
                *Status,int32(Data->Seconds)/60,int32(Data->Seconds)%60,Data->Progress()*100,Data->CompletedPlots,Data->Plots.Num(),Data->Alerts);
        },13)];
        Header->AddSlot().AutoHeight().Padding(0,7)[Progress([this](){return Data->Progress();},[](){return Green;})];
        TSharedRef<SHorizontalBox> Controls=SNew(SHorizontalBox);
        Controls->AddSlot().AutoWidth().Padding(2)[Button(TEXT("开始 / 继续"),[this](){Command(0);})];
        Controls->AddSlot().AutoWidth().Padding(2)[Button(TEXT("暂停"),[this](){Command(1);})];
        Controls->AddSlot().AutoWidth().Padding(2)[Button(TEXT("重置"),[this](){Command(2);})];
        Controls->AddSlot().AutoWidth().Padding(2)[Button(TEXT("全景"),[this](){Command(12);})];
        Controls->AddSlot().AutoWidth().Padding(2)[Button(TEXT("俯视"),[this](){Command(13);})];
        Controls->AddSlot().FillWidth(1).VAlign(VAlign_Center).Padding(12,0)[SNew(SSlider)
            .MinValue(.25f).MaxValue(8).StepSize(.25f).MouseUsesStep(true)
            .Value_Lambda([this](){return Data->Speed;})
            .OnValueChanged_Lambda([this](float Value){if(Owner.IsValid() && IsValid(Owner->Manager)) {Owner->Manager->SetSlomo(Value);Owner->RefreshView();}})];
        Controls->AddSlot().AutoWidth().VAlign(VAlign_Center)[DynamicText([this](){return FString::Printf(TEXT("%.2f×"),Data->Speed);})];
        Controls->AddSlot().AutoWidth().Padding(8,0)[Button(TEXT("恢复 1×"),[this](){if(Owner.IsValid() && IsValid(Owner->Manager)){Owner->Manager->SetSlomo(1);Owner->RefreshView();}})];
        Header->AddSlot().AutoHeight()[Controls];
        Header->AddSlot().AutoHeight().Padding(0,5,0,0)[DynamicText([this]()
        {return Data->MinSeparationMetres<0 ? TEXT("当前机间距：—") : FString::Printf(TEXT("当前最小机间距 %.1f m / 安全阈值 %.1f m   ·   已覆盖 %.0f / %.0f m²"),Data->MinSeparationMetres,Data->SafetyMetres,Data->Covered,Data->Area);},10,Muted)];
        return Header;
    }

    TSharedRef<SWidget> BuildDemoControls()
    {
        TSharedRef<SVerticalBox> Box=SNew(SVerticalBox);
        const TCHAR* Names[]={TEXT("协同植保"),TEXT("共享补给"),TEXT("优先级调度"),TEXT("故障恢复")};
        for(int32 I=0;I<4;++I) Box->AddSlot().AutoHeight().Padding(0,2)[Button(Names[I],[this,I]()
            {if(Owner.IsValid() && IsValid(Owner->Manager)){Owner->Manager->SelectPreset(I);Owner->RefreshView();}})];
        Box->AddSlot().AutoHeight().Padding(0,4)[Button(TEXT("插入紧急地块"),[this](){Command(7);})];
        Box->AddSlot().AutoHeight()[DynamicText([this]()
        {const TCHAR* Labels[]={TEXT("协同植保"),TEXT("共享补给"),TEXT("优先级调度"),TEXT("故障恢复")};return FString(TEXT("当前预设："))+Labels[FMath::Clamp(Data->Preset,0,3)];},10,Muted)];
        return Box;
    }

    static FString ObjectID(int32 ID) {return ID==INDEX_NONE ? TEXT("—") : FString::FromInt(ID);}
    void AddCard(ECooperationObject Type,int32 ID,TSharedRef<SWidget> Content);
    void BuildCards();
    void BuildDetails();
};
}

void SCooperationDashboard::AddCard(ECooperationObject Type,int32 ID,TSharedRef<SWidget> Content)
{
    Cards->AddSlot().AutoHeight().Padding(0,4)[SNew(SButton)
        .ButtonColorAndOpacity_Lambda([this,Type,ID](){return Data->Selection==Type && Data->SelectedID==ID ? FLinearColor(.15f,.4f,.65f) : FLinearColor(.1f,.15f,.22f);})
        .ContentPadding(10).OnClicked_Lambda([this,Type,ID](){if(Owner.IsValid()) Owner->SelectObject(Type,ID);return FReply::Handled();})[Content]];
}

void SCooperationDashboard::BuildCards()
{
    Cards->ClearChildren();
    if(Tab==0) for(const auto& View:Data->Agents)
    {
        const int32 ID=View.State.AgentID;
        TSharedRef<SVerticalBox> Card=SNew(SVerticalBox);
        Card->AddSlot().AutoHeight()[DynamicText([this,ID](){const auto* A=Agent(ID);return A ? FString::Printf(TEXT("UAV %d   ·   %s"),ID,*FCooperationViewData::PhaseName(A->State.Phase)) : TEXT("—");},12,FCooperationViewData::AgentColor(ID))];
        Card->AddSlot().AutoHeight().Padding(0,6)[DynamicText([this,ID](){const auto* A=Agent(ID);return A ? FString::Printf(TEXT("电量 %.0f%%   药液 %.1f L"),A->State.Battery*100,A->State.LiquidLitres) : TEXT("—");},10)];
        Card->AddSlot().AutoHeight()[Progress([this,ID](){const auto* A=Agent(ID);return A ? A->State.Battery : 0;},[this,ID](){const auto* A=Agent(ID);return A && A->State.Battery>Data->BatteryReserve ? Green : Red;})];
        Card->AddSlot().AutoHeight().Padding(0,4)[Progress([this,ID](){const auto* A=Agent(ID);return A && Data->TankCapacity>0 ? A->State.LiquidLitres/Data->TankCapacity : 0;},[](){return FLinearColor(.15f,.65f,1);})];
        Card->AddSlot().AutoHeight()[DynamicText([this,ID](){const auto* A=Agent(ID);return A ? FString::Printf(TEXT("地块 %s   机场 %s"),*ObjectID(A->State.PlotID),*ObjectID(A->State.AirportID)) : TEXT("—");},10,Muted)];
        AddCard(ECooperationObject::Agent,ID,Card);
    }
    else if(Tab==1) for(const auto& State:Data->Airports)
    {
        const int32 ID=State.Config.AirportID;
        TSharedRef<SVerticalBox> Card=SNew(SVerticalBox);
        Card->AddSlot().AutoHeight()[DynamicText([this,ID](){const auto* A=Airport(ID);return A ? FString::Printf(TEXT("机场 %d   ·   %s"),ID,*FCooperationViewData::AirportStatus(*A)) : TEXT("—");},12)];
        Card->AddSlot().AutoHeight().Padding(0,6)[DynamicText([this,ID](){const auto* A=Airport(ID);return A ? FString::Printf(TEXT("占用 UAV %s   排队 %d\n清水 %.0f L   原液 %.1f L"),*ObjectID(A->OccupantID),A->Queue.Num(),A->Config.WaterLitres,A->Config.ConcentrateLitres) : TEXT("—");},10,Muted)];
        AddCard(ECooperationObject::Airport,ID,Card);
    }
    else if(Tab==2) for(const auto& State:Data->Plots)
    {
        const int32 ID=State.ID;
        TSharedRef<SVerticalBox> Card=SNew(SVerticalBox);
        Card->AddSlot().AutoHeight()[DynamicText([this,ID](){const auto* P=Plot(ID);return P ? FString::Printf(TEXT("地块 %d   ·   %s"),ID,P->bFailed ? TEXT("失败") : P->bCompleted ? TEXT("完成") : P->AgentIDs.IsEmpty() ? TEXT("待分配") : TEXT("作业中")) : TEXT("—");},12)];
        Card->AddSlot().AutoHeight().Padding(0,6)[Progress([this,ID](){const auto* P=Plot(ID);return P ? P->Progress() : 0;},[](){return Green;})];
        Card->AddSlot().AutoHeight()[DynamicText([this,ID](){const auto* P=Plot(ID);return P ? FString::Printf(TEXT("%.1f%%   %.0f / %.0f m²\n执行 UAV %s"),P->Progress()*100,P->Covered,P->Area,*P->AgentLabel()) : TEXT("—");},10,Muted)];
        AddCard(ECooperationObject::Plot,ID,Card);
    }
    if(Cards->NumSlots()==0) Cards->AddSlot().AutoHeight()[Text(TEXT("暂无对象"),11,Muted)];
}

void SCooperationDashboard::BuildDetails()
{
    Details->ClearChildren();
    const int32 ID=Data->SelectedID;
    if(Data->Selection==ECooperationObject::Agent)
    {
        Details->AddSlot().AutoHeight()[DynamicText([this,ID](){const auto* A=Agent(ID);return A ? FString::Printf(TEXT("UAV %d · %s\n速度 %.1f m/s   高度 %.1f m（世界）\n电量 %.0f%%   药液 %.1f / %.0f L\n地块 %s   机场 %s\n阶段经过 %.1f s"),ID,*FCooperationViewData::PhaseName(A->State.Phase),A->SpeedMetres,A->Position.Z/100,A->State.Battery*100,A->State.LiquidLitres,Data->TankCapacity,*ObjectID(A->State.PlotID),*ObjectID(A->State.AirportID),A->State.PhaseSeconds) : TEXT("对象已移除");})];
        Details->AddSlot().AutoHeight().Padding(0,6)[Button(TEXT("跟随视角"),[this](){if(Owner.IsValid()) Owner->FollowSelected();})];
        Details->AddSlot().AutoHeight()[Button(TEXT("请求返航补给"),[this](){AgricultureCommand(0);},[this,ID]()
        {
            const auto* A=Agent(ID);if(!A) return false;
            const auto Phase=A->State.Phase;
            return Phase==EAgriculturePhase::Idle || Phase==EAgriculturePhase::TakingOff || Phase==EAgriculturePhase::Transit ||
                Phase==EAgriculturePhase::Spraying || Phase==EAgriculturePhase::Resuming;
        })];
        Details->AddSlot().AutoHeight().Padding(0,6)[SNew(SExpandableArea).InitiallyCollapsed(true)
            .HeaderContent()[Text(TEXT("故障注入"),10,Muted)]
            .BodyContent()[Button(TEXT("故障所选飞机"),[this](){AgricultureCommand(3);})]];
    }
    else if(Data->Selection==ECooperationObject::Airport)
    {
        Details->AddSlot().AutoHeight()[DynamicText([this,ID]()
        {
            const auto* A=Airport(ID);if(!A) return FString(TEXT("对象已移除"));
            TArray<FString> Queue;for(int32 AgentID:A->Queue) Queue.Add(FString::Printf(TEXT("U%d"),AgentID));
            return FString::Printf(TEXT("机场 %d · %s\n占用 UAV %s\n排队顺序：%s\n清水 %.0f L   原液 %.1f L\n废液 %.1f / %.0f L"),ID,*FCooperationViewData::AirportStatus(*A),*ObjectID(A->OccupantID),Queue.IsEmpty() ? TEXT("无") : *FString::Join(Queue,TEXT(" → ")),A->Config.WaterLitres,A->Config.ConcentrateLitres,A->WasteLitres,A->Config.WasteCapacityLitres);
        })];
        Details->AddSlot().AutoHeight().Padding(0,6)[Button(TEXT("机场视角"),[this](){AgricultureCommand(5);})];
        Details->AddSlot().AutoHeight()[SNew(SExpandableArea).InitiallyCollapsed(true)
            .HeaderContent()[Text(TEXT("机场状态注入"),10,Muted)]
            .BodyContent()[SNew(SVerticalBox)
                +SVerticalBox::Slot().AutoHeight()[Button(TEXT("切换启用 / 停用"),[this](){AgricultureCommand(1);})]
                +SVerticalBox::Slot().AutoHeight().Padding(0,4)[Button(TEXT("清空清水库存"),[this](){AgricultureCommand(2);})]]];
    }
    else if(Data->Selection==ECooperationObject::Plot)
        Details->AddSlot().AutoHeight()[DynamicText([this,ID](){const auto* P=Plot(ID);return P ? FString::Printf(TEXT("地块 %d\n已覆盖 %.0f / %.0f m²（%.1f%%）\n累计施药 %.1f L\n执行 UAV %s\n%s"),ID,P->Covered,P->Area,P->Progress()*100,P->AppliedLitres,*P->AgentLabel(),P->bFailed ? TEXT("任务失败") : P->bCompleted ? TEXT("覆盖完成") : TEXT("尚未完成")) : TEXT("对象已移除");})];
    else Details->AddSlot().AutoHeight()[Text(TEXT("点击卡片或态势图中的飞机、机场、地块查看详情。"),11,Muted)];
}

TSharedRef<SWidget> UCooperationPanel::RebuildWidget()
{
    if(!View) View=MakeShared<FCooperationViewData>();
    RefreshView();
    return SNew(SCooperationDashboard).Visibility(EVisibility::SelfHitTestInvisible).Owner(this).Data(View);
}

void UCooperationPanel::RefreshView()
{
    if(!IsValid(Manager) || !View) return;
    View->Capture(*Manager);
    if(!CoverageMesh)
    {
        FActorSpawnParameters Parameters;
        Parameters.Owner=Manager;
        Parameters.ObjectFlags|=RF_Transient;
        CoverageActor=GetWorld()->SpawnActor<AActor>(Parameters);
        if(!CoverageActor) return;
        CoverageActor->Tags.Add(TEXT("CooperationCoverage"));
        CoverageMesh=NewObject<UInstancedStaticMeshComponent>(CoverageActor);
        CoverageMesh->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));
        CoverageMesh->SetMaterial(0,LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/Environment/Cooperation/MI_Green.MI_Green")));
        CoverageMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        CoverageMesh->SetCastShadow(false);
        CoverageMesh->SetCanEverAffectNavigation(false);
        CoverageActor->SetRootComponent(CoverageMesh);
        CoverageActor->AddInstanceComponent(CoverageMesh);
        CoverageMesh->RegisterComponent();
    }
    TArray<FTransform> NewTransforms;
    for(const auto& Plot:View->Plots) for(const FBox& Box:Plot.Coverage)
        NewTransforms.Emplace(FQuat::Identity,Box.GetCenter(),Box.GetSize()/100);
    if(NewTransforms.Num()!=CoverageTransforms.Num())
    {
        CoverageMesh->ClearInstances();
        if(!NewTransforms.IsEmpty()) CoverageMesh->AddInstances(NewTransforms,false,true);
    }
    else
    {
        bool bChanged=false;
        for(int32 I=0;I<NewTransforms.Num();++I) if(!NewTransforms[I].Equals(CoverageTransforms[I]))
        {CoverageMesh->UpdateInstanceTransform(I,NewTransforms[I],true,false,true);bChanged=true;}
        if(bChanged) CoverageMesh->MarkRenderStateDirty();
    }
    CoverageTransforms=MoveTemp(NewTransforms);
}

void UCooperationPanel::SelectObject(ECooperationObject Type,int32 ID)
{if(View) {View->Selection=Type;View->SelectedID=ID;}}

void UCooperationPanel::RunCommand(int32 Command)
{if(IsValid(Manager)) {Manager->DemoCommand(Command);RefreshView();}}

void UCooperationPanel::RunAgricultureCommand(int32 Command)
{
    if(!IsValid(Manager) || !View || View->SelectedID==INDEX_NONE) return;
    const bool bAgentCommand=Command==0 || Command==3;
    if(View->Selection!=(bAgentCommand ? ECooperationObject::Agent : ECooperationObject::Airport)) return;
    Manager->AgricultureCommand(Command,View->SelectedID);RefreshView();
}

void UCooperationPanel::FollowSelected()
{
    if(View && View->Selection==ECooperationObject::Agent)
        if(auto* PC=Cast<AUAVPlayerController>(GetOwningPlayer())) PC->SwitchView(View->SelectedID);
}

void UCooperationPanel::NativeDestruct()
{
    if(IsValid(CoverageActor)) CoverageActor->Destroy();
    CoverageActor=nullptr;CoverageMesh=nullptr;CoverageTransforms.Empty();
    Super::NativeDestruct();
}
