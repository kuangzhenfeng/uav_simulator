#include "SCooperationMap.h"
#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"
#include "InputCoreTypes.h"
#include "Blueprint/WidgetLayoutLibrary.h"
#include "GameFramework/PlayerController.h"

void SCooperationMap::Construct(const FArguments& Args)
{
    Data = Args._Data;
    OnSelect = Args._OnSelect;
    WorldController = Args._WorldController;
    SetClipping(EWidgetClipping::ClipToBounds);
#if WITH_ACCESSIBILITY
    SetAccessibleBehavior(EAccessibleBehavior::Custom, FText::FromString(WorldController.IsValid() ? TEXT("三维作业态势") : TEXT("俯视作业态势")));
#endif
}

void SCooperationMap::Tick(const FGeometry& Geometry, double Now, float Delta)
{
    SLeafWidget::Tick(Geometry, Now, Delta);
    if (Data && (Data->Revision != LastRevision || Data->Selection != LastSelection || Data->SelectedID != LastSelectedID))
    {
        LastRevision = Data->Revision; LastSelection = Data->Selection; LastSelectedID = Data->SelectedID;
        Invalidate(EInvalidateWidgetReason::Paint);
    }
}

FVector2D SCooperationMap::Project(const FGeometry& Geometry, const FVector& Position) const
{
    if (WorldController.IsValid())
    {
        FVector2D Point;
        if (UWidgetLayoutLibrary::ProjectWorldLocationToWidgetPosition(WorldController.Get(), Position, Point, true)) return Point;
        return FVector2D(-100000, -100000);
    }
    const FVector2D Size = Geometry.GetLocalSize();
    const FVector Extent = Data->MapBounds.GetSize();
    const float Scale = FMath::Min(FMath::Max(1.f, float(Size.X)-32) / FMath::Max(1.f, float(Extent.X)),
        FMath::Max(1.f, float(Size.Y)-32) / FMath::Max(1.f, float(Extent.Y)));
    const FVector Offset = Position - Data->MapBounds.GetCenter();
    return Size/2 + FVector2D(Offset.X, Offset.Y)*Scale;
}

int32 SCooperationMap::OnPaint(const FPaintArgs&, const FGeometry& G, const FSlateRect&,
    FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle&, bool) const
{
    if (!Data) return Layer;
    const auto* Brush = FCoreStyle::Get().GetBrush("WhiteBrush");
    const auto Line = [&](const TArray<FVector2D>& Points, FLinearColor Color, float Width = 1.f)
    {
        for (const FVector2D& Point : Points) if (Point.X <= -100000 || Point.Y <= -100000) return;
        FSlateDrawElement::MakeLines(Out, Layer+2, G.ToPaintGeometry(), Points, ESlateDrawEffect::None, Color, true, Width);
    };
    const auto Label = [&](FVector2D At, const FString& Text, FLinearColor Color)
    {
        if (!WorldController.IsValid())
        {
            At.X = FMath::Clamp(At.X, 4., FMath::Max(4., G.GetLocalSize().X-65));
            At.Y = FMath::Clamp(At.Y, 4., FMath::Max(4., G.GetLocalSize().Y-20));
        }
        FSlateDrawElement::MakeText(Out, Layer+3, G.ToPaintGeometry(FVector2D(100,20), FSlateLayoutTransform(At)),
            Text, FCoreStyle::GetDefaultFontStyle("Regular", 12), ESlateDrawEffect::None, Color);
    };
    const auto Rect = [&](const FBox& Box, FLinearColor Color)
    {
        if (WorldController.IsValid()) return;
        const FVector2D A = Project(G, FVector(Box.Min.X, Box.Min.Y, 0));
        const FVector2D B = Project(G, FVector(Box.Max.X, Box.Max.Y, 0));
        FSlateDrawElement::MakeBox(Out, Layer+1, G.ToPaintGeometry(B-A, FSlateLayoutTransform(A)),
            Brush, ESlateDrawEffect::None, Color);
    };
    if (!WorldController.IsValid())
        FSlateDrawElement::MakeBox(Out, Layer, G.ToPaintGeometry(), Brush, ESlateDrawEffect::None, FLinearColor(.015f,.025f,.045f));
    for (const auto& Plot : Data->Plots)
    {
        Rect(Plot.Bounds, Plot.bFailed ? FLinearColor(.25f,.07f,.07f) : FLinearColor(.09f,.14f,.12f));
        for (const FBox& Coverage : Plot.Coverage) Rect(Coverage, FLinearColor(.12f,.55f,.3f));
        const FBox& B = Plot.Bounds;
        const TArray<FVector2D> Outline = {Project(G,FVector(B.Min.X,B.Min.Y,0)), Project(G,FVector(B.Max.X,B.Min.Y,0)),
            Project(G,FVector(B.Max.X,B.Max.Y,0)), Project(G,FVector(B.Min.X,B.Max.Y,0)), Project(G,FVector(B.Min.X,B.Min.Y,0))};
        const bool Selected = Data->Selection == ECooperationObject::Plot && Data->SelectedID == Plot.ID;
        Line(Outline, Selected ? FLinearColor::White : FLinearColor(.3f,.45f,.38f), Selected ? 2.f : 1.f);
        Label(Project(G,B.GetCenter())-FVector2D(25,8), FString::Printf(TEXT("地块%d"),Plot.ID), FLinearColor::White);
    }
    for (const auto& Agent : Data->Agents) if (Data->Selection == ECooperationObject::Agent && Data->SelectedID == Agent.State.AgentID)
    {
        for (int32 I = 1; I < Agent.Route.Num(); I += 2)
            Line({Project(G,Agent.Route[I-1]), Project(G,Agent.Route[I])}, FCooperationViewData::AgentColor(Agent.State.AgentID), 1.5f);
        const auto* Airport = Data->Airports.FindByPredicate([&](const auto& A){return A.Config.AirportID == Agent.State.AirportID;});
        if (Airport && Agent.State.Phase != EAgriculturePhase::Completed)
            Label(Project(G,Airport->Config.DockPosition)+FVector2D(8,12), TEXT("补给目标"), FCooperationViewData::AgentColor(Agent.State.AgentID));
    }
    for (const auto& Airport : Data->Airports)
    {
        const FVector2D P = Project(G, Airport.Config.DockPosition);
        const FString Status = FCooperationViewData::AirportStatus(Airport);
        const FLinearColor Color = Status == TEXT("可用") ? FLinearColor(.2f,.8f,.45f) :
            Status == TEXT("占用") ? FLinearColor(1,.65f,.2f) : FLinearColor(1,.25f,.25f);
        Line({P+FVector2D(-5,-5),P+FVector2D(5,-5),P+FVector2D(5,5),P+FVector2D(-5,5),P+FVector2D(-5,-5)}, Color, 2);
        if (Data->Selection == ECooperationObject::Airport && Data->SelectedID == Airport.Config.AirportID)
            Line({P+FVector2D(-8,-8),P+FVector2D(8,-8),P+FVector2D(8,8),P+FVector2D(-8,8),P+FVector2D(-8,-8)}, FLinearColor::White, 1);
        Label(P+FVector2D(8,-10), FString::Printf(TEXT("机场%d"),Airport.Config.AirportID), Color);
    }
    for (const auto& Agent : Data->Agents)
    {
        const FVector2D P = Project(G, Agent.Position);
        const float Angle = FMath::DegreesToRadians(Agent.Yaw);
        const FVector2D Forward(FMath::Cos(Angle),FMath::Sin(Angle));
        const FVector2D Side(-Forward.Y,Forward.X);
        const FLinearColor Color = Agent.State.Phase == EAgriculturePhase::Failed ? FLinearColor(1,.2f,.2f) : FCooperationViewData::AgentColor(Agent.State.AgentID);
        Line({P+Forward*8,P-Forward*5+Side*5,P-Forward*5-Side*5,P+Forward*8}, Color, 2);
        if (Data->Selection == ECooperationObject::Agent && Data->SelectedID == Agent.State.AgentID)
            Line({P+FVector2D(-11,-11),P+FVector2D(11,-11),P+FVector2D(11,11),P+FVector2D(-11,11),P+FVector2D(-11,-11)}, FLinearColor::White);
        Label(P+FVector2D(10,2),FString::Printf(TEXT("U%d"),Agent.State.AgentID),Color);
    }
    if (!WorldController.IsValid()) Label(FVector2D(8,6), TEXT("+Y ↓   +X →"), FLinearColor(.65f,.75f,.85f));
    return Layer+3;
}

FReply SCooperationMap::OnMouseButtonDown(const FGeometry& G, const FPointerEvent& Event)
{
    if (!Data || !OnSelect || Event.GetEffectingButton() != EKeys::LeftMouseButton) return FReply::Unhandled();
    const FVector2D Mouse = G.AbsoluteToLocal(Event.GetScreenSpacePosition());
    int32 ID = INDEX_NONE; float Nearest = 18;
    for (const auto& Agent : Data->Agents)
    {
        const float Distance = FVector2D::Distance(Mouse, Project(G, Agent.Position));
        if (Distance < Nearest) { Nearest = Distance; ID = Agent.State.AgentID; }
    }
    if (ID != INDEX_NONE) { OnSelect(ECooperationObject::Agent, ID); return FReply::Handled(); }
    for (const auto& Airport : Data->Airports)
        if (FVector2D::Distance(Mouse, Project(G, Airport.Config.DockPosition)) < 18)
        { OnSelect(ECooperationObject::Airport, Airport.Config.AirportID); return FReply::Handled(); }
    for (const auto& Plot : Data->Plots)
    {
        if (WorldController.IsValid())
        {
            if (FVector2D::Distance(Mouse, Project(G, Plot.Bounds.GetCenter())) < 26)
            { OnSelect(ECooperationObject::Plot, Plot.ID); return FReply::Handled(); }
            continue;
        }
        const FVector2D A = Project(G,FVector(Plot.Bounds.Min.X,Plot.Bounds.Min.Y,0));
        const FVector2D B = Project(G,FVector(Plot.Bounds.Max.X,Plot.Bounds.Max.Y,0));
        if (Mouse.X >= A.X && Mouse.X <= B.X && Mouse.Y >= A.Y && Mouse.Y <= B.Y)
        { OnSelect(ECooperationObject::Plot, Plot.ID); return FReply::Handled(); }
    }
    if (WorldController.IsValid()) return FReply::Unhandled();
    OnSelect(ECooperationObject::None, INDEX_NONE);
    return FReply::Handled();
}
