#pragma once

#include "Widgets/SLeafWidget.h"
#include "CooperationViewData.h"

class APlayerController;

/** 俯视图与卡片共用对象 ID，绘制坐标和命中检测使用同一变换。 */
class SCooperationMap : public SLeafWidget
{
public:
    using FSelectObject = TFunction<void(ECooperationObject, int32)>;
    SLATE_BEGIN_ARGS(SCooperationMap) {}
        SLATE_ARGUMENT(TSharedPtr<FCooperationViewData>, Data)
        SLATE_ARGUMENT(FSelectObject, OnSelect)
        SLATE_ARGUMENT(TWeakObjectPtr<APlayerController>, WorldController)
    SLATE_END_ARGS()
    void Construct(const FArguments& Args);
    virtual void Tick(const FGeometry&, double, float) override;
    virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(300, 230); }
    virtual int32 OnPaint(const FPaintArgs&, const FGeometry&, const FSlateRect&, FSlateWindowElementList&,
        int32, const FWidgetStyle&, bool) const override;
    virtual FReply OnMouseButtonDown(const FGeometry&, const FPointerEvent&) override;
private:
    TSharedPtr<FCooperationViewData> Data;
    TFunction<void(ECooperationObject, int32)> OnSelect;
    TWeakObjectPtr<APlayerController> WorldController;
    uint64 LastRevision = 0;
    ECooperationObject LastSelection = ECooperationObject::None;
    int32 LastSelectedID = INDEX_NONE;
    FVector2D Project(const FGeometry& Geometry, const FVector& Position) const;
};
