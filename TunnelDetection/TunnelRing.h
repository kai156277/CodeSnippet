#pragma once

#include "TunnelCrossSection.h"

#include <QVector>

class TunnelRing
{
public:
    TunnelRing();

    int32_t mRingNumber;
    double  mProjectX;
    double  mProjectY;

    QVector<TunnelCrossSection> mRingData;

    // TODO: 处理径向错台
    QVector<double> mRadialDislocation;
    void            calcuateRadialDislocation();

    // TODO: 处理收敛直径
    double mDiameterValue;
    double mAbsoluteDeformation;
    double mRelativeDeformation;
    void   calcuateConvergenceDiameter();

    // TODO: 处理形变量
    double mOvality;
    double mAxisDelta;
    void   calcuateEllipseDeformation(double design_radial);
};
