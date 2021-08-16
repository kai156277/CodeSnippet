#pragma once

#include "ScanPoint.h"
#include <stdint.h>

#include <QVector>

struct EllipseGeneralParam
{
    double A;
    double B;
    double C;
    double D;
    double E;
    double F;
};

struct EllipseStandardParam
{
    double x0;
    double y0;
    double alpha;
    double majorSemiAxis;
    double minorSemiAxis;
};

class ScanLine
{
public:
    ScanLine();

    int32_t            mIndex;
    QVector<ScanPoint> mLineData;

    QVector<EllipseGeneralParam>  mGeneralParams;
    QVector<EllipseStandardParam> mStandardParams;

    void calculateAll();

    void calEllipseGeneralParam();
    void calEllipseStandardParam();

    void setIndex(int32_t index);
};
