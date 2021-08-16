#pragma once
#include "PointXYZIT.h"

#include <QString>
#include <QVector>
//#include <stdint.h>
//定义椭圆一般方程的参数A、B、C、D、E、F
struct EllipseCoefficients
{
    double A, B, C, D, E, F;
};

//定义椭圆标准方程参数，包括中点(x0,y0),长半轴majorAxis，短半轴minorAxis，长半轴相对于短半轴的旋转角alpha
struct EllipseStandardParam
{
    double x0;
    double y0;
    double alpha;
    double majorAxis;
    double minorAxis;
};

struct ellipseCenter
{
    double x0 = 0.0;
    double y0 = 0.0;
};

class TunnelCrossSection
{
public:
    TunnelCrossSection();

    int32_t                     mIndex;
    pcl::PointCloud<PointXYZIT> mLineData;

    double               mMean;
    double               mStd;
    QVector<int>         mInlierflags;
    QVector<int>         mUnlierflags;
    EllipseCoefficients  mBestEllipseCoefficients;
    EllipseStandardParam mBestStandardParam;
    bool                 ransac(int iterations, int n);

    // TODO: 处理收敛直径
    double mDiameterValue;
    double mAbsoluteDeformation;
    double mRelativeDeformation;
    void   calcuateConvergenceDiameter();

    // TODO: 处理形变量
    double mOvality;
    double mAxisDelta;
    void   calcuateEllipseDeformation(double radial);

private:
    void calcuateEllipseStandParam();
    bool computeModelCoefficients(const QVector<int> &randflags, EllipseCoefficients *ellipse_coefficients);
    bool countWithinDistance(const EllipseCoefficients &coefficient,
                             QVector<int> &             inliers,
                             QVector<int> &             unInliers,
                             double &                   mean,
                             double &                   std);
};
