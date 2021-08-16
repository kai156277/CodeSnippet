#pragma once

#include "DiseaseObject.h"
#include "GeoJsonFeature.h"
#include "GeoJsonGeometry.h"

#include <QJsonObject>
#include <QVector>

class DislocationObject : public DiseaseObject
{
public:
    struct RadialDislocation
    {
        double direction;
        double dislocation;
    };
    explicit DislocationObject(DiseaseType type = Type::Dislocation);
    DislocationObject(const GeoJsonGeometryPoint &geometry);

    GeoJsonGeometryPoint geometry() const;
    void                 setGeometry(const GeoJsonGeometryPoint &geometry);

    int  ringNum() const;
    void setRingNum(int ringNum);

    double circumferentialDislocation() const;
    void   setCircumferentialDislocation(double circumferentialDislocation);

    QVector<RadialDislocation> radialDislocation() const;
    void                       setRadialDislocation(const QVector<RadialDislocation> &radialDislocation);

    QJsonObject toGeoJsonObject() const;
    bool        fromGeoJsonObject(const QJsonObject &json);

    QString mileage() const;
    void    setMileage(const QString &mileage);

private:
    GeoJsonGeometryPoint       mGeometry;   // 管片中心位置在二维平面坐标系中的位置
    QString                    mMileage;
    int                        mRingNum;                      // 环号
    double                     mCircumferentialDislocation;   // 环向误差 (m)
    QVector<RadialDislocation> mRadialDislocation;            // 从封顶块开始顺时针的衬砌块之间的径向误差 (m)
};

using RadialDislocation  = DislocationObject::RadialDislocation;
using RadialDislocations = QVector<DislocationObject::RadialDislocation>;
