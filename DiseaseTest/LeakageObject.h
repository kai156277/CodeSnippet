#pragma once

#include "DiseaseObject.h"
#include "GeoJsonFeature.h"
#include "GeoJsonGeometry.h"

#include <QJsonObject>

class LeakageObject : public DiseaseObject
{
public:
    explicit LeakageObject(DiseaseType type = Type::Leakage);
    LeakageObject(const GeoJsonGeometryPolygon &geometry);

    GeoJsonGeometryPolygon geometry() const;
    void                   setGeometry(const GeoJsonGeometryPolygon &geometry);

    int  ringNum() const;
    void setRingNum(int ringNum);

    double leakageArea() const;
    void   setLeakageArea(double leakageArea);

    QJsonObject toGeoJsonObject() const;
    bool        fromGeoJsonObject(const QJsonObject &json);

    QString mileage() const;
    void    setMileage(const QString &mileage);

private:
    GeoJsonGeometryPolygon mGeometry;
    int                    mRingNum;
    double                 mLeakageArea;
    QString                mMileage;
};
