#pragma once

#include "DiseaseObject.h"
#include "GeoJsonFeature.h"
#include "GeoJsonGeometry.h"

#include <QJsonObject>

class AxialDeformationObject : public DiseaseObject
{
public:
    explicit AxialDeformationObject(DiseaseType type = Type::AxialDeformation);
    AxialDeformationObject(const GeoJsonGeometryLineString &geometry);

    GeoJsonGeometryLineString geometry() const;
    void                      setGeometry(const GeoJsonGeometryLineString &geometry);

    Points deformations() const;
    void   setDeformations(const Points &deformations);

    QJsonObject toGeoJsonObject() const override;
    bool        fromGeoJsonObject(const QJsonObject &json) override;

private:
    GeoJsonGeometryLineString mGeometry;       // 每一对坐标描述一个采样点的位置
    Points                    mDeformations;   // 采样点所在位置的水平和竖直方向上的偏差值
};
