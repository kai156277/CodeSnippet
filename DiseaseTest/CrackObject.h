#pragma once

#include "DiseaseObject.h"
#include "GeoJsonFeature.h"
#include "GeoJsonGeometry.h"

#include <QJsonObject>

class CrackObject : public DiseaseObject
{
public:
    explicit CrackObject(DiseaseType type = DiseaseType::Crack);
    CrackObject(const GeoJsonGeometryPolygon &geometry);

    enum Direction
    {
        UnDefine,
        Lengthways,
        Hoop,
        Slant
    };
    static QString   DirectionTypeToString(Direction type);
    static Direction StringToDirectionType(const QString &str);

    DiseaseType diseaseType() const;

    int  ringNum() const;
    void setRingNum(int ringNum);

    double crackArea() const;
    void   setCrackArea(double crackArea);

    double crackLength() const;
    void   setCrackLength(double crackLength);

    double crackWidth() const;
    void   setCrackWidth(double crackWidth);

    Direction direction() const;
    void      setDirection(const Direction &direction);

    GeoJsonGeometryPolygon geometry() const;
    void                   setGeometry(const GeoJsonGeometryPolygon &geometry);

    QJsonObject toGeoJsonObject() const;
    bool        fromGeoJsonObject(const QJsonObject &json);

    QString mileage() const;
    void    setMileage(const QString &mileage);

private:
    GeoJsonGeometryPolygon mGeometry;   // 裂缝在平面二维坐标系下的范围
    QString                mMileage;
    int                    mRingNum;       // 环号
    double                 mCrackArea;     // 裂缝面积 mm^2
    double                 mCrackLength;   // 裂缝长度 mm
    double                 mCrackWidth;    // 裂缝宽度 mm
    Direction              mDirection;
};

using CrackDirection = CrackObject::Direction;
