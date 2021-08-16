#pragma once

#include <QJsonObject>

class DiseaseObject
{
public:
    enum class Type
    {
        NoDisease = 0,
        Crack     = 1,        // 裂缝
        Leakage,              // 渗水
        Dislocation,          // 错台
        AxialDeformation,     // 轴线变形
        EllipseDeformation,   // 椭圆变形
    };

    DiseaseObject();
    explicit DiseaseObject(Type type);

    virtual QJsonObject toGeoJsonObject() const                    = 0;
    virtual bool        fromGeoJsonObject(const QJsonObject &json) = 0;

    Type type() const;
    void setType(const Type &type);

private:
    Type mType = Type::NoDisease;
};

using DiseaseType = DiseaseObject::Type;

QString DiseaseTypeToString(DiseaseType type);
