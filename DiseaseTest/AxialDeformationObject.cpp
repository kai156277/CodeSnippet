#include "AxialDeformationObject.h"

#include <spdlog/qt_spdlog.h>

AxialDeformationObject::AxialDeformationObject(DiseaseType type)
    : DiseaseObject(type)
{
}

AxialDeformationObject::AxialDeformationObject(const GeoJsonGeometryLineString &geometry)
    : DiseaseObject(Type::AxialDeformation)
    , mGeometry(geometry)
{
}

GeoJsonGeometryLineString AxialDeformationObject::geometry() const
{
    return mGeometry;
}

void AxialDeformationObject::setGeometry(const GeoJsonGeometryLineString &geometry)
{
    mGeometry = geometry;
}

Points AxialDeformationObject::deformations() const
{
    return mDeformations;
}

void AxialDeformationObject::setDeformations(const Points &deformations)
{
    mDeformations = deformations;
}

QJsonObject AxialDeformationObject::toGeoJsonObject() const
{
    QJsonObject feature;
    QJsonObject properties;
    properties.insert(GeoJsonFeature::key_disease(), DiseaseTypeToString(type()));
    properties.insert(GeoJsonFeature::key_deformations(), PointsToJsonArray(mDeformations));

    feature.insert(GeoJsonFeature::key_type(), GeoJsonFeature::value_feature());
    feature.insert(GeoJsonFeature::key_geometry(), mGeometry.toGeoJson());
    feature.insert(GeoJsonFeature::key_properties(), properties);
    return feature;
}

bool AxialDeformationObject::fromGeoJsonObject(const QJsonObject &json)
{
    static const QString disease_str = DiseaseTypeToString(type());

    QString type_string = json.value(GeoJsonFeature::key_type()).toString();
    if (GeoJsonFeature::value_feature() != type_string)
    {
        SPDLOG_ERROR("features 类型错误！读入类型应为: Feature，实际类型为: {}", type_string);
        return false;
    }
    auto    properties       = json.value(GeoJsonFeature::key_properties()).toObject();
    QString json_disease_str = properties.value(GeoJsonFeature::key_disease()).toString();
    if (disease_str != json_disease_str)
    {
        SPDLOG_ERROR("病害类型错误！读入类型应为: AxialDeformation，实际类型为: {}", json_disease_str);
        return false;
    }

    QJsonArray deformations_array = properties.value(GeoJsonFeature::key_deformations()).toArray();

    if (!JsonArrayToPoints(deformations_array, mDeformations))
    {
        SPDLOG_ERROR("轴线变形采样点点不正确");
        return false;
    }

    auto geometry = json.value(GeoJsonFeature::key_geometry()).toObject();
    return mGeometry.fromGeoJson(geometry);
}
