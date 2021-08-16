#include "LeakageObject.h"

#include <spdlog/qt_spdlog.h>

LeakageObject::LeakageObject(DiseaseType type)
    : DiseaseObject(type)
{
}

LeakageObject::LeakageObject(const GeoJsonGeometryPolygon &geometry)
    : DiseaseObject(Type::Leakage)
    , mGeometry(geometry)
{
}

GeoJsonGeometryPolygon LeakageObject::geometry() const
{
    return mGeometry;
}

void LeakageObject::setGeometry(const GeoJsonGeometryPolygon &geometry)
{
    mGeometry = geometry;
}

int LeakageObject::ringNum() const
{
    return mRingNum;
}

void LeakageObject::setRingNum(int ringNum)
{
    mRingNum = ringNum;
}

double LeakageObject::leakageArea() const
{
    return mLeakageArea;
}

void LeakageObject::setLeakageArea(double leakageArea)
{
    mLeakageArea = leakageArea;
}

QJsonObject LeakageObject::toGeoJsonObject() const
{
    QJsonObject feature;
    QJsonObject properties;
    properties.insert(GeoJsonFeature::key_ring_num(), mRingNum);
    properties.insert(GeoJsonFeature::key_mileage(), mMileage);
    properties.insert(GeoJsonFeature::key_disease(), DiseaseTypeToString(type()));
    properties.insert(GeoJsonFeature::key_leakage_area(), mLeakageArea);

    feature.insert(GeoJsonFeature::key_type(), GeoJsonFeature::value_feature());
    feature.insert(GeoJsonFeature::key_geometry(), mGeometry.toGeoJson());
    feature.insert(GeoJsonFeature::key_properties(), properties);
    return feature;
}

bool LeakageObject::fromGeoJsonObject(const QJsonObject &json)
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
        SPDLOG_ERROR("病害类型错误！读入类型应为: Crack，实际类型为: {}", json_disease_str);
        return false;
    }

    mRingNum     = properties.value(GeoJsonFeature::key_ring_num()).toInt();
    mMileage     = properties.value(GeoJsonFeature::key_mileage()).toString();
    mLeakageArea = properties.value(GeoJsonFeature::key_leakage_area()).toDouble();

    auto geometry = json.value(GeoJsonFeature::key_geometry()).toObject();
    return mGeometry.fromGeoJson(geometry);
}

QString LeakageObject::mileage() const
{
    return mMileage;
}

void LeakageObject::setMileage(const QString &mileage)
{
    mMileage = mileage;
}
