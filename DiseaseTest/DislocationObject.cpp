#include "DislocationObject.h"

#include <spdlog/qt_spdlog.h>

DislocationObject::DislocationObject(DiseaseType type)
    : DiseaseObject(type)
{
}

DislocationObject::DislocationObject(const GeoJsonGeometryPoint &geometry)
    : DiseaseObject(Type::Dislocation)
    , mGeometry(geometry)
{
}

GeoJsonGeometryPoint DislocationObject::geometry() const
{
    return mGeometry;
}

void DislocationObject::setGeometry(const GeoJsonGeometryPoint &geometry)
{
    mGeometry = geometry;
}

int DislocationObject::ringNum() const
{
    return mRingNum;
}

void DislocationObject::setRingNum(int ringNum)
{
    mRingNum = ringNum;
}

double DislocationObject::circumferentialDislocation() const
{
    return mCircumferentialDislocation;
}

void DislocationObject::setCircumferentialDislocation(double circumferentialDislocation)
{
    mCircumferentialDislocation = circumferentialDislocation;
}

QVector<DislocationObject::RadialDislocation> DislocationObject::radialDislocation() const
{
    return mRadialDislocation;
}

void DislocationObject::setRadialDislocation(const QVector<RadialDislocation> &radialDislocation)
{
    mRadialDislocation = radialDislocation;
}

QJsonObject DislocationObject::toGeoJsonObject() const
{
    QJsonObject feature;
    QJsonObject properties;
    properties.insert(GeoJsonFeature::key_ring_num(), mRingNum);
    properties.insert(GeoJsonFeature::key_mileage(), mMileage);
    properties.insert(GeoJsonFeature::key_disease(), DiseaseTypeToString(type()));
    properties.insert(GeoJsonFeature::key_circumferential_dislocation(), mCircumferentialDislocation);
    QJsonArray radials;
    for (int i = 0; i < mRadialDislocation.size(); ++i)
    {
        QJsonObject radial_obj;
        radial_obj.insert(GeoJsonFeature::key_direction(), mRadialDislocation[i].direction);
        radial_obj.insert(GeoJsonFeature::key_dislocation(), mRadialDislocation[i].dislocation);
        radials.push_back(radial_obj);
    }
    properties.insert(GeoJsonFeature::key_radial_dislocation(), radials);

    feature.insert(GeoJsonFeature::key_type(), GeoJsonFeature::value_feature());
    feature.insert(GeoJsonFeature::key_geometry(), mGeometry.toGeoJson());
    feature.insert(GeoJsonFeature::key_properties(), properties);
    return feature;
}

bool DislocationObject::fromGeoJsonObject(const QJsonObject &json)
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
    if (disease_str != properties.value(GeoJsonFeature::key_disease()).toString())
    {
        SPDLOG_ERROR("病害类型错误！读入类型应为: Dislocation，实际类型为: {}", json_disease_str);
        return false;
    }

    mRingNum                    = properties.value(GeoJsonFeature::key_ring_num()).toInt();
    mMileage                    = properties.value(GeoJsonFeature::key_mileage()).toString();
    mCircumferentialDislocation = properties.value(GeoJsonFeature::key_circumferential_dislocation()).toDouble();
    QJsonArray radials          = properties.value(GeoJsonFeature::key_radial_dislocation()).toArray();
    mRadialDislocation.clear();
    mRadialDislocation.resize(radials.size());
    for (int i = 0; i < radials.size(); ++i)
    {
        QJsonObject radialObj             = radials[i].toObject();
        mRadialDislocation[i].direction   = radialObj.value(GeoJsonFeature::key_direction()).toDouble();
        mRadialDislocation[i].dislocation = radialObj.value(GeoJsonFeature::key_dislocation()).toDouble();
    }

    auto geometry = json.value(GeoJsonFeature::key_geometry()).toObject();
    return mGeometry.fromGeoJson(geometry);
}

QString DislocationObject::mileage() const
{
    return mMileage;
}

void DislocationObject::setMileage(const QString &mileage)
{
    mMileage = mileage;
}
