#include "EllipseDeformationObject.h"

#include <spdlog/qt_spdlog.h>

EllipseDeformationObject::EllipseDeformationObject(DiseaseType type)
    : DiseaseObject(type)
{
}

EllipseDeformationObject::EllipseDeformationObject(const GeoJsonGeometryPoint &geometry)
    : DiseaseObject(Type::EllipseDeformation)
    , mGeometry(geometry)
{
}

GeoJsonGeometryPoint EllipseDeformationObject::geometry() const
{
    return mGeometry;
}

void EllipseDeformationObject::setGeometry(const GeoJsonGeometryPoint &geometry)
{
    mGeometry = geometry;
}

int EllipseDeformationObject::ringNum() const
{
    return mRingNum;
}

void EllipseDeformationObject::setRingNum(int ringNum)
{
    mRingNum = ringNum;
}

double EllipseDeformationObject::ovality() const
{
    return mOvality;
}

void EllipseDeformationObject::setOvality(double ovality)
{
    mOvality = ovality;
}

double EllipseDeformationObject::axisDelta() const
{
    return mAxisDelta;
}

void EllipseDeformationObject::setAxisDelta(double axisDelta)
{
    mAxisDelta = axisDelta;
}

QJsonObject EllipseDeformationObject::toGeoJsonObject() const
{
    QJsonObject feature;
    QJsonObject properties;
    properties.insert(GeoJsonFeature::key_ring_num(), mRingNum);
    properties.insert(GeoJsonFeature::key_mileage(), mMileage);
    properties.insert(GeoJsonFeature::key_disease(), DiseaseTypeToString(type()));
    properties.insert(GeoJsonFeature::key_ovality(), mOvality);
    properties.insert(GeoJsonFeature::key_axis_delta(), mAxisDelta);
    properties.insert(GeoJsonFeature::key_diameter_value(), mDiameterValue);
    properties.insert(GeoJsonFeature::key_absolute_deformation(), mAbsoluteDeformation);
    properties.insert(GeoJsonFeature::key_relative_deformation(), mRelativeDeformation);

    QJsonArray deformations;
    for (int i = 0; i < mDeformations.size(); ++i)
    {
        QJsonObject deformation_bj;
        deformation_bj.insert(GeoJsonFeature::key_direction(), mDeformations[i].direction);
        deformation_bj.insert(GeoJsonFeature::key_deformation(), mDeformations[i].deformation);
        deformations.push_back(deformation_bj);
    }
    properties.insert(GeoJsonFeature::key_deformations(), deformations);

    feature.insert(GeoJsonFeature::key_type(), GeoJsonFeature::value_feature());
    feature.insert(GeoJsonFeature::key_geometry(), mGeometry.toGeoJson());
    feature.insert(GeoJsonFeature::key_properties(), properties);
    return feature;
}

bool EllipseDeformationObject::fromGeoJsonObject(const QJsonObject &json)
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
        SPDLOG_ERROR("病害类型错误！读入类型应为: EllipseDeformation，实际类型为: {}", json_disease_str);
        return false;
    }

    mRingNum             = properties.value(GeoJsonFeature::key_ring_num()).toInt();
    mMileage             = properties.value(GeoJsonFeature::key_mileage()).toString();
    mOvality             = properties.value(GeoJsonFeature::key_ovality()).toDouble();
    mAxisDelta           = properties.value(GeoJsonFeature::key_axis_delta()).toDouble();
    mDiameterValue       = properties.value(GeoJsonFeature::key_diameter_value()).toDouble();
    mAbsoluteDeformation = properties.value(GeoJsonFeature::key_absolute_deformation()).toDouble();
    mRelativeDeformation = properties.value(GeoJsonFeature::key_relative_deformation()).toDouble();

    QJsonArray deformations = properties.value(GeoJsonFeature::key_deformations()).toArray();
    mDeformations.clear();
    mDeformations.resize(deformations.size());
    for (int i = 0; i < deformations.size(); ++i)
    {
        QJsonObject deformation_obj  = deformations[i].toObject();
        mDeformations[i].direction   = deformation_obj.value(GeoJsonFeature::key_direction()).toDouble();
        mDeformations[i].deformation = deformation_obj.value(GeoJsonFeature::key_deformation()).toDouble();
    }

    auto geometry = json.value(GeoJsonFeature::key_geometry()).toObject();
    return mGeometry.fromGeoJson(geometry);
}

QString EllipseDeformationObject::mileage() const
{
    return mMileage;
}

void EllipseDeformationObject::setMileage(const QString &mileage)
{
    mMileage = mileage;
}

double EllipseDeformationObject::diameterValue() const
{
    return mDiameterValue;
}

void EllipseDeformationObject::setDiameterValue(double diameterValue)
{
    mDiameterValue = diameterValue;
}

double EllipseDeformationObject::absoluteDeformation() const
{
    return mAbsoluteDeformation;
}

void EllipseDeformationObject::setAbsoluteDeformation(double absoluteDeformation)
{
    mAbsoluteDeformation = absoluteDeformation;
}

double EllipseDeformationObject::relativeDeformation() const
{
    return mRelativeDeformation;
}

void EllipseDeformationObject::setRelativeDeformation(double relativeDeformation)
{
    mRelativeDeformation = relativeDeformation;
}

QVector<EllipseDeformationObject::Deformation> EllipseDeformationObject::deformations() const
{
    return mDeformations;
}

void EllipseDeformationObject::setDeformations(const QVector<Deformation> &deformations)
{
    mDeformations = deformations;
}
