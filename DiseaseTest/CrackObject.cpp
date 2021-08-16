#include "CrackObject.h"

#include <QMap>

#include <spdlog/qt_spdlog.h>

CrackObject::CrackObject(DiseaseType type)
    : DiseaseObject(type)
{
}

CrackObject::CrackObject(const GeoJsonGeometryPolygon &geometry)
    : DiseaseObject(Type::Crack)
    , mGeometry(geometry)
{
}

static QMap<CrackDirection, QString> direction_mapping = {
    {CrackDirection::UnDefine, "UnDefine"},
    {CrackDirection::Hoop, "Hoop"},
    {CrackDirection::Lengthways, "Lengthways"},
    {CrackDirection::Slant, "Slant"},
};

QString CrackObject::DirectionTypeToString(CrackObject::Direction type)
{
    return direction_mapping.value(type, "UnDefine");
}

CrackObject::Direction CrackObject::StringToDirectionType(const QString &str)
{
    return direction_mapping.key(str, CrackDirection::UnDefine);
}

int CrackObject::ringNum() const
{
    return mRingNum;
}

void CrackObject::setRingNum(int ringNum)
{
    mRingNum = ringNum;
}

double CrackObject::crackArea() const
{
    return mCrackArea;
}

void CrackObject::setCrackArea(double crackArea)
{
    mCrackArea = crackArea;
}

GeoJsonGeometryPolygon CrackObject::geometry() const
{
    return mGeometry;
}

void CrackObject::setGeometry(const GeoJsonGeometryPolygon &geometry)
{
    mGeometry = geometry;
}

QJsonObject CrackObject::toGeoJsonObject() const
{
    QJsonObject feature;
    QJsonObject properties;
    properties.insert(GeoJsonFeature::key_ring_num(), mRingNum);
    properties.insert(GeoJsonFeature::key_mileage(), mMileage);
    properties.insert(GeoJsonFeature::key_disease(), DiseaseTypeToString(type()));
    properties.insert(GeoJsonFeature::key_crack_area(), mCrackArea);
    properties.insert(GeoJsonFeature::key_crack_length(), mCrackLength);
    properties.insert(GeoJsonFeature::key_crack_width(), mCrackWidth);
    properties.insert(GeoJsonFeature::key_crack_direction(), DirectionTypeToString(direction()));

    feature.insert(GeoJsonFeature::key_type(), GeoJsonFeature::value_feature());
    feature.insert(GeoJsonFeature::key_geometry(), mGeometry.toGeoJson());
    feature.insert(GeoJsonFeature::key_properties(), properties);
    return feature;
}

bool CrackObject::fromGeoJsonObject(const QJsonObject &json)
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
    mCrackArea   = properties.value(GeoJsonFeature::key_crack_area()).toDouble();
    mCrackLength = properties.value(GeoJsonFeature::key_crack_length()).toDouble();
    mCrackWidth  = properties.value(GeoJsonFeature::key_crack_width()).toDouble();
    mMileage     = properties.value(GeoJsonFeature::key_mileage()).toString();
    mDirection   = StringToDirectionType(properties.value(GeoJsonFeature::key_crack_direction()).toString());

    auto geometry = json.value(GeoJsonFeature::key_geometry()).toObject();
    return mGeometry.fromGeoJson(geometry);
}

QString CrackObject::mileage() const
{
    return mMileage;
}

void CrackObject::setMileage(const QString &mileage)
{
    mMileage = mileage;
}

double CrackObject::crackLength() const
{
    return mCrackLength;
}

void CrackObject::setCrackLength(double crackLength)
{
    mCrackLength = crackLength;
}

double CrackObject::crackWidth() const
{
    return mCrackWidth;
}

void CrackObject::setCrackWidth(double crackWidth)
{
    mCrackWidth = crackWidth;
}

CrackDirection CrackObject::direction() const
{
    return mDirection;
}

void CrackObject::setDirection(const Direction &direction)
{
    mDirection = direction;
}
