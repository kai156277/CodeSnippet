#include "GeoJsonGeometry.h"

#include <QJsonObject>
#include <QMap>

#include <spdlog/qt_spdlog.h>

static QMap<GeometryType, QString> type_mapping = {
    {GeometryType::ErrorType, "ErrorType"},
    {GeometryType::Point, "Point"},
    {GeometryType::LineString, "LineString"},
    {GeometryType::Polygon, "Polygon"},
    {GeometryType::MultiPoint, "MultiPoint"},
    {GeometryType::MultiLineString, "MultiLineString"},
    {GeometryType::MultiPolygon, "MultiPolygon"},
    {GeometryType::GeometryCollection, "GeometryCollection"},
};

static const int point_size = 2;

QString GeometryTypeToString(GeometryType type)
{
    return type_mapping.value(type);
}

GeometryType StringToGeometryType(const QString &str)
{
    return type_mapping.key(str);
}

PointXY GeoJsonGeometryPoint::coordinates() const
{
    return mCoordinates;
}

void GeoJsonGeometryPoint::setCoordinates(const PointXY &coordinates)
{
    mCoordinates = coordinates;
}

QJsonObject GeoJsonGeometryPoint::toGeoJson() const
{
    QJsonObject geojson;
    geojson.insert(GeoJsonFeature::key_type(), GeometryTypeToString(mType));
    geojson.insert(GeoJsonFeature::key_coordinates(), PointXYToJsonArray(mCoordinates));
    return geojson;
}

bool GeoJsonGeometryPoint::fromGeoJson(const QJsonObject &json)
{
    QString type_string = json.value(GeoJsonFeature::key_type()).toString();
    auto    type        = StringToGeometryType(type_string);
    if (mType != type)
    {
        SPDLOG_ERROR("geometry的类型错误！读入类型应为: Point，实际类型为: {}", type_string);
        return false;
    }

    QJsonArray array = json.value(GeoJsonFeature::key_coordinates()).toArray();
    if (array.size() != point_size)
    {
        SPDLOG_ERROR("GeoJson geometry的coordinates 长度不足2位");
        return false;
    }

    mCoordinates.x = array[0].toDouble();
    mCoordinates.y = array[1].toDouble();
    return true;
}

GeometryType GeoJsonGeometryPoint::type() const
{
    return mType;
}

Points GeoJsonGeometryLineString::coordinates() const
{
    return mCoordinates;
}

void GeoJsonGeometryLineString::setCoordinates(const Points &coordinates)
{
    mCoordinates = coordinates;
}

QJsonObject GeoJsonGeometryLineString::toGeoJson() const
{
    QJsonObject geojson;
    geojson.insert(GeoJsonFeature::key_type(), GeometryTypeToString(mType));
    QJsonArray array;
    for (int i = 0; i < mCoordinates.size(); ++i)
    {
        array.push_back(PointXYToJsonArray(mCoordinates[i]));
    }
    geojson.insert(GeoJsonFeature::key_coordinates(), array);
    return geojson;
}

bool GeoJsonGeometryLineString::fromGeoJson(const QJsonObject &json)
{
    QString type_string = json.value(GeoJsonFeature::key_type()).toString();
    auto    type        = StringToGeometryType(type_string);
    if (mType != type)
    {
        SPDLOG_ERROR("未正确识别GeoJson geometry的type，读入类型应为: LineString，实际类型为: {}", type_string);
        return false;
    }

    QJsonArray array = json.value(GeoJsonFeature::key_coordinates()).toArray();
    const int  size  = array.size();

    mCoordinates.clear();
    mCoordinates.resize(size);
    QJsonArray point_json;
    for (int i = 0; i < size; ++i)
    {
        point_json = array[i].toArray();
        if (point_json.size() != point_size)
        {
            SPDLOG_ERROR("LineString 的第{}个坐标长度不足2位", i);
            return false;
        }
        mCoordinates[i].x = point_json[0].toDouble();
        mCoordinates[i].y = point_json[1].toDouble();
    }
    return true;
}

GeometryType GeoJsonGeometryLineString::type() const
{
    return mType;
}

Points GeoJsonGeometryPolygon::coordinates() const
{
    return mCoordinates;
}

void GeoJsonGeometryPolygon::setCoordinates(const Points &coordinates)
{
    mCoordinates = coordinates;
}

GeometryType GeoJsonGeometryPolygon::type() const
{
    return mType;
}

QJsonObject GeoJsonGeometryPolygon::toGeoJson() const
{
    QJsonObject geojson;
    geojson.insert(GeoJsonFeature::key_type(), GeometryTypeToString(mType));
    QJsonArray outer_boundary;
    for (int i = 0; i < mCoordinates.size(); ++i)
    {
        outer_boundary.push_back(PointXYToJsonArray(mCoordinates[i]));
    }
    QJsonArray array;
    array.push_back(outer_boundary);

    geojson.insert(GeoJsonFeature::key_coordinates(), array);
    return geojson;
}

bool GeoJsonGeometryPolygon::fromGeoJson(const QJsonObject &json)
{
    QString type_string = json.value(GeoJsonFeature::key_type()).toString();
    auto    type        = StringToGeometryType(type_string);
    if (mType != type)
    {
        SPDLOG_ERROR("未正确识别GeoJson geometry的type，读入类型应为: Polygon，实际类型为: {}", type_string);
        return false;
    }

    QJsonArray array = json.value(GeoJsonFeature::key_coordinates()).toArray();

    if (array.isEmpty())
    {
        SPDLOG_ERROR("Polygon 中未包含外界边框！");
        return false;
    }
    QJsonArray outer_boundary = array.first().toArray();
    const int  size           = outer_boundary.size();

    mCoordinates.clear();
    mCoordinates.resize(size);
    QJsonArray point_json;
    for (int i = 0; i < size; ++i)
    {
        point_json = outer_boundary[i].toArray();
        if (point_json.size() != point_size)
        {
            SPDLOG_ERROR("Polygon 的第{}个坐标长度不足2位", i);
            return false;
        }
        mCoordinates[i].x = point_json[0].toDouble();
        mCoordinates[i].y = point_json[1].toDouble();
    }
    return true;
}
