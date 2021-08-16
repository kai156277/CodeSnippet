#pragma once

#include "GeoJsonFeature.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QVector>

enum class GeometryType
{
    ErrorType = 0,
    Point,
    LineString,
    Polygon,
    MultiPoint,
    MultiLineString,
    MultiPolygon,
    GeometryCollection
};

QString      GeometryTypeToString(GeometryType type);
GeometryType StringToGeometryType(const QString &str);

class GeoJsonGeometryPoint
{
public:
    GeoJsonGeometryPoint() = default;

    PointXY coordinates() const;
    void    setCoordinates(const PointXY &coordinates);

    QJsonObject toGeoJson() const;
    bool        fromGeoJson(const QJsonObject &json);

    GeometryType type() const;

private:
    PointXY      mCoordinates;
    GeometryType mType = GeometryType::Point;
};

class GeoJsonGeometryLineString
{
public:
    GeoJsonGeometryLineString() = default;

    Points coordinates() const;
    void   setCoordinates(const Points &coordinates);

    QJsonObject toGeoJson() const;
    bool        fromGeoJson(const QJsonObject &json);

    GeometryType type() const;

private:
    Points       mCoordinates;
    GeometryType mType = GeometryType::LineString;
};

class GeoJsonGeometryPolygon
{
public:
    Points coordinates() const;
    void   setCoordinates(const Points &coordinates);

    GeometryType type() const;

    QJsonObject toGeoJson() const;
    bool        fromGeoJson(const QJsonObject &json);

private:
    Points       mCoordinates;   // 外侧边界
    GeometryType mType = GeometryType::Polygon;
};
