#pragma once

#include <QString>
#include <QVector>

#include <QJsonArray>

struct PointXY
{
    double x;
    double y;
};

using Points = QVector<PointXY>;

QJsonArray PointsToJsonArray(const Points &points);
bool       JsonArrayToPoints(const QJsonArray &array, Points &out_points);

QJsonArray PointXYToJsonArray(const PointXY &xy);
bool       JsonArrayToPointXY(const QJsonArray &array, PointXY &out_point);

struct GeoJsonFeature
{
    static QString value_feature_collection() { return "FeatureCollection"; }
    static QString value_feature() { return "Feature"; }

    static QString key_features() { return "features"; }
    static QString key_type() { return "type"; }
    static QString key_geometry() { return "geometry"; }
    static QString key_properties() { return "properties"; }
    static QString key_coordinates() { return "coordinates"; }

    // properties key:
    static QString key_ring_num() { return "ring_num"; }
    static QString key_mileage() { return "mileage"; }
    static QString key_disease() { return "disease"; }
    static QString key_deformations() { return "deformations"; }
    static QString key_direction() { return "direction"; }
    static QString key_deformation() { return "deformation"; }

    // 裂缝病害
    static QString key_crack_area() { return "crack_area"; }
    static QString key_crack_length() { return "crack_length"; }
    static QString key_crack_width() { return "crack_width"; }
    static QString key_crack_direction() { return "crack_direction"; }

    // 渗水病害
    static QString key_leakage_area() { return "leakage_area"; }

    // 错台
    static QString key_circumferential_dislocation() { return "circumferential_dislocation"; }
    static QString key_radial_dislocation() { return "radial_dislocation"; }
    static QString key_dislocation() { return "dislocation"; }

    // 形变量
    static QString key_ovality() { return "ovality"; }
    static QString key_axis_delta() { return "axis_delta"; }
    static QString key_diameter_value() { return "diameter_value"; }
    static QString key_absolute_deformation() { return "absolute_deformation"; }
    static QString key_relative_deformation() { return "relative_deformation"; }
};
