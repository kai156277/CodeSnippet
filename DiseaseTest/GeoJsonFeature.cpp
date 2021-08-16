#include "GeoJsonFeature.h"
#include <QMap>
#include <QString>

QJsonArray PointsToJsonArray(const Points &points)
{
    QJsonArray array;
    for (int i = 0; i < points.size(); ++i)
    {
        array.push_back(PointXYToJsonArray(points[i]));
    }
    return array;
}

bool JsonArrayToPoints(const QJsonArray &array, Points &out_points)
{
    out_points.clear();
    out_points.resize(array.size());
    for (int i = 0; i < array.size(); ++i)
    {
        if (!JsonArrayToPointXY(array[i].toArray(), out_points[i]))
        {
            out_points.clear();
            return false;
        }
    }
    return true;
}

QJsonArray PointXYToJsonArray(const PointXY &xy)
{
    return {xy.x, xy.y};
}

bool JsonArrayToPointXY(const QJsonArray &array, PointXY &out_point)
{
    if (array.size() != 2)
        return false;

    out_point.x = array[0].toDouble(0);
    out_point.y = array[1].toDouble(0);
    return true;
}
