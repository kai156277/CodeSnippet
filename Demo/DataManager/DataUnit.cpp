#include "DataUnit.h"
#include <map>
#include <qstring.h>

namespace {
struct UnitStrings
{
    QString str;
    QString human;
};
const std::map<pos::DataUnit, UnitStrings> kUnitStringMapping = {
    {pos::DataUnit::kNone, {"None", QString::fromUtf8("无")}},
    {pos::DataUnit::kMeter, {"Meter", QString::fromUtf8("米")}},
    {pos::DataUnit::kRadian, {"Radian", QString::fromUtf8("弧度")}},
    {pos::DataUnit::kMeterPerSec, {"Meter Per Sec", QString::fromUtf8("米/每秒")}},
    {pos::DataUnit::kRadPerSec, {"Radian Per Sec", QString::fromUtf8("弧度/每秒")}},
    {pos::DataUnit::kDefaultUnit, {"Default Unit", QString::fromUtf8("标准单位")}},
    {pos::DataUnit::kCentimeter, {"Centimeter", QString::fromUtf8("厘米")}},
    {pos::DataUnit::kMillimeter, {"Millimeter", QString::fromUtf8("毫米")}},
    {pos::DataUnit::kKilometer, {"Kilometer", QString::fromUtf8("千米")}},
    {pos::DataUnit::kDegreeMinuteSecondSigned, {"Degree Minute Second [Signed]", QString::fromUtf8("度 分 秒 (带符号)")}},
    {pos::DataUnit::kDegreeSigned, {"Degree [Signed]", QString::fromUtf8("度 (带符号)")}},
    {pos::DataUnit::kKilometersPerHour, {"Kilometers Per Hour", QString::fromUtf8("千米/每小时")}},
};
}   // namespace

namespace pos {

QString toString(DataUnit uint)
{
    auto search = kUnitStringMapping.find(uint);
    if (search != kUnitStringMapping.end())
    {
        return search->second.str;
    }
    return QString();
}

QString toHumanString(DataUnit uint)
{
    auto search = kUnitStringMapping.find(uint);
    if (search != kUnitStringMapping.end())
    {
        return search->second.human;
    }
    return QString();
}

bool pos::MeterToOther(double meter, DataUnit type, double *value)
{
    if (value == nullptr)
        return false;

    switch (type)
    {
    case DataUnit::kMeter:
        (*value) = meter;
        break;
    case DataUnit::kCentimeter:
        (*value) = meter * 100.0;
        break;
    case DataUnit::kMillimeter:
        (*value) = meter * 10000.0;
        break;
    case DataUnit::kKilometer:
        (*value) = meter / 1000.0;
        break;
    default:
        return false;
    }

    return true;
}

}   // namespace pos
