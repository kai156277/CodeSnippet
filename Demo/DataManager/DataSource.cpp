#include "DataSource.h"
#include <qdatetime.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qvector.h>

namespace pos {

QString DataSource::ErrorString(ErrorType error) const
{
    switch (error)
    {
    case kOk:
        return "Ok";
    case kUnsupportedValue:
        return "Unsupported Value";
    default:
        return "";
    }
}

RandomTimeData::RandomTimeData()
{
    mapping_.insert("RandomTime::Year",
                    {DataUnit::kNone,
                     QString::fromUtf8("年"),
                     []() { return QDate::currentDate().year(); },
                     "Year"});
    mapping_.insert("RandomTime::Month",
                    {DataUnit::kMeter,
                     QString::fromUtf8("月"),
                     []() { return QDate::currentDate().month(); },
                     "Month"});
    mapping_.insert("RandomTime::Day",
                    {DataUnit::kMeterPerSec,
                     QString::fromUtf8("日"),
                     []() { return QDate::currentDate().day(); },
                     "Day"});
    mapping_.insert("RandomTime::Hour",
                    {DataUnit::kRadian,
                     QString::fromUtf8("时"),
                     []() { return QTime::currentTime().hour(); },
                     "Hour"});
    mapping_.insert("RandomTime::Minute",
                    {DataUnit::kRadPerSec,
                     QString::fromUtf8("分"),
                     []() { return QTime::currentTime().minute(); },
                     "Minute"});
    mapping_.insert("RandomTime::Second",
                    {DataUnit::kNone,
                     QString::fromUtf8("秒"),
                     []() { return QTime::currentTime().second(); },
                     "Second"});
}

DataUnit RandomTimeData::Unit(const QString &value_name) const
{
    if (mapping_.contains(value_name))
        return mapping_[value_name].unit;

    return DataUnit::kNone;
}

QStringList RandomTimeData::SupportVariables() const
{
    return mapping_.keys();
}

QString RandomTimeData::HumanReadableVariable(const QString &value_name) const
{
    if (mapping_.contains(value_name))
        return mapping_[value_name].humanString;

    return "";
}

QString RandomTimeData::Description(const QString &value_name) const
{
    if (mapping_.contains(value_name))
        return mapping_[value_name].description;

    return "";
}

DataSource::ErrorType RandomTimeData::GetValues(const QString &value_name, QVector<double> *values) const
{
    if (!values)
        return kEmptyInput;

    if (mapping_.contains(value_name))
    {
        values->clear();
        values->push_back(mapping_[value_name].values());
    }
    else
    {
        return kUnsupportedValue;
    }
    return kOk;
}

QString RandomTimeData::ErrorString(ErrorType error) const
{
    return DataSource::ErrorString(error);
}

}   // namespace pos
