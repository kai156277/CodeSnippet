#pragma once
#include "DataUnit.h"
#include <memory>
#include <qmap.h>
#include <qstring.h>
#include <qstringlist.h>

#include <qplugin.h>

namespace pos {

class DataSource
{
public:
    typedef int32_t ErrorType;
    enum : ErrorType
    {
        kOk = 0,
        kEmptyInput,
        kUnsupportedValue,
    };
    DataSource() = default;

    virtual QString     Name() const                                                                    = 0;
    virtual QString     HumanReadableName() const                                                       = 0;
    virtual QString     Version() const                                                                 = 0;
    virtual int8_t      Major() const                                                                   = 0;
    virtual int8_t      Minor() const                                                                   = 0;
    virtual int8_t      Patch() const                                                                   = 0;
    virtual QStringList SupportVariables() const                                                        = 0;
    virtual DataUnit    Unit(const QString &variable_name) const                                        = 0;
    virtual ErrorType   GetValues(const QString &variable_name, /*time*/ QVector<double> *values) const = 0;
    virtual QString     Description(const QString &variable_name) const                                 = 0;
    virtual QString     ErrorString(ErrorType error) const                                              = 0;
    virtual QString     HumanReadableVariable(const QString &variable_name) const                       = 0;
};

typedef std::shared_ptr<pos::DataSource> DataSourceSptr;
typedef std::weak_ptr<pos::DataSource>   DataSourceWptr;
}   // namespace pos

#define DataSource_iid "supersurs.VSursPOS.core.DataSource/1.0.0"
Q_DECLARE_INTERFACE(pos::DataSource, DataSource_iid)

namespace pos {
class RandomTimeData : public DataSource
{
public:
    RandomTimeData();
    QString     Name() const override { return "RandomTime"; }
    QString     HumanReadableName() const override { return QString::fromUtf8("随机时间"); };
    QString     Version() const override { return "1.0.0"; }
    int8_t      Major() const override { return 1; }
    int8_t      Minor() const override { return 0; };
    int8_t      Patch() const override { return 0; };
    DataUnit    Unit(const QString &variable_name) const override;
    QStringList SupportVariables() const override;
    QString     HumanReadableVariable(const QString &variable_name) const;
    QString     Description(const QString &variable_name) const override;
    ErrorType   GetValues(const QString &variable_name, /*time*/ QVector<double> *values) const override;
    QString     ErrorString(ErrorType error) const override;

    typedef std::function<double()> value_fun;

private:
    struct Package
    {
        DataUnit  unit;
        QString   humanString;
        value_fun values;
        QString   description;
    };
    QMap<QString, Package> mapping_;
};
}   // namespace pos
