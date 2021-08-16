#pragma once
#include "DataSource.h"
#include <QVector>
#include <qmap.h>

namespace pos {

class DataManager
{
public:
    enum ErrorType
    {
        kOk = 0,
        kEmptyInput,
        kExisted,
        kUnregisteredDataSource,
    };

    static DataManager *Instance();

    ErrorType      RegisterDataSource(DataSource *src);
    ErrorType      UnregisterDataSource(const QString &src_name);
    int            NumOfDataSource() const;
    QStringList    DataSourceNames() const;
    DataSourceWptr GetDataSource(const QString &src_name) const;
    bool           Contains(const QString &src_name) const;

private:
    QMap<QString, DataSourceSptr> data_sources_;
    DataManager();
};

}   // namespace pos
