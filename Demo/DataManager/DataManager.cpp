#include "DataManager.h"
#include "DataSource.h"

namespace pos {

DataManager::DataManager()
{
}

DataManager *DataManager::Instance()
{
    static DataManager manager;
    return &manager;
}

DataManager::ErrorType DataManager::RegisterDataSource(DataSource *src)
{
    if (src == nullptr)
    {
        return kEmptyInput;
    }

    DataSourceSptr src_sptr(src);

    if (data_sources_.contains(src->Name()))
    {
        return kExisted;
    }

    data_sources_.insert(src->Name(), src_sptr);
    return kOk;
}

DataManager::ErrorType DataManager::UnregisterDataSource(const QString &src_name)
{
    auto search = data_sources_.find(src_name);

    if (search != data_sources_.end())
    {
        data_sources_.erase(search);
    }
    return kOk;
}

int DataManager::NumOfDataSource() const
{
    return data_sources_.size();
}

QStringList DataManager::DataSourceNames() const
{
    return data_sources_.keys();
}

DataSourceWptr DataManager::GetDataSource(const QString &src_name) const
{
    auto search = data_sources_.find(src_name);

    if (search != data_sources_.end())
    {
        return search.value();
    }
    return DataSourceWptr();
}

bool DataManager::Contains(const QString &src_name) const
{
    return data_sources_.contains(src_name);
}

}   // namespace pos
