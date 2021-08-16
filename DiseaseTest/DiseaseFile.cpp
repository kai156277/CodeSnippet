#include "DiseaseFile.h"
#include "AxialDeformationObject.h"
#include "CrackObject.h"
#include "DiseaseObject.h"
#include "DislocationObject.h"
#include "EllipseDeformationObject.h"
#include "LeakageObject.h"

#include <QDeadlineTimer>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>

#include <spdlog/qt_spdlog.h>

template <typename T>
bool objectConstruct(QVector<DiseaseObject *> &diseases, const QJsonArray &array, DiseaseType type)
{
    const int size = array.size();
    diseases.reserve(size);
    for (int i = 0; i < size; ++i)
    {
        diseases.push_back(new T(type));
        if (!diseases[i]->fromGeoJsonObject(array[i].toObject()))
        {
            SPDLOG_ERROR("第 {} 个病害数据读取错误!", i);
            qDeleteAll(diseases);
            diseases.clear();
            return false;
        }
    }
    return true;
}

DiseaseFile::DiseaseFile()
{
}

DiseaseFile::~DiseaseFile()
{
    qDeleteAll(mDiseases);
    mDiseases.clear();
}

void DiseaseFile::saveToGeoJson(const QString &file) const
{
    QJsonObject obj;
    obj.insert(GeoJsonFeature::key_type(), GeoJsonFeature::value_feature_collection());
    QJsonArray array;
    for (int i = 0; i < mDiseases.size(); ++i)
    {
        array.push_back(mDiseases[i]->toGeoJsonObject());
    }
    obj.insert(GeoJsonFeature::key_features(), array);

    QJsonDocument doc(obj);

    QFileInfo save_file_info(file);
    QDir      root(save_file_info.absoluteDir());

    root.mkpath(save_file_info.absolutePath());

    QFile save_file(file);
    if (!save_file.open(QIODevice::WriteOnly))
    {
        SPDLOG_ERROR("打开文件: {} 失败", file);
        return;
    }
    save_file.write(doc.toJson(QJsonDocument::Indented));
}

bool DiseaseFile::readFromGeoJsonFile(const QString &file, DiseaseType type)
{
    QFile save_file(file);
    if (!save_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_ERROR("打开文件: {} 失败", file);
        return false;
    }

    auto        doc      = QJsonDocument::fromJson(save_file.readAll());
    QJsonObject root     = doc.object();
    QJsonArray  features = root.value(GeoJsonFeature::key_features()).toArray();

    switch (type)
    {
    case DiseaseType::AxialDeformation:
        return objectConstruct<AxialDeformationObject>(mDiseases, features, type);
    case DiseaseType::Crack:
        return objectConstruct<CrackObject>(mDiseases, features, type);
    case DiseaseType::Dislocation:
        return objectConstruct<DislocationObject>(mDiseases, features, type);
    case DiseaseType::EllipseDeformation:
        return objectConstruct<EllipseDeformationObject>(mDiseases, features, type);
    case DiseaseType::Leakage:
        return objectConstruct<LeakageObject>(mDiseases, features, type);
    default:
        SPDLOG_ERROR("不正确的病害数据类型: {}", type);
        return false;
    }
}
