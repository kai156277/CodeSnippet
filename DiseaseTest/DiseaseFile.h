#pragma once

#include <QJsonObject>
#include <QVector>

#include "DiseaseObject.h"
#include "GeoJsonGeometry.h"

class DiseaseFile
{
public:
    QVector<DiseaseObject *> mDiseases;

    DiseaseFile();
    ~DiseaseFile();

    void saveToGeoJson(const QString &file) const;
    bool readFromGeoJsonFile(const QString &file, DiseaseType type);
};
