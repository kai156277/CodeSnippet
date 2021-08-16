#include "DiseaseObject.h"

#include <QMap>

static QMap<DiseaseType, QString> disease_mapping = {
    {DiseaseType::NoDisease, "NoDisease"},
    {DiseaseType::Crack, "Crack"},
    {DiseaseType::Leakage, "Leakage"},
    {DiseaseType::Dislocation, "Dislocation"},
    {DiseaseType::AxialDeformation, "AxialDeformation"},
    {DiseaseType::EllipseDeformation, "EllipseDeformation"},
};

QString DiseaseTypeToString(DiseaseType type)
{
    return disease_mapping.value(type);
}

DiseaseObject::DiseaseObject()
{
}

DiseaseObject::DiseaseObject(DiseaseObject::Type type)
    : mType(type)
{
}

DiseaseObject::Type DiseaseObject::type() const
{
    return mType;
}

void DiseaseObject::setType(const Type &type)
{
    mType = type;
}
