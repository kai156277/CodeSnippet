#pragma once

#include "DiseaseObject.h"
#include "GeoJsonFeature.h"
#include "GeoJsonGeometry.h"

#include <QJsonObject>

class EllipseDeformationObject : public DiseaseObject
{
public:
    explicit EllipseDeformationObject(DiseaseType type = Type::EllipseDeformation);
    EllipseDeformationObject(const GeoJsonGeometryPoint &geometry);

    struct Deformation
    {
        double direction;
        double deformation;
    };

    GeoJsonGeometryPoint geometry() const;
    void                 setGeometry(const GeoJsonGeometryPoint &geometry);

    int  ringNum() const;
    void setRingNum(int ringNum);

    double ovality() const;
    void   setOvality(double ovality);

    double axisDelta() const;
    void   setAxisDelta(double axisDelta);

    double diameterValue() const;
    void   setDiameterValue(double diameterValue);

    double absoluteDeformation() const;
    void   setAbsoluteDeformation(double absoluteDeformation);

    double relativeDeformation() const;
    void   setRelativeDeformation(double relativeDeformation);

    QVector<Deformation> deformations() const;
    void                 setDeformations(const QVector<Deformation> &deformations);

    QJsonObject toGeoJsonObject() const;
    bool        fromGeoJsonObject(const QJsonObject &json);

    QString mileage() const;
    void    setMileage(const QString &mileage);

private:
    GeoJsonGeometryPoint mGeometry;
    QString              mMileage;
    int                  mRingNum;               // 环号
    double               mOvality;               // 椭圆度
    double               mAxisDelta;             // 长半轴和短半轴的差值(mm)
    double               mDiameterValue;         // 收敛直径(mm)
    double               mAbsoluteDeformation;   // 测量值与设计值的绝对误差 (mm)
    double               mRelativeDeformation;   // 测量值与设计值的相对误差
    QVector<Deformation> mDeformations;
};

using EllipseDeformation  = EllipseDeformationObject::Deformation;
using EllipseDeformations = QVector<EllipseDeformation>;
