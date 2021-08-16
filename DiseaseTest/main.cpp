#include <QCoreApplication>

//#include "WiAcrLib/disease/AxialDeformationObject.h"
//#include "WiAcrLib/disease/CrackObject.h"
//#include "WiAcrLib/disease/DiseaseFile.h"
//#include "WiAcrLib/disease/DislocationObject.h"
//#include "WiAcrLib/disease/EllipseDeformationObject.h"
//#include "WiAcrLib/disease/GeoJsonFeature.h"
//#include "WiAcrLib/disease/GeoJsonGeometry.h"
//#include "WiAcrLib/disease/LeakageObject.h"

#include "AxialDeformationObject.h"
#include "CrackObject.h"
#include "DiseaseFile.h"
#include "DislocationObject.h"
#include "EllipseDeformationObject.h"
#include "GeoJsonFeature.h"
#include "GeoJsonGeometry.h"
#include "LeakageObject.h"

#include <QDebug>
#include <QFile>
#include <QJsonDocument>

#include <random>

void axial_deformation_test();
void convergence_diameter_test();
void crack_test();
void dislocation_test();
void ellipse_deformation_test();
void leakage_test();

std::random_device                     rd;          //Will be used to obtain a seed for the random number engine
std::mt19937                           gen(1996);   //Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<double> real_dis(-5.0, 5.0);
std::uniform_int_distribution<int32_t> int_dis(0, 100);
std::uniform_real_distribution<double> angle_dis(60.0, 240.0);

using namespace wim;
using namespace wim::acr;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    axial_deformation_test();
    // convergence_diameter_test();
    crack_test();
    dislocation_test();
    ellipse_deformation_test();
    leakage_test();
    return 0;
}

void axial_deformation_test()
{
    DiseaseFile save_file;

    QString file_name  = "axial_deformation.json";
    QString file_name1 = "axial_deformation1.json";
    for (int i = 1; i < 5; ++i)
    {
        GeoJsonGeometryLineString geometry;
        Points                    line_string;
        for (int j = 0; j < 10; ++j)
        {
            line_string.push_back({real_dis(gen), real_dis(gen)});
        }
        geometry.setCoordinates(line_string);

        AxialDeformationObject *dislocation = new AxialDeformationObject(geometry);
        dislocation->setDeformations(line_string);

        save_file.mDiseases.push_back(dislocation);
    }
    save_file.saveToGeoJson(file_name);

    DiseaseFile read_file;
    read_file.readFromGeoJsonFile(file_name, DiseaseType::AxialDeformation);

    for (int i = 0; i < read_file.mDiseases.size(); ++i)
    {
        QJsonObject   obj = read_file.mDiseases[i]->toGeoJsonObject();
        QJsonDocument doc(obj);
        qDebug() << i << ":" << doc.toJson(QJsonDocument::Indented);
    }
    read_file.saveToGeoJson(file_name1);
}
void crack_test()
{
    DiseaseFile save_file;

    QString file_name  = "crack.json";
    QString file_name1 = "crack1.json";

    for (int i = 1; i < 5; ++i)
    {
        GeoJsonGeometryPolygon geometry;
        Points                 line_string;
        for (int j = 0; j < 10; ++j)
        {
            line_string.push_back({real_dis(gen), real_dis(gen)});
        }
        line_string.push_back(line_string.first());
        geometry.setCoordinates(line_string);

        CrackObject *dislocation = new CrackObject(geometry);
        dislocation->setRingNum(int_dis(gen));
        dislocation->setCrackArea(real_dis(gen));
        dislocation->setCrackLength(real_dis(gen));
        dislocation->setCrackWidth(real_dis(gen));
        dislocation->setDirection((CrackDirection)(i % 3));

        save_file.mDiseases.push_back(dislocation);
    }
    save_file.saveToGeoJson(file_name);

    DiseaseFile read_file;
    read_file.readFromGeoJsonFile(file_name, DiseaseType::Crack);

    for (int i = 0; i < read_file.mDiseases.size(); ++i)
    {
        QJsonObject   obj = read_file.mDiseases[i]->toGeoJsonObject();
        QJsonDocument doc(obj);
        qDebug() << i << ":" << doc.toJson(QJsonDocument::Indented);
    }
    read_file.saveToGeoJson(file_name1);
}

void dislocation_test()
{
    DiseaseFile save_file;

    for (int i = 1; i < 5; ++i)
    {
        GeoJsonGeometryPoint geometry_point;
        geometry_point.setCoordinates({0.77, 10.5 / i});

        RadialDislocations radials;
        for (int j = 0; j < 6; ++j)
        {
            radials.push_back({angle_dis(gen), real_dis(gen)});
        }
        DislocationObject *dislocation = new DislocationObject(geometry_point);
        dislocation->setRingNum(int_dis(gen));
        dislocation->setCircumferentialDislocation(real_dis(gen));
        dislocation->setRadialDislocation(radials);

        save_file.mDiseases.push_back(dislocation);
    }
    save_file.saveToGeoJson("dislocation_demo.json");

    DiseaseFile read_file;
    read_file.readFromGeoJsonFile("dislocation_demo.json", DiseaseType::Dislocation);

    for (int i = 0; i < read_file.mDiseases.size(); ++i)
    {
        QJsonObject   obj = read_file.mDiseases[i]->toGeoJsonObject();
        QJsonDocument doc(obj);
        qDebug() << i << ":" << doc.toJson(QJsonDocument::Indented);
    }
    read_file.saveToGeoJson("dislocation_demo_copy.json");
}

void ellipse_deformation_test()
{
    DiseaseFile save_file;

    QString file_name  = "ellipse_deformation.json";
    QString file_name1 = "ellipse_deformation1.json";

    for (int i = 1; i < 5; ++i)
    {
        GeoJsonGeometryPoint geometry_point;
        geometry_point.setCoordinates({real_dis(gen), real_dis(gen)});

        EllipseDeformations deformations;

        for (int j = 0; j < 3; ++j)
        {
            deformations.push_back({angle_dis(gen), real_dis(gen)});
        }

        EllipseDeformationObject *dislocation = new EllipseDeformationObject(geometry_point);
        dislocation->setRingNum(int_dis(gen));
        dislocation->setOvality(real_dis(gen));
        dislocation->setAxisDelta(real_dis(gen));
        dislocation->setDiameterValue(real_dis(gen));
        dislocation->setAbsoluteDeformation(real_dis(gen));
        dislocation->setRelativeDeformation(real_dis(gen));
        dislocation->setDeformations(deformations);

        save_file.mDiseases.push_back(dislocation);
    }
    save_file.saveToGeoJson(file_name);

    DiseaseFile read_file;
    read_file.readFromGeoJsonFile(file_name, DiseaseType::EllipseDeformation);

    for (int i = 0; i < read_file.mDiseases.size(); ++i)
    {
        QJsonObject   obj = read_file.mDiseases[i]->toGeoJsonObject();
        QJsonDocument doc(obj);
        qDebug() << i << ":" << doc.toJson(QJsonDocument::Indented);
    }
    read_file.saveToGeoJson(file_name1);
}

void leakage_test()
{
    DiseaseFile save_file;

    QString file_name  = "leakage.json";
    QString file_name1 = "leakage1.json";

    for (int i = 1; i < 5; ++i)
    {
        GeoJsonGeometryPolygon geometry;
        Points                 line_string;
        for (int j = 0; j < 10; ++j)
        {
            line_string.push_back({real_dis(gen), real_dis(gen)});
        }
        line_string.push_back(line_string.first());
        geometry.setCoordinates(line_string);

        LeakageObject *dislocation = new LeakageObject(geometry);
        dislocation->setRingNum(int_dis(gen));
        dislocation->setLeakageArea(real_dis(gen));

        save_file.mDiseases.push_back(dislocation);
    }
    save_file.saveToGeoJson(file_name);

    DiseaseFile read_file;
    read_file.readFromGeoJsonFile(file_name, DiseaseType::Leakage);

    for (int i = 0; i < read_file.mDiseases.size(); ++i)
    {
        QJsonObject   obj = read_file.mDiseases[i]->toGeoJsonObject();
        QJsonDocument doc(obj);
        qDebug() << i << ":" << doc.toJson(QJsonDocument::Indented);
    }
    read_file.saveToGeoJson(file_name1);
}
