#include "SectionFitting.h"

#include <QString>
#include <QStringList>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <thread>
#pragma execution_character_set("utf-8")
QString SectionFitting::TypeToQString(SectionFitting::Type type)
{
    QStringList strs = {
        QString::fromUtf8("三次B样条曲线拟合法"),
        QString::fromUtf8("贝塞尔曲线拟合法"),
        QString::fromUtf8("None"),
    };
    return strs.value(type, "None");
}

int SectionFitting::algorithmCount()
{
    return SectionFitting::None;
}

bool SectionFitting::isProcess()
{
    return process;
}

void SectionFitting::fitting(SectionFitting::Type type, const TunnelPointCloud &section, TunnelPointCloud &newvector, int numberofnewpoint)
{
    if (process)
        return;
    process = true;
    switch (type)
    {
    case SectionFitting::Bspline:
        bspline(section, newvector, numberofnewpoint);
        break;
    case SectionFitting::Bezier:
        bezier(section, newvector, numberofnewpoint);
        break;
    default:
        break;
    }
    process = false;
}

void SectionFitting::async_fitting(SectionFitting::Type type, const TunnelPointCloud &section, TunnelPointCloud &newvector, int numberofnewpoint)
{
    std::thread tmp(&SectionFitting::fitting, this, type, section, std::ref(newvector), numberofnewpoint);
    tmp.detach();
}

void SectionFitting::bezier(const TunnelPointCloud &section, TunnelPointCloud &newvector, int numberofnewpoint)
{
    int  process_num = 0;
    emit processNum(process_num);
    newvector = section;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(section, minPt, maxPt);
    int MARK;
    if (maxPt.y - minPt.y > maxPt.x - minPt.x)   //断面走向方向比较薄
    {
        MARK = 1;   //走向沿X轴
    }
    else
    {
        MARK = 0;   //走向沿Y轴
    }

    double n0, n1, n2, n3;
    double dt = 1.0 / numberofnewpoint;
    dt        = dt - 0.000000000000000000000000000000001;
    double z0 = (maxPt.z + minPt.z) / 2;
    double MARK0;
    if (MARK == 0)
    {
        MARK0 = (maxPt.x + minPt.x) / 2;
    }
    else
    {
        MARK0 = (maxPt.y + minPt.y) / 2;
    }
    pcl::PointCloud<pcl::PointXYZI> SectionWithAzimuth;
    pcl::copyPointCloud(section, SectionWithAzimuth);
    process_num = 5;
    emit processNum(process_num);
    //SectionWithAzimuth.points[0].
    //计算方位角
    int process_time = SectionWithAzimuth.points.size();
    int add_count    = process_time / 30;
    if (MARK == 0)
    {
        for (int i = 0; i < SectionWithAzimuth.points.size(); i++)
        {
            if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].x > MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].x < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = 360 + (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z < z0 && SectionWithAzimuth.points[i].x < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535 + 180;
            }
            else
            {
                SectionWithAzimuth.points[i].intensity = 180 + (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            if (i % add_count == 0)
                emit processNum(process_num += 1);
        }
    }
    else
    {
        for (int i = 0; i < SectionWithAzimuth.points.size(); i++)
        {
            if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].y > MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].y < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = 360 + (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z < z0 && SectionWithAzimuth.points[i].y < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535 + 180;
            }
            else
            {
                SectionWithAzimuth.points[i].intensity = 180 + (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            if (i % add_count == 0)
                emit processNum(process_num += 1);
        }
    }

    for (int i = 0; i < SectionWithAzimuth.points.size() - 1; i++)   //冒泡法排序
    {
        for (int j = 0; j < SectionWithAzimuth.points.size() - 1 - i; j++)   // j开始等于0，
        {
            if (SectionWithAzimuth.points[j].intensity < SectionWithAzimuth.points[j + 1].intensity)
            {

                pcl::PointXYZI point0            = SectionWithAzimuth.points[j];
                SectionWithAzimuth.points[j]     = SectionWithAzimuth.points[j + 1];
                SectionWithAzimuth.points[j + 1] = point0;
            }
        }
        if (i % add_count == 0)
            emit processNum(process_num += 1);
    }

    for (int i = 0; i < SectionWithAzimuth.points.size() - 3; i++)
    {
        for (double t = dt; t < 1; t = t + dt)
        {
            n0 = (1 - t) * (1 - t) * (1 - t);
            n1 = (3 * t) * (1 - t) * (1 - t);
            n2 = (3 * t * t) * (1 - t);
            n3 = t * t * t;
            pcl::PointXYZ point0;
            point0.x = SectionWithAzimuth.points[i].x * n0 + SectionWithAzimuth.points[i + 1].x * n1 + SectionWithAzimuth.points[i + 2].x * n2 + SectionWithAzimuth.points[i + 3].x * n3;
            point0.y = SectionWithAzimuth.points[i].y * n0 + SectionWithAzimuth.points[i + 1].y * n1 + SectionWithAzimuth.points[i + 2].y * n2 + SectionWithAzimuth.points[i + 3].y * n3;
            point0.z = SectionWithAzimuth.points[i].z * n0 + SectionWithAzimuth.points[i + 1].z * n1 + SectionWithAzimuth.points[i + 2].z * n2 + SectionWithAzimuth.points[i + 3].z * n3;
            newvector.points.push_back(point0);
        }
        if (i % add_count == 0)
            emit processNum(process_num += 1);
    }
    emit processNum(100);
}

void SectionFitting::bspline(const TunnelPointCloud &section, TunnelPointCloud &newvector, int numberofnewpoint)
{
    int  process_num = 0;
    emit processNum(process_num);
    newvector = section;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(section, minPt, maxPt);
    int MARK;
    if (maxPt.y - minPt.y > maxPt.x - minPt.x)   //断面走向方向比较薄
    {
        MARK = 1;   //走向沿X轴
    }
    else
    {
        MARK = 0;   //走向沿Y轴
    }

    double n0, n1, n2, n3;
    double dt = 1.0 / numberofnewpoint;
    dt        = dt - 0.000000000000000000000000000000001;
    double z0 = (maxPt.z + minPt.z) / 2;
    double MARK0;
    if (MARK == 0)
    {
        MARK0 = (maxPt.x + minPt.x) / 2;
    }
    else
    {
        MARK0 = (maxPt.y + minPt.y) / 2;
    }
    pcl::PointCloud<pcl::PointXYZI> SectionWithAzimuth;
    pcl::copyPointCloud(section, SectionWithAzimuth);
    process_num = 5;
    emit processNum(process_num);
    //SectionWithAzimuth.points[0].
    //计算方位角
    int process_time = SectionWithAzimuth.points.size();
    int add_count    = process_time / 30;
    if (MARK == 0)
    {
        for (int i = 0; i < SectionWithAzimuth.points.size(); i++)
        {
            if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].x > MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].x < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = 360 + (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z < z0 && SectionWithAzimuth.points[i].x < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535 + 180;
            }
            else
            {
                SectionWithAzimuth.points[i].intensity = 180 + (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            if (i % add_count == 0)
                emit processNum(process_num += 1);
        }
    }
    else
    {
        for (int i = 0; i < SectionWithAzimuth.points.size(); i++)
        {
            if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].y > MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].y < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = 360 + (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z < z0 && SectionWithAzimuth.points[i].y < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535 + 180;
            }
            else
            {
                SectionWithAzimuth.points[i].intensity = 180 + (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            if (i % add_count == 0)
                emit processNum(process_num += 1);
        }
    }

    for (int i = 0; i < SectionWithAzimuth.points.size() - 1; i++)   //冒泡法排序
    {
        for (int j = 0; j < SectionWithAzimuth.points.size() - 1 - i; j++)   // j开始等于0，
        {
            if (SectionWithAzimuth.points[j].intensity < SectionWithAzimuth.points[j + 1].intensity)
            {

                pcl::PointXYZI point0            = SectionWithAzimuth.points[j];
                SectionWithAzimuth.points[j]     = SectionWithAzimuth.points[j + 1];
                SectionWithAzimuth.points[j + 1] = point0;
            }
        }
        if (i % add_count == 0)
            emit processNum(process_num += 1);
    }

    for (int i = 0; i < SectionWithAzimuth.points.size() - 3; i++)
    {
        for (double t = dt; t < 1; t = t + dt)
        {
            n0 = (-t * t * t + 3 * t * t - 3 * t + 1) / 6.0;
            n1 = (3 * t * t * t - 6 * t * t + 4) / 6.0;
            n2 = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6.0;
            n3 = t * t * t / 6.0;
            pcl::PointXYZ point0;
            point0.x = SectionWithAzimuth.points[i].x * n0 + SectionWithAzimuth.points[i + 1].x * n1 + SectionWithAzimuth.points[i + 2].x * n2 + SectionWithAzimuth.points[i + 3].x * n3;
            point0.y = SectionWithAzimuth.points[i].y * n0 + SectionWithAzimuth.points[i + 1].y * n1 + SectionWithAzimuth.points[i + 2].y * n2 + SectionWithAzimuth.points[i + 3].y * n3;
            point0.z = SectionWithAzimuth.points[i].z * n0 + SectionWithAzimuth.points[i + 1].z * n1 + SectionWithAzimuth.points[i + 2].z * n2 + SectionWithAzimuth.points[i + 3].z * n3;
            newvector.points.push_back(point0);
        }
        if (i % add_count == 0)
            emit processNum(process_num += 1);
    }
    emit processNum(100);
}
