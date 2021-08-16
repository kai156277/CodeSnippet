#pragma once

#include "TunnelCrossSection.h"

#include "TunnelRing.h"

#include <QString>
#include <QVector>

class TunnelPointCloud
{
public:
    TunnelPointCloud();

    QVector<TunnelRing> mPointCloud;

    // TODO: 可以先按一定的线数将整段点云的数据分割成环
    QVector<TunnelCrossSection> mAllLineData;
    bool                        saveLineFile(int begin_line, int end_line, QString save_file);
    bool                        readDataFromLASFile(const QString &las_file);

    // TODO: AxialDeformation
    struct AxialDeformationValue
    {
        double x;
        double y;
    };

    QVector<AxialDeformationValue> mDeformations;
    void                           calcuateAxialDeformation();

    // TODO: CircumferentialDislocation
    QVector<double> mCircumferentialDislocation;
    void            calcuateCircumferentialDislocation();

    static bool flsToLAS(const QString &fls_file, const QString &las_file);

    //    bool readDataFromLASFile(const QString &las_file);
};
