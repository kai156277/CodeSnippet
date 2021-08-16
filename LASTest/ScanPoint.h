#pragma once

#include "PointType.h"

class ScanPoint
{
public:
    ScanPoint();
    explicit ScanPoint(const PointXYZ &xyz, PointClassification classification = NeverClassified, bool auto_calc = false);

    /**
     * @brief setPointXYZ
     * @param xyz
     * @param auto_calc 当 auto_calc 为 ture 时将自动计算 ScanPoint 的属性
     */
    void setPointXYZ(const PointXYZ &xyz, bool auto_calc = false);

    // 计算 ScanPoint 的属性
    double distance();
    void   preFiltering(double   min_dis,
                        double   max_dis,
                        uint16_t min_intensity,
                        uint16_t max_intensity);
    void   zero();

    static double distance(const PointXYZ &xyz);
    static double xozProjDistance(const PointXYZ &xyz);

    // 原始值
    PointXYZ mData;

    // 属性, 如果有一些计算值经常使用，可以在处理过程中保存为本地变量，
    // 或者将其修改为 ScanPoint 的属性，方便之后使用（同时也能节约计算时间，使用空间换时间）
    PointClassification mClassification = NeverClassified;
    double              mDistance       = 0.0;
};
