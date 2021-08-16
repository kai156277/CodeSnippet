#pragma once

#include <pcl/point_types.h>
//#include <stdint.h>
//自定义点类型PointXYZIT，包括XYZ坐标和强度，GPS时间
struct PointXYZIT
{
    PCL_ADD_POINT4D
    float  intensity;
    double gpsTime;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
// clang-format off

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, gpsTime, gpsTime))
