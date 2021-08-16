#pragma once

#include <stdint.h>

struct ScannerInformation
{
    uint16_t intensity;   // 反射强度
    uint32_t lineId;
    double   time;     // GPS [s]
    double   dist;     // 距离 [m]
    double   hAngle;   // 水平角 [rad]
    double   vAngle;   // 竖直角 [rad]
    double   x;        // [m]
    double   y;        // [m]
    double   z;        // [m]
};

struct ScannerCalibrationInfo
{
    double time;   // GPS [s]

    // 扫描仪坐标系下
    double dist;     // 距离 [m]
    double hAngle;   // 水平角 [rad]
    double vAngle;   // 竖直角 [rad]

    // 惯导坐标系下
    double roll;      // 横滚 [rad]
    double pitch;     // 俯仰 [rad]
    double heading;   // 航向 [rad]

    // ECEF 坐标系下 惯导中心位置
    double latitude;         // 纬度 [rad]
    double longitude;        // 经度 [rad]
    double geodeticHeight;   // 大地高 [m]
};
