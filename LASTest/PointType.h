#pragma once

#include <stdint.h>

namespace wimapping {

}

enum PointClassification : uint8_t
{
    NeverClassified = 0,   /// 默认、不分类的
    Unclassified,          /// 未分类的
    Noise,                 /// 噪声
    KeyPoint,              /// 关键点
    Water,                 /// 水体
    Ground,                /// 土壤
    Vegetaion,             /// 植被
    Building,              /// 建筑
    UsrDefined = 0x80      /// 用户自定义类别应大于此值
};

/**
 * @brief The PointXYZ struct
 *
 * @note intensity: 强度值是脉冲返回幅度的整数表示。
 * 如果不包括强度，则必须将该值设置为零。
 * 当包含强度时，总是通过将该值乘以 65536/(传感器的强度动态范围)归一化为16位无符号值。
 * 例如，如果传感器的动态范围为10位，则缩放值为(65536/ 1024)。
 * 需要这种规范化来确保来自不同传感器的数据可以正确地合并。
 */
struct PointXYZ
{
    double   x         = 0.0;
    double   y         = 0.0;
    double   z         = 0.0;
    double   gpsTime   = 0.0;
    uint16_t intensity = 0;
};
