#include "ScanPoint.h"

#include <math.h>

ScanPoint::ScanPoint()
{
}

void ScanPoint::setPointXYZ(const PointXYZ &xyz, bool auto_calc)
{
}

double ScanPoint::distance()
{
    return mDistance;
}

void ScanPoint::zero()
{
    mDistance = 0.0;
}

double ScanPoint::distance(const PointXYZ &xyz)
{
    return sqrt(xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z);
}

double ScanPoint::xozProjDistance(const PointXYZ &xyz)
{
    return sqrt(xyz.x * xyz.x + xyz.z * xyz.z);
}
