#pragma once

#include <qstring.h>
#include <stdint.h>

namespace pos {

class Unit
{
public:
    enum Type : uint32_t
    {
        kNone        = 0,
        kMeter       = 1 << 16,
        kRadian      = 1 << 17,
        kMeterPerSec = 1 << 18,
        kRadPerSec   = 1 << 19,
        kDefaultUnit = kNone | kMeter | kRadian | kMeterPerSec | kRadPerSec,
        /* 长度单位 */
        kCentimeter = kMeter + 1,
        kMillimeter,
        kKilometer,
        /* 角度单位 */
        kDegreeMinuteSecondSigned = kRadian + 1,
        kDegreeSigned,
        /* 速度单位 */
        kKilometersPerHour = kMeterPerSec + 1,
    };
};

typedef Unit::Type DataUnit;

QString toString(DataUnit uint);
QString toHumanString(DataUnit uint);

bool MeterToOther(double meter, DataUnit type, double *kilometer);
}   // namespace pos
