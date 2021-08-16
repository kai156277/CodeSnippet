#pragma once
#include "DataUnit.h"
#include <qstring.h>

namespace pos {
class VariableFormat
{
public:
    enum Alignment
    {
        kLeftAlignment = 0,
        kRightAlighment
    };
    VariableFormat()          = default;
    virtual ~VariableFormat() = default;
    VariableFormat(int32_t field_width, int32_t decimals, Alignment alignment, bool pad_field);

    int32_t field_width() const;
    void    set_field_width(int32_t field_width);

    int32_t decimals() const;
    void    set_decimals(int32_t decimals);

    bool pad_field_with_zero() const;
    void set_pad_field_with_zero(bool with_zero);

    Alignment alignment() const;
    void      set_alignment(Alignment align);

    DataUnit unit() const;
    void     set_unit(DataUnit unit);

    virtual QString DecorativeData(double value) const;

    QString toJsonString() const;
    void    fromJsonString(const QString &str);

private:
    Alignment alignment_           = kRightAlighment;
    bool      pad_field_with_zero_ = false;
    int32_t   decimals_            = 3;
    int32_t   field_width_         = 12;
    DataUnit  unit_                = DataUnit::kNone;
};

}   // namespace pos
