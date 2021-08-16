#include "VariableFormat.h"
#include "DataUnit.h"
#include <qjsondocument.h>
#include <qjsonobject.h>

namespace {
const QString kAlignment        = QStringLiteral("alignment");
const QString kPadFieldWithZero = QStringLiteral("pad_field_with_zro");
const QString kDecimals         = QStringLiteral("decimals");
const QString kFieldWidth       = QStringLiteral("fieldWidth");
const QString kUnit             = QStringLiteral("unit");
}   // namespace

namespace pos {
VariableFormat::VariableFormat(int32_t field_width, int32_t decimals, Alignment alignment, bool pad_field)
    : field_width_(field_width)
    , decimals_(decimals)
    , alignment_(alignment)
    , pad_field_with_zero_(pad_field)
{
}

int32_t VariableFormat::field_width() const
{
    return field_width_;
}

void VariableFormat::set_field_width(int32_t field_width)
{
    field_width_ = field_width;
}

int32_t VariableFormat::decimals() const
{
    return decimals_;
}

void VariableFormat::set_decimals(int32_t decimals)
{
    decimals_ = decimals;
}

bool VariableFormat::pad_field_with_zero() const
{
    return pad_field_with_zero_;
}

void VariableFormat::set_pad_field_with_zero(bool with_zero)
{
    pad_field_with_zero_ = with_zero;
}

VariableFormat::Alignment VariableFormat::alignment() const
{
    return alignment_;
}

void VariableFormat::set_alignment(Alignment align)
{
    alignment_ = align;
}

DataUnit VariableFormat::unit() const
{
    return unit_;
}

void VariableFormat::set_unit(DataUnit unit)
{
    unit_ = unit;
}

QString VariableFormat::DecorativeData(double value) const
{
    if ((unit_ | DataUnit::kDefaultUnit) == DataUnit::kDefaultUnit)
    {
        if (kLeftAlignment)
            return QString::asprintf("%-*.*lf\n", field_width_, decimals_, value);
        else
        {
            if (pad_field_with_zero_)
                return QString::asprintf("%0*.*lf\n", field_width_, decimals_, value);
            else
                return QString::asprintf("% *.*lf\n", field_width_, decimals_, value);
        }
    }
    return QString();
}

QString VariableFormat::toJsonString() const
{
    QJsonObject object {
        {kAlignment, ((alignment_ == kLeftAlignment) ? "Left" : "Right")},
        {kPadFieldWithZero, pad_field_with_zero_},
        {kDecimals, decimals_},
        {kFieldWidth, field_width_},
        {kUnit, (int) unit_}};
    QJsonDocument doc(object);
    return doc.toJson(QJsonDocument::Compact);
}
void VariableFormat::fromJsonString(const QString &str)
{
    QJsonParseError err;
    QJsonDocument   doc = QJsonDocument::fromJson(str.toUtf8(), &err);
    if (err.error == QJsonParseError::NoError && doc.isObject())
    {
        auto json = doc.object();

        QString alignment = json.value(kAlignment).toString();
        if (alignment == "Left")
            alignment_ = kLeftAlignment;
        else
            alignment_ = kRightAlighment;

        pad_field_with_zero_ = json.value(kPadFieldWithZero).toBool();
        decimals_            = json.value(kDecimals).toInt(12);
        field_width_         = json.value(kFieldWidth).toInt(3);
        unit_                = static_cast<DataUnit>(json.value(kUnit).toInt());
    }
}
}   // namespace pos
