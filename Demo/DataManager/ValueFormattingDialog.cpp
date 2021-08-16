#include "ValueFormattingDialog.h"
#include "DataManager.h"
#include <QtWidgets/qdialog.h>
#include <QtWidgets/qpushbutton.h>

ValueFormattingDialog::ValueFormattingDialog(QWidget *parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    connect(ui.ok_pushButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(ui.cancel_pushButton, &QPushButton::clicked, this, &QDialog::reject);
}

pos::VariableFormat ValueFormattingDialog::format() const
{
    return format_;
}

void ValueFormattingDialog::SetValueFormat(const QString &            src_name,
                                           const QString &            var_name,
                                           const pos::VariableFormat &format)
{
    auto wptr = pos::DataManager::Instance()->GetDataSource(src_name);

    if (auto sptr = wptr.lock())
    {
        QString var_human_str = sptr->HumanReadableVariable(var_name);
        setWindowTitle(var_human_str + QString::fromUtf8(" 的输出格式"));
        pos::DataUnit unit = sptr->Unit(var_name);
        if (unit == pos::DataUnit::kNone)
        {
            ui.format_or_unit_comboBox->clear();
            ui.format_or_unit_comboBox->setEnabled(false);
        }
        else if (unit == pos::DataUnit::kMeter)
        {
            uint32_t tmp = pos::DataUnit::kMeter;
            ui.format_or_unit_comboBox->clear();
            for (; tmp < pos::DataUnit::kRadian; ++tmp)
            {
                QString str = pos::toHumanString((pos::DataUnit) tmp);
                if (str.isEmpty())
                    break;

                ui.format_or_unit_comboBox->addItem(str, tmp);
            }
        }
        else if (unit == pos::DataUnit::kRadian)
        {
            uint32_t tmp = unit;
            ui.format_or_unit_comboBox->clear();
            for (; tmp < pos::DataUnit::kMeterPerSec; ++tmp)
            {
                QString str = pos::toHumanString((pos::DataUnit) tmp);
                if (str.isEmpty())
                    break;

                ui.format_or_unit_comboBox->addItem(str, tmp);
            }
        }
        else if (unit == pos::DataUnit::kMeterPerSec)
        {
            uint32_t tmp = unit;
            ui.format_or_unit_comboBox->clear();
            for (; tmp < pos::DataUnit::kRadPerSec; ++tmp)
            {
                QString str = pos::toHumanString((pos::DataUnit) tmp);
                if (str.isEmpty())
                    break;

                ui.format_or_unit_comboBox->addItem(str, tmp);
            }
        }
        else if (unit == pos::DataUnit::kRadPerSec)
        {
            uint32_t tmp = unit;
            ui.format_or_unit_comboBox->clear();
            for (; tmp < pos::DataUnit::kDefaultUnit; ++tmp)
            {
                QString str = pos::toHumanString((pos::DataUnit) tmp);
                if (str.isEmpty())
                    break;

                ui.format_or_unit_comboBox->addItem(str, tmp);
            }
        }
        format_              = format;
        source_name_         = src_name;
        variable_name_       = var_name;
        variable_human_name_ = var_human_str;

        if (format_.unit() != pos::DataUnit::kNone)
            ui.format_or_unit_comboBox->setCurrentIndex(format_.unit() - unit);

        ui.field_width_spinBox->setValue(format_.field_width());
        ui.num_of_decimals_spinBox->setValue(format_.decimals());
        ui.pad_field_with_zero_checkBox->setChecked(format_.pad_field_with_zero());
        ui.left_justify_checkBox->setChecked(format_.alignment() == pos::VariableFormat::kLeftAlignment);
    }
}

QString ValueFormattingDialog::source_name() const
{
    return source_name_;
}

QString ValueFormattingDialog::variable_name() const
{
    return variable_name_;
}

QString ValueFormattingDialog::variable_human_name() const
{
    return variable_human_name_;
}

void ValueFormattingDialog::accept()
{
    format_.set_unit(static_cast<pos::DataUnit>(ui.format_or_unit_comboBox->currentData().toInt()));
    format_.set_alignment(ui.left_justify_checkBox->isChecked() ? pos::VariableFormat::kLeftAlignment : pos::VariableFormat::kRightAlighment);
    format_.set_decimals(ui.num_of_decimals_spinBox->value());
    format_.set_field_width(ui.field_width_spinBox->value());
    format_.set_pad_field_with_zero(ui.pad_field_with_zero_checkBox->isChecked());

    QDialog::accept();
}
