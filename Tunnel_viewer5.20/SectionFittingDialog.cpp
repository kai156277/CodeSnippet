#include "SectionFittingDialog.h"

#include "SectionFitting.h"
#include "ui_SectionFittingDialog.h"

#include <QRadioButton>
#include <QPushButton>
#pragma execution_character_set("utf-8")
SectionFittingDialog::SectionFittingDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::SectionFittingDialog)
{
    ui->setupUi(this);
    this->ui->buttonBox->button(QDialogButtonBox::Ok)->setText("确定");
    this->ui->buttonBox->button(QDialogButtonBox::Cancel)->setText("取消");
    for (int i = 0; i < SectionFitting::algorithmCount(); ++i)
    {
        SectionFitting::Type type    = static_cast<SectionFitting::Type>(i);
        QRadioButton *       fitting = new QRadioButton(SectionFitting::TypeToQString(type), this);
        ui->verticalLayout->insertWidget(0, fitting);
        fittings_.push_back(fitting);
    }
}

SectionFittingDialog::~SectionFittingDialog()
{
    delete ui;
}

SectionFitting::Type SectionFittingDialog::selectType() const
{
    return type_;
}

void SectionFittingDialog::on_buttonBox_accepted()
{
    for (int i = 0; i < fittings_.count(); ++i)
    {
        if (fittings_[i]->isChecked())
            type_ = static_cast<SectionFitting::Type>(i);
    }
}
