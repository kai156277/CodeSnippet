#include "AxisDialog.h"
#include "ui_AxisDialog.h"
#pragma execution_character_set("utf-8")
AxisDialog::AxisDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AxisDialog)
{
    ui->setupUi(this);
    connect(this->ui->pushButton,SIGNAL(clicked(bool)),this,SLOT(getMethod()));
}

AxisDialog::~AxisDialog()
{
    delete ui;
}
void AxisDialog::getMethod()
{
    if(ui->radioButton_projection->isChecked())
    {
        method=0;
    }
    else if(ui->radioButton_normal->isChecked())
    {
        method=1;
    }
    this->close();
}
