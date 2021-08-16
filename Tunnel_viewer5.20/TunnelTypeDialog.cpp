#include "TunnelTypeDialog.h"
#include "ui_TunnelTypeDialog.h"
#pragma execution_character_set("utf-8")
TunnelTypeDialog::TunnelTypeDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TunnelTypeDialog)
{
    ui->setupUi(this);
}

TunnelTypeDialog::~TunnelTypeDialog()
{
    delete ui;
}

void TunnelTypeDialog::on_pushButton_ok_clicked()
{
    this->close();
}

void TunnelTypeDialog::on_pushButton_cancel_clicked()
{
    this->close();
}
