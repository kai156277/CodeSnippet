#include "ExtractionDialog.h"
#include "ui_ExtractionDialog.h"
#pragma execution_character_set("utf-8")
ExtractionDialog::ExtractionDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ExtractionDialog)
{
    ui->setupUi(this);
    connect(this->ui->pushButton,SIGNAL(clicked(bool)),this,SLOT(getMethod()));
}

ExtractionDialog::~ExtractionDialog()
{
    delete ui;
}
void ExtractionDialog::getMethod()
{
    if(ui->radioButton_section->isChecked())
    {
        method=0;
    }
    else if(ui->radioButton_sections->isChecked())
    {
        method=1;
    }
    this->close();
}
