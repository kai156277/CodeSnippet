#include "dialog_sections.h"
#include "ui_dialog_sections.h"
#include<QMessageBox>
#include< QFileDialog>
#pragma execution_character_set("utf-8")
Dialog_Sections::Dialog_Sections(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog_Sections)
{
    ui->setupUi(this);
    connect(this->ui->btn_ok,SIGNAL(clicked(bool)),this,SLOT(GetParameter()));
    this->ui->lineEdit_mileageInterval->setText("2");
    this->ui->lineEdit_d->setText("0.1");
}

Dialog_Sections::~Dialog_Sections()
{
    delete ui;
}
void Dialog_Sections::GetParameter()
{
    int parameter=0;

    if(!ui->lineEdit_d->text().isEmpty())
    {
        d=ui->lineEdit_d->text().toFloat();
        parameter++;
    }
    if(!ui->lineEdit_mileageInterval->text().isEmpty())
    {
        MileageInterval=ui->lineEdit_mileageInterval->text().toFloat();
        parameter++;
    }
    if(parameter==2)
    {
        QString SavePath=QFileDialog::getExistingDirectory(this,"Save Sections Path","D:\\qt_data\\tunnel\\processed\\");
        if(!SavePath.isNull())
        {
            SaveSectionPath=SavePath;
               this->close();
        }
        else {
            this->close();
        }

    }
    else {
        QMessageBox::information(NULL, "Error", "Please set parameter.");
        return;
    }
}
