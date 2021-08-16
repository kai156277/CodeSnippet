#include "dialog_attribute.h"
#include "ui_dialog_attribute.h"
#include<QMessageBox>
#pragma execution_character_set("utf-8")
Dialog_Attribute::Dialog_Attribute(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog_Attribute)
{
    ui->setupUi(this);
    connect(this->ui->pushButton,SIGNAL(clicked(bool)),this,SLOT(GetParameter()));
    this->ui->lineEdit_z0->setText("2.3");
    this->ui->lineEdit_1->setText("1.5");
    this->ui->lineEdit_2->setText("0.8");
    this->ui->lineEdit_3->setText("0");
    this->ui->lineEdit_4->setText("-1.1");
}

Dialog_Attribute::~Dialog_Attribute()
{
    delete ui;
}
void Dialog_Attribute::GetParameter()
{
    if(ui->box_TopPoint->isChecked())
    {
        TopPoint=true;
    }
    else
    {
        TopPoint=false;
    }
    if(ui->lineEdit_z0->text().isEmpty())
    {
        QMessageBox::information(NULL, "Error", "Please set the height of target point!");
        return;
    }
    else
    {
        z0=ui->lineEdit_z0->text().toFloat();
        if(!ui->lineEdit_1->text().isEmpty())
        {
            parameter.push_back(ui->lineEdit_1->text().toFloat());
        }
        if(!ui->lineEdit_2->text().isEmpty())
        {
            parameter.push_back(ui->lineEdit_2->text().toFloat());
        }
        if(!ui->lineEdit_3->text().isEmpty())
        {
            parameter.push_back(ui->lineEdit_3->text().toFloat());
        }
        if(!ui->lineEdit_4->text().isEmpty())
        {
            parameter.push_back(ui->lineEdit_4->text().toFloat());
        }


        if(parameter.size()==0)
        {
            QMessageBox::information(NULL, "Error", "Please set parameter!");
            return;
        }else
        {
            for(int i=0;i<parameter.size();i++)
            {
                parameter[i]=parameter[i]+z0;
            }
            this->close();
        }
    }

}
