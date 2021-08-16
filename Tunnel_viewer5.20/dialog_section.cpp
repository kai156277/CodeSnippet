#include "dialog_section.h"

#include "ui_dialog_section.h"

#include <QFileDialog>
#include <QMessageBox>
#pragma execution_character_set("utf-8")
Dialog_Section::Dialog_Section(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog_Section)
{
    ui->setupUi(this);
    connect(this->ui->btn_ok, SIGNAL(clicked(bool)), this, SLOT(GetParameter()));
    this->ui->lineEdit_mileage->setText("3940");
    this->ui->lineEdit_d->setText("0.1");
}

Dialog_Section::~Dialog_Section()
{
    delete ui;
}
void Dialog_Section::GetParameter()
{
    int parameter = 0;

    if (!ui->lineEdit_d->text().isEmpty())
    {
        d = ui->lineEdit_d->text().toFloat();
        parameter++;
    }
    if (!ui->lineEdit_mileage->text().isEmpty())
    {
        Mileage = ui->lineEdit_mileage->text().toFloat();
        parameter++;
    }
    if (parameter == 2)
    {
        QString SavePath = QFileDialog::getSaveFileName(this, tr("Save Section Path"), "D:\\qt_data\\tunnel\\processed\\SaveSectionFile.pcd", tr("*.pcd *.ply"));
        if (!SavePath.isNull())
        {
            SaveSectionPath = SavePath;
            this->close();
        }
        else
        {
            this->close();
        }
    }
    else
    {
        QMessageBox::information(NULL, "Error", "Please set parameter.");
        return;
    }
}
