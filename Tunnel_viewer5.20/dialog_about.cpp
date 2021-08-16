#include "dialog_about.h"
#include "ui_dialog_about.h"
#pragma execution_character_set("utf-8")
Dialog_About::Dialog_About(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog_About)
{
    ui->setupUi(this);
}

Dialog_About::~Dialog_About()
{
    delete ui;
}
