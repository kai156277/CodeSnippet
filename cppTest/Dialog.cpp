#include "Dialog.h"
#include "ui_Dialog.h"

#include <QMessageBox>

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
{
    ui->setupUi(this);
    connect(ui->pushButton, &QPushButton::clicked, this, &Dialog::open1);
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::open1()
{
    QMessageBox::information(this, "test", "test");
}
