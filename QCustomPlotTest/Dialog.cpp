#include "Dialog.h"
#include "ui_Dialog.h"

#include <QDebug>

#include <QMessageBox>

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
{
    ui->setupUi(this);
    init();
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::init()
{
    for (int i = 0; i < 11; ++i)
    {
        a.push_back(i);
    }
}

void Dialog::on_BPSKPushButton_clicked()
{
    qDebug() << a;
    mB;
}

void Dialog::on_pushButton_clicked()
{
    int sum = 0;
    for (int i = 0; i < a.size(); ++i)
    {
        sum += i;
    }
    qDebug() << sum;

    QMessageBox::information(this, "sum", QString::number(sum));
    mB;
}
