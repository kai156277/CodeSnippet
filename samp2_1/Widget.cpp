#include "Widget.h"
#include "ui_Widget.h"

#include <QFileDialog>
#include <QMessageBox>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    connect(ui->pushButton_openFile, &QPushButton::clicked, this, &Widget::when_pushButton_openFile_clicked);
}

Widget::~Widget()
{
    delete ui;
}

//void Widget::on_pushButton_openFile_clicked()
//{
//}

void Widget::when_pushButton_openFile_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", tr("Images (*.png *.xpm *.jpg);;Text files (*.txt);;XML files (*.xml)"));
    if (fileName.isEmpty())
    {
        QMessageBox::warning(this, "warning", " open a empty file");
        return;
    }
    ui->lineEdit_openFile->setText(fileName);
}
