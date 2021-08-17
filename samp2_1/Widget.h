#pragma once

#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui {
class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    //    void on_pushButton_openFile_clicked();
    void when_pushButton_openFile_clicked();

private:
    Ui::Widget *ui;
};
