#pragma once

#include <QTcpSocket>
#include <QWidget>

#include "HydrinsReceiver.h"

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

    void readyRead();

private slots:
    void on_resetPushButton_clicked();

    void on_configPushButton_clicked();

private:
    Ui::Widget *ui;
    QTcpSocket *tcpSocket;

    HydrinsReceiver *hydrins;
};
