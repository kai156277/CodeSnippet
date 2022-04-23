#include "Widget.h"
#include "ui_Widget.h"

#include <QByteArray>
#include <QDateTime>
#include <QTcpSocket>

QString getCheckSum(const QString &cmd)
{
    int hh = 0;
    for (int i = 1; i < (cmd.length() - 1); ++i)
    {
        hh = hh ^ cmd[i].toLatin1();
    }

    return QString("%1").arg(hh, 2, 16);
}

// raw: $PIXSE,CONFIG,RESET_*
// return: $PIXSE,CONFIG,RESET_*57\r\n
QString generateCmd(const QString &raw_cmd)
{
    QString cmd;
    cmd = QString("%1%2\r\n").arg(raw_cmd).arg(getCheckSum(raw_cmd));
    return cmd;
}

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , tcpSocket(new QTcpSocket)
    , hydrins(new HydrinsReceiver("C:\\Users\\zhao\\Documents\\Data\\test.ini"))
{
    ui->setupUi(this);
    int port = 81;
    //    while (true)
    //    {
    //        tcpSocket->connectToHost("192.168.36.128", port);
    //        if (tcpSocket->waitForConnected(1000))
    //        {
    //            qDebug() << " 192.168.36.128:" << port << " connected!";
    //            break;
    //        }
    //        ++port;
    //        if (port % 10 == 0)
    //            qDebug() << "port: " << port;
    //    }
    //    tcpSocket->connectToHost("192.168.36.128", 8110);
    connect(tcpSocket, &QTcpSocket::readyRead, this, &Widget::readyRead);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::readyRead()
{
    QByteArray ba = tcpSocket->readAll();
    qDebug() << ba << QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
}

void Widget::on_resetPushButton_clicked()
{
    //    QString config_reset = "$PIXSE,CONFIG,RESET_*";
    //    qDebug() << generateCmd(config_reset);
    //    tcpSocket->write("$PIXSE,CONFIG,RESET_*57\r\n");
    hydrins->connectToCtrlSocket("192.168.36.128", 8110);
    hydrins->connectToDataSocket("192.168.36.128", 8113);
}

void Widget::on_configPushButton_clicked()
{
    //    hydrins->configDefaultParam();
    hydrins->disconnectToSocket();
}
