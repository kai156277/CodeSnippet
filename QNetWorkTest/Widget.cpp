#include "Widget.h"
#include "ui_Widget.h"

#include <QByteArray>
#include <QDateTime>
#include <QTcpSocket>

#include <WiAcrLib/common/log.h>

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
    , mHyrins(new HydrinsTcpReciver)
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
    //    hydrins->connectToCtrlSocket("192.168.36.128", 8110);
    //    hydrins->connectToDataSocket("192.168.36.128", 8113);
    emit mHyrins->connectToHost("192.168.36.128", 8110, QIODevice::ReadWrite, QAbstractSocket::AnyIPProtocol);
}

void Widget::on_configPushButton_clicked()
{
    //    hydrins->configDefaultParam();
    //    hydrins->disconnectToSocket();
    emit mHyrins->disconnectFromHost();
}

Q_DECLARE_METATYPE(QIODevice::OpenMode)
Q_DECLARE_METATYPE(QAbstractSocket::NetworkLayerProtocol)

SocketWithThread::SocketWithThread(QAbstractSocket *socket, QObject *parent)
    : QObject(parent)
    , mWorkThread(new QThread)
    , mSocket(socket)
{
    qRegisterMetaType<QIODevice::OpenMode>();
    qRegisterMetaType<QAbstractSocket::NetworkLayerProtocol>();
    qRegisterMetaType<QAbstractSocket::SocketState>();
    qRegisterMetaType<QAbstractSocket::SocketError>();

    initConnection(mSocket);

    this->moveToThread(mWorkThread);      // this 在主线程中创建所以要移动
    mSocket->moveToThread(mWorkThread);   // mTcpSockect 在主线程中创建所以要移动
    mWorkThread->start();
}

SocketWithThread::~SocketWithThread()
{
    mWorkThread->quit();
    mWorkThread->wait();
    delete mWorkThread;
    delete mSocket;
}

void SocketWithThread::connectToHostFun(const QString &host_name, quint16 port, QIODevice::OpenMode open_mode, QAbstractSocket::NetworkLayerProtocol protocol)
{
    mSocket->connectToHost(host_name, port, open_mode, protocol);
}

void SocketWithThread::initConnection(QAbstractSocket *socket)
{
    // 带有默认参数的只能用 string-based connection
    // string-based 调用的 slot函数 必须要定义在 slots 之后, 但是Qt源码中没有这样定义，故需要外面再套一层
    connect(
        this,
        SIGNAL(connectToHost(const QString &, quint16, QIODevice::OpenMode, QAbstractSocket::NetworkLayerProtocol)),
        this,
        SLOT(connectToHostFun(const QString &, quint16, QIODevice::OpenMode, QAbstractSocket::NetworkLayerProtocol)));
    connect(this, &SocketWithThread::disconnectFromHost, socket, &QTcpSocket::disconnectFromHost);
    connect(this, &SocketWithThread::connected, socket, &QTcpSocket::connected);
    connect(this, &SocketWithThread::disconnected, socket, &QTcpSocket::disconnected);
    connect(socket, &QTcpSocket::readyRead, this, &SocketWithThread::readyRead);
    connect(socket, &QTcpSocket::bytesWritten, this, &SocketWithThread::bytesWritten);
    connect(socket, &QTcpSocket::aboutToClose, this, &SocketWithThread::aboutToClose);
    connect(socket, &QTcpSocket::channelReadyRead, this, &SocketWithThread::channelReadyRead);
    connect(socket, &QTcpSocket::channelBytesWritten, this, &SocketWithThread::channelBytesWritten);
    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::error), this, QOverload<QAbstractSocket::SocketError>::of(&SocketWithThread::tcpSocketError));
    connect(socket, &QTcpSocket::hostFound, this, &SocketWithThread::hostFound);
    connect(socket, &QTcpSocket::stateChanged, this, &SocketWithThread::stateChanged);
}

HydrinsTcpReciver::HydrinsTcpReciver(QObject *parent)
    : TcpSocketWithThread(parent)
{
}

void HydrinsTcpReciver::readyRead()
{
    mSocket->readAll();
    SPDLOG_INFO("ready read");
}
