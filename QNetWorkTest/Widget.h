#pragma once

#include <QTcpSocket>
#include <QThread>
#include <QUdpSocket>
#include <QWidget>

#include "HydrinsReceiver.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class Widget;
}
QT_END_NAMESPACE

class HydrinsTcpReciver;
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

    HydrinsReceiver *  hydrins;
    HydrinsTcpReciver *mHyrins;
};

class RealTimeServer : public QThread
{
    Q_OBJECT
public:
    explicit RealTimeServer(QObject *parent = nullptr);
};

class SocketWithThread : public QObject
{
    Q_OBJECT
public:
    // socket 由 SocketWithTread 销毁
    explicit SocketWithThread(QAbstractSocket *socket, QObject *parent = nullptr);
    ~SocketWithThread() override;

signals:
    void connectToHost(const QString &                       host_name,
                       quint16                               port,
                       QIODevice::OpenMode                   open_mode,
                       QAbstractSocket::NetworkLayerProtocol protocol);
    void connected();
    void disconnectFromHost();
    void disconnected();
    void tcpSocketError(QAbstractSocket::SocketError socket_error);
    void stateChanged(QAbstractSocket::SocketState socket_state);
    void hostFound();

    void aboutToClose();
    void bytesWritten(qint64 bytes);
    void channelBytesWritten(int channel, qint64 bytes);

public slots:
    void connectToHostFun(const QString &                       host_name,
                          quint16                               port,
                          QIODevice::OpenMode                   open_mode,
                          QAbstractSocket::NetworkLayerProtocol protocol);

protected:
    void initConnection(QAbstractSocket *socket);

    virtual void readyRead() {}
    virtual void channelReadyRead(int channel) {}

    QAbstractSocket *mSocket;
    QThread *        mWorkThread;
};
class TcpSocketWithThread : public SocketWithThread
{
    Q_OBJECT
public:
    explicit TcpSocketWithThread(QObject *parent = nullptr)
        : SocketWithThread(new QTcpSocket, parent)
    {
    }
};

class UdpSocketWithTread : public SocketWithThread
{
    Q_OBJECT
public:
    explicit UdpSocketWithTread(QObject *parent = nullptr)
        : SocketWithThread(new QUdpSocket, parent)
    {
    }
};

class HydrinsTcpReciver : public TcpSocketWithThread
{
    Q_OBJECT
public:
    explicit HydrinsTcpReciver(QObject *parent = nullptr);

protected:
    void readyRead() override;
};
