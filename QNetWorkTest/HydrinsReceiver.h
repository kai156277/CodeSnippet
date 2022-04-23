#pragma once

#include <QObject>
#include <QTcpSocket>

class HydrinsReceiver : public QObject
{
    Q_OBJECT
public:
    explicit HydrinsReceiver(const QString &config_file, QObject *parent = nullptr);

    void connectToCtrlSocket(const QString &ip, int port);
    void connectToDataSocket(const QString &ip, int port);
    void disconnectToSocket();
    void configDefaultParam();
signals:

private:
    void ctrlSocketReadyRead();
    void dataSocketReadyRead();

    QTcpSocket *mCtrlTcpSocket = nullptr;
    QTcpSocket *mDataTcpSocket = nullptr;

    QString mConfigFile;
};
