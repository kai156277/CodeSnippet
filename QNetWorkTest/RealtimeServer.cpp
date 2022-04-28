#include "RealtimeServer.h"

#include <QSettings>
#include <QVariant>

namespace {

const static double Pi       = 3.1415926535897932;
const static double DegToRad = (Pi / 180.0);
const static double RadToDeg = (180.0 / Pi);

int32_t reverseBytes_int32t(const char *data)
{
    int32_t value0 = 0xff000000 & (data[0] << 24);
    int32_t value1 = 0x00ff0000 & (data[1] << 16);
    int32_t value2 = 0x0000ff00 & (data[2] << 8);
    int32_t value3 = 0x000000ff & (data[3]);
    return value0 | value1 | value2 | value3;
}

float reverseBytes_float(const char *data)
{
    int32_t value0    = 0xff000000 & (data[0] << 24);
    int32_t value1    = 0x00ff0000 & (data[1] << 16);
    int32_t value2    = 0x0000ff00 & (data[2] << 8);
    int32_t value3    = 0x000000ff & (data[3]);
    int32_t value     = value0 | value1 | value2 | value3;
    float   end_value = 0;
    memcpy_s(&end_value, 4, &value, 4);
    return end_value;
}
}   // namespace

RealtimeServer::RealtimeServer(const QString &config_file, QObject *parent)
    : QThread(parent)
    , mConfigFile(config_file)
{
    QSettings ini_file(config_file, QSettings::IniFormat);

    auto default_key_setting = [&, this](const QString &key, const QVariant &value) -> QVariant {
        if (!ini_file.contains(key))
        {
            ini_file.setValue(key, value);
        }
        return ini_file.value(key).toString();
    };

    {
        mNavigationServer.ip   = default_key_setting(key_navigation_server_ip, "127.0.0.1").toString();
        mNavigationServer.port = default_key_setting(key_navigation_server_port, 1260).toInt();

        mNavigationServer.socket = new QUdpSocket;
        mNavigationServer.socket->moveToThread(this);
    }

    {
        mMultibeamServer.ip   = default_key_setting(key_multibeam_server_ip, "127.0.0.1").toString();
        mMultibeamServer.port = default_key_setting(key_multibeam_server_port, 1261).toInt();

        mMultibeamServer.socket = new QUdpSocket;
        mMultibeamServer.socket->moveToThread(this);
    }

    {
        mLidarServer.ip   = default_key_setting(key_lidar_server_ip, "127.0.0.1").toString();
        mLidarServer.port = default_key_setting(key_lidar_server_port, 1262).toInt();

        mLidarServer.socket = new QUdpSocket;
        mLidarServer.socket->moveToThread(this);
    }

    {
        mNavigationSocket.ip   = default_key_setting(key_navigation_socket_ip, "192.168.36.128").toString();
        mNavigationSocket.port = default_key_setting(key_navigation_socket_port, 8113).toInt();

        mNavigationSocket.socket = new QTcpSocket;
        mNavigationSocket.socket->connectToHost(mNavigationSocket.ip, mNavigationSocket.port);
        connect(mNavigationSocket.socket, &QTcpSocket::connected, [this]() {
            mIsNavigationConnected = true;
        });
        connect(mNavigationSocket.socket, &QTcpSocket::readyRead, this, &RealtimeServer::navigationDataReadyRead);

        //        mNavigationSocket.socket->moveToThread(this);
    }

    {
        mMultibeamSocket.ip   = default_key_setting(key_multibeam_socket_ip, "192.168.36.120").toString();
        mMultibeamSocket.port = default_key_setting(key_multibeam_socket_port, 7000).toInt();

        mMultibeamSocket.socket = new QUdpSocket;
        mMultibeamSocket.socket->connectToHost(mMultibeamSocket.ip, mMultibeamSocket.port);
        connect(mMultibeamSocket.socket, &QUdpSocket::connected, [this]() {
            mIsMultibeamConnected = true;
        });
        mMultibeamSocket.socket->bind(QHostAddress::AnyIPv4, 346);
    }
}

void RealtimeServer::run()
{
    while (mBroadcastDataFlag)
    {
        // 解析
        const char *data = record.data();
        //char data[84];

        const double lat_factor  = 90.0 / pow(2, 31);
        const double long_factor = 180.0 / pow(2, 31);

        if (data[0] == (char) 0x24)
        {
            NavItem item;
            item.heading   = reverseBytes_float(&data[17]) * RadToDeg;
            item.roll      = reverseBytes_float(&data[21]) * RadToDeg;
            item.pitch     = reverseBytes_float(&data[25]) * RadToDeg;
            item.latitude  = reverseBytes_int32t(&data[29]) * lat_factor;
            item.longitude = reverseBytes_int32t(&data[33]) * long_factor;
            item.altitude  = reverseBytes_float(&data[37]);
            item.heave     = reverseBytes_float(&data[53]);
            //            item.latitude_sd  = reverseBytes_float(&data[57]);
            //            item.longitude_sd = reverseBytes_float(&data[61]);
            //            item.altitude_sd  = reverseBytes_float(&data[65]);
            //            item.heading_sd   = reverseBytes_float(&data[69]) * RadToDeg;
            //            item.roll_sd      = reverseBytes_float(&data[73]) * RadToDeg;
            //            item.pitch_sd     = reverseBytes_float(&data[77]) * RadToDeg;

            // uint32_t d = ((data[2] >> 4) & 0x0f) * 10 + (data[2] & 0x0f);
            uint32_t h = ((data[3] >> 4) & 0x0f) * 10 + (data[3] & 0x0f);
            uint32_t m = ((data[4] >> 4) & 0x0f) * 10 + (data[4] & 0x0f);
            uint32_t s = ((data[5] >> 4) & 0x0f) * 10 + (data[5] & 0x0f);

            uint32_t ms  = ((data[6] >> 4) & 0x0f) * 100 + (data[6] & 0x0f) * 10 + ((data[7] >> 4) & 0x0f);
            uint32_t us  = (data[7] & 0x0f) * 100 + ((data[8] >> 4) & 0x0f) * 10 + (data[8] & 0x0f);
            item.utcTime = h * 3600.0 + m * 60 + s + ms / 1000.0 + us / 1000000.0;

            // 融合

            // 发送
            QByteArray nav_ba;
            mNavigationServer.socket->writeDatagram(nav_ba, mNavigationServer.ip, mNavigationServer.port);
            mMultibeamServer.socket->writeDatagram(nav_ba, mMultibeamServer.ip, mMultibeamServer.port);
            //        mLidarServer.socket->writeDatagram(nav_ba, mLidarServer.ip, mLidarServer.port);
        }
    }
}

void RealtimeServer::navigationDataReadyRead()
{
    QByteArray ba = mNavigationSocket.socket->readAll();
    if (!ba.isEmpty())
    {
        mNavigationRawData.push_back(ba);
    }
}

void RealtimeServer::multibeamDataReadyRead()
{
    QByteArray ba = mMultibeamSocket.socket->readAll();
    if (!ba.isEmpty())
    {
        mMultibeamRawData.push_back(ba);
    }
}
