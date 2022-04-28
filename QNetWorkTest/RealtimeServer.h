#pragma once

#include <QList>
#include <QTcpSocket>
#include <QThread>
#include <QUdpSocket>

constexpr auto key_navigation_socket_ip   = "navigation_socket_ip";
constexpr auto key_navigation_socket_port = "navigation_socket_port";
constexpr auto key_multibeam_socket_ip    = "multibeam_socket_ip";
constexpr auto key_multibeam_socket_port  = "multibeam_socket_port";
constexpr auto key_lidar_socket_ip        = "lidar_socket_ip";
constexpr auto key_lidar_socket_port      = "lidar_socket_port";

constexpr auto key_navigation_server_ip   = "navigation_server_ip";
constexpr auto key_navigation_server_port = "navigation_server_port";
constexpr auto key_multibeam_server_ip    = "multibeam_server_ip";
constexpr auto key_multibeam_server_port  = "multibeam_server_port";
constexpr auto key_lidar_server_ip        = "lidar_server_ip";
constexpr auto key_lidar_server_port      = "lidar_server_port";

#pragma pack(push)
#pragma pack(1)
struct NavItem
{
    double utcTime;

    float heading;   // 航向
    float roll;      // 横滚 左侧抬起为 +
    float pitch;     // 俯仰 低头 +

    //    double heading_sd;
    //    double roll_sd;
    //    double pitch_sd;

    //    double heave;   // 垂荡, 向上为 +
    //    double surge;   // 浪涌
    //    double sway;    // 横摇
    //    double heave_sd;
    //    double surge_sd;
    //    double sway_sd;

    float latitude;
    float longitude;
    float altitude;
    //    double latitude_sd;
    //    double longitude_sd;
    //    double altitude_sd;

    //    double northSpeed;
    //    double eastSpeed;
    //    double upSpeed;
    //    double speednormal;
};

struct PointPack
{
    float   latitude;
    float   longitude;
    int16_t depth;
    uint8_t intensity;
};

#pragma pack(pop)

class RealtimeServer : public QThread
{
    Q_OBJECT
public:
    struct TcpSocketParam
    {
        QTcpSocket *socket;
        QString     ip;
        qint16      port;
    };
    struct UdpSocketParam
    {
        QUdpSocket *socket;
        QString     ip;
        qint16      port;
    };

    explicit RealtimeServer(const QString &config_file, QObject *parent = nullptr);

    void run() override;

private:
    void navigationDataReadyRead();
    void multibeamDataReadyRead();

    QList<QByteArray> mNavigationRawData;
    QList<QByteArray> mMultibeamRawData;

    bool mBroadcastDataFlag = true;

    QString mConfigFile;

    bool mIsNavigationConnected = false;
    bool mIsMultibeamConnected  = false;

    TcpSocketParam mNavigationSocket;
    TcpSocketParam mMultibeamSocket;
    TcpSocketParam mLidarSocket;

    UdpSocketParam mNavigationServer;
    UdpSocketParam mMultibeamServer;
    UdpSocketParam mLidarServer;
};
