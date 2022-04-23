#include "HydrinsReceiver.h"

#include <WiAcrLib/common/log.h>

#include <QDateTime>
#include <QSettings>
#include <QString>
#include <QStringList>
#include <QTcpSocket>

namespace {

const QStringList port_list                     = {"None", "A", "B", "C", "D", "E"};
const QStringList stopbits_list                 = {"0.5", "1.0", "1.5", "2.0"};
const QStringList parity_list                   = {"None", "Even", "Odd"};
const QStringList output_device_list            = {"None", "Serial only", "Ethernet only", "Serial+Ethernet"};
const QStringList ethernet_transport_layer_list = {"TCP Server", "TCP Client", "UDP", "UDP Broadcast", "UDP Multicast"};
const QStringList altitude_reference_list       = {"Geoid(MSL)", "Ellipsoid"};
const QStringList baudrate_list                 = {
    "600",
    "1200",
    "2400",
    "4800",
    "9600",
    "19200",
    "38400",
    "57600",
    "115200",
    "230400",
    "460800"};
const QMap<int, QString> data_protocol_list   = {{13, "HEHDT"}, {33, "TSS1 DMS"}, {70, "GAPS BIN"}};
const QStringList        lever_arm_list       = {"Primary", "arm A", "arm B", "arm C"};
const QStringList        serial_standard_list = {"RS232", "RS422"};
const QStringList        pps_protocol_list    = {"None", "PPS Rising+Time", "PPS Falling+Time", "Time+PPS Rising", "Time+PPS Falling"};

const QString key_gps_lever_arm_x = "gps_lever_arm_x";
const QString key_gps_lever_arm_y = "gps_lever_arm_y";
const QString key_gps_lever_arm_z = "gps_lever_arm_z";

const QString key_serial_port        = "serial_port";
const QString key_serial_parity      = "serial_parity";
const QString key_serial_stop_bits   = "serial_stop_bits";
const QString key_serial_baud_rate   = "serial_baud_rate";
const QString key_output_protocol    = "output_protocol";
const QString key_output_rate        = "output_rate";
const QString key_output_rs_level    = "output_rs_level";
const QString key_output_lever_arm   = "output_lever_arm";
const QString key_output_ip_mode     = "output_ip_mode";
const QString key_output_ip_port     = "output_ip_port";
const QString key_input_protocol     = "input_protocol";
const QString key_altitude_reference = "altitude_reference";
const QString key_pps_protocol       = "pps_protocol";

const QString key_output_device = "output_device";
const QString key_input_device  = "input_device";

void gpsLeverAramConfig(QSettings &plugin_ini, QStringList &cmds);

const QString key_gps_input = "gps_input";

const QString key_gps_port = "gps_port";
const QString key_utc_port = "utc_port";
const QString key_pps_port = "pps_port";

void gpsInputConfig(QSettings &plugin_ini, QStringList &cmds);

const QString key_utc_pps_input = "utc_pps_input";

void utcAndPpsInputConfig(QSettings &plugin_ini, QStringList &cmds);

const QString key_montion_output = "montion_output";
const QString key_montion_port   = "montion_port";

void montionOutputConfig(QSettings &plugin_ini, QStringList &cmds);

const QString key_heading_output = "heading_output";
const QString key_heading_port   = "heading_port";

void headingOutputConfig(QSettings &plugin_ini, QStringList &cmds);

const QString key_integrated_nav_output = "integrated_nav_output";
const QString key_integrated_nav_port   = "integrated_nav_port";

void integratedNavOutputConfig(QSettings &plugin_ini, QStringList &cmds);

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

}   // namespace

HydrinsReceiver::HydrinsReceiver(const QString &config_file, QObject *parent)
    : QObject(parent)
    , mCtrlTcpSocket(new QTcpSocket)
    , mDataTcpSocket(new QTcpSocket)
    , mConfigFile(config_file)
{
    connect(mCtrlTcpSocket, &QTcpSocket::readyRead, this, &HydrinsReceiver::ctrlSocketReadyRead);
    connect(mDataTcpSocket, &QTcpSocket::readyRead, this, &HydrinsReceiver::dataSocketReadyRead);

    connect(mCtrlTcpSocket, &QTcpSocket::connected, [this]() {
        SPDLOG_INFO("连接至 hydrins 控制端口");
        //        configDefaultParam();
    });

    connect(mCtrlTcpSocket, &QTcpSocket::disconnected, [this]() {
        SPDLOG_INFO("断开 hydrins 控制端口: {}", mCtrlTcpSocket->errorString());
    });

    connect(mDataTcpSocket, &QTcpSocket::connected, [this]() {
        SPDLOG_INFO("连接至 hydrins 数据端口");
    });

    connect(mDataTcpSocket, &QTcpSocket::disconnected, [this]() {
        SPDLOG_INFO("断开 hydrins 数据端口:{}", mDataTcpSocket->errorString());
    });
}

void HydrinsReceiver::connectToCtrlSocket(const QString &ip, int port)
{
    mCtrlTcpSocket->connectToHost(ip, port);
}

void HydrinsReceiver::connectToDataSocket(const QString &ip, int port)
{
    mDataTcpSocket->connectToHost(ip, port);
}

void HydrinsReceiver::disconnectToSocket()
{
    mCtrlTcpSocket->disconnectFromHost();
    mDataTcpSocket->disconnectFromHost();
}

void HydrinsReceiver::ctrlSocketReadyRead()
{
    mCtrlTcpSocket->readAll();
    //    Sleep(50);
    SPDLOG_INFO("ctrl socket");
}

void HydrinsReceiver::dataSocketReadyRead()
{
    mDataTcpSocket->readAll();
    //    Sleep(50);
    SPDLOG_INFO("data socket");
    //    mCtrlTcpSocket->readAll();
}

void HydrinsReceiver::configDefaultParam()
{
    QSettings plugin_ini(mConfigFile, QSettings::IniFormat);

    QStringList default_param_cmds;

    // GPS 位置对准模式
    QString starting_mode_cmd = "$PIXSE,CONFIG,START_,1*";
    default_param_cmds.push_back(generateCmd(starting_mode_cmd));

    // GPS 高程模式
    QString altitude_calculation_mode_cmd = "$PIXSE,CONFIG,ALTMDE,1*";
    default_param_cmds.push_back(generateCmd(altitude_calculation_mode_cmd));

    gpsLeverAramConfig(plugin_ini, default_param_cmds);
    gpsInputConfig(plugin_ini, default_param_cmds);
    utcAndPpsInputConfig(plugin_ini, default_param_cmds);
    montionOutputConfig(plugin_ini, default_param_cmds);
    headingOutputConfig(plugin_ini, default_param_cmds);
    integratedNavOutputConfig(plugin_ini, default_param_cmds);

    QString save_cmd = "$PIXSE,CONFIG,SAVE__*5C\r\n";
    default_param_cmds.push_back(save_cmd);

    for (int i = 0; i < default_param_cmds.size(); ++i)
    {
        mCtrlTcpSocket->write(default_param_cmds[i].toLocal8Bit());
        SPDLOG_DEBUG("hydrins config cmd: {}", default_param_cmds[i]);
    }
}

namespace {

void gpsLeverAramConfig(QSettings &plugin_ini, QStringList &cmds)
{
    // GPS 杆臂值
    if (!plugin_ini.contains(key_gps_lever_arm_x))
    {
        SPDLOG_WARN("未读取到 gps_lever_arm_x 配置项！设置为 0");
        plugin_ini.setValue(key_gps_lever_arm_x, 0);
    }
    float gps_x = plugin_ini.value(key_gps_lever_arm_x).toFloat();

    if (!plugin_ini.contains(key_gps_lever_arm_y))
    {
        SPDLOG_WARN("未读取到 gps_lever_arm_y 配置项！设置为 0");
        plugin_ini.setValue(key_gps_lever_arm_y, 0);
    }
    float gps_y = plugin_ini.value(key_gps_lever_arm_y).toFloat();

    if (!plugin_ini.contains(key_gps_lever_arm_z))
    {
        SPDLOG_WARN("未读取到 gps_lever_arm_z 配置项！设置为 0");
        plugin_ini.setValue(key_gps_lever_arm_z, 0);
    }
    float gps_z = plugin_ini.value(key_gps_lever_arm_z).toFloat();

    QString gps_lever_arm_cmd = QString("$PIXSE,CONFIG,GPSLV_,%1,%2,%3*").arg(gps_x, 0, 'f').arg(gps_y, 0, 'f').arg(gps_z, 0, 'f');
    cmds.push_back(generateCmd(gps_lever_arm_cmd));
}

void gpsInputConfig(QSettings &plugin_ini, QStringList &cmds)
{
    // GPS 输入接口
    plugin_ini.beginGroup(key_gps_input);

    if (!plugin_ini.contains(key_gps_port))
    {
        SPDLOG_ERROR("未读取到 gps_input/gps_interface 配置项！设置为 Port B");
        plugin_ini.setValue(key_gps_port, port_list[2]);
    }
    QString port_str  = plugin_ini.value(key_gps_port).toString();
    int     gps_index = port_list.indexOf(port_str);

    QString gps_interface_cmd = QString("$PIXSE,CONFIG,GPSINT,%1*").arg(gps_index);
    cmds.push_back(generateCmd(gps_interface_cmd));

    QString input_device = QString("$PHCNF,EDIRI%1,1*").arg(port_str);   // serial only
    cmds.push_back(generateCmd(input_device));

    // GPS 输入B接口配置
    if (!plugin_ini.contains(key_serial_baud_rate))
    {
        SPDLOG_ERROR("未读取到 gps_input/serial_baud_rate 配置项！设置为 9600 baud (4)");
        plugin_ini.setValue(key_serial_baud_rate, 4);
    }
    // NOTE! ref 8-INS-AdvancedConfiguration_MU-INSIII-AN-004-F.p41
    QString baud_str       = plugin_ini.value(key_serial_baud_rate).toString();   // 9600 = 4
    int     baud_index     = baudrate_list.indexOf(baud_str);
    QString parity_str     = plugin_ini.value(key_serial_parity, parity_list[0]).toString();
    int     parity_index   = parity_list.indexOf(parity_str);
    QString stopbits_str   = plugin_ini.value(key_serial_stop_bits, stopbits_list[1]).toString();
    int     stopbits_index = stopbits_list.indexOf(stopbits_str);

    QString serial_io_param = QString("$PIXSE,CONFIG,RSCM_%1,%2,%3,0,0*").arg(port_str).arg(parity_index).arg(stopbits_index);
    cmds.push_back(generateCmd(serial_io_param));

    QString gps_input_cmd = QString("$PIXSE,CONFIG,RSIN_%1,%2,1,-1*").arg(port_str).arg(baud_index);
    cmds.push_back(generateCmd(gps_input_cmd));
    plugin_ini.endGroup();
}

void montionOutputConfig(QSettings &plugin_ini, QStringList &cmds)
{
    plugin_ini.beginGroup(key_montion_output);

    if (!plugin_ini.contains(key_montion_port))
    {
        SPDLOG_ERROR("没有指定输出 montion 数据输出端口！默认设置为: D端口, TSS1 DMS, 115200, 50Hz, RS232, 无奇偶校验, 1 stop bit");
        plugin_ini.setValue(key_montion_port, "D");
    }

    // 串口 montion IO配置
    QString port            = plugin_ini.value(key_montion_port).toString();
    QString parity_str      = plugin_ini.value(key_serial_parity, parity_list[0]).toString();
    int     parity_index    = parity_list.indexOf(parity_str);
    QString stopbits_str    = plugin_ini.value(key_serial_stop_bits, stopbits_list[1]).toString();
    int     stopbits_index  = stopbits_list.indexOf(stopbits_str);
    QString baud_str        = plugin_ini.value(key_serial_baud_rate, "115200").toString();   // 115200
    int     baud_index      = baudrate_list.indexOf(baud_str);
    QString protocol_str    = plugin_ini.value(key_output_protocol, data_protocol_list[33]).toString();   // TSS1 DMS
    int     protocol_index  = data_protocol_list.key(protocol_str);
    int     rate            = plugin_ini.value(key_output_rate, 20).toInt();               // 20ms = 50Hz
    QString rs_level_str    = plugin_ini.value(key_output_rs_level, "RS232").toString();   // RS232
    int     rs_level_index  = serial_standard_list.indexOf(rs_level_str);
    QString lever_arm_str   = plugin_ini.value(key_output_lever_arm, "Primary").toString();   // Main lever arm
    int     lever_arm_index = lever_arm_list.indexOf(lever_arm_str);

    QString output_device_cmd = QString("$PHCNF,EDIRO%1,1*").arg(port);   // serial only
    cmds.push_back(generateCmd(output_device_cmd));

    QString serial_io_param = QString("$PIXSE,CONFIG,RSCM_%1,%2,%3,0,0*").arg(port).arg(parity_index).arg(stopbits_index);
    cmds.push_back(generateCmd(serial_io_param));

    QString serial_io_output_param = QString("$PIXSE,CONFIG,RSOUT%1,%2,%3,%4,%5,%6,*")
                                         .arg(port)
                                         .arg(baud_index)
                                         .arg(protocol_index)
                                         .arg(rate)
                                         .arg(rs_level_index)
                                         .arg(lever_arm_index);
    cmds.push_back(generateCmd(serial_io_output_param));
    plugin_ini.endGroup();
}

void headingOutputConfig(QSettings &plugin_ini, QStringList &cmds)
{
    plugin_ini.beginGroup(key_heading_output);

    if (!plugin_ini.contains(key_heading_port))
    {
        SPDLOG_ERROR("没有指定输出 heading 数据输出端口！默认设置为: E端口, HEHDT, 115200, 50Hz, RS232, 无奇偶校验, 1 stop bit");
        plugin_ini.setValue(key_heading_port, "E");
    }

    QString port            = plugin_ini.value(key_heading_port).toString();
    QString parity_str      = plugin_ini.value(key_serial_parity, parity_list[0]).toString();
    int     parity_index    = parity_list.indexOf(parity_str);
    QString stopbits_str    = plugin_ini.value(key_serial_stop_bits, stopbits_list[1]).toString();
    int     stopbits_index  = stopbits_list.indexOf(stopbits_str);
    QString baud_str        = plugin_ini.value(key_serial_baud_rate, "9600").toString();   // 115200
    int     baud_index      = baudrate_list.indexOf(baud_str);
    QString protocol_str    = plugin_ini.value(key_output_protocol, data_protocol_list[13]).toString();   // HEHDT
    int     protocol_index  = data_protocol_list.key(protocol_str);
    int     rate            = plugin_ini.value(key_output_rate, 200).toInt();              // 200ms = 5Hz
    QString rs_level_str    = plugin_ini.value(key_output_rs_level, "RS232").toString();   // RS232
    int     rs_level_index  = serial_standard_list.indexOf(rs_level_str);
    QString lever_arm_str   = plugin_ini.value(key_output_lever_arm, "Primary").toString();   // Main lever arm
    int     lever_arm_index = lever_arm_list.indexOf(lever_arm_str);

    QString output_device_cmd = QString("$PHCNF,EDIRO%1,1*").arg(port);   // serial only
    cmds.push_back(generateCmd(output_device_cmd));

    QString serial_io_param = QString("$PIXSE,CONFIG,RSCM_%1,%2,%3,0,0*").arg(port).arg(parity_index).arg(stopbits_index);
    cmds.push_back(generateCmd(serial_io_param));

    QString serial_io_output_param = QString("$PIXSE,CONFIG,RSOUT%1,%2,%3,%4,%5,%6,*")
                                         .arg(port)
                                         .arg(baud_index)
                                         .arg(protocol_index)
                                         .arg(rate)
                                         .arg(rs_level_index)
                                         .arg(lever_arm_index);
    cmds.push_back(generateCmd(serial_io_output_param));
    plugin_ini.endGroup();
}

void integratedNavOutputConfig(QSettings &plugin_ini, QStringList &cmds)
{
    plugin_ini.beginGroup(key_integrated_nav_output);
    if (!plugin_ini.contains(key_integrated_nav_port))
    {
        SPDLOG_ERROR("没有指定输出组合导航数据输出端口！默认设置为: C端口, GapsBin, 100Hz, TCP Server, 8113");
        plugin_ini.setValue(key_integrated_nav_port, "C");
    }

    QString port                = plugin_ini.value(key_integrated_nav_port).toString();
    QString protocol_str        = plugin_ini.value(key_output_protocol, "GAPS BIN").toString();   // GAPS BIN
    int     protocol_index      = data_protocol_list.key(protocol_str);
    int     rate                = plugin_ini.value(key_output_rate, 10).toInt();                      // 10ms = 100Hz
    QString altitude_ref_str    = plugin_ini.value(key_altitude_reference, "Ellipsoid").toString();   // WGS84 Ellipsoidal
    int     altitude_ref_index  = altitude_reference_list.indexOf(altitude_ref_str);
    QString output_device_str   = plugin_ini.value(key_output_device, "Ethernet only").toString();   //  Ethernet only;
    int     output_device_index = output_device_list.indexOf(output_device_str);
    QString mode_str            = plugin_ini.value(key_output_ip_mode, "TCP Server").toString();   // TCP Server
    int     mode_index          = ethernet_transport_layer_list.indexOf(mode_str);
    int     ip_port             = plugin_ini.value(key_output_ip_port, 8113).toInt();   // IP port

    QString output_device_cmd = QString("$PHCNF,EDIRO%1,%2*").arg(port).arg(output_device_index);
    cmds.push_back(generateCmd(output_device_cmd));

    QString output_param = QString("$PIXSE,CONFIG,RSOUT%1,1,%2,%3,0,0,0,%4*")
                               .arg(port)
                               .arg(protocol_index)
                               .arg(rate)
                               .arg(altitude_ref_index);
    cmds.push_back(generateCmd(output_param));

    // 设置网口输出的参数包括输出端口，模式，目标IP，IP port
    QString ip_output_config = QString("$PHCNF,ELCFO%1,%2,192.168.36.100,%3*")
                                   .arg(port)
                                   .arg(mode_index)
                                   .arg(ip_port);
    cmds.push_back(generateCmd(ip_output_config));

    plugin_ini.endGroup();
}

void utcAndPpsInputConfig(QSettings &plugin_ini, QStringList &cmds)
{
    plugin_ini.beginGroup(key_utc_pps_input);
    // UTC 输入接口
    if (!plugin_ini.contains(key_utc_port))
    {
        SPDLOG_ERROR("未读取到 utc_interface 配置项！设置为 Port B(2)");
        plugin_ini.setValue(key_utc_port, "B");
    }
    QString utc_str   = plugin_ini.value(key_utc_port).toString();
    int     utc_index = port_list.indexOf(utc_str);

    QString utc_interface_cmd = QString("$PIXSE,CONFIG,UTCINT,%1*").arg(utc_index);
    cmds.push_back(generateCmd(utc_interface_cmd));

    // PPS 输入
    if (!plugin_ini.contains(key_pps_port))
    {
        SPDLOG_ERROR("未读取到 pps_interface 配置项！设置为 Port A(1)");
        plugin_ini.setValue(key_pps_port, "A");
    }
    QString pps_str = plugin_ini.value(key_pps_port).toString();
    // NOTE! 这个地方的protocol的编号和文档中给出的不同
    // PPS Rising + Time  = 1
    // PPS Falling + Time = 2
    // Time + PPS Rising  = 3
    // Time + PPS Falling = 4
    QString pps_protocol_str   = plugin_ini.value(key_pps_protocol, "Time+PPS Rising").toString();   // SPS 系列是时间标在前，标记PPS上升沿
    int     pps_protocol_index = pps_protocol_list.indexOf(pps_protocol_str);

    QString pps_input_cmd = QString("$PIXSE,CONFIG,IOIN_%1,0.0,%2*").arg(pps_str).arg(pps_protocol_index);
    cmds.push_back(generateCmd(pps_input_cmd));

    plugin_ini.endGroup();
}
}   // namespace
