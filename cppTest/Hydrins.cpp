#include "Hydrins.h"

#include <QDateTime>
#include <QFile>
#include <QFileDialog>
#include <QMap>
#include <QString>
#include <QTextStream>

#include <spdlog/qt_spdlog.h>

#include "IEWriter.h"

const QString key_hehdt    = "$HEHDT";
const QString key_pixse    = "$PIXSE";
const QString key_atitude  = "ATITUD";
const QString key_position = "POSITI";
const QString key_speed    = "SPEED_";
const QString key_utmwgs   = "UTMWGS";
const QString key_heave    = "HEAVE_";
const QString key_time     = "TIME__";
const QString key_stdhrp   = "STDHRP";
const QString key_stdpos   = "STDPOS";
const QString key_stdspd   = "STDSPD";
const QString key_algsts   = "ALGSTS";
const QString key_status   = "STATUS";
const QString key_htsts    = "HT_STS";
//const QString key_login    = "LOGIN_";
//const QString key_logdvl   = "LOGDVL";
//const QString key_logwat   = "LOGWAT";
const QString key_gpsin = "GPSIN_";
//const QString key_gp2in    = "GP2IN_";
//const QString key_gpmin    = "GPMIN_";
//const QString key_depin    = "DEPIN_";
//const QString key_usbin    = "USBIN_";
//const QString key_lblin    = "LBLIN_";
const QString key_utcin = "UTCIN_";
//const QString key_lmnin    = "LMNIN_";
//const QString key_ddreck   = "DDRECK";
//const QString key_sorsts   = "SORSTS";

using logToData = std::function<void(const QStringList &, PhinsStandard &)>;

using PhinsStandards = QVector<PhinsStandard>;

QMap<QString, logToData> initLogFuns();

void transformBLH2WGS84(double latitude, double longitude, double height, double &wgs84X, double &wgs84Y, double &wgs84Z)
{
    double latitude_rad  = latitude;
    double longitude_rad = longitude;
    latitude_rad *= DegToRad;
    longitude_rad *= DegToRad;
    double sinLat = sin(latitude_rad);
    double cosLat = cos(latitude_rad);
    double sinLon = sin(longitude_rad);
    double cosLon = cos(longitude_rad);
    double w = 0.0, N = 0.0;
    w      = sqrt(1 - E2 * sinLat * sinLat);
    N      = WGS84A / w;
    wgs84X = (N + height) * cosLat * cosLon;
    wgs84Y = (N + height) * cosLat * sinLon;
    wgs84Z = sinLat * (N * (1 - E2) + height);
}

int32_t reverseBytes_int32t(char *data)
{
    int32_t value0 = 0xff000000 & (data[0] << 24);
    int32_t value1 = 0x00ff0000 & (data[1] << 16);
    int32_t value2 = 0x0000ff00 & (data[2] << 8);
    int32_t value3 = 0x000000ff & (data[3]);
    return value0 | value1 | value2 | value3;
}

float reverseBytes_float(char *data)
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

int16_t reverseBytes_int16t(char *data)
{
    int16_t value0 = 0xff00 & (data[0] << 8);
    int16_t value1 = 0x00ff & (data[1]);
    return value0 | value1;
}
uint16_t reverseBytes_uint16t(char *data)
{
    uint16_t value0 = 0xff00 & (data[0] << 8);
    uint16_t value1 = 0x00ff & (data[1]);
    return value0 | value1;
}

int32_t U24ToU32(char *data)
{
    int32_t value0 = 0x00ff0000 & (data[0] << 16);
    int32_t value1 = 0x0000ff00 & (data[1] << 8);
    int32_t value2 = 0x000000ff & (data[2]);
    return value0 | value1 | value2;
}

int32_t S24ToS32(char *data)
{
    int32_t value0 = 0x00ff0000 & (data[0] << 16);
    int32_t value1 = 0x0000ff00 & (data[1] << 8);
    int32_t value2 = 0x000000ff & (data[2]);
    int32_t value  = value0 | value1 | value2;

    if ((value & 0x00800000) == 0x00800000)
    {
        value |= 0xff000000;
    }
    return value;
}

IELogItem phinsStandardToIELogItem(const PhinsStandard &item, int32_t date)
{
    IELogItem ie_log = {0};
    transformBLH2WGS84(item.latitude, item.longitude, item.altitude, ie_log.ecef_x, ie_log.ecef_y, ie_log.ecef_z);
    ie_log.heading    = item.heading;
    ie_log.heading_sd = item.headingSD;
    ie_log.heave      = item.heave;
    ie_log.surge      = item.surge;
    ie_log.sway       = item.sway;
    ie_log.height     = item.altitude;
    ie_log.latitude   = item.latitude;
    ie_log.longitude  = item.longitude;
    ie_log.pitch      = item.pitch;
    ie_log.pitch_sd   = item.pitchSD;
    ie_log.roll       = item.roll;
    ie_log.roll_sd    = item.rollSD;
    ie_log.utcTime    = item.time + date;
    return ie_log;
}

IELogItem NavItemToIELogItem(const NavItem &item)
{
    IELogItem ie_log = {0};
    transformBLH2WGS84(item.latitude, item.longitude, item.altitude, ie_log.ecef_x, ie_log.ecef_y, ie_log.ecef_z);
    ie_log.heading    = item.heading;
    ie_log.heading_sd = item.heading_sd;
    ie_log.heave      = item.heave;
    ie_log.surge      = item.surge;
    ie_log.sway       = item.sway;
    ie_log.height     = item.altitude;
    ie_log.latitude   = item.latitude;
    ie_log.longitude  = item.longitude;
    ie_log.pitch      = item.pitch;
    ie_log.pitch_sd   = item.pitch_sd;
    ie_log.roll       = item.roll;
    ie_log.roll_sd    = item.roll_sd;
    ie_log.utcTime    = item.utcTime;
    return ie_log;
}

Hydrins::Hydrins()
{
}
int parsePhinsStandard()
{
    QString file = QFileDialog::getOpenFileName(nullptr, "open phins stand file", "D:\\Data\\hydrins");
    if (file.isEmpty())
    {
        SPDLOG_INFO("empty file! end!");
        return 0;
    }

    QFile phins_file(file);
    if (!phins_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_INFO("opne phins file: {} faild!", file);
        return 1;
    }
    QTextStream              read_stream(&phins_file);
    QString                  log_line;
    QStringList              log_items;
    uint32_t                 count          = 0;
    QMap<QString, logToData> parse_log_funs = initLogFuns();

    PhinsStandards logs;

    QFileInfo file_info(file);
    IEWriter  ie_writer;
    ie_writer.setup(file_info.absolutePath(), "ie_" + file_info.baseName());
    ie_writer.setAntLeverArm(0.1543, 0.0102, 0.6624);
    logs.reserve(5000);

    QDateTime date({2021, 6, 26}, {0, 0, 0});
    QDateTime UTC_begin({1970, 1, 1}, {0, 0, 0});
    double    date_s = UTC_begin.secsTo(date);
    while (!read_stream.atEnd())
    {
        log_line  = read_stream.readLine().replace("*", ",");
        log_items = log_line.split(",");
        ++count;
        if (log_items.size() >= 2)
        {
            if (log_items[0] == key_hehdt)
            {
                PhinsStandard item = {};
                item.heading       = log_items[1].toDouble();
                do
                {
                    log_line  = read_stream.readLine().replace("*", ",");
                    log_items = log_line.split(",");
                    ++count;
                    if (log_items.size() < 2)
                        break;
                    if (log_items[0] == key_pixse)
                    {
                        auto fun = parse_log_funs.value(log_items[1], nullptr);
                        if (fun)
                            fun(log_items, item);
                        else
                            SPDLOG_WARN("not parse log data :{}", log_items[1]);

                        if (log_items[1] == key_htsts)
                            break;
                    }
                    if (count % 10000 == 0)
                        SPDLOG_INFO("read line: {}", count);
                } while (!read_stream.atEnd());
                IELogItem ie_item = phinsStandardToIELogItem(item, date_s);
                ie_writer.addLogItem(ie_item);
                logs.push_back(item);
            }
            else
            {
                SPDLOG_INFO("skip log: {}", log_line);
            }
        }
        else
        {
            SPDLOG_INFO("line: {} item length error", count);
        }
    };
}

// pitch 和 phins standard 相反
int parseBinaryNavHR(const QDateTime &date)
{
    QString file = "D:\\Data\\hydrins\\new_output\\binary_nav_hr.log";

    // QFileDialog::getOpenFileName(nullptr, "open phins stand file", "D:\\Data\\hydrins");

    if (file.isEmpty())
    {
        SPDLOG_INFO("empty file! end!");
        return 0;
    }

    QFile phins_file(file);
    if (!phins_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_INFO("opne phins file: {} faild!", file);
        return 1;
    }
    char data[42];

    qint64           read_flag;
    QVector<NavItem> navItems;
    navItems.reserve(5000);
    const double lat_factor     = 180.0 / pow(2, 31);
    const double roll_factor    = 45.0 / pow(2, 15);
    const double heading_factor = 360.0 / pow(2, 16);
    while (!phins_file.atEnd())
    {
        if (phins_file.read(&data[0], 1) == 1 && data[0] == (char) 0x71)
        {
            read_flag = phins_file.read(&data[1], 41);
            if (read_flag != 41)
            {
                SPDLOG_WARN("error read \"binary nav hr\" log! at: {}", phins_file.pos());
                continue;
            }
            NavItem item   = {0};
            item.utcTime   = date.toSecsSinceEpoch() + reverseBytes_int32t(&data[1]) + data[5] / 100.0;
            item.latitude  = reverseBytes_int32t(&data[6]) * lat_factor;
            item.longitude = reverseBytes_int32t(&data[10]) * lat_factor;
            item.altitude  = reverseBytes_int32t(&data[14]) / 100.0;
            item.heave     = -reverseBytes_int16t(&data[18]) / 100.0;
            item.roll      = reverseBytes_int16t(&data[26]) * roll_factor;
            item.pitch     = reverseBytes_int16t(&data[28]) * roll_factor;
            item.heading   = reverseBytes_uint16t(&data[30]) * heading_factor;

#ifdef _DEBUG
            QDateTime time = QDateTime::fromSecsSinceEpoch(item.utcTime);
#endif

            navItems.push_back(item);
        }
    }
    return 0;
}

QMap<QString, logToData> initLogFuns()
{
    QMap<QString, logToData> parse_log_funs;
    parse_log_funs.insert(key_atitude,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.roll  = items[2].toDouble();
                              item.pitch = items[3].toDouble();
                          });
    parse_log_funs.insert(key_position,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.latitude  = items[2].toDouble();
                              item.longitude = items[3].toDouble();
                              item.altitude  = items[4].toDouble();
                          });
    parse_log_funs.insert(key_speed,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.eastSpeed  = items[2].toDouble();
                              item.northSpeed = items[3].toDouble();
                              item.upSpeed    = items[4].toDouble();
                          });
    parse_log_funs.insert(key_utmwgs,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.latitudeUtmZone  = items[2];
                              item.longitudeUtmZone = items[3].toDouble();
                              item.utmEast          = items[4].toDouble();
                              item.utmNorth         = items[5].toDouble();
                              item.utmAltitude      = items[6].toDouble();
                          });
    parse_log_funs.insert(key_heave,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.surge = items[2].toDouble();
                              item.sway  = items[3].toDouble();
                              item.heave = items[4].toDouble();
                          });
    parse_log_funs.insert(key_time,
                          [](const QStringList &items, PhinsStandard &item) {
                              QString time_str = items[2];
                              int     h        = time_str.mid(0, 2).toInt();
                              int     m        = time_str.mid(2, 2).toInt();
                              int     s        = time_str.mid(4, 2).toInt();
                              int     us       = time_str.mid(7).toDouble();
                              item.time        = h * 3600 + m * 60 + s + us / 1000000.0;
                          });
    parse_log_funs.insert(key_stdhrp,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.headingSD = items[2].toDouble();
                              item.rollSD    = items[3].toDouble();
                              item.pitchSD   = items[4].toDouble();
                          });
    parse_log_funs.insert(key_stdpos,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.latitudeSD  = items[2].toDouble();
                              item.longitudeSD = items[3].toDouble();
                              item.altitudeSD  = items[4].toDouble();
                          });
    parse_log_funs.insert(key_stdspd,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.northSpeedSD = items[2].toDouble();
                              item.eastSpeedSD  = items[3].toDouble();
                              item.upSpeedSD    = items[4].toDouble();
                          });
    // not use
    parse_log_funs.insert(key_gpsin,
                          [](const QStringList &items, PhinsStandard &item) {
                          });
    parse_log_funs.insert(key_utcin,
                          [](const QStringList &items, PhinsStandard &item) {
                          });
    parse_log_funs.insert(key_algsts,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.insAlgorithmStatus = items[2].toDouble();
                          });
    parse_log_funs.insert(key_status,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.insSysteamStatus = items[2].toDouble();
                          });
    parse_log_funs.insert(key_htsts,
                          [](const QStringList &items, PhinsStandard &item) {
                              item.insUserStatus = items[2].toDouble();
                          });
    return parse_log_funs;
}

int parseNavigtionHDLC(const QDateTime &date)
{
    QString file = "D:\\Data\\hydrins\\new_output\\navigation_hdlc.log";

    // QFileDialog::getOpenFileName(nullptr, "open phins stand file", "D:\\Data\\hydrins");

    if (file.isEmpty())
    {
        SPDLOG_INFO("empty file! end!");
        return 0;
    }

    QFile phins_file(file);
    if (!phins_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_INFO("opne phins file: {} faild!", file);
        return 1;
    }

    char raw_data[47] = {0};

    const double     lat_factor  = 90 * pow(2, -30);
    const double     roll_factor = 90 * pow(2, -22);
    const double     time_factor = pow(2, -10);
    const double     head_factor = 180.0 / pow(2, 32);
    qint64           read_flag;
    QVector<NavItem> navItems;
    navItems.reserve(5000);
    while (!phins_file.atEnd())
    {
        read_flag = phins_file.read(&raw_data[0], 1);
        if (read_flag == 1 && raw_data[0] == 0x02)
        {
            read_flag = phins_file.read(&raw_data[1], 46);
            if (read_flag != 46)
            {
                SPDLOG_WARN("error read \"navigtion hdlc\" log! at: {}", phins_file.pos());
                continue;
            }
            NavItem item = {0};
            double  ms   = U24ToU32(&raw_data[1]) * time_factor;
#ifdef _DEBUG
            QTime test = QTime::fromMSecsSinceStartOfDay(ms);
#endif
            item.utcTime   = date.toSecsSinceEpoch() + ms / 1000.0;
            item.latitude  = reverseBytes_int32t(&raw_data[5]) * lat_factor;
            item.longitude = reverseBytes_int32t(&raw_data[9]) * lat_factor;
            item.altitude  = reverseBytes_int32t(&raw_data[13]) / 100.0;
            item.heave     = -reverseBytes_int16t(&raw_data[17]) / 100.0;
            item.heading   = U24ToU32(&raw_data[25]) * head_factor;
            item.roll      = S24ToS32(&raw_data[29]) * roll_factor;
            item.pitch     = S24ToS32(&raw_data[33]) * roll_factor;
            navItems.push_back(item);
        }
    }
    return 0;
}

int parseNavigtionShort(const QDateTime &date)
{
    QString file = "D:\\Data\\hydrins\\2021-06-26_1\\navigation_short.log";

    // QFileDialog::getOpenFileName(nullptr, "open phins stand file", "D:\\Data\\hydrins");

    if (file.isEmpty())
    {
        SPDLOG_INFO("empty file! end!");
        return 0;
    }

    QFile phins_file(file);
    if (!phins_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_INFO("opne phins file: {} faild!", file);
        return 1;
    }
    char data[54];

    QFileInfo file_info(file);
    IEWriter  ie_writer;
    ie_writer.setup(file_info.absolutePath(), "ie_" + file_info.baseName());
    ie_writer.setAntLeverArm(0.1543, 0.0102, 0.6624);

    double       secs = date.toSecsSinceEpoch();
    qint64       read_flag;
    const double lat_factor = 180.0 / pow(2, 31);
    int          count      = 0;
    while (!phins_file.atEnd())
    {
        if (phins_file.read(&data[0], 1) == 1 && data[0] == (char) 0x24)
        {
            if (phins_file.read(&data[1], 1) == 1 && data[1] == (char) 0xaa)
            {
                read_flag = phins_file.read(&data[2], 52);
                if (read_flag != 52)
                {
                    SPDLOG_WARN("error read \"navigtion short\" log! at: {}", phins_file.pos());
                    continue;
                }
                ++count;
                NavItem item   = {0};
                item.heading   = reverseBytes_float(&data[14]) * RadToDeg;
                item.roll      = reverseBytes_float(&data[18]) * RadToDeg;
                item.pitch     = reverseBytes_float(&data[22]) * RadToDeg;
                item.latitude  = reverseBytes_int32t(&data[38]) * lat_factor;
                item.longitude = reverseBytes_int32t(&data[42]) * lat_factor;
                item.altitude  = reverseBytes_float(&data[46]);

                uint32_t h  = (data[50] >> 3) & 0x1fU;
                uint32_t m  = ((data[50] << 3) & 0x38U) | ((data[51] >> 5) & 0x07U);
                uint32_t s  = (data[51] << 1 & 0x3eU) | ((data[52] >> 7) & 0x01U);
                uint32_t ms = (((((uint32_t) data[52] << 8) & 0xff00U) | (data[53] & 0x00ffU)) >> 5) & 0x03ffU;
#ifdef _DEBUG
                QTime time(h, m, s, ms);
#endif
                item.utcTime      = secs + h * 3600.0 + m * 60 + s + ms / 1000.0;
                IELogItem ie_item = NavItemToIELogItem(item);
                ie_writer.addLogItem(ie_item);

                if (count % 10000 == 0)
                    SPDLOG_INFO("navitem count: {}", count);
            }
        }
    }
    SPDLOG_INFO("parse navitem end!");
    return 0;
}

int parseGAPSBin(const QDateTime &date)
{
    QString file = QFileDialog::getOpenFileName(nullptr, "open phins stand file", "E:\\Data\\0717\\gaps_bin2.log");

    if (file.isEmpty())
    {
        SPDLOG_INFO("empty file! end!");
        return 0;
    }

    QFile phins_file(file);
    if (!phins_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_INFO("opne phins file: {} faild!", file);
        return 1;
    }
    char data[84];

    double secs = date.toSecsSinceEpoch();
    qint64 read_flag;

    QFileInfo file_info(file);
    IEWriter  ie_writer;
    ie_writer.setup(file_info.absolutePath(), "ie_" + file_info.baseName());
    ie_writer.setAntLeverArm(0.1543, 0.0102, 0.6624);

    const double lat_factor  = 90.0 / pow(2, 31);
    const double long_factor = 180.0 / pow(2, 31);
    int32_t      count       = 0;
    while (!phins_file.atEnd())
    {
        if (phins_file.read(&data[0], 1) == 1 && data[0] == (char) 0x24)
        {
            read_flag = phins_file.read(&data[1], 83);
            ++count;
            if (read_flag != 83)
            {
                SPDLOG_WARN("error read \"gaps bin\" log! at: {}", phins_file.pos());
                continue;
            }
            NavItem item      = {0};
            item.heading      = reverseBytes_float(&data[17]) * RadToDeg;
            item.roll         = reverseBytes_float(&data[21]) * RadToDeg;
            item.pitch        = reverseBytes_float(&data[25]) * RadToDeg;
            item.latitude     = reverseBytes_int32t(&data[29]) * lat_factor;
            item.longitude    = reverseBytes_int32t(&data[33]) * long_factor;
            item.altitude     = reverseBytes_float(&data[37]);
            item.heave        = reverseBytes_float(&data[53]);
            item.latitude_sd  = reverseBytes_float(&data[57]);
            item.longitude_sd = reverseBytes_float(&data[61]);
            item.altitude_sd  = reverseBytes_float(&data[65]);
            item.heading_sd   = reverseBytes_float(&data[69]);
            item.roll_sd      = reverseBytes_float(&data[73]);
            item.pitch_sd     = reverseBytes_float(&data[77]);

            uint32_t d = ((data[2] >> 4) & 0x0f) * 10 + (data[2] & 0x0f);
            uint32_t h = ((data[3] >> 4) & 0x0f) * 10 + (data[3] & 0x0f);
            uint32_t m = ((data[4] >> 4) & 0x0f) * 10 + (data[4] & 0x0f);
            uint32_t s = ((data[5] >> 4) & 0x0f) * 10 + (data[5] & 0x0f);

            uint32_t ms = ((data[6] >> 4) & 0x0f) * 100 + (data[6] & 0x0f) * 10 + ((data[7] >> 4) & 0x0f);
            uint32_t us = (data[7] & 0x0f) * 100 + ((data[8] >> 4) & 0x0f) * 10 + (data[8] & 0x0f);
#ifdef _DEBUG
            QTime time(h, m, s, ms);
#endif
            item.utcTime = secs + h * 3600.0 + m * 60 + s + ms / 1000.0 + us / 1000000.0;

            IELogItem ie_item = NavItemToIELogItem(item);
            ie_writer.addLogItem(ie_item);
            if (count % 10000 == 0)
                SPDLOG_INFO("gaps bin item: {}", count);
        }
    }
    return 0;
}
