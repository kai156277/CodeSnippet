#include <QApplication>
#include <QDebug>
#include <iomanip>
#include <iostream>
#include <time.h>
#include <windows.h>

#include <QDateTime>
#include <QFile>
#include <QString>
#include <QStringRef>
#include <QTextStream>
#include <QVector>

//#include <execution>
#include <spdlog/qt_spdlog.h>

#define _USE_MATH_DEFINES
#include <math.h>

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

using namespace std;

struct Pose
{
    int64_t usec_in_day = -1;
    double  latitude    = 0.0;
    double  longitude   = 0.0;
    double  height      = 0.0;
    double  x           = 0.0;
    double  y           = 0.0;
    double  z           = 0.0;
    double  roll        = 0.0;
    double  pitch       = 0.0;
    double  yaw         = 0.0;
    double  heave       = 0.0;

    static Pose interpolation(const Pose &b, const Pose e, uint64_t time);
};

Pose operator-(const Pose &left, const Pose &right)
{
    Pose sub;
    sub.usec_in_day = left.usec_in_day - right.usec_in_day;
    sub.latitude    = left.latitude - right.latitude;
    sub.longitude   = left.longitude - right.longitude;
    sub.height      = left.height - right.height;
    sub.x           = left.x - right.x;
    sub.y           = left.y - right.y;
    sub.z           = left.z - right.z;
    sub.roll        = left.roll - right.roll;
    sub.pitch       = left.pitch - right.pitch;
    sub.yaw         = left.yaw - right.yaw;
    sub.heave       = left.heave - right.heave;
    return sub;
}

Pose operator+(const Pose &left, const Pose &right)
{
    Pose tmp;
    tmp.usec_in_day = left.usec_in_day + right.usec_in_day;
    tmp.latitude    = left.latitude + right.latitude;
    tmp.longitude   = left.longitude + right.longitude;
    tmp.height      = left.height + right.height;
    tmp.x           = left.x + right.x;
    tmp.y           = left.y + right.y;
    tmp.z           = left.z + right.z;
    tmp.roll        = left.roll + right.roll;
    tmp.pitch       = left.pitch + right.pitch;
    tmp.yaw         = left.yaw + right.yaw;
    tmp.heave       = left.heave + right.heave;
    return tmp;
}

Pose operator*(const Pose &left, double scale)
{
    Pose tmp;
    tmp.usec_in_day = left.usec_in_day * scale;
    tmp.latitude    = left.latitude * scale;
    tmp.longitude   = left.longitude * scale;
    tmp.height      = left.height * scale;
    tmp.x           = left.x * scale;
    tmp.y           = left.y * scale;
    tmp.z           = left.z * scale;
    tmp.roll        = left.roll * scale;
    tmp.pitch       = left.pitch * scale;
    tmp.yaw         = left.yaw * scale;
    tmp.heave       = left.heave * scale;
    return tmp;
}

Pose Pose::interpolation(const Pose &b, const Pose e, uint64_t time)
{
    Pose inter_pose;
    if (b.usec_in_day <= time && time < e.usec_in_day)
    {
        double scale = (double) (time - b.usec_in_day) / (double) (e.usec_in_day - b.usec_in_day);
        inter_pose   = b + (e - b) * scale;

        inter_pose.usec_in_day = time;
    }
    return inter_pose;
}

struct PoseStd
{
    float latitude_std  = 0.0f;
    float longitude_std = 0.0f;
    float height_std    = 0.0f;
    float x_std         = 0.0f;
    float y_std         = 0.0f;
    float z_std         = 0.0f;
    float roll_std      = 0.0f;
    float pitch_std     = 0.0f;
    float yaw_std       = 0.0f;
    float heave_std     = 0.0f;
};

using RangeIndex = std::pair<size_t, size_t>;   // [first, second)

struct IEPose
{
    QDate   date;
    Pose    pos;
    PoseStd std;
    int     quality = -1;

    static IEPose interpolation(const IEPose &b, const IEPose e, uint64_t time)
    {
        IEPose inter;
        inter.pos = Pose::interpolation(b.pos, e.pos, time);
        if (inter.pos.usec_in_day != -1)
        {
            inter.date    = b.date;
            inter.quality = b.quality;
        }
        return inter;
    }

    static RangeIndex findMinRange(const std::vector<IEPose> &poses, uint64_t time)
    {
        RangeIndex range;

        auto upper = std::upper_bound(poses.begin(), poses.end(), time, [](uint64_t usec, const IEPose &pose) {
            return usec < pose.pos.usec_in_day;
        });

        size_t index = std::distance(poses.begin(), upper);

        if (upper == poses.begin() || upper == poses.end())
        {
            range.second = index;
            range.first  = index;
        }
        else
        {
            range.second = index;
            range.first  = index - 1;
        }

        return range;
    }
};
std::vector<IEPose> readIEPoseFile(const QString &filename)
{
    QFile pose_file(filename);
    if (!pose_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_ERROR("打开轨迹文件失败！！！");
        return {};
    }

    QTextStream read_stream(&pose_file);

    QString             read_line;
    QStringList         items;
    std::vector<IEPose> poses;
    IEPose              pose;
    int                 old_msec;
    size_t              count = 0;
    while (!read_stream.atEnd())
    {
        read_line = read_stream.readLine();
        count++;
        if (count % 100000 == 0)
        {
            SPDLOG_DEBUG("read pos num: {}", count);
        }
        items = read_line.split(" ", QString::SkipEmptyParts);
        if (items.size() < 23)
            continue;

        pose.date = QDate::fromString(items[0], "MM/dd/yyyy");
        if (!pose.date.isValid())
            continue;

        double sec  = items[3].toDouble();
        int    msec = std::round((sec - (int) sec) * 1000);
        QTime  time(items[1].toInt(), items[2].toInt(), (int) sec, msec);

        old_msec             = pose.pos.usec_in_day;
        pose.pos.usec_in_day = static_cast<int64_t>(time.msecsSinceStartOfDay()) * 1000;
        if (old_msec > pose.pos.usec_in_day)
        {
            SPDLOG_ERROR("轨迹文件中的数据未按时间排序！！！");
        }
        pose.pos.latitude  = (items[4].toInt() + items[5].toInt() / 60.0 + items[6].toDouble() / 3600.0) * DEG2RAD;
        pose.pos.longitude = (items[7].toInt() + items[8].toInt() / 60.0 + items[9].toDouble() / 3600.0) * DEG2RAD;
        pose.pos.height    = items[10].toDouble();
        pose.pos.x         = items[11].toDouble();
        pose.std.x_std     = items[12].toFloat();
        pose.pos.y         = items[13].toDouble();
        pose.std.y_std     = items[14].toFloat();
        pose.pos.z         = items[15].toDouble();
        pose.std.z_std     = items[16].toFloat();
        pose.pos.roll      = items[17].toDouble() * DEG2RAD;
        pose.std.roll_std  = items[18].toFloat() * DEG2RAD;
        pose.pos.pitch     = items[19].toDouble() * DEG2RAD;
        pose.std.pitch_std = items[20].toFloat() * DEG2RAD;
        pose.pos.yaw       = items[21].toDouble() * DEG2RAD;
        pose.std.yaw_std   = items[22].toFloat() * DEG2RAD;
        pose.quality       = items.value(23, "0").toInt();
        pose.pos.heave     = items.value(24, "0.0").toDouble();
        poses.push_back(pose);
    }
    return poses;
}

IEPose string2IEPose(const QString &str)
{
    IEPose pose;

    QStringList items = str.split(" ", QString::SkipEmptyParts);
    if (items.size() < 23)
        return pose;

    pose.date = QDate::fromString(items[0], "MM/dd/yyyy");
    if (!pose.date.isValid())
        return pose;

    double sec  = items[3].toDouble();
    int    msec = std::round((sec - (int) sec) * 1000);
    QTime  time(items[1].toInt(), items[2].toInt(), (int) sec, msec);

    pose.pos.usec_in_day = static_cast<int64_t>(time.msecsSinceStartOfDay()) * 1000;
    pose.pos.latitude    = (items[4].toInt() + items[5].toInt() / 60.0 + items[6].toDouble() / 3600.0) * DEG2RAD;
    pose.pos.longitude   = (items[7].toInt() + items[8].toInt() / 60.0 + items[9].toDouble() / 3600.0) * DEG2RAD;
    pose.pos.height      = items[10].toDouble();
    pose.pos.x           = items[11].toDouble();
    pose.std.x_std       = items[12].toFloat();
    pose.pos.y           = items[13].toDouble();
    pose.std.y_std       = items[14].toFloat();
    pose.pos.z           = items[15].toDouble();
    pose.std.z_std       = items[16].toFloat();
    pose.pos.roll        = items[17].toDouble() * DEG2RAD;
    pose.std.roll_std    = items[18].toFloat() * DEG2RAD;
    pose.pos.pitch       = items[19].toDouble() * DEG2RAD;
    pose.std.pitch_std   = items[20].toFloat() * DEG2RAD;
    pose.pos.yaw         = items[21].toDouble() * DEG2RAD;
    pose.std.yaw_std     = items[22].toFloat() * DEG2RAD;
    pose.quality         = items.value(23, "0").toInt();
    pose.pos.heave       = items.value(24, "0.0").toDouble();
    return pose;
}

#include <QFileDialog>

#pragma pack(push, 1)
//! bin文件字段
struct InterpolateAllInformation
{
    int32_t lineID;     ///<扫描仪信息-扫描仪线号。
    double  lineTime;   ///<扫描仪信息-线第一个点时间作为lineTime时间。
    double  gpsTime;    ///<扫描仪信息-插值后扫描仪时间。

    double dist;     ///<扫描仪信息-扫描仪极径。
    double hAngle;   ///<扫描仪信息-扫描仪水平距离角。
    double vAngle;   ///<扫描仪信息-扫描仪竖直距离角。

    uint16_t intensity;   ///<扫描仪信息-扫描仪强度。

    double sigma_distance;   ///<扫描仪信息-扫描仪极径误差。
    double sigma_hAngle;     ///<扫描仪信息-扫描仪极径误差。
    double sigma_vAngle;     ///<扫描仪信息-扫描仪极径误差。

    double roll;      ///<惯导信息-同时刻POS横滚角。
    double pitch;     ///<惯导信息-同时刻POS俯仰角。
    double heading;   ///<惯导信息-同时刻POS方向角。

    double sigma_roll;      ///<惯导信息-同时刻POS横滚角误差。
    double sigma_pitch;     ///<惯导信息-同时刻POS俯仰角误差。
    double sigma_heading;   ///<惯导信息-同时刻POS方向角误差。

    double latitude;     ///<POS信息-同时刻POS纬度-B。
    double longtitude;   ///<POS信息-同时刻POS经度-L。
    double height;       ///<POS信息-同时刻POS高度-H。

    double x_oe;   ///<WGS84  X
    double y_oe;   ///<WGS84  Y
    double z_oe;   ///<WGS84  Z

    double sigma_x_oe;
    double sigma_y_oe;
    double sigma_z_oe;

    double laser_x_e;
    double laser_y_e;
    double laser_z_e;

    uint16_t r;   ///<点云颜色R
    uint16_t g;   ///<点云颜色G
    uint16_t b;   ///<点云颜色B
};
#pragma pack(pop)

#include <Dialog.h>
#include <random>

#include <QInputDialog>

int main(int argc, char *argv[])
{
    spdlog::set_level(spdlog::level::trace);
    QApplication a(argc, argv);

    QStringList files = QFileDialog::getOpenFileNames(nullptr, "pcap", "E:");

    QString num = QInputDialog::getText(nullptr, "set num", "num");
    for (auto filename : files)
    {
        QFileInfo   file_info(filename);
        QString     name  = file_info.baseName();
        QStringList items = name.split("_");
        if (items.size() < 3)
        {
            items.push_back(num);
        }
        QDateTime datetime = QDateTime::fromString(items[0], "yyyy-MM-dd-HH-mm-ss");
        QString   new_name = QString("%1_vlp%2.%3").arg(datetime.toString("yyMMdd-HHmmss")).arg(items[2]).arg(file_info.suffix());
        QFile     file(filename);
        qDebug() << name << "-->" << new_name << ":" << file.rename(file_info.absolutePath() + "/" + new_name);
    }

    /*
    QFile bin_file(filename);

    if (!bin_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_ERROR("打开 bin 文件失败！！！");
        return -1;
    }

    QVector<InterpolateAllInformation> bins;
    while (!bin_file.atEnd())
    {
        InterpolateAllInformation bin_item;
        bin_file.read((char *) &bin_item, sizeof(InterpolateAllInformation));
        bins.push_back(bin_item);
    }
    */

    SPDLOG_INFO("END");
    return 0;
}
