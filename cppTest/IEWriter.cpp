#include "IEWriter.h"
#include <QDateTime>
#include <spdlog/qt_spdlog.h>

// clang-format off
const QString IEWriter::mHeaderTemplate =
R"(Project:     %1/%2
Program:     Inertial Explorer Version 8.60.4609
Profile:     VSurs(%3))_Or_calibration (copy) (heave)
Source:      GNSS/INS Epochs(Smoothed TC Combined)
ProcessInfo: %4/%5 by Unknown on %6
Datum:       WGS84, (processing datum)
Master 1:    Name BMAW1442, Status ENABLED
Antenna height 0.000 m, to L1PC [NOV703GGG.R2(NONE)]
Position %7 %8 %9 , %10 %11 %12 , %13 m (WGS84, Ellipsoidal hgt)
Remote:      Antenna height 0.000 m, to L1PC [Generic(NONE)
IMU to GNSS Antenna Lever Arms
x=%14, y=%15, z=%16 m (x-right, y-fwd, z-up)
Body to Sensor Rotations:
xRot=0.000, yRot=0.000, zRot=0.000 degrees (Rotate IMU into Vehicle Frame)
UTC Offset:  %17s
SD Scaling Settings:
Position: 1.0000
Velocity: 1.0000
Attitude: 1.0000


UTCDate     UT UT UTCTim       Latitude        Longitude         H-Ell         X-ECEF        SDX-ECEF       Y-ECEF       SDY-ECEF       Z-ECEF       SDZ-ECEF       Roll       RollSD      Pitch      PitchSD      Heading     HdngSD     Q        Heave
(MDY)       ho mi  (sec)      (+/-D M S)       (+/-D M S)         (m)           (m)             (m)           (m)          (m)            (m)          (m)          (deg)       (deg)      (deg)       (deg)        (deg)      (deg)                (m)
)";

const QString IEWriter::mDataItemTemplate =
R"(%1   %2 %3 %4     %5 %6 %7     %8      %9      %10      %11      %12      %13      %14      %15      %16      %17      %18      %19      %20      1      %21
)";

// clang-format on

IEWriter::IEWriter()
{
}

bool IEWriter::setup(const QString &folder, const QString &project)
{

    if (folder.isEmpty() || project.isEmpty())
    {
        return false;
    }
    QString fpath = folder + "/" + project + "_000.txt";
    SPDLOG_INFO("file path: {}", fpath);
    mIEFile.setFileName(fpath);
    if (!mIEFile.open(QFile::WriteOnly))
    {
        return false;
    }
    mLogs.clear();
    mProjectPath    = folder;
    mProjectName    = project;
    mHeaderWritten  = false;
    mUtcOffsetReady = true;   // 待修正
    return true;
}

bool IEWriter::addLogItem(const IELogItem &item)
{
    if (!mHeaderWritten)
    {
        if (mUtcOffsetReady)
        {   // 可以添加其他条件
            mInitLatitude  = item.latitude;
            mInitLongitude = item.longitude;
            mInitHeight    = item.height;
            writeHeader();
            for (const auto &j : mLogs)
            {
                writeItem(j);
            }
            writeItem(item);
            mLogs.clear();
        }
        else
        {
            mLogs.push_back(item);
        }
    }
    else
    {
        writeItem(item);
    }
    return true;
}

bool IEWriter::setUtfOffset(double off)
{
    mUtcOffset      = off;
    mUtcOffsetReady = true;
    SPDLOG_INFO("UTC offset: {}", off);
    return true;
}

bool IEWriter::setAntLeverArm(float x, float y, float z)
{
    mAntLeverArmX = x;
    mAntLeverArmY = y;
    mAntLeverArmZ = z;
    return true;
}

bool IEWriter::finish()
{
    if (!mHeaderWritten)
    {
        writeHeader();
        for (const auto &i : mLogs)
        {
            writeItem(i);
        }
    }
    mLogs.clear();
    return true;
}

bool IEWriter::writeHeader()
{

    int   latitudeDegree, latitudeMinute, longitudeDegree, longitudeMinute;
    float latitudeSecond, longitudeSecond;
    degToDMS(mInitLatitude, latitudeDegree, latitudeMinute, latitudeSecond);
    degToDMS(mInitLongitude, longitudeDegree, longitudeMinute, longitudeSecond);

    QDateTime datetime = QDateTime::currentDateTime();
    QString   header   = mHeaderTemplate.arg("ProjectPath").arg("ProjectName")   // line 1
                         .arg("ProjectName")                                     // line 3
                         .arg("ProjectPath")
                         .arg("ProjectName")
                         .arg(datetime.toString(("MM/dd/yyyy hh:mm:ss")))   // line 5
                         // line 9
                         .arg(latitudeDegree)
                         .arg(latitudeMinute)
                         .arg(QString::number(latitudeSecond, 'f', 3))
                         .arg(longitudeDegree)
                         .arg(longitudeMinute)
                         .arg(QString::number(longitudeSecond, 'f', 3))
                         .arg(QString::number(mInitHeight, 'f', 3))
                         .arg(mAntLeverArmX)
                         .arg(mAntLeverArmY)
                         .arg(mAntLeverArmZ)   // line 12
                         .arg(-mUtcOffset)     // line 15
        ;
    mIEFile.write(header.toUtf8());
    mHeaderWritten = true;
    return true;
}

bool IEWriter::writeItem(const IELogItem &item)
{
    QDateTime dateTime = QDateTime::fromTime_t(item.utcTime, Qt::UTC);

    double ds = item.utcTime - floor(item.utcTime);
    ds *= 1000.0;
    int     ms   = floor(ds);
    QString time = dateTime.toString("MM/dd/yyyy  hh mm ss") + "." + QString("%1").arg(ms, 3, 10, QChar('0'));

    int    latitudeDegree = item.latitude;
    int    latitudeMinute = (item.latitude - latitudeDegree) * 60;
    double latitudeSecond = ((item.latitude - latitudeDegree) * 60.0 - latitudeMinute) * 60.0;

    int    longitudeDegree = item.longitude;
    int    longitudeMinute = (item.longitude - longitudeDegree) * 60;
    double longitudeSecond = ((item.longitude - longitudeDegree) * 60.0 - longitudeMinute) * 60.0;

    float azi = item.heading;

    if (azi > 180.0)
    {
        azi -= 360.0;
    }
    QString dstr = mDataItemTemplate.arg(time)
                       .arg(latitudeDegree)
                       .arg(latitudeMinute)
                       .arg(QString::number(latitudeSecond, 'f', 6))
                       .arg(longitudeDegree)
                       .arg(longitudeMinute)
                       .arg(QString::number(longitudeSecond, 'f', 6))
                       .arg(QString::number(item.height, 'f', 6))
                       .arg(QString::number(item.ecef_x, 'f', 3))
                       .arg(QString::number(item.ecef_x_sd, 'f', 3))
                       .arg(QString::number(item.ecef_y, 'f', 3))
                       .arg(QString::number(item.ecef_y_sd, 'f', 3))
                       .arg(QString::number(item.ecef_z, 'f', 3))
                       .arg(QString::number(item.ecef_z_sd, 'f', 3))
                       .arg(QString::number(item.roll, 'f', 6))
                       .arg(QString::number(item.roll_sd, 'f', 3))
                       .arg(QString::number(item.pitch, 'f', 6))
                       .arg(QString::number(item.pitch_sd, 'f', 3))
                       .arg(QString::number(azi, 'f', 6))
                       .arg(QString::number(item.heading_sd, 'f', 3))
                       .arg(QString::number(item.heave, 'f', 6));

    mIEFile.write(dstr.toUtf8());

    return true;
}

void IEWriter::degToDMS(double in, int &d, int &m, float &s)
{
    d = int(in);
    m = int((in - d) * 60);
    s = float(((in - d) * 60 - m) * 60.0);
}
