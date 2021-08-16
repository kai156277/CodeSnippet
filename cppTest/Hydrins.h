#pragma once

#include <QDate>
#include <QString>
#include <stdint.h>

static double Pi       = 3.1415926535897932;
static double DegToRad = (Pi / 180.0);
static double RadToDeg = (180.0 / Pi);
static double WGS84A   = 6378137.0;
static double WGS84F   = (1.0 / 298.257223563);
static double E2       = (WGS84F * (2 - WGS84F));

struct PhinsStandard
{
    // hehdt
    double heading;   // 航向

    // atitud
    double roll;    // 横滚
    double pitch;   // 俯仰

    // stdhrp
    double headingSD;
    double rollSD;
    double pitchSD;

    // heave_
    double heave;   // 垂荡
    double surge;   // 浪涌
    double sway;    // 横摇

    // time__
    double time;

    // positi
    double latitude;
    double longitude;
    double altitude;

    // stdpos
    double latitudeSD;
    double longitudeSD;
    double altitudeSD;

    // speed_
    double northSpeed;
    double eastSpeed;
    double upSpeed;

    // stdspd
    double northSpeedSD;
    double eastSpeedSD;
    double upSpeedSD;
    // utmwgs
    QString latitudeUtmZone;
    int32_t longitudeUtmZone;
    double  utmEast;
    double  utmNorth;
    double  utmAltitude;

    // status
    int32_t insSysteamStatus;

    // algsts
    int32_t insAlgorithmStatus;

    // ht_sts
    int32_t insUserStatus;
    int32_t insSensorStatus;
    int32_t otherStatus;

    //    double  logSpeed;
    //    double  logHeadingMisalignment;
    //    double  logPitchMisalignment;
    //    double  logScaleFactorError;
    //    double  soundVelocity;
    //    double  depth;
    //    double  gpsLatitude;
    //    double  gpsLongitude;
    //    double  gpsAltitude;
    //    double  usblLatitude;
    //    double  usblLongitude;
    //    double  usblAltitude;
    //    double  lblLatitude;
    //    double  lblLongitude;
    //    double  lblAltitude;
    //    double  lochEMSpeed;
    //    double  deadReckoningData;
    //    double  utcTime;
};

class Hydrins
{
public:
    Hydrins();
};

int parsePhinsStandard();

int parseBinaryNavHR(const QDateTime &date);
int parseNavigtionHDLC(const QDateTime &date);
int parseNavigtionShort(const QDateTime &date);
int parseGAPSBin(const QDateTime &date);

struct NavItem
{
    double utcTime;

    double heading;   // 航向
    double roll;      // 横滚 左侧抬起为 +
    double pitch;     // 俯仰 低头 +
    double heading_sd;
    double roll_sd;
    double pitch_sd;

    double heave;   // 垂荡, 向上为 +
    double surge;   // 浪涌
    double sway;    // 横摇
    double heave_sd;
    double surge_sd;
    double sway_sd;

    double latitude;
    double longitude;
    double altitude;
    double latitude_sd;
    double longitude_sd;
    double altitude_sd;
};
