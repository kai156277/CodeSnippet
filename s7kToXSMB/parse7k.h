#ifndef PARSE7K_H
#define PARSE7K_H
#include "7kdef.h"
#include <QDebug>
#include <QFile>
#include <QString>
#include <QTextStream>

typedef struct beam
{
    float Depth;
    float Along_track;
    float Across_track;
    float Pointing_angle;
    float Azimuth_angle;
} BEAM;

typedef struct record_data
{
    quint16 Beam_descriptor;
    float   Detection_point;
    quint32 Flags;
    float   Auto_min;
    float   Auto_max;
    float   User_min;
    float   User_max;
    quint32 Quality;
    float   Uncertainty;
} RECORD7017;

typedef struct _geoinfo
{
    float dir_ang[512];

} K7004GEOINFO;

typedef struct _bathinfo
{
    float        velocity;
    float        range[512];
    QList<float> intensite;
} K7006BATHINFO;

typedef struct _pinginfo
{
    quint32       flag;
    S7KTime       K7Time;
    quint32       K7PointNum;
    K7004GEOINFO  K7004Geometry;
    K7006BATHINFO K7006Bathmetry;
    quint32       K7006PingNum;
    R7000RTH      K7000SonarSettings;
} PINGINFO;

/*  **********the following are real time functions****************/
PINGINFO parse_real_time_Data(QByteArray data);
PINGINFO _parseData7000(char *buffer, int init);
PINGINFO _parseData7004(char *buffer, int init);
PINGINFO _parseData7006(char *buffer, int init);

/*********** functions for PINGINFO type variables*****************/
PINGINFO ping_merge(PINGINFO p7000, PINGINFO p7004, PINGINFO p7006);
bool     ping_diff(PINGINFO p1, PINGINFO p2, PINGINFO p3);

#endif   // PARSE7K_H
