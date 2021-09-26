#include "parse7k.h"
#include <QDataStream>
#include <QDebug>
#include <QFile>
#include <QString>
#include <QTextStream>
#include <QtCore/QCoreApplication>
#include <qmath.h>

PINGINFO parse_real_time_Data(QByteArray data)
{
    PINGINFO temp;
    char *   buffer;
    buffer = data.data();
    quint16 version0, version1;
    quint16 offset0, offset1;
    memcpy(&version0, buffer, 2);
    memcpy(&offset0, buffer + 2, 2);
    memcpy(&version1, buffer + 4, 2);
    memcpy(&offset1, buffer + 6, 2);
    int init = 0;
    if ((version0 == 5) && (offset0 == 36))
    {
        init = 0;
    }
    else if ((version1 == 5) && (offset1 == 36))
    {
        init = 4;
    }
    else
    {
        qDebug() << "data transmit error!" << endl;
    }
    quint32 recordType;
    memcpy(&recordType, init + buffer + 68, 4);
    switch (recordType)
    {
    case 7000: {
        temp = _parseData7000(buffer, init);
        break;
    }
    case 7004: {
        temp = _parseData7004(buffer, init);
        break;
    }
    case 7006: {
        temp = _parseData7006(buffer, init);
        break;
    }
    default: break;
    }
    return temp;
}

PINGINFO _parseData7000(char *buffer, int init)
{
    PINGINFO         temp;
    NetWorkFrame *   _NF  = (NetWorkFrame *) (buffer + init);
    DataRecordFrame *_DRF = (DataRecordFrame *) (buffer + init + sizeof(NetWorkFrame));
    R7000RTH *       _RTH = (R7000RTH *) (buffer + init + sizeof(NetWorkFrame) + sizeof(DataRecordFrame));
    temp.flag             = 7000;
    temp.K7Time.mYear     = _DRF->mTimeStamp.mYear;      // 年份 2018
    temp.K7Time.mDay      = _DRF->mTimeStamp.mDay;       // 1-366
    temp.K7Time.mHours    = _DRF->mTimeStamp.mHours;     // 0-23
    temp.K7Time.mMinutes  = _DRF->mTimeStamp.mMinutes;   // 0-59
    temp.K7Time.mSeconds  = _DRF->mTimeStamp.mSeconds;   // 0.000000-59.999999

    temp.K7000SonarSettings = *_RTH;

    return temp;
}

PINGINFO _parseData7004(char *buffer, int init)
{
    PINGINFO temp;
    //    QFile writefile("parse_7004.txt");
    //    if(!writefile.open((QIODevice::ReadWrite|QIODevice::Text|QIODevice::Append))) {
    //        qDebug() << "open writefile fail!"<<endl;
    //    }
    //    QTextStream out(&writefile);
    quint16 version;
    memcpy(&version, init + buffer + 0, 2);
    //    out << "1_version:"<<version<<'\n';
    quint16 offset;
    memcpy(&offset, init + buffer + 2, 2);
    //    out << "2_Offset:"<<offset <<'\n';
    quint32 total_packets;
    memcpy(&total_packets, init + buffer + 4, 4);
    //    out << "3_Total packets:"<<total_packets << '\n';
    quint16 size4;
    memcpy(&size4, init + buffer + 8, 2);
    //    out << "4_Total records:"<<size4 << '\n';
    quint16 size5;
    memcpy(&size5, init + buffer + 10, 2);
    //    out << "5_transmission identifier:"<<size5 << '\n';
    quint32 size6;
    memcpy(&size6, init + buffer + 12, 4);
    //    out << "6_packet size:"<<size6 << '\n';
    quint32 size7;
    memcpy(&size7, init + buffer + 16, 4);
    //    out << "7_Total size:"<<size7 << '\n';
    quint32 size8;
    memcpy(&size8, init + buffer + 20, 4);
    //    out << "8_Sequence number:"<<size8 << '\n';
    quint32 size9;
    memcpy(&size9, init + buffer + 24, 4);
    //    out << "9_Desitination device identifier:"<<size9 << '\n';
    quint16 size10;
    memcpy(&size10, init + buffer + 28, 2);
    //    out << "10_Destination enumerator:"<<size10 << '\n';
    quint16 size11;
    memcpy(&size11, init + buffer + 30, 2);
    //    out << "11_Source enumerator:"<<size11 << '\n';
    quint32 size12;
    memcpy(&size12, init + buffer + 32, 4);
    //    out << "12_Source device identifier:"<<size12 << '\n';
    /**Data Record Frame**/
    quint16 size13;
    memcpy(&size13, init + buffer + 36, 2);
    //    out << "13_Protocol version:"<<size13 << '\n';
    quint16 size14;
    memcpy(&size14, init + buffer + 38, 2);
    //    out << "14_Offset:"<<size14 << '\n';
    quint32 size15;
    memcpy(&size15, init + buffer + 40, 4);
    //    out << "15_Sync pattern:"<< hex<<size15 <<dec<< '\n';
    quint32 size16;
    memcpy(&size16, init + buffer + 44, 4);
    //    out << "16_Size:"<<size16 << '\n';
    quint32 size17;
    memcpy(&size17, init + buffer + 48, 4);
    //    out << "17_Optinal data offset:"<<size17 << '\n';
    quint32 size18;
    memcpy(&size18, init + buffer + 52, 4);
    //    out << "18_Optinal data identifier:"<<size18 << '\n';

    //    out << "19_7KTIME:\n";
    quint16 year;
    memcpy(&year, init + buffer + 56, 2);
    //    out << "19_year:" << year<<'\n';
    quint16 day2;
    memcpy(&day2, init + buffer + 58, 2);
    //    out << "19_day:" << day2<<'\n';
    quint8 hours;
    memcpy(&hours, init + buffer + 64, 1);
    //    out << "19_hours:" << hours<<'\n';
    quint8 minutes;
    memcpy(&minutes, init + buffer + 65, 1);
    //    out << "19_minutes:" << minutes<<'\n';
    float seconds;
    memcpy(&seconds, init + buffer + 60, 4);
    //    out << "19_seconds:" << seconds<<'\n';
    /*********assignment temp*************************/
    temp.flag            = 7004;
    temp.K7Time.mYear    = year;      // 年份 2018
    temp.K7Time.mDay     = day2;      // 1-366
    temp.K7Time.mHours   = hours;     // 0-23
    temp.K7Time.mMinutes = minutes;   // 0-59
    temp.K7Time.mSeconds = seconds;   // 0.000000-59.999999
    /*********assignment temp*************************/
    quint16 size20;
    memcpy(&size20, init + buffer + 66, 2);
    //    out << "20_Reserved:"<<size20 << '\n';
    quint32 size21;
    memcpy(&size21, init + buffer + 68, 4);
    //    out << "21_Record type identifier:"<<size21 << '\n';
    quint32 size22;
    memcpy(&size22, init + buffer + 72, 4);
    //    out << "22_Device identifier:"<<size22 << '\n';
    quint32 size23;
    memcpy(&size23, init + buffer + 76, 4);
    //    out << "23_System enumerator:"<<size23 << '\n';
    quint32 size24;
    memcpy(&size24, init + buffer + 80, 4);
    //    out << "24_Reserved:"<<size24 << '\n';
    quint16 size25;
    memcpy(&size25, init + buffer + 84, 2);
    //    out << "25_Flags:"<<bin<<size25 << dec<<'\n';
    quint16 size26;
    memcpy(&size26, init + buffer + 86, 2);
    //    out << "26_Reserved:"<<size26 << '\n';
    quint32 size27;
    memcpy(&size27, init + buffer + 88, 4);
    //    out << "27_Reserved:"<<size27 << '\n';
    quint32 size28;
    memcpy(&size28, init + buffer + 92, 4);
    //    out << "28_Total records in fragmentd data:"<<size28 << '\n';
    quint32 size29;
    memcpy(&size29, init + buffer + 96, 4);
    //    out << "29_Fragment number:"<<size29 << '\n';
    /***7004 Record Type Header****/
    quint64 size30;
    memcpy(&size30, init + buffer + 100, 8);
    //    out << "30_Sonar Id:"<<size30 << '\n';
    quint32 N;
    memcpy(&N, init + buffer + 108, 4);
    //    out << "31_Ping numbers:"<< N << '\n';
    /*********assignment temp*************************/
    temp.K7PointNum = N;
    /*********assignment temp*************************/
    float size32[512];
    for (quint32 i = 0; i < N; i++)
    {
        memcpy(&size32[i], init + buffer + 112 + 4 * i, 4);
        //    out << "32_Beam vertical direction angle["<<i+1<<"]:"<<size32[i] << '\n';          // 通常是0；
    }
    float size33[512];
    for (quint32 i = 0; i < N; i++)
    {
        memcpy(&size33[i], init + buffer + 112 + 4 * N + 4 * i, 4);
        //    out << "33_Beam horizontal direction angle["<<i+1<<"]:"<<size33[i] << '\n';
        /*********assignment temp*************************/
        temp.K7004Geometry.dir_ang[i] = size33[i];
        /*********assignment temp*************************/
    }

    float size34[512];
    for (quint32 i = 0; i < N; i++)
    {
        memcpy(&size34[i], init + buffer + 112 + 8 * N + 4 * i, 4);
        //    out << "34_-3dB Beam width Y["<<i+1<<"]:"<<size34[i] << '\n';
    }
    float size35[512];
    for (quint32 i = 0; i < N; i++)
    {
        memcpy(&size35[i], init + buffer + 112 + 12 * N + 4 * i, 4);
        //    out << "35_-3dB Beam width X["<<i+1<<"]:"<<size35[i] << '\n';
    }

    quint32 size36;
    memcpy(&size36, init + buffer + 112 + 16 * N, 4);
    //    out << "36_CheckSum:"<<size36 << '\n';
    //    writefile.close();
    return temp;
}

PINGINFO _parseData7006(char *buffer, int init)
{
    PINGINFO temp;
    //    QFile writefile("parse_7006.txt");
    //    if(!writefile.open((QIODevice::ReadWrite|QIODevice::Text|QIODevice::Append))) {
    //        qDebug() << "open writefile fail!"<<endl;
    //    }
    //    QTextStream out(&writefile);
    quint16 version;
    memcpy(&version, init + buffer + 0, 2);
    //    out << "1_version:"<<version<<'\n';
    quint16 offset;
    memcpy(&offset, init + buffer + 2, 2);
    //    out << "2_Offset:"<<offset <<'\n';
    quint32 total_packets;
    memcpy(&total_packets, init + buffer + 4, 4);
    //    out << "3_Total packets:"<<total_packets << '\n';
    quint16 size4;
    memcpy(&size4, init + buffer + 8, 2);
    //    out << "4_Total records:"<<size4 << '\n';
    quint16 size5;
    memcpy(&size5, init + buffer + 10, 2);
    //    out << "5_transmission identifier:"<< size5 <<  '\n';
    quint32 size6;
    memcpy(&size6, init + buffer + 12, 4);
    //    out << "6_packet size:"<<size6 << '\n';
    quint32 size7;
    memcpy(&size7, init + buffer + 16, 4);
    //    out << "7_Total size:"<<size7 << '\n';
    quint32 size8;
    memcpy(&size8, init + buffer + 20, 4);
    //    out << "8_Sequence number:"<<size8 << '\n';
    quint32 size9;
    memcpy(&size9, init + buffer + 24, 4);
    //    out << "9_Desitination device identifier:"<<size9 << '\n';
    quint16 size10;
    memcpy(&size10, init + buffer + 28, 2);
    //    out << "10_Destination enumerator:"<<size10 << '\n';
    quint16 size11;
    memcpy(&size11, init + buffer + 30, 2);
    //    out << "11_Source enumerator:"<<size11 << '\n';
    quint32 size12;
    memcpy(&size12, init + buffer + 32, 4);
    //    out << "12_Source device identifier:"<<size12 << '\n';
    /**Data Record Frame**/
    quint16 size13;
    memcpy(&size13, init + buffer + 36, 2);
    //    out << "13_Protocol version:"<<size13 << '\n';
    quint16 size14;
    memcpy(&size14, init + buffer + 38, 2);
    //    out << "14_Offset:"<<size14 << '\n';
    quint32 size15;
    memcpy(&size15, init + buffer + 40, 4);
    //    out << "15_Sync pattern:"<< hex<<size15 <<dec<< '\n';
    quint32 size16;
    memcpy(&size16, init + buffer + 44, 4);
    //    out << "16_Size:"<<size16 << '\n';
    quint32 size17;
    memcpy(&size17, init + buffer + 48, 4);
    //    out << "17_Optinal data offset:"<<size17 << '\n';
    quint32 size18;
    memcpy(&size18, init + buffer + 52, 4);
    //    out << "18_Optinal data identifier:"<<size18 << '\n';

    //    out << "19_7KTIME:\n";
    quint16 year;
    memcpy(&year, init + buffer + 56, 2);
    //    out << "19_year:" << year<<'\n';
    quint16 day2;
    memcpy(&day2, init + buffer + 58, 2);
    //    out << "19_day:" << day2<<'\n';
    quint8 hours;
    memcpy(&hours, init + buffer + 64, 1);
    //    out << "19_hours:" << hours<<'\n';
    quint8 minutes;
    memcpy(&minutes, init + buffer + 65, 1);
    //    out << "19_minutes:" << minutes<<'\n';
    float seconds;
    memcpy(&seconds, init + buffer + 60, 4);
    //    out << "19_seconds:" << seconds<<'\n';
    /*********assignment temp*************************/
    temp.flag            = 7006;
    temp.K7Time.mYear    = year;      // 年份 2018
    temp.K7Time.mDay     = day2;      // 1-366
    temp.K7Time.mHours   = hours;     // 0-23
    temp.K7Time.mMinutes = minutes;   // 0-59
    temp.K7Time.mSeconds = seconds;   // 0.000000-59.999999
    /*********assignment temp*************************/
    quint16 size20;
    memcpy(&size20, init + buffer + 66, 2);
    //    out << "20_Reserved:"<<size20 << '\n';
    quint32 size21;
    memcpy(&size21, init + buffer + 68, 4);
    //    out << "21_Record type identifier:"<<size21 << '\n';
    quint32 size22;
    memcpy(&size22, init + buffer + 72, 4);
    //    out << "22_Device identifier:"<<size22 << '\n';
    quint32 size23;
    memcpy(&size23, init + buffer + 76, 4);
    //    out << "23_System enumerator:"<<size23 << '\n';
    quint32 size24;
    memcpy(&size24, init + buffer + 80, 4);
    //    out << "24_Reserved:"<<size24 << '\n';
    quint16 size25;
    memcpy(&size25, init + buffer + 84, 2);
    //    out << "25_Flags:"<<bin<<size25 << dec<<'\n';
    quint16 size26;
    memcpy(&size26, init + buffer + 86, 2);
    //    out << "26_Reserved:"<<size26 << '\n';
    quint32 size27;
    memcpy(&size27, init + buffer + 88, 4);
    //    out << "27_Reserved:"<<size27 << '\n';
    quint32 size28;
    memcpy(&size28, init + buffer + 92, 4);
    //    out << "28_Total records in fragmentd data:"<<size28 << '\n';
    quint32 size29;
    memcpy(&size29, init + buffer + 96, 4);
    //    out << "29_Fragment number:"<<size29 << '\n';
    /***7006 Record Type Header****/
    quint64 size30;
    memcpy(&size30, init + buffer + 100, 8);
    //    out << "30_Sonar Id:"<<size30 << '\n';
    quint32 size31;
    memcpy(&size31, init + buffer + 108, 4);
    //    out << "31_Ping number:"<<size31 << '\n';
    temp.K7006PingNum = size31;
    quint16 size32;
    memcpy(&size32, init + buffer + 112, 2);
    //    out << "32_Multi-ping sequence:"<<size32 << '\n';
    quint32 N; /**pings in all **/
    memcpy(&N, init + buffer + 114, 4);
    //    out << "33_N:"<<N << '\n';
    /*********assignment temp*************************/
    temp.K7PointNum = N;
    /*********assignment temp*************************/
    quint8 size34;
    memcpy(&size34, init + buffer + 118, 1);
    //    out << "34_Flags:"<<bin<<size34 << dec<<'\n';
    quint8 size35;
    memcpy(&size35, init + buffer + 119, 1);
    //    out << "35_Sound velocity flag:"<<bin<<size35 << dec<<'\n';
    float size36;
    memcpy(&size36, init + buffer + 120, 4);
    //    out << "36_Sound velocity:"<<size36 << '\n';
    /*********assignment temp*************************/
    temp.K7006Bathmetry.velocity = size36;
    /*********assignment temp*************************/
    /***7006 Record Data****/
    float Range[512];
    for (quint32 i = 0; i < N; i++)
    {
        memcpy(&Range[i], init + buffer + 124 + i * 4, 4);
        //    out << "37_Range["<<i+1<<"]:"<<Range[i] << '\n';
        /*********assignment temp*************************/
        temp.K7006Bathmetry.range[i] = Range[i];
        /*********assignment temp*************************/
    }

    quint8 Quality[512];
    for (quint32 i = 0; i < N; i++)
    {
        memcpy(&Quality[i], init + buffer + 124 + 4 * N + i, 1);
        //    out << "38_Quality["<<i+1<<"]:"<<bin<<Quality[i] << dec<<'\n';
    }

    float Intensity[512];
    for (quint32 i = 0; i < N; i++)
    {
        memcpy(&Intensity[i], init + buffer + 124 + 5 * N + i * 4, 4);
        //    out << "39_Intensity["<<i+1<<"]:"<<Intensity[i] <<'\n';
        temp.K7006Bathmetry.intensite.push_back(Intensity[i]);
    }
    float Min_filter[512];
    for (quint32 i = 0; i < N; i++)
    {
        memcpy(&Min_filter[i], init + buffer + 124 + 9 * N + i * 4, 4);
        //    out << "40_Min_filter["<<i+1<<"]:"<<Min_filter[i] <<'\n';
    }
    float Max_filter[512];
    for (quint32 i = 0; i < N; i++)
    {
        memcpy(&Max_filter[i], init + buffer + 124 + 13 * N + i * 4, 4);
        //    out << "41_Max_filter["<<i+1<<"]:"<<Max_filter[i] <<'\n';
    }
    switch (size17)
    {
    case 0: {
        quint32 CheckSum;
        memcpy(&CheckSum, init + buffer + 124 + 17 * N, 4);
        //    out << "42_CheckSum:" << CheckSum << '\n';
        break;
    }
    default: {
        float size42;
        memcpy(&size42, init + buffer + 124 + 17 * N, 4);
        //    out << "42_Frequency:"<<size42 << '\n';
        double size43;
        memcpy(&size43, init + buffer + 128 + 17 * N, 8);
        //    out << "43_Latitude:"<<size43 << '\n';
        double size44;
        memcpy(&size44, init + buffer + 136 + 17 * N, 8);
        //    out << "44_longitude:"<<size44 << '\n';
        float size45;
        memcpy(&size45, init + buffer + 144 + 17 * N, 4);
        //    out << "45_Frequency:"<<size45 << '\n';
        quint8 size46;
        memcpy(&size46, init + buffer + 148 + 17 * N, 1);
        //    out << "46_Height source:"<<size46 << '\n';
        float size47;
        memcpy(&size47, init + buffer + 149 + 17 * N, 4);
        //    out << "47_Tide:"<<size47 << '\n';
        float size48;
        memcpy(&size48, init + buffer + 153 + 17 * N, 4);
        //    out << "48_Roll:"<<size48 << '\n';
        float size49;
        memcpy(&size49, init + buffer + 157 + 17 * N, 4);
        //    out << "49_Pitch:"<<size49 << '\n';
        float size50;
        memcpy(&size50, init + buffer + 161 + 17 * N, 4);
        //    out << "50_Heave:"<<size50 << '\n';
        float size51;
        memcpy(&size51, init + buffer + 165 + 17 * N, 4);
        //    out << "51_Vehicle depth:"<<size51 << '\n';
        BEAM beam[512];
        for (quint32 i = 0; i < N; i++)
        {
            memcpy(&beam[i].Depth, init + buffer + 169 + 17 * N + 20 * i, 4);
            //    out << "beam["<<i+1<<"].Depth"<<beam[i].Depth << '\n';
            memcpy(&beam[i].Along_track, init + buffer + 169 + 17 * N + 20 * i + 4, 4);
            //    out << "beam["<<i+1<<"].Along track"<<beam[i].Along_track << '\n';
            memcpy(&beam[i].Across_track, init + buffer + 169 + 17 * N + 20 * i + 8, 4);
            //    out << "beam["<<i+1<<"].Across track"<<beam[i].Across_track << '\n';
            memcpy(&beam[i].Pointing_angle, init + buffer + 169 + 17 * N + 20 * i + 12, 4);
            //    out << "beam["<<i+1<<"].Pointing angle"<<beam[i].Pointing_angle << '\n';
            memcpy(&beam[i].Azimuth_angle, init + buffer + 169 + 17 * N + 20 * i + 16, 4);
            //    out << "beam["<<i+1<<"].Azimuth angle"<<beam[i].Azimuth_angle << '\n';
        }
        quint32 CheckSum;
        memcpy(&CheckSum, init + buffer + 169 + 37 * N, 4);
        //    out << "53_CheckSum:" << CheckSum << '\n';
        break;
    }
    }
    //    writefile.close();
    return temp;
}

PINGINFO ping_merge(PINGINFO p7000, PINGINFO p7004, PINGINFO p7006)
{
    if ((p7000.flag == 7000 && p7004.flag == 7004 && p7006.flag == 7006))
    {
        p7000.flag                    = 0;
        p7000.K7PointNum              = p7004.K7PointNum;
        p7000.K7Time.mYear            = p7004.K7Time.mYear;
        p7000.K7Time.mDay             = p7004.K7Time.mDay;
        p7000.K7Time.mHours           = p7004.K7Time.mHours;
        p7000.K7Time.mMinutes         = p7004.K7Time.mMinutes;
        p7000.K7Time.mSeconds         = p7004.K7Time.mSeconds;
        p7000.K7006PingNum            = p7006.K7006PingNum;
        p7000.K7006Bathmetry.velocity = p7006.K7006Bathmetry.velocity;
        for (quint32 i = 0; i < p7004.K7PointNum; i++)
        {
            p7000.K7004Geometry.dir_ang[i] = p7004.K7004Geometry.dir_ang[i];
            p7000.K7006Bathmetry.range[i]  = p7006.K7006Bathmetry.range[i];
        }
    }
    else
    {
        qDebug() << "merge error!" << endl;
    }
    return p7000;
}

bool ping_diff(PINGINFO p1, PINGINFO p2, PINGINFO p3)
{
    /*
    if (p1.K7Time.year == p2.K7Time.year && p1.K7Time.year == p3.K7Time.year)
    {
        if (p1.K7Time.day == p2.K7Time.day && p1.K7Time.day == p3.K7Time.day)
        {
            if (p1.K7Time.hour == p2.K7Time.hour && p1.K7Time.hour == p3.K7Time.hour)
            {
                if (p1.K7Time.minute == p2.K7Time.minute && p1.K7Time.minute == p3.K7Time.minute)
                {
                    if (p1.K7Time.second == p2.K7Time.second && p1.K7Time.second == p3.K7Time.second)
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    */
    return true;
}
