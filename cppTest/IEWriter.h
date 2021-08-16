#pragma once
#include <QFile>
#include <QString>
#include <QVector>

struct IELogItem
{
    double utcTime;
    double latitude;
    double longitude;
    double height;
    double ecef_x;
    double ecef_x_sd;
    double ecef_y;
    double ecef_y_sd;
    double ecef_z;
    double ecef_z_sd;
    double roll;
    double roll_sd;
    double pitch;
    double pitch_sd;
    double heading;
    double heading_sd;
    double heave;   // 垂荡
    double surge;   // 浪涌
    double sway;    // 横摇
};

class IEWriter
{
public:
    IEWriter();
    bool setup(const QString &folder, const QString &project);
    bool addLogItem(const IELogItem &item);
    bool setUtfOffset(double off);
    bool setAntLeverArm(float x, float y, float z);
    bool finish();

private:
    bool writeHeader();
    bool writeItem(const IELogItem &item);
    void degToDMS(double in, int &d, int &m, float &s);
    bool mUtcOffsetReady = true;
    bool mHeaderWritten  = false;

    QVector<IELogItem> mLogs;

    QFile   mIEFile;
    QString mProjectPath;
    QString mProjectName;
    double  mInitLatitude {0.0};
    double  mInitLongitude {0.0};
    double  mInitHeight {0.0};

    double mUtcOffset {-18};
    double mAntLeverArmX {0.0};
    double mAntLeverArmY {0.0};
    double mAntLeverArmZ {0.0};

    const static QString mHeaderTemplate;
    const static QString mDataItemTemplate;
};
