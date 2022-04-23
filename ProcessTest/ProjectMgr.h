#pragma once

#include <QString>

#include <ProjectInterface.h>

class ProjectManage
{
public:
    static ProjectManage *getInstance();

    QString getTidFile() const;
    void    setTidFile(const QString &tidFile);
    QString getHeaveFile() const;
    void    setHeaveFile(const QString &heaveFile);

    QString getLasDirPath() const;
    void    setLasDirPath(const QString &lasDirPath);

    QString getMBDirPath() const;
    void    setMBDirPath(const QString &mBDirPath);

    QString getDataDirPath() const;
    void    setDataDirPath(const QString &dataDirPath);

    QString getSpanFilePath() const;
    void    setSpanFilePath(const QString &value);

    float getDraft(float &hOff) const;
    void  setDraft(float draft);

    bool isSingle() const;

    QString getTimeFile() const;
    void    setTimeFile(const QString &timeFile);

    QString getJobDirPath() const;
    void    setJobDirPath(const QString &jobDirPath);

    QString getSoundFile() const;
    void    setSoundFile(const QString &soundFile);

    QString getMBType() const;

    void setMBType(const QString &mBType);

    QString getScanType() const;
    void    setScanType(const QString &scanType);

    void getCurJobTime(double &min_time, double &max_time);

private:
    ProjectManage();

    QString mScanType;
    QString mMBType;
    QString mSoundFile;
    QString mJobDirPath;
    QString mTimeFile;
    QString mTidFile;
    QString mHeaveFile;
    QString mLasDirPath;
    QString mMBDirPath;
    QString mDataDirPath;
    QString mSpanFilePath;
    float   mDraft;
    bool    mIsSingle;
};
