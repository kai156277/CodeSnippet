#include "ProjectMgr.h"

#include <QMap>

ProjectManage *ProjectManage::getInstance()
{
    static ProjectManage mgr;
    return &mgr;
}

QString ProjectManage::getTidFile() const
{
    return mTidFile;
}

ProjectManage::ProjectManage()
{
}

QString ProjectManage::getScanType() const
{
    return mScanType;
}

void ProjectManage::setScanType(const QString &scanType)
{
    mScanType = scanType;
}

void ProjectManage::getCurJobTime(double &min_time, double &max_time)
{
}

void ProjectManage::setMBType(const QString &mBType)
{
    mMBType = mBType;
}

QString ProjectManage::getMBType() const
{
    return mMBType;
}

QString ProjectManage::getSoundFile() const
{
    return mSoundFile;
}

void ProjectManage::setSoundFile(const QString &soundFile)
{
    mSoundFile = soundFile;
}

QString ProjectManage::getJobDirPath() const
{
    return mJobDirPath;
}

void ProjectManage::setJobDirPath(const QString &jobDirPath)
{
    mJobDirPath = jobDirPath;
}

QString ProjectManage::getTimeFile() const
{
    return mTimeFile;
}

void ProjectManage::setTimeFile(const QString &timeFile)
{
    mTimeFile = timeFile;
}

bool ProjectManage::isSingle() const
{
    return mIsSingle;
}

float ProjectManage::getDraft(float &hOff) const
{
    return mDraft;
}

void ProjectManage::setDraft(float draft)
{
    mDraft = draft;
}

QString ProjectManage::getSpanFilePath() const
{
    return mSpanFilePath;
}

void ProjectManage::setSpanFilePath(const QString &value)
{
    mSpanFilePath = value;
}

QString ProjectManage::getDataDirPath() const
{
    return mDataDirPath;
}

void ProjectManage::setDataDirPath(const QString &dataDirPath)
{
    mDataDirPath = dataDirPath;
}

QString ProjectManage::getMBDirPath() const
{
    return mMBDirPath;
}

void ProjectManage::setMBDirPath(const QString &mBDirPath)
{
    mMBDirPath = mBDirPath;
}

QString ProjectManage::getLasDirPath() const
{
    return mLasDirPath;
}

void ProjectManage::setLasDirPath(const QString &lasDirPath)
{
    mLasDirPath = lasDirPath;
}

QString ProjectManage::getHeaveFile() const
{
    return mHeaveFile;
}

void ProjectManage::setHeaveFile(const QString &heaveFile)
{
    mHeaveFile = heaveFile;
}

void ProjectManage::setTidFile(const QString &tidFile)
{
    mTidFile = tidFile;
}
