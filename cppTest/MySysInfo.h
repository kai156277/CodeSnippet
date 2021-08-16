#ifndef RESOURCE_MINITOR_H
#define RESOURCE_MINITOR_H
#include <QDebug>
#include <QObject>
#include <QProcess>
#include <QString>
#include <QTimer>
#if defined(Q_OS_LINUX)
#    include "sys/statfs.h"
#else
#    include <windows.h>

#    pragma comment(lib, "Kernel32.lib")
#    pragma comment(lib, "Psapi.lib")
#    include <direct.h>
#    include <psapi.h>
#    include <tlhelp32.h>
#    include <winternl.h>
//#include <atlconv.h>
#    include <cmath>
#    include <string.h>
#endif

class MySysInfo : public QObject
{
    Q_OBJECT
public:
    explicit MySysInfo(QObject *parent = nullptr);
private slots:
    void GetResource();

public:
    bool GetMemUsage(double &nMemTotal, double &nMemUsed);
    bool GetNetUsage();
    bool GetDiskSpeed();
    bool GetCpuUsage(double &nCpuRate);
    bool GetdiskSpace(unsigned long &lFreeAll, unsigned long &lTotalAll);
    bool GetPathSpace(const QString &path);

private:
    const int m_timer_interval__ = 1000;
    QTimer    monitor_timer__;
    double    m_send_bytes__ = 0;
    double    m_recv_bytes__ = 0;
    double    m_disk_read__  = 0;
    double    m_disk_write__ = 0;
    double    m_cpu_total__  = 0;
    double    m_cpu_use__    = 0;
};
#endif   // RESOURCE_MINITOR_H
