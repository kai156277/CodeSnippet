#include <QCoreApplication>

#include "CrossSection.h"
#include "EllipseRANSAC.h"
#include "LASALL.h"
#include "LASFile.h"
#include "TunnelPointCloud.h"
#include <LASlib/lasreader.hpp>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <iostream>
#include <spdlog/qt_spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <vector>

#include <windows.h>

#include <DbgHelp.h>

#pragma comment(lib, "DbgHelp.lib")

LONG WINAPI TopLevelFilter(struct _EXCEPTION_POINTERS *pExceptionInfo)
{
    // 返回EXCEPTION_CONTINUE_SEARCH，让程序停止运行
    LONG ret = EXCEPTION_CONTINUE_SEARCH;

    // 设置core文件生成目录和文件名
    QString currentTime = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".dmp";

    HANDLE hFile = ::CreateFile(currentTime.toStdWString().data(), GENERIC_WRITE, FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    if (hFile != INVALID_HANDLE_VALUE)
    {
        MINIDUMP_EXCEPTION_INFORMATION ExInfo;

        ExInfo.ThreadId          = ::GetCurrentThreadId();
        ExInfo.ExceptionPointers = pExceptionInfo;
        ExInfo.ClientPointers    = NULL;

        // write the dump
        BOOL bOK = MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile, MiniDumpNormal, &ExInfo, NULL, NULL);
        ret      = EXCEPTION_EXECUTE_HANDLER;
        ::CloseHandle(hFile);
    }

    return ret;
}

int main(int argc, char *argv[])
{

    system("chcp 65001");
    ::SetUnhandledExceptionFilter(TopLevelFilter);
    QCoreApplication a(argc, argv);
    QString          logfile_name = "logs/lastest_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".log";
    spdlog::flush_every(std::chrono::seconds(1));

    qDebug() << "logfile_name: " << logfile_name;
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logfile_name.toStdString(), true);
    qDebug() << "make_shared ";
    file_sink->set_level(spdlog::level::trace);
    file_sink->set_pattern("[%H:%M:%S.%F] [%=8l] [%=8t] [%s:%#] %v");

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    console_sink->set_pattern("[%Y/%m/%d %H:%M:%S.%F] %^[%=8l] [%=8t] [%s:%#] %v");

    spdlog::sinks_init_list         sinks {file_sink, console_sink};
    std::shared_ptr<spdlog::logger> logger = std::make_shared<spdlog::logger>("cloudpoint", sinks);
    spdlog::set_default_logger(logger);
    spdlog::set_level(spdlog::level::trace);
    spdlog::flush_every(std::chrono::seconds(1));

    QString   fls_file = "D:\\Data\\TunnelMonitor\\jn2w\\jn2w012.fls\\jn2w012.fls";
    QString   las_file = "D:\\Data\\TunnelMonitor\\jn2w\\jn2w012.las";
    QFileInfo las_file_info(las_file);
    QString   las_root_file = las_file_info.absolutePath() + "\\" + las_file_info.baseName();
    if (false)
    {
        // ScanAllLine::flsToLAS(fls_file, las_file);

        TunnelPointCloud scan_data;
        qDebug() << "readDataFromLASFile: " << scan_data.readDataFromLASFile(las_file);

        //        int size = scan_data.mAllLineData.size() / 10;
        int size = 100;
        scan_data.saveLineFile(0, size, las_root_file + QString("_0-%1.las").arg(size));
    }

    TunnelPointCloud scan_data;
    bool             flag = scan_data.readDataFromLASFile("D:\\Data\\TunnelMonitor\\jn2w\\jn2w012_0-100.las");
    SPDLOG_INFO("readDataFromLASFile: {}", flag);

    const int                   count = scan_data.mAllLineData.size();
    pcl::PointCloud<PointXYZIT> use_pts;
    use_pts.reserve(count * 5000);
    SPDLOG_INFO("start to run RANSAC");
    for (int i = 0; i < count; ++i)
    {
        //        scan_data.mAllLineData[i].racsac();
        //        // OUTPUT:
        //        scan_data.mAllLineData[i].ellipseFit();
        //        // OUTPUT:
        //        ransac.compute(scan_data.mAllLineData[i], inliers);
        //        scan_data.mAllLineData[i].flags = inliers;
        //        ellipseCoefficients ell_coeff;
        //        scan_data.mAllLineData[i].EllipseFit(ransac.mInliners, ransac.mEll_coeff);
        SPDLOG_INFO("This is line:{}", i);
        scan_data.mAllLineData[i].ransac(scan_data.mAllLineData[i].mLineData.size(), 7);

        for (int j = 0; j < scan_data.mAllLineData[i].mInlierflags.size(); ++j)
        {
            int index = scan_data.mAllLineData[i].mInlierflags[j];
            use_pts.push_back(scan_data.mAllLineData[i].mLineData[index]);
        }
        //    use_pts.clear();
    }
    pcl::io::savePCDFileBinary("D:\\Data\\TunnelMonitor\\jn2w\\jn2w012_0-" + std::to_string(count) + ".pcd", use_pts);
    SPDLOG_INFO("END!");

    return 0;
}
