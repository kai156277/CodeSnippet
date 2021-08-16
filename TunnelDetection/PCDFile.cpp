#include "PCDFile.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <qstring.h>
#include <spdlog/custom_log.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <windows.h>

#import "C:\Windows\WinSxS\amd64_faro.ls_1d23f5635ba800ab_1.1.703.1_none_3591b17b356ae3ff\iQOpen.dll" no_namespace

PCDFile::PCDFile()
{
}

bool PCDFile::flsToPCD(const QString &fls_file, const QString &pcd_file)
{
    CoInitialize(NULL);
    const wchar_t *licenseCode =
        L"FARO Open Runtime License\n"
        L"Key:W2CEL7NRTCTXXKJT6KZYSPUP2\n"
        L"\n"
        L"The software is the registered property of FARO Scanner "
        L"Production GmbH, Stuttgart, Germany.\n"
        L"All rights reserved.\n"
        L"This software may only be used with written permission of "
        L"FARO Scanner Production GmbH, Stuttgart, Germany.";
    IiQLicensedInterfaceIfPtr liPtr(__uuidof(iQLibIf));
    liPtr->License     = licenseCode;
    IiQLibIfPtr libRef = static_cast<IiQLibIfPtr>(liPtr);
    SPDLOG_INFO("Load fls: {}", fls_file);
    std::wstring flsfile_name = fls_file.toStdWString();
    libRef->load(flsfile_name.data());

    int numRows = libRef->getScanNumRows(0);
    int numCols = libRef->getScanNumCols(0);

    SPDLOG_INFO("fls rows:{}, cols:{}", numRows, numCols);

    QDir      root("D:");
    QFileInfo pcd_file_info(pcd_file);
    if (!pcd_file_info.absoluteDir().exists())
    {
        root.mkpath(pcd_file_info.absolutePath());
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;

    cloud.points.reserve(cloud.width * cloud.width);

    pcl::PointXYZI point;

    double *first_pos_arr      = new double[numRows * 3];
    int *   first_reflect_arr  = new int[numRows];
    double *second_pos_arr     = new double[numRows * 3];
    int *   second_reflect_arr = new int[numRows];
    int32_t half_cols          = numCols / 2;

    for (int col = 0; col < half_cols; ++col)
    //    for (int col = 0; col < 6; ++col)
    {
        if (col % 1000 == 0)
        {
            SPDLOG_INFO("nums: {}", col);
        }
        int res = libRef->getXYZScanPoints(0, 0, col, numRows, first_pos_arr, first_reflect_arr);                 //getXYZScanPoints2
        res     = libRef->getXYZScanPoints(0, 0, col + half_cols, numRows, second_pos_arr, second_reflect_arr);   //getXYZScanPoints2

        for (int nRow = numRows - 1; nRow >= 0; nRow--)
        {

            int row = nRow;

            row *= 3;

            point.x         = first_pos_arr[row + 0];
            point.y         = col * 0.01;
            point.z         = first_pos_arr[row + 2];
            point.intensity = first_reflect_arr[nRow];

            cloud.push_back(point);

            point.x         = second_pos_arr[row + 0];
            point.y         = col * 0.01;
            point.z         = second_pos_arr[row + 2];
            point.intensity = second_reflect_arr[nRow];

            cloud.push_back(point);
        }
    }
    cloud.width    = numRows * 2;
    cloud.height   = numCols / 2;
    cloud.is_dense = true;

    SPDLOG_INFO("cloud width: {}, height:{}", cloud.width, cloud.height);
    //    pcl::io::savePCDFileBinary(pcd_file.toStdString(), cloud);
    pcl::io::savePCDFileASCII(pcd_file.toStdString(), cloud);

    delete[] first_pos_arr;
    delete[] first_reflect_arr;
    first_pos_arr     = NULL;
    first_reflect_arr = NULL;
    delete[] second_pos_arr;
    delete[] second_reflect_arr;
    second_pos_arr     = NULL;
    second_reflect_arr = NULL;
    SPDLOG_INFO("flsToPCD END");
    return true;
}

void PCDFile::splitPCDFile(const QString &pcd_file, const QString &pcd_out_root, int split)
{
    SPDLOG_INFO("param: {}, {}, {}", pcd_file, pcd_out_root, split);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    SPDLOG_INFO("start to read {}", pcd_file);
    pcl::io::loadPCDFile(pcd_file.toStdString(), cloud);
    SPDLOG_INFO("read {} end! ", pcd_file);

    QDir root("D:");
    root.mkpath(pcd_out_root);

    pcl::PointCloud<pcl::PointXYZI> line_cloud;
    line_cloud.resize(cloud.width);
    line_cloud.height = split;
    for (int i = 0; i < cloud.height; ++i)
    {
        for (int j = 0; j < cloud.width; ++j)
        {
            line_cloud[j] = cloud[i * cloud.width + j];
        }
        if (i % split == 0)
        {
            QString save_file = pcd_out_root + QString("/line_%1.pcd").arg(i);
            line_cloud.width  = cloud.width;
            line_cloud.height = split;
            pcl::io::savePCDFileBinary(save_file.toStdString(), line_cloud);
        }
        if (i % 1000 == 0)
        {
            SPDLOG_INFO("line: {}", i);
        }
    }
    SPDLOG_INFO("splitPCDFile END");
}
