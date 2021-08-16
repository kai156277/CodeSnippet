#include "stdafx.h"

#include "TunnelPointCloud.h"

#include <windows.h>

#import "C:/Windows/WinSxS/amd64_faro.ls_1d23f5635ba800ab_1.1.703.1_none_3591b17b356ae3ff/iQOpen.dll" no_namespace

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <LASlib/lasreader.hpp>
#include <LASlib/laswriter.hpp>

TunnelPointCloud::TunnelPointCloud()
{
}

bool TunnelPointCloud::saveLineFile(int begin_line, int end_line, QString save_file)
{
    SPDLOG_INFO("param: {}, {}, {}", begin_line, end_line, save_file);

    if (begin_line > end_line)
    {
        SPDLOG_INFO("ERROR:input error,begin_line>end_line");
        return false;
    }
    LASwriteOpener laswritelineopener;
    char *         las_out_root_char;
    QByteArray     b  = save_file.toLatin1();
    las_out_root_char = b.data();
    laswritelineopener.set_file_name(las_out_root_char);
    if (!laswritelineopener.active())
    {
        SPDLOG_ERROR("ERROR:no output specified");
        return false;
    }
    // set header
    LASheader lasheaderline;
    lasheaderline.clean();
    lasheaderline.version_major = 1;
    lasheaderline.version_minor = 2;
    // lasheaderline.global_encoding          = 1;
    lasheaderline.x_scale_factor           = 0.00001;
    lasheaderline.y_scale_factor           = 0.00001;
    lasheaderline.z_scale_factor           = 0.00001;
    lasheaderline.x_offset                 = 0.0;
    lasheaderline.y_offset                 = 0.0;
    lasheaderline.z_offset                 = 0.0;
    lasheaderline.point_data_format        = 1;
    lasheaderline.point_data_record_length = 28;

    LASpoint laslinepoint;

    laslinepoint.init(&lasheaderline, lasheaderline.point_data_format, lasheaderline.point_data_record_length, 0);
    LASwriter *laslinewriter = laswritelineopener.open(&lasheaderline);
    if (laslinewriter == 0)
    {
        SPDLOG_INFO("ERROR: could not open laslinewriter");
        return false;
    }

    for (int index = begin_line; index < end_line; ++index)
    {
        int point_count = mAllLineData[index].mLineData.size();   //第index条线中点的数量
        for (int i = 0; i < point_count; ++i)                     //循环每个点
        {

            laslinepoint.set_x(mAllLineData[index].mLineData[i].x);
            laslinepoint.set_y(mAllLineData[index].mLineData[i].y);
            laslinepoint.set_z(mAllLineData[index].mLineData[i].z);

            laslinepoint.set_intensity(mAllLineData[index].mLineData[i].intensity);
            laslinepoint.set_point_source_ID(mAllLineData[index].mIndex);

            laslinewriter->write_point(&laslinepoint);
            laslinewriter->update_inventory(&laslinepoint);
        }
    }

    laslinewriter->update_header(&lasheaderline);

    laslinewriter->close();
    delete laslinewriter;
    laslinewriter = NULL;
    return true;
}

bool TunnelPointCloud::flsToLAS(const QString &fls_file, const QString &las_file)
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
    libRef->load(flsfile_name.c_str());

    int numRows = libRef->getScanNumRows(0);
    int numCols = libRef->getScanNumCols(0);

    SPDLOG_INFO("fls rows:{}, cols:{}", numRows, numCols);

    double *  first_pos_arr      = new double[numRows * 3];
    int *     first_reflect_arr  = new int[numRows];
    double *  second_pos_arr     = new double[numRows * 3];
    int *     second_reflect_arr = new int[numRows];
    int32_t   half_cols          = numCols / 2;
    QDir      root("D:");
    QFileInfo las_file_info(las_file);
    if (!las_file_info.absoluteDir().exists())
    {
        root.mkpath(las_file_info.absolutePath());
    }
    char *     las_file_char;
    QByteArray ba = las_file.toLatin1();
    las_file_char = ba.data();
    LASwriteOpener laswriteopener;
    laswriteopener.set_file_name(las_file_char);
    if (!laswriteopener.active())
    {
        SPDLOG_INFO("ERROR:no output specified");
        return false;
    }

    LASheader lasheader;
    lasheader.version_major = 1;
    lasheader.version_minor = 2;
    // lasheader.global_encoding          = 1;
    lasheader.x_scale_factor           = 0.00001;
    lasheader.y_scale_factor           = 0.00001;
    lasheader.z_scale_factor           = 0.00001;
    lasheader.x_offset                 = 0.0;
    lasheader.y_offset                 = 0.0;
    lasheader.z_offset                 = 0.0;
    lasheader.point_data_format        = 1;
    lasheader.point_data_record_length = 28;

    LASpoint laspoint;
    laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

    LASwriter *laswriter = laswriteopener.open(&lasheader);
    if (laswriter == 0)
    {
        SPDLOG_INFO("ERROR: could not open laswriter");
    }

    for (int col = 0; col < half_cols; ++col)
    {
        if (col % 1000 == 0)
        {
            SPDLOG_INFO("nums: {}", col);
        }
        int res          = libRef->getXYZScanPoints(0, 0, col, numRows, first_pos_arr, first_reflect_arr);                 //getXYZScanPoints2
        res              = libRef->getXYZScanPoints(0, 0, col + half_cols, numRows, second_pos_arr, second_reflect_arr);   //getXYZScanPoints2
        uint64_t pptime  = 0;
        uint64_t pptime1 = 0;
        for (int nRow = numRows - 1; nRow >= 0; nRow--)
        {

            int row = nRow;

            row *= 3;
            double         x, y, z;
            unsigned short intensity;

            libRef->getAutomationTimeOfScanPoint(0, nRow, col, &pptime);
            x         = (first_pos_arr[row + 0] - lasheader.x_offset) / lasheader.x_scale_factor;
            y         = (col * 0.01 - lasheader.y_offset) / lasheader.y_scale_factor;
            z         = (first_pos_arr[row + 2] - lasheader.z_offset) / lasheader.z_scale_factor;
            intensity = first_reflect_arr[nRow];

            laspoint.set_X(x);
            laspoint.set_Y(y);
            laspoint.set_Z(z);
            laspoint.set_intensity(intensity);
            laspoint.set_gps_time(pptime);
            laspoint.set_point_source_ID(col);

            laswriter->write_point(&laspoint);
            laswriter->update_inventory(&laspoint);

            libRef->getAutomationTimeOfScanPoint(0, nRow, col + half_cols, &pptime1);
            x         = (second_pos_arr[row + 0] - lasheader.x_offset) / lasheader.x_scale_factor;
            y         = (col * 0.01 - lasheader.y_offset) / lasheader.y_scale_factor;
            z         = (second_pos_arr[row + 2] - lasheader.z_offset) / lasheader.z_scale_factor;
            intensity = second_reflect_arr[nRow];

            laspoint.set_X(x);
            laspoint.set_Y(y);
            laspoint.set_Z(z);
            laspoint.set_intensity(intensity);
            laspoint.set_gps_time(pptime1);
            laspoint.set_point_source_ID(col);

            laswriter->write_point(&laspoint);
            laswriter->update_inventory(&laspoint);
        }
    }
    laswriter->update_header(&lasheader, TRUE);

    delete[] first_pos_arr;
    delete[] first_reflect_arr;
    first_pos_arr     = NULL;
    first_reflect_arr = NULL;
    delete[] second_pos_arr;
    delete[] second_reflect_arr;
    second_pos_arr     = NULL;
    second_reflect_arr = NULL;

    laswriter->close();
    delete laswriter;
    laswriter = NULL;

    SPDLOG_INFO("flsToLAS END");
    return true;
}

bool TunnelPointCloud::readDataFromLASFile(const QString &las_file)
{
    SPDLOG_INFO("param: {} ", las_file);

    LASreadOpener las_read_opener;
    std::string   las_file_name = las_file.toStdString();
    SPDLOG_INFO("qstring => string :{}", las_file_name);
    las_read_opener.set_file_name(las_file_name.c_str());

    if (!las_read_opener.active())
    {
        SPDLOG_ERROR("ERROR: no input specified");
        return false;
    }

    LASreader *las_reader = las_read_opener.open();
    if (las_reader == 0)
    {
        SPDLOG_ERROR("ERROR: could not open lasreader");
        return false;
    }
    SPDLOG_INFO("start to read {}", las_file);

    //    QDir root("D:");
    //    root.mkpath(las_out_root);

    TunnelCrossSection current_line;
    PointXYZIT         pclpoint;

    int      line_index  = -1;
    int      point_index = 0;
    LASpoint point;

    point.init(&las_reader->header, las_reader->header.point_data_format, las_reader->header.point_data_record_length);

    //    mAllLineData.reserve(current_line.mlineData.size());
    //TODO:mAllLineData.reserve(从头文件获得总点数/第一条线的大小)
    mAllLineData.reserve(5000);
    while (las_reader->read_point())
    {
        point = las_reader->point;
        if (point.point_source_ID != line_index)
        {
            // 进入此判断时，一条线已经读完。

            if (!current_line.mLineData.empty())
            {
                current_line.mIndex = line_index;
                mAllLineData.push_back(current_line);
            }

            if (line_index % 100 == 0)
            {
                SPDLOG_INFO("line index: {}, point index: {}", line_index, point_index);
            }

            line_index = point.point_source_ID;
            current_line.mLineData.clear();
        }
        pclpoint.x         = point.get_x();
        pclpoint.y         = point.get_y();
        pclpoint.z         = point.get_z();
        pclpoint.intensity = point.get_intensity();
        pclpoint.gpsTime   = point.get_gps_time();
        current_line.mLineData.push_back(pclpoint);
        ++point_index;
    }
    current_line.mIndex = line_index;
    mAllLineData.push_back(current_line);
    SPDLOG_INFO("read {} end! ", las_file);

    //    int begin_line = 0;
    //    int end_line   = ringnum(begin_line);
    //    //    QString base_file_name = "D:/YM";
    //    current_line.index = 0;

    //    while (current_line.index < mAllLineData.size())
    //    {
    //        SPDLOG_INFO("file.data.size:{}", mAllLineData.size());
    //        SPDLOG_INFO("current_ling.index:{}", current_line.index);
    //        // TODO: 循环不变式是什么
    //        QString file_name = las_out_root + "/" + QString::number(end_line) + ".las";

    //        saveLineFile(begin_line, end_line, file_name);

    //        begin_line = end_line;
    //        end_line   = ringnum(begin_line);

    //        current_line.index++;
    //    }
    las_reader->close();
    delete las_reader;
    return true;
}

void TunnelPointCloud::calcuateAxialDeformation()
{
    // FIXME: 计算轴线变形
}

void TunnelPointCloud::calcuateCircumferentialDislocation()
{
    //FIXME: 计算环向误差
}

//void ScanAllLine::ellipseAllLine(const QString &pcd_file, QVector<QVector<ellipseCenter>> &allline_center_points, const QString &output_root)
//{
//}

//bool ScanAllLine::readDataFromLASFile(const QString &las_file)
//{
//    return true;
//}
