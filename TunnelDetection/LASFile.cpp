#include "LASFile.h"
#include <LASlib/lasreader.hpp>
#include <LASlib/laswriter.hpp>
#include <QDir>
#include <QFile>
#include <QFileInfo>

#import "C:\Windows\WinSxS\amd64_faro.ls_1d23f5635ba800ab_1.1.703.1_none_3591b17b356ae3ff\iQOpen.dll" no_namespace
#include "LASALL.h"
#include "TunnelCrossSection.h"
#include <QVector>
#include <qstring.h>
#include <spdlog/custom_log.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include <windows.h>

LASFile::LASFile()
{
}

bool LASFile::flsToLAS(const QString &fls_file, const QString &las_file)
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

    double *  first_pos_arr      = new double[numRows * 3];
    int *     first_reflect_arr  = new int[numRows];
    double *  second_pos_arr     = new double[numRows * 3];
    int *     second_reflect_arr = new int[numRows];
    uint64_t *first_pulsetime    = new uint64_t[numCols * numRows + 1];
    uint64_t *second_pulsetime   = new uint64_t[numCols * numRows + 1];
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
    }

    LASheader lasheader;
    lasheader.version_major            = 1;
    lasheader.version_minor            = 2;
    lasheader.global_encoding          = 1;
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

            x                    = (first_pos_arr[row + 0] - lasheader.x_offset) / lasheader.x_scale_factor;
            y                    = (col * 0.01 - lasheader.y_offset) / lasheader.y_scale_factor;
            z                    = (first_pos_arr[row + 2] - lasheader.z_offset) / lasheader.z_scale_factor;
            intensity            = first_reflect_arr[nRow];
            first_pulsetime[row] = pptime;

            laspoint.set_X(x);
            laspoint.set_Y(y);
            laspoint.set_Z(z);
            laspoint.set_intensity(intensity);
            laspoint.set_gps_time(first_pulsetime[row]);
            laspoint.set_point_source_ID(col);

            laswriter->write_point(&laspoint);
            laswriter->update_inventory(&laspoint);

            x                     = (second_pos_arr[row + 0] - lasheader.x_offset) / lasheader.x_scale_factor;
            y                     = (col * 0.01 - lasheader.y_offset) / lasheader.y_scale_factor;
            z                     = (second_pos_arr[row + 2] - lasheader.z_offset) / lasheader.z_scale_factor;
            intensity             = second_reflect_arr[nRow];
            second_pulsetime[row] = pptime1;

            laspoint.set_X(x);
            laspoint.set_Y(y);
            laspoint.set_Z(z);
            laspoint.set_intensity(intensity);
            laspoint.set_gps_time(second_pulsetime[row]);
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
    delete[] first_pulsetime;
    delete[] second_pulsetime;
    first_pulsetime  = NULL;
    second_pulsetime = NULL;
    laswriter->close();
    delete laswriter;
    laswriter = NULL;

    SPDLOG_INFO("flsToLAS END");
    return true;
}

int ringnum(int begin_num);
int ringnum(int begin_num)
{

    int end_num = begin_num + 1;
    return end_num;
}

void LASFile::splitLASFile(const QString &las_file, const QString &las_out_root, int split)
{
    SPDLOG_INFO("param: {}, {}, {}", las_file, las_out_root, split);

    LASreadOpener lasreadopener;
    if (!lasreadopener.active())
    {
        SPDLOG_INFO("ERROR: no input specified");
    }
    char *     las_file_char;
    QByteArray ba = las_file.toLatin1();
    las_file_char = ba.data();
    lasreadopener.set_file_name(las_file_char);

    LASreader *lasreader = lasreadopener.open();
    if (lasreader == 0)
    {
        SPDLOG_INFO("ERROR: could not open lasreader");
    }
    SPDLOG_INFO("start to read {}", las_file);

    QDir root("D:");
    root.mkpath(las_out_root);

    LASALL   file;
    scanline current_line;
    current_line.index = 0;

    int      line_index  = 0;
    int      point_index = 0;
    LASpoint point;
    point.init(&lasreader->header, lasreader->header.point_data_format, lasreader->header.point_data_record_length);
    file.data.reserve(current_line.lineData.size());
    while (lasreader->read_point())
    {
        point = lasreader->point;
        if (point_index % 1000 == 0)
        {
            SPDLOG_INFO("line index: {}, point index: {}", line_index);
        }

        if (point.point_source_ID != line_index)
        {
            // TODO:
            // 进入此判断时，一条线已经读完。
            file.data.push_back(current_line);
            current_line.index = line_index;
            current_line.lineData.clear();
            ++line_index;
        }
        current_line.lineData.push_back(point);
        ++point_index;
    }
    file.data.push_back(current_line);
    SPDLOG_INFO("read {} end! ", las_file);

    int begin_line = 0;
    int end_line   = ringnum(begin_line);
    //    QString base_file_name = "D:/YM";
    current_line.index = 0;

    while (current_line.index < file.data.size())
    {
        SPDLOG_INFO("file.data.size:{}", file.data.size());
        SPDLOG_INFO("current_ling.index:{}", current_line.index);
        // TODO: 循环不变式是什么
        QString file_name = las_out_root + "/" + QString::number(end_line) + ".las";

        file.saveLineFile(begin_line, end_line, file_name);

        begin_line = end_line;
        end_line   = ringnum(begin_line);

        current_line.index++;
    }
    lasreader->close();
    delete lasreader;
    SPDLOG_INFO("splitlasFile END");
}
