#include <QCoreApplication>

#include <LASlib/lasdefinitions.hpp>
#include <LASlib/laspoint.hpp>
#include <LASlib/lasreader.hpp>

#include <spdlog/qt_spdlog.h>

#include <QVector>
#include <iostream>
#include <vector>

bool readDataFromLASFile(const QString &las_file);

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    LASheader lasheader;
    lasheader.x_scale_factor           = 0.1;
    lasheader.y_scale_factor           = 0.01;
    lasheader.z_scale_factor           = 0.001;
    lasheader.x_offset                 = 1000.0;
    lasheader.y_offset                 = 2000.0;
    lasheader.z_offset                 = 0.0;
    lasheader.point_data_format        = 1;
    lasheader.point_data_record_length = 28;

    LASpoint laspoint;
    laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

    std::vector<LASpoint> points;

    for (int i = 0; i < 10; ++i)
    {
        laspoint.set_X(i);
        laspoint.set_Y(i);
        laspoint.set_Z(i);
        laspoint.set_intensity((U16) i);
        laspoint.set_gps_time(0.0006 * i);
        points.push_back(laspoint);
    }

    std::cout << "size: " << points.size() << std::endl;

    std::cout << "double:" << sizeof(double) << std::endl;
    std::cout << "uint16_t:" << sizeof(uint16_t) << std::endl;

    readDataFromLASFile("D:\\Data\\TunnelMonitor\\jn2w\\jn2w012.las");
    return 0;
}

struct PointXYZIT
{
    float  x;
    float  y;
    float  z;
    float  intensity;
    double gpsTime;
};

using TunnelCrossSection = QVector<PointXYZIT>;

bool readDataFromLASFile(const QString &las_file)
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
    while (las_reader->read_point())
    {
        point = las_reader->point;
        if (point_index == 546742)
        {
            SPDLOG_INFO("point source id: {}", point.point_source_ID);
            break;
        }
        ++point_index;
    }
    SPDLOG_INFO("read {} end! ", las_file);

    las_reader->close();
    delete las_reader;
    return true;
}
