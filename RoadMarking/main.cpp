#define NOMINMAX
#include <QCoreApplication>

#include <QDebug>
#include <lasreader.hpp>
#include <laswriter.hpp>
#include <spdlog/spdlog.h>
#include <vector>
#include <map>
#include <limits>

#include <QTime>

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>


struct TPoint
{
    double x,y,z;
    int8_t scan_angle_rank;
    uint16_t intensity;
    uint64_t index;
};

TPoint operator-(TPoint a, TPoint b)
{
    TPoint tmp = {0};
    tmp.x = a.x -b.x;
    tmp.y = a.y -b.y;
    tmp.z = a.z -b.z;
    return tmp;
}

struct ScanLine
{
    int64_t index = 0;
    std::vector<TPoint> line;
};

struct PointCloud
{
    std::vector<ScanLine> point_cloud;
};

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

//    for(int intensity = 5000; intensity <= 10000; intensity+=2500)
    {
//        for(int angle = 20; angle <= 40; angle+=5)
        {
            PointCloud pc;
            LASheader las_header;
        //        SPDLOG_INFO("读取 las 文件: {}", file);
            LASreadOpener  las_read_opener;
            LASwriteOpener las_write_opener;
            QString filetime = QTime::currentTime().toString("HH-mm-ss");
            std::string    las_file_name        = "C:\\Users\\zhaokai\\Desktop\\YangMi\\1_152957_00001_0.las";
            std::string las_output_file_str = "C:\\Users\\zhaokai\\Desktop\\YangMi\\1_152957_00001_0-" + filetime.toStdString() + QString(".las").toStdString();
            las_write_opener.set_file_name(las_output_file_str.c_str());
            las_read_opener.set_file_name(las_file_name.c_str());

            std::map<uint16_t, int64_t> hist;
            for(uint16_t i = 0; i < std::numeric_limits<uint16_t>::max(); ++i)
                hist.insert({i,0});


            LASreader *las_reader = las_read_opener.open();

            las_header            = las_reader->header;
            LASwriter *las_writer = las_write_opener.open(&las_header);

            int      line_index  = -1;
            int      point_index = 0;
            LASpoint point;

            point.init(&las_header, las_header.point_data_format, las_header.point_data_record_length);

            ScanLine sl;
            TPoint p;
            while (las_reader->read_point())
            {
                point = las_reader->point;
                if (point.point_source_ID != line_index)
                {
                    if (!sl.line.empty())
                    {
                        sl.index = line_index;
                        pc.point_cloud.push_back(sl);
                    }
                    // 进入此判断时，一条线已经读完。
                    line_index  = point.point_source_ID;
                    point_index = 0;
                    sl.line.clear();;
                }
                point_index++;
                p.x = point.get_x();
                p.y = point.get_y();
                p.z = point.get_z();
//                p.scan_angle_rank = point.get_scan_angle_rank();
                p.intensity = point.get_intensity();
//                p.index = point.point_source_ID;
                sl.line.push_back(p);

            }
            for(int i = 0; i < pc.point_cloud.size(); ++i)
            {
                if(pc.point_cloud[i].line.size() < 3000)
                    continue;
                //-----------------------------拟合直线-----------------------------
                double ave_z = pc.point_cloud[i].line[0].z;
                int ave_zi = 1;
                std::vector<TPoint> line_sum;
                for(int j = 0; j < pc.point_cloud[i].line.size() -1; j++)
//                for(int j = pc.point_cloud[i].line.size() -1; j >0 ; j--)
                {
                    auto p1 = pc.point_cloud[i].line[j];
                    auto p2 = pc.point_cloud[i].line[j+1];
                    double dz = abs(p1.z - ave_z);
                    auto dp = p2 -p1;
                    double dis = sqrt(dp.x * dp.x + dp.y * dp.y);
                    auto max_dp = pc.point_cloud[i].line[0] - p1;
                    double max_dis = sqrt(max_dp.x * max_dp.x + max_dp.y * max_dp.y + max_dp.z * max_dp.z); //道路最大宽度
                    double slop = atan2(dp.z, dis)* 180.0 / M_PI;

                    if((abs(slop) < 10.0 || (abs(slop) > 170.0)) && (abs(dz) < 0.20 && dis < 1) && max_dis < 20)
                    {
                        if(!line_sum.empty())
                        {
                            auto last_dp = p1 - line_sum[line_sum.size()-1];
                            double last_dis = sqrt(last_dp.x * last_dp.x + last_dp.y * last_dp.y);
                            if(abs(last_dp.z) > 0.1 && last_dis < 10)
                                continue; // 这是个台阶
                        }

                        double sum_z = ave_z * ave_zi + p1.z;
                        ave_zi++;
                        ave_z = sum_z / ave_zi;
                        line_sum.push_back(p1);
                        point.set_x(p1.x);
                        point.set_y(p1.y);
                        point.set_z(p1.z);
                        point.set_intensity(p1.intensity);
                        las_writer->write_point(&point);
                        las_writer->update_inventory(&point);
                    }

                }
//                TPoint last_true_point = line_sum[0];
//                    if(abs(last_true_point.z - line_sum[j].z) > 0.15)
//                    {
//                        auto dp = last_true_point - line_sum[j];
//                        if(sqrt(dp.x * dp.x + dp.y * dp.y) > 1.0)
//                            last_true_point = line_sum[j];// 跟新最后点
//                    }
//                    else{
//                        last_true_point = line_sum[j];
//                    }
//
//                    point.set_x(line_sum[j].x);
//                    point.set_y(line_sum[j].y);
//                    point.set_z(line_sum[j].z);
//                    point.set_intensity(line_sum[j].intensity);
//                    las_writer->write_point(&point);
//                    las_writer->update_inventory(&point);
                qDebug() << "ave_zi: " << ave_z ;//<< "sigma: " << sigma << endl;
            }


//            for(int i = 0; i < pv.size(); ++i)
//            {
//                point.set_x(pv[i].x);
//                point.set_y(pv[i].y);
//                point.set_z(pv[i].z);
//                point.set_scan_angle_rank(pv[i].scan_angle_rank);
//                if(abs(pv[i].intensity-mach_intensity) < max_sigma)
//                    point.set_intensity(pv[i].intensity - intensity * (1.0 - (abs(pv[i].scan_angle_rank) / (double)angle)));
//                else
//                    point.set_intensity(mach_intensity+max_sigma);
//                point.set_point_source_ID(pv[i].index);
//                las_writer->write_point(&point);
//                las_writer->update_inventory(&point);
//            }
//            for(int i = 0; i < pvi.size(); ++i)
//            {
//                point.set_x(pvi[i].x);
//                point.set_y(pvi[i].y);
//                point.set_z(pvi[i].z);
//                point.set_scan_angle_rank(pvi[i].scan_angle_rank);
//                point.set_intensity(pvi[i].intensity);
//                point.set_point_source_ID(pvi[i].index);
//                las_writer->write_point(&point);
//                las_writer->update_inventory(&point);
//            }
//
            las_writer->update_header(&las_header, true);
            las_writer->close();
            las_reader->close();
            delete las_reader;
            delete las_writer;
        }
    }
    qDebug() << "END! ";
//    return a.exec();
    return 0;
}
