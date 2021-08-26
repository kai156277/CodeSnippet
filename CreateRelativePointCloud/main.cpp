#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QString>
#include <QStringList>
#include <QTextStream>

#include <lasreader.hpp>
#include <laswriter.hpp>

struct RingSeamTag
{
    int    index;
    int    startLine;
    int    endLine;
    double startMileage;
    double endMileage;
    double lineMileageInterval;
};

struct PointXYZIT
{
    float    x;
    float    y;
    float    z;
    uint16_t intensity;
    double   gpsTime;
};

struct TunnelCrossSection
{
    int32_t             mIndex;
    QVector<PointXYZIT> mPointData;
};

struct TunnelRing
{
    int32_t                     mRingNum;
    RingSeamTag                 mRingTag;
    QVector<TunnelCrossSection> mCrossSectionData;
};

QVector<RingSeamTag>               readRingSeamTagFile(const QString &file);
static QVector<TunnelCrossSection> readLasFile(const QString &file);

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QStringList raw_las_files        = QFileDialog::getOpenFileNames(nullptr, "las files", "D:/Data/TunnelMonitor/test_seamAuto", {"*.las"});
    QStringList ring_seam_info_files = QFileDialog::getOpenFileNames(nullptr, "ring info files", "D:/Data/TunnelMonitor/test_seamAuto", {"*.ringInfo"});

    bool hasIncomplateRing = false;

    TunnelRing mIncomplateRing;
    for (int file_index = 0; file_index < raw_las_files.size(); ++file_index)
    {
        QString raw_las_file        = raw_las_files[file_index];
        QString ring_seam_info_file = ring_seam_info_files[file_index];

        QFileInfo raw_file_info(raw_las_file);

        QVector<TunnelRing> mAllRingData;

        //读环号文件

        QVector<RingSeamTag>        ring_tags = readRingSeamTagFile(ring_seam_info_file);
        QVector<TunnelCrossSection> mAllLineData;

        LASreadOpener las_read_opener;
        std::string   las_file_name = raw_las_file.toStdString();
        las_read_opener.set_file_name(las_file_name.c_str());

        if (!las_read_opener.active())
        {
            qDebug() << ("ERROR: no input specified");
            return false;
        }

        LASreader *las_reader = las_read_opener.open();
        if (las_reader == 0)
        {
            qDebug() << ("ERROR: could not open lasreader");
            return false;
        }

        // 从 las 文件中读取数据
        {
            TunnelCrossSection current_line;
            PointXYZIT         pclpoint;

            int      line_index  = -1;
            int      point_index = 0;
            LASpoint point;

            point.init(&las_reader->header, las_reader->header.point_data_format, las_reader->header.point_data_record_length);

            mAllLineData.reserve(5000);
            while (las_reader->read_point())
            {
                point = las_reader->point;
                if (point.point_source_ID != line_index)
                {
                    // 进入此判断时，一条线已经读完。
                    if (!current_line.mPointData.empty())
                    {
                        current_line.mIndex = line_index;
                        mAllLineData.push_back(current_line);
                    }
                    if (line_index % 100 == 0)
                    {
                        // TODO: 增加进度
                        qDebug() << QString("line index: %1, point index: %2").arg(line_index).arg(point_index);
                    }
                    line_index = point.point_source_ID;
                    current_line.mPointData.clear();
                }
                pclpoint.x         = point.get_x();
                pclpoint.y         = point.get_y();
                pclpoint.z         = point.get_z();
                pclpoint.intensity = point.get_intensity();
                pclpoint.gpsTime   = point.get_gps_time();
                current_line.mPointData.push_back(pclpoint);
                ++point_index;
            }
            current_line.mIndex = line_index;
            mAllLineData.push_back(current_line);

            las_reader->close();
        }

        {
            //从mAllLineData里面按扫描线和环号信息将其按环保存，并进行相对定位
            if (ring_tags.isEmpty())
            {
                qDebug() << "没有数据";
                continue;
            }

            // 处理第一环, 如果有上一环的数据融合它
            {
                TunnelRing    current_ring;
                const int32_t ring_index = 0;
                current_ring.mCrossSectionData.reserve(5000);
                current_ring.mRingTag = ring_tags[ring_index];
                current_ring.mRingNum = ring_tags[ring_index].index;
                if (hasIncomplateRing)
                {
                    current_ring.mCrossSectionData.swap(mIncomplateRing.mCrossSectionData);
                    // 每个文件里的第一环的数据默认起始里程为0, 需要用上一个环的里程数据更新
                    current_ring.mRingTag.startMileage = mIncomplateRing.mRingTag.startMileage;

                    for (int32_t line_index = ring_tags[ring_index].startLine; line_index < ring_tags[ring_index].endLine; ++line_index)
                    {
                        current_ring.mCrossSectionData.push_back(std::move(mAllLineData[line_index]));
                    }

                    // 重新计算y值
                    const int32_t ring_line_count = current_ring.mCrossSectionData.size();
                    // 校正里程间隔是将上一环剩下的部分和这一环开始的部分合并后才能确认。
                    double corrected_line_mileage_interval = (current_ring.mRingTag.endMileage - current_ring.mRingTag.startMileage) / ring_line_count;
                    for (int32_t ring_line_index = 0; ring_line_index < ring_line_count; ++ring_line_index)
                    {
                        const int32_t line_data_count = current_ring.mCrossSectionData[ring_line_index].mPointData.size();
                        for (int line_point_index = 0; line_point_index < line_data_count; ++line_point_index)
                        {
                            double line_increase_mileage = ring_line_index * corrected_line_mileage_interval;

                            current_ring.mCrossSectionData[ring_line_index].mPointData[line_point_index].y = current_ring.mRingTag.startMileage + line_increase_mileage;
                        }
                    }

                    mAllRingData.push_back(std::move(current_ring));
                }
            }

            for (int32_t ring_index = 1; ring_index < ring_tags.size() - 1; ++ring_index)
            {
                TunnelRing current_ring;
                current_ring.mCrossSectionData.reserve(5000);
                current_ring.mRingTag = ring_tags[ring_index];
                current_ring.mRingNum = ring_tags[ring_index].index;

                for (int32_t line_index = ring_tags[ring_index].startLine; line_index < ring_tags[ring_index].endLine; ++line_index)
                {
                    const int32_t line_data_count = mAllLineData[line_index].mPointData.size();
                    for (int line_point_index = 0; line_point_index < line_data_count; ++line_point_index)
                    {
                        double increase_line_mileage = (line_index - ring_tags[ring_index].startLine) * ring_tags[ring_index].lineMileageInterval;

                        mAllLineData[line_index].mPointData[line_point_index].y = current_ring.mRingTag.startMileage + increase_line_mileage;
                    }
                    current_ring.mCrossSectionData.push_back(std::move(mAllLineData[line_index]));
                }

                mAllRingData.push_back(std::move(current_ring));
            }

            // 处理最后一环
            {
                const int32_t ring_index = ring_tags.size() - 1;
                mIncomplateRing.mCrossSectionData.reserve(5000);
                mIncomplateRing.mRingTag = ring_tags[ring_index];
                mIncomplateRing.mRingNum = ring_tags[ring_index].index;

                for (int32_t line_index = ring_tags[ring_index].startLine; line_index < ring_tags[ring_index].endLine; ++line_index)
                {
                    mIncomplateRing.mCrossSectionData.push_back(std::move(mAllLineData[line_index]));
                }

                hasIncomplateRing = true;
            }
        }

        // 保存相对定位后的 las 点云
        {

            for (int ring_indx = 0; ring_indx < mAllRingData.size(); ++ring_indx)
            {
                qDebug() << "las ring index:" << ring_indx;
                LASwriteOpener las_write_opener;
                QString        las_file = QString("%1/%2_%3.las")
                                       .arg(raw_file_info.absolutePath())
                                       .arg(raw_file_info.baseName())
                                       .arg(mAllRingData[ring_indx].mRingNum);
                las_write_opener.set_file_name(las_file.toUtf8().data());

                LASwriter *las_writer = las_write_opener.open(&las_reader->header);
                if (!las_writer)
                {
                    qDebug() << QString("无法写入 %1 文件").arg(las_file);
                    continue;
                }

                LASpoint las_point;

                las_point.init(&las_reader->header, las_reader->header.point_data_format, las_reader->header.point_data_record_length, 0);

                PointXYZIT tdi_point;
                const int  line_count = mAllRingData[ring_indx].mCrossSectionData.size();
                for (int line_index = 0; line_index < line_count; ++line_index)
                {
                    const int point_count = mAllRingData[ring_indx].mCrossSectionData[line_index].mPointData.size();
                    for (int point_index = 0; point_index < point_count; ++point_index)
                    {
                        tdi_point = mAllRingData[ring_indx].mCrossSectionData[line_index].mPointData[point_index];
                        las_point.set_x(tdi_point.x);
                        las_point.set_y(tdi_point.y);
                        las_point.set_z(tdi_point.z);
                        las_point.set_intensity(tdi_point.intensity);
                        las_point.set_gps_time(tdi_point.gpsTime);
                        las_point.set_point_source_ID(line_index);
                        las_writer->write_point(&las_point);
                        las_writer->update_inventory(&las_point);
                    }
                }
                las_writer->update_header(&las_reader->header, true);
                las_writer->close();
                delete las_writer;
                las_writer = nullptr;
            }
        }
        delete las_reader;
    }

    qDebug() << "END!";
    return 0;
}
QVector<RingSeamTag> readRingSeamTagFile(const QString &file)
{
    QVector<RingSeamTag> ring_seam_tags;
    QFile                ring_file(file);
    QString              line;
    QStringList          lines;
    QStringList          split;
    RingSeamTag          ringtag;
    ring_seam_tags.reserve(30);
    if (ring_file.open(QIODevice::ReadOnly))
    {
        QTextStream stream_text(&ring_file);
        while (!stream_text.atEnd())
        {
            lines.push_back(stream_text.readLine());
        }
        for (int j = 0; j < lines.size(); j++)
        {

            line  = lines.at(j);
            split = line.split(",");
            bool    ok;
            QString startLine           = split.value(0);
            QString endLine             = split.value(1);
            QString startMileage        = split.value(2);
            QString endMileage          = split.value(3);
            QString index               = split.value(4);
            ringtag.index               = index.toInt(&ok);
            ringtag.startLine           = startLine.toInt(&ok);
            ringtag.endLine             = endLine.toInt(&ok);
            ringtag.startMileage        = startMileage.toDouble(&ok);
            ringtag.endMileage          = endMileage.toDouble(&ok);
            ringtag.lineMileageInterval = (ringtag.endMileage - ringtag.startMileage) / (ringtag.endLine - ringtag.startLine);
            ring_seam_tags.push_back(ringtag);
        }
    }
    return ring_seam_tags;

    ring_file.close();
}
