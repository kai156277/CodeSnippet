#include "RieglDataImporter.h"
#include <cmath>
#include <QFile>

RieglVZ2000iDataImporter::RieglVZ2000iDataImporter() : pointcloud(true)
{  

}

RieglVZ2000iDataImporter::~RieglVZ2000iDataImporter(){    
       mParseFile.close();
       mExposureFile.close();
       mTriggerFile.close();
}

void RieglVZ2000iDataImporter::on_hk_gps_hr(const scanlib::hk_gps_hr<iterator_type> &arg)
{
    pointcloud::on_hk_gps_hr(arg);
    QString sync_status = "not synchronized";
    if(arg.SYNC_STATUS == 1)
        sync_status = "lost synchronized";
    else if(arg.SYNC_STATUS == 3)
        sync_status = "correctly synchronized";
}

void RieglVZ2000iDataImporter::on_unsolicited_message(const scanlib::unsolicited_message<scanlib::basic_packets::iterator_type> &arg)
{
    basic_packets::on_unsolicited_message(arg);
    for(int i = 0; i < 128; ++i)
    {
        std::cout << arg.message[i];
    }
    std::cout << std::endl;
}

void RieglVZ2000iDataImporter::on_pps_synchronized(){
    qDebug() << "on_pps_synchronized" ;
}

void RieglVZ2000iDataImporter::on_pps_sync_lost(){
    qDebug() << "on_pps_sync_lost" ;
}

void RieglVZ2000iDataImporter::on_counter_sync_2angles_hr(const scanlib::counter_sync_2angles_hr<basic_packets::iterator_type> &arg)
{
    pointcloud::on_counter_sync_2angles_hr(arg);
    if ((arg.source % 2) == 1)
    {
        mExposureFile.write(QString(QString::number(arg.count) + "," +
                                    QString::number(time_event,'f',8) + "," +
                                    QString::number(frame_angle,'f',8) + "\r\n").toLocal8Bit());
    }
    else if ((arg.source % 2) == 0)
    {
        mTriggerFile.write(QString(QString::number(arg.count) + "," +
                                   QString::number(time_event,'f',8) + "," +
                                   QString::number(frame_angle,'f',8) + "\r\n").toLocal8Bit());
    }
}

void RieglVZ2000iDataImporter::setSaveDirPath(QString dirPath)
{
    QString time = QTime::currentTime().toString("hhmmss");

    mParseFile.setFileName(dirPath + "/" + time + "rxp.txt");
    if(!mParseFile.open(QIODevice::ReadWrite|QIODevice::Append))
        qDebug() << "mParseFile open failed!";
    mExposureFile.setFileName(dirPath + "/" + time + "exposureTime.txt");
    if(!mExposureFile.open(QIODevice::ReadWrite|QIODevice::Append))
        qDebug() << "mExposureFile open failed!";
    mTriggerFile.setFileName(dirPath + "/" + time + "triggerTime.txt");
    if(!mTriggerFile.open(QIODevice::ReadWrite|QIODevice::Append))
        qDebug() << "mTriggerFile open failed!";
}

void RieglVZ2000iDataImporter::on_echo_transformed(echo_type echo)
{
    echo;
    scanlib::target& _t(targets[target_count -1]);
    mParseFile.write((
                         QString::number(_t.time,'f',8)+ "," +
                         QString::number(_t.echo_range,'f',8) + "," +
                         QString::number(frame_angle,'f',8) + "," +
                         QString::number(line_angle,'f',8) + "," +
                         QString::number(_t.vertex[0],'f',8) + "," +
                     QString::number(_t.vertex[1],'f',8) + "," +
            QString::number(_t.vertex[2],'f',8) + "\r\n").toLatin1());
}
