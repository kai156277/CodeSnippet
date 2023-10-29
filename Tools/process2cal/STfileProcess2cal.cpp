//
// Created by zhaokai on 2023/6/16.
//
#include <QFileDialog>
#include <QString>
#include <QApplication>
#include <QStringList>
#include <QTextStream>
#include <QFile>
#include <QDebug>

#include <iostream>


constexpr double PI         = (3.14159265358979323846);   // 圆周率
constexpr double RAD_TO_DEG = (180.0 / PI);               // 弧度转度
constexpr double DEG_TO_RAD = (PI / 180.0);               // 度转弧度

using namespace std;

bool process2cal(const QString& process_file);

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    QStringList process_file_names = QFileDialog::getOpenFileNames(nullptr, "open process st file");
    if(process_file_names.isEmpty())
    {
        qDebug() << "not open file" ;
        return 0;
    }

    for(const auto & process_file_name : process_file_names)
    {
        process2cal(process_file_name);
    }

    cout << "Hello CMake." << endl;
    return 0;
}
bool process2cal(const QString& process_file)
{

    QFile process_cal_file(process_file);
    QFileInfo pcal_file_info(process_file);
    QString new_cal_file = pcal_file_info.absolutePath() + "/cal_" + pcal_file_info.fileName();
    QFile save_cal_file(new_cal_file);

    if(!process_cal_file.open(QIODevice::ReadOnly))
    {
        qDebug() << "can`t open VSursProcess cal file: " << process_file ;
        return false;
    }
    if(!save_cal_file.open(QIODevice::WriteOnly))
    {
        qDebug() << "can`t open new cal file: " << new_cal_file;
        return false;
    }
    QTextStream save_cal_stream(&save_cal_file);

    QString line;
    QStringList items;
    while (!process_cal_file.atEnd())
    {
        line = process_cal_file.readLine().simplified();
        items = line.split(',', Qt::SkipEmptyParts);
        if(items.size() != 4)
            continue;

        QStringList cal_items;
        cal_items << items[1] << items[2] << items[3] << items[0];
        save_cal_stream << cal_items.join(',') << Qt::endl;
    }
    qDebug() << "finish write: " << new_cal_file;
    return true;
}
