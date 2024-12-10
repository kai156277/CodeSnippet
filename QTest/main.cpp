#include "PrintFormat.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QProgressBar>
#include <QProgressDialog>
#include <QString>
#include <QTextStream>
#include <QTime>
#include <QVector>
#include <fmt/format.h>
#include <map>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

//#pragma execution_character_set("utf-8")

struct A
{
    double a;
    double b;
};

template <typename T>
bool search_map(const std::map<T, A> &map, T v)
{
    auto search = map.find(v);
    if (search != map.end())
    {
        qDebug() << QString::fromUtf8(u8"ABCDEFC\u03BC");
        qDebug() << "a:" << search->second.a;
        qDebug() << "b:" << search->second.b;
        return true;
    }
    return false;
}

#include <iostream>
#include <Eigen/Eigen>
#include "unb3m.h"

using namespace Eigen;

int main(int argc, char *argv[])
{
    unb3m unb3;
    double map = 0.0;
    unb3.trop_unb3m(100.0, 100.0, 1, 10, map);
    MatrixXd AVG(5,6);
    AVG=MatrixXd::Zero(5,6);
    AVG<<15.0,  1013.25,  299.65, 75.00,  6.30,  2.77,
               30.0,  1017.25,  294.15, 80.00,  6.05,  3.15,
               45.0,  1015.75,  283.15, 76.00,  5.58,  2.57,
               60.0,  1011.75,  272.15, 77.50,  5.39,  1.81,
               75.0,  1013.00,  263.65, 82.50,  4.53, 1.55;

    double M = 0.5;
    int P1 = 2, P2 = 3;
    double PAVG = M * ( AVG(P2-1,1) - AVG(P1-1,1) ) + AVG(P1-1,1);
    double t = AVG(0,0);
    std::cout << AVG << std::endl;
    std::cout << PAVG << std::endl;

    return 0;
    QApplication    a(argc, argv);
    QProgressDialog progress("load image", "cancel", 0, 20, nullptr);
    progress.setMinimumDuration(0);
    progress.setWindowModality(Qt::WindowModal);
    QProgressBar *bar = new QProgressBar;
    bar->setMinimum(0);
    bar->setMaximum(0);
    progress.setBar(bar);
    progress.setValue(20);
    for (int i = 0; i < 20; ++i)
    {
        if (progress.wasCanceled())
            break;

        qDebug() << i;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    a.exec();
}
