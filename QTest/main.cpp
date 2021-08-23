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

int main(int argc, char *argv[])
{
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
