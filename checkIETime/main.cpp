#include "Widget.h"

#include <QApplication>
#include <QDateTime>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QString>
#include <QTextStream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QString      ie_file_str = QFileDialog::getOpenFileName(nullptr, "ie file", "D:\\Data\\0806-7125\\hyrdrins");
    if (ie_file_str.isEmpty())
    {
        qDebug() << "empty file str";
        return -1;
    }

    QFile ie_file(ie_file_str);
    if (!ie_file.open(QIODevice::ReadOnly))
    {
        qDebug() << "not open file";
        return -1;
    }

    QTextStream read_stream(&ie_file);
    int         index = 0;
    for (; index < 23; ++index)
    {
        read_stream.readLine();
    }

    QString read_line_str;

    QDateTime current_time;
    QDateTime last_time(QDate(2000, 1, 1), QTime(0, 0, 0));
    while (!read_stream.atEnd())
    {
        read_line_str = read_stream.readLine();
        ++index;
        QString time_str = read_line_str.mid(0, 24);
        current_time     = QDateTime::fromString(time_str, "MM/dd/yyyy  hh mm ss.zzz");
        if (last_time >= current_time)
        {
            qDebug() << "read line: " << index << " error!";
        }
        last_time = current_time;

        if (index % 10000 == 0)
        {
            qDebug() << "read line: " << index;
        }
    }

    qDebug() << "END!";
    return 0;
}
