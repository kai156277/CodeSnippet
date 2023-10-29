#include "ChartWidget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ChartWidget  w;
    w.show();
    return a.exec();
    return 0;
}
