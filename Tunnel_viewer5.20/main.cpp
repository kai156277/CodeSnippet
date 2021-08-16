#include "mainwindow.h"

#include <QApplication>

#pragma comment(lib, "User32.lib")   // __imp_MessageBoxA
#pragma comment(lib, "gdi32.lib")    // __imp_GetStockObject
#pragma execution_character_set("utf-8")
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow   w;
    w.show();

    return a.exec();
}
