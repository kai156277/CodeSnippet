#include "Dialog.h"
#include "Widget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget       w;
    w.show();
    //    Dialog d;
    //    d.show();
    return a.exec();
}
