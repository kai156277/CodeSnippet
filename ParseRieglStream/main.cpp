#include "ParseRieglStream.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ParseRieglStream w;
    w.show();

    return a.exec();
}
