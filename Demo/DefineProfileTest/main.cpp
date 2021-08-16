#include "DefineProfileTest.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication      a(argc, argv);
    DefineProfileTest w;
    w.show();
    return a.exec();
}
