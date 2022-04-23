#include "Widget.h"

#include <QApplication>

#include <WiAcrLib/common/log.h>

int main(int argc, char *argv[])
{
    system("chcp 65001");
    QApplication a(argc, argv);
    spdlog::set_level(spdlog::level::trace);
    Widget w;
    w.show();
    return a.exec();
}
