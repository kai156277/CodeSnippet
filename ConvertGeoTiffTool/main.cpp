#include "ConvertGeoTiffWidget.h"
#include <QtGui>
#include <QtCore>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QCoreApplication::setApplicationName("ConvertGeoTiffTool");
    QCoreApplication::setOrganizationName("XiuShan");
    QCoreApplication::setOrganizationDomain("supersurs.com");
    if((argc == 2) && QString(argv[1]) == QString("--uninstall"))
    {
        QSettings config;
        config.clear();
        return 0;
    }
    QTextCodec *codec = QTextCodec::codecForName("UTF-8");
    QTextCodec::setCodecForCStrings(codec);
    QTranslator glp_cn;
    glp_cn.load("ConvertGeoTiffTool_cn.qm", QApplication::applicationDirPath());
    a.installTranslator(&glp_cn);

    QTranslator zh_cn;
    zh_cn.load("qt_zh_cn.qm", QApplication::applicationDirPath());
    a.installTranslator(&zh_cn);
    ConvertGeoTiffWidget w;
    w.show();

    return a.exec();
}
