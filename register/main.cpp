#include "RegisterWidget.h"

#include <QApplication>
#include <QCryptographicHash>
#include <QDebug>
#include <QMetaEnum>
#include <QSettings>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QCoreApplication::setOrganizationDomain("luoyu.com");
    QCoreApplication::setOrganizationName("luoyu");
    QApplication::setApplicationName("register");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings settings;

    settings.beginGroup("xxx");
    if (!settings.contains("name"))
    {
        settings.setValue("name", "xxx");
    }
    if (!settings.contains("basekey"))
    {
        settings.setValue("basekey", "12%*$*094fwda`9082`1dawf,ma");
    }
    if (!settings.contains("algorithm"))
    {
        settings.setValue("algorithm", QCryptographicHash::Md4);
    }
    settings.endGroup();

    QMetaEnum metaEnum = QMetaEnum::fromType<QCryptographicHash::Algorithm>();
    for (int i = 0; i < metaEnum.keyCount(); ++i)
    {
        qDebug() << metaEnum.key(i) << " : " << QCryptographicHash::hashLength((QCryptographicHash::Algorithm) i);
    }
    RegisterWidget w;
    w.show();
    return a.exec();
}
