#include "RegisterDialog.h"

#include <QApplication>
#include <QCryptographicHash>

#include <QMessageBox>
#include <QSettings>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QCoreApplication::setOrganizationDomain("luoyu.com");
    QCoreApplication::setOrganizationName("luoyu");
    QApplication::setApplicationName("register-user");

    QSettings settings;
    if (!settings.contains("basekey"))
    {
        settings.setValue("basekey", "12%*$*094fwda`9082`1dawf,ma");
    }
    if (!settings.contains("algorithm"))
    {
        settings.setValue("algorithm", QCryptographicHash::Md4);
    }
    if (!settings.contains("machine code"))
    {
        settings.setValue("machine code", getMACCode());
    }

    while (true)
    {
        if (RegisterDialog::isRegistered())
            break;

        RegisterDialog w;
        int            exec_code = w.exec();
        if (exec_code == QDialog::Rejected)
            return 0;
        if (exec_code == QDialog::Accepted && RegisterDialog::isRegistered())
            break;
    };

    QMessageBox::information(nullptr, "ok", "ok");
    return a.exec();
}
