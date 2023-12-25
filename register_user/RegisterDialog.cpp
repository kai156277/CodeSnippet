#include "RegisterDialog.h"
#include "ui_RegisterDialog.h"
#include <QBuffer>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QNetworkInterface>
#include <QSettings>

QByteArray getMACCode()
{
    QList<QNetworkInterface> netList = QNetworkInterface::allInterfaces();
    foreach (QNetworkInterface item, netList)
    {
        if ((QNetworkInterface::IsUp & item.flags()) && (QNetworkInterface::IsRunning & item.flags()))
        {
            if (item.type() == QNetworkInterface::Ethernet)
            {
                return item.hardwareAddress().toUtf8().toBase64();
            }
        }
    }
}

RegisterDialog::RegisterDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::RegisterDialog)
{
    ui->setupUi(this);
    ui->machineCodeLineEdit->setText(getMACCode());
}

RegisterDialog::~RegisterDialog()
{
    delete ui;
}

void RegisterDialog::on_licensePushButton_clicked()
{
    QString license_name = QFileDialog::getOpenFileName(this, "open license file");
    if (license_name.isEmpty())
        return;

    QFile liense_file(license_name);
    if (liense_file.open(QIODevice::ReadOnly))
    {
        QSettings settings;
        settings.setValue("register-code", liense_file.readAll());
    }

    if (isRegistered())
    {
        QMessageBox::information(this, "register", "register ok");
        accept();
    }
    else
    {
        QMessageBox::critical(this, "regiser", "register error");
    }
}

bool RegisterDialog::isRegistered()
{
    QSettings settings;
    if (!settings.contains("register-code"))
    {
        return false;
    }
    QByteArray ba = settings.value("register-code").toByteArray();
    QBuffer    buffer(&ba);
    buffer.open(QIODevice::ReadOnly);
    QDataStream out(&buffer);

    QSettings                     mSettings;
    QString                       basekey  = mSettings.value("basekey").toString();
    QCryptographicHash::Algorithm hash_alg = (QCryptographicHash::Algorithm) mSettings.value("algorithm").toInt();

    QByteArray machine_code = mSettings.value("machine code").toByteArray();
    QString    basekey_hash = QCryptographicHash::hash(basekey.toUtf8(), hash_alg);
    QString    machine_hash = QCryptographicHash::hash(machine_code, hash_alg);

    QString   basekey_1, machine_1;
    QDateTime datetime;
    out >> basekey_1;
    out >> machine_1;
    out >> datetime;
    if (basekey_hash != basekey_1)
    {
        QMessageBox::critical(nullptr, "Error!", "base key error");
        return false;
    }

    if (machine_1 != machine_hash)
    {
        QMessageBox::critical(nullptr, "Error!", "machine code error");
        return false;
    }

    if (QDateTime::currentDateTime() > datetime)
    {
        QMessageBox::critical(nullptr, "Error!", "datetime error");
        return false;
    }
    return true;
}
