#include "RegisterWidget.h"
#include "ui_RegisterWidget.h"

#include <QBuffer>
#include <QCryptographicHash>
#include <QFileDialog>
#include <QSettings>

RegisterWidget::RegisterWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::RegisterWidget)
{
    ui->setupUi(this);
    QDate last_date = QDate::currentDate();
    ui->dateEdit->setDate(last_date.addMonths(3));
    mSoftwareLists = mSettings.childGroups();
    for (int i = 0; i < mSoftwareLists.size(); ++i)
    {
        ui->softwareComboBox->addItem(mSoftwareLists[i]);
    }
}

RegisterWidget::~RegisterWidget()
{
    delete ui;
}

void RegisterWidget::on_generatePushButton_clicked()
{
    QString software_name = ui->softwareComboBox->currentText();

    mSettings.beginGroup(software_name);
    QString                       basekey  = mSettings.value("basekey").toString();
    QCryptographicHash::Algorithm hash_alg = (QCryptographicHash::Algorithm) mSettings.value("algorithm").toInt();

    QString basekey_hash = QCryptographicHash::hash(basekey.toUtf8(), hash_alg);
    QString machine_hash = QCryptographicHash::hash(ui->machineCodeLineEdit->text().toUtf8(), hash_alg);

    QString open_filename = QFileDialog::getSaveFileName(this, "save licenses", software_name + "_" + ui->machineCodeLineEdit->text(), tr("licenses file (*.lic"));

    QFile license_file(open_filename);
    if (license_file.open(QIODevice::WriteOnly))
    {
        QDataStream ds(&license_file);
        ds << basekey_hash << machine_hash << ui->dateEdit->dateTime();
    }
    license_file.close();
}

void RegisterWidget::on_addSoftwarePushButton_clicked()
{
}

void RegisterWidget::on_softwareInfoPushButton_clicked()
{
}
