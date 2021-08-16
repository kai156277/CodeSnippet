#include "HeaderAndFooterDialog.h"
#include <QtCore/qjsondocument.h>
#include <QtCore/qjsonobject.h>
#include <QtWidgets/qfiledialog.h>

HeaderAndFooterDialog::HeaderAndFooterDialog(QWidget *parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    connect(ui.ok_pushButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(ui.cancel_pushButton, &QPushButton::clicked, this, &QDialog::reject);

    connect(ui.header_file_checkBox,
            &QCheckBox::clicked,
            [this](bool checked) {
                ui.header_file_browse_pushButton->setEnabled(checked);
                ui.header_file_lineEdit->setEnabled(checked);
            });

    connect(ui.header_string_checkBox,
            &QCheckBox::clicked,
            [this](bool checked) {
                ui.header_string_lineEdit->setEnabled(checked);
            });

    connect(ui.footer_file_checkBox,
            &QCheckBox::clicked,
            [this](bool checked) {
                ui.footer_file_browse_pushButton->setEnabled(checked);
                ui.footer_file_lineEdit->setEnabled(checked);
            });

    connect(ui.footer_string_checkBox,
            &QCheckBox::clicked,
            [this](bool checked) {
                ui.footer_string_lineEdit->setEnabled(checked);
            });

    connect(ui.header_file_browse_pushButton,
            &QCheckBox::clicked,
            [this](bool checked) {
                QString file = QFileDialog::getOpenFileName(this,
                                                            QString::fromUtf8("页眉插入文件"),
                                                            "",
                                                            "文本文件(*.txt)");
                if (!file.isEmpty())
                    ui.header_file_lineEdit->setText(file);
            });

    connect(ui.footer_file_browse_pushButton,
            &QCheckBox::clicked,
            [this](bool checked) {
                QString file = QFileDialog::getOpenFileName(this,
                                                            QString::fromUtf8("页脚插入文件"),
                                                            "",
                                                            "文本文件(*.txt)");
                if (!file.isEmpty())
                    ui.footer_file_lineEdit->setText(file);
            });
}

void HeaderAndFooterDialog::set_profile(const HeaderAndFooterProfile &profile)
{
    profile_ = profile;

    if (!profile_.header_file.isEmpty())
    {
        ui.header_file_checkBox->setChecked(true);
        ui.header_file_lineEdit->setEnabled(true);
        ui.header_file_lineEdit->setText(profile_.header_file);
        ui.header_file_browse_pushButton->setEnabled(true);
    }
    else
    {
        ui.header_file_checkBox->setChecked(false);
        ui.header_file_lineEdit->setEnabled(false);
        ui.header_file_browse_pushButton->setEnabled(false);
        ui.header_file_lineEdit->clear();
    }

    ui.header_info_checkBox->setChecked(profile_.project_info);
    ui.variable_info_checkBox->setChecked(profile_.variable_info);

    if (!profile_.header_string.isEmpty())
    {
        ui.header_string_checkBox->setChecked(true);
        ui.header_string_lineEdit->setEnabled(true);
        ui.header_string_lineEdit->setText(profile_.header_string);
    }
    else
    {
        ui.header_string_checkBox->setChecked(false);
        ui.header_string_lineEdit->setEnabled(false);
        ui.header_string_lineEdit->clear();
    }

    ui.variable_title_checkBox->setChecked(profile_.variable_title);
    ui.variable_unit_checkBox->setChecked(profile_.variable_unit);

    if (!profile_.footer_file.isEmpty())
    {
        ui.footer_file_checkBox->setChecked(true);
        ui.footer_file_lineEdit->setEnabled(true);
        ui.footer_file_lineEdit->setText(profile_.footer_file);
        ui.footer_file_browse_pushButton->setEnabled(true);
    }
    else
    {
        ui.footer_file_checkBox->setChecked(false);
        ui.footer_file_lineEdit->setEnabled(false);
        ui.footer_file_lineEdit->clear();
        ui.footer_file_browse_pushButton->setEnabled(false);
    }

    ui.error_and_warning_checkBox->setChecked(profile_.warning);
    ui.processing_summary_info_checkBox->setChecked(profile_.process_summary_info);

    if (!profile_.footer_string.isEmpty())
    {
        ui.footer_string_checkBox->setChecked(true);
        ui.footer_string_lineEdit->setEnabled(true);
        ui.footer_string_lineEdit->setText(profile_.footer_string);
    }
    else
    {
        ui.footer_string_checkBox->setChecked(false);
        ui.footer_string_lineEdit->setEnabled(false);
        ui.footer_string_lineEdit->clear();
    }
}

HeaderAndFooterProfile HeaderAndFooterDialog::profile() const
{
    return profile_;
}

void HeaderAndFooterDialog::accept()
{
    if (ui.header_file_checkBox->isChecked())
        profile_.header_file = ui.header_file_lineEdit->text();

    profile_.project_info  = ui.header_info_checkBox->isChecked();
    profile_.variable_info = ui.variable_info_checkBox->isChecked();

    if (ui.header_string_checkBox->isChecked())
        profile_.header_string = ui.header_string_lineEdit->text();

    profile_.variable_title = ui.variable_title_checkBox->isChecked();
    profile_.variable_unit  = ui.variable_unit_checkBox->isChecked();

    if (ui.footer_file_checkBox->isChecked())
        profile_.footer_file = ui.footer_file_lineEdit->text();

    profile_.warning              = ui.error_and_warning_checkBox->isChecked();
    profile_.process_summary_info = ui.processing_summary_info_checkBox->isChecked();

    if (ui.footer_string_checkBox->isChecked())
        profile_.footer_string = ui.footer_string_lineEdit->text();

    QDialog::accept();
}

namespace {
const QString kHeaderFile         = QStringLiteral("header_file");
const QString kProjectInfo        = QStringLiteral("project_info");
const QString kVariableInfo       = QStringLiteral("variable_info");
const QString kHeaderString       = QStringLiteral("header_string");
const QString kVariableTitle      = QStringLiteral("variable_title");
const QString kVariableUnit       = QStringLiteral("variable_unit");
const QString kFooterFile         = QStringLiteral("footer_file");
const QString kWarning            = QStringLiteral("warning");
const QString kProcessSummaryInfo = QStringLiteral("process_summary_info");
const QString kFooterString       = QStringLiteral("footer_string");
}   // namespace

QString HeaderAndFooterProfile::toJson()
{
    QJsonObject obj {
        {kHeaderFile, header_file},
        {kProjectInfo, project_info},
        {kVariableInfo, variable_info},
        {kHeaderString, header_string},
        {kVariableTitle, variable_title},
        {kVariableUnit, variable_unit},
        {kFooterFile, footer_file},
        {kWarning, warning},
        {kProcessSummaryInfo, process_summary_info},
        {kFooterString, footer_string},
    };
    QJsonDocument doc(obj);
    return doc.toJson();
}

void HeaderAndFooterProfile::fromJson(const QString &json)
{
    QJsonParseError error;
    QJsonDocument   doc = QJsonDocument::fromJson(json.toUtf8(), &error);
    if (error.error == QJsonParseError::NoError)
    {
        QJsonObject obj      = doc.object();
        header_file          = obj.value(kHeaderFile).toString();
        project_info         = obj.value(kProjectInfo).toBool();
        variable_info        = obj.value(kVariableInfo).toBool();
        header_string        = obj.value(kHeaderString).toString();
        variable_title       = obj.value(kVariableTitle).toBool();
        variable_unit        = obj.value(kVariableUnit).toBool();
        footer_file          = obj.value(kFooterFile).toString();
        warning              = obj.value(kWarning).toBool();
        process_summary_info = obj.value(kProcessSummaryInfo).toBool();
        footer_string        = obj.value(kFooterString).toString();
    }
}
