#pragma once

#include "ui_HeaderAndFooterDialog.h"

#include <QtWidgets/qdialog.h>
#include <qstring.h>

struct HeaderAndFooterProfile
{
    QString header_file;
    bool    project_info;
    bool    variable_info;
    QString header_string;
    bool    variable_title;
    bool    variable_unit;

    QString footer_file;
    bool    warning;
    bool    process_summary_info;
    QString footer_string;

    QString toJson();
    void    fromJson(const QString &json);
};

class HeaderAndFooterDialog : public QDialog
{
    Q_OBJECT
public:
    HeaderAndFooterDialog(QWidget *parent = nullptr);

    void set_profile(const HeaderAndFooterProfile &profile);

    HeaderAndFooterProfile profile() const;

    void accept() override;

private:
    Ui::HeaderFooterOptionsDialog ui;

    HeaderAndFooterProfile profile_;
};
