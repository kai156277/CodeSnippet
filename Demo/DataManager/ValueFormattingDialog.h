#pragma once
#include "DataUnit.h"
#include "VariableFormat.h"
#include "ui_ValueFormattingDialog.h"
#include <QtWidgets/qdialog.h>

class ValueFormattingDialog : public QDialog
{
    Q_OBJECT
public:
    ValueFormattingDialog(QWidget *parent = nullptr);

    void SetValueFormat(const QString &src_name, const QString &var_name, const pos::VariableFormat &format);

    pos::VariableFormat format() const;

    QString source_name() const;
    QString variable_name() const;
    QString variable_human_name() const;

    void accept() override;

private:
    QString             variable_human_name_;
    QString             source_name_;
    QString             variable_name_;
    pos::VariableFormat format_;

    Ui::ValueFormattingDialog ui;
};
