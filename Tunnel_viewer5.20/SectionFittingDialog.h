#ifndef SECTIONFITTINGDIALOG_H
#define SECTIONFITTINGDIALOG_H

#include "SectionFitting.h"

#include <QDialog>

class QRadioButton;
namespace Ui {
class SectionFittingDialog;
}

class SectionFittingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SectionFittingDialog(QWidget *parent = nullptr);
    ~SectionFittingDialog();

    SectionFitting::Type selectType() const;

private slots:
    void on_buttonBox_accepted();

private:
    Ui::SectionFittingDialog *ui;

    SectionFitting::Type    type_ = SectionFitting::None;
    QVector<QRadioButton *> fittings_;
};

#endif   // SECTIONFITTINGDIALOG_H
