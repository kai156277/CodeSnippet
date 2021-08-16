#pragma once //CONVERTPARAMDIALOG_H

#include <QDialog>

namespace Ui {
class ConvertParamDialog;
}

class ConvertParamDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ConvertParamDialog(double lng = 120, QWidget *parent = 0);
    ~ConvertParamDialog();

    int epsg;
private slots:
    void on_mLngSpinBox_valueChanged(int arg1);

    void on_mExportCoordSysComboBox_currentIndexChanged(int index);

    void on_buttonBox_accepted();

private:
    Ui::ConvertParamDialog *ui;
};

