#pragma once

#include <QDialog>

namespace Ui {
class RegisterDialog;
}

class RegisterDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RegisterDialog(QWidget *parent = nullptr);
    ~RegisterDialog();

    static bool isRegistered();
private slots:
    void on_licensePushButton_clicked();

private:
    Ui::RegisterDialog *ui;
};

QByteArray getMACCode();
