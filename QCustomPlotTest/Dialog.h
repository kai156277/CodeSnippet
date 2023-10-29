#pragma once

#include <QDialog>
#include <QVector>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = nullptr);
    ~Dialog();

    void init();
private slots:
    void on_BPSKPushButton_clicked();

    void on_pushButton_clicked();

private:
    Ui::Dialog *ui;

    QVector<int> a;

    QVector<QVector<int>> mB;
};
