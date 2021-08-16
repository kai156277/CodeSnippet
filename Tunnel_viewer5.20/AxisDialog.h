#ifndef AXISDIALOG_H
#define AXISDIALOG_H

#include <QDialog>

namespace Ui {
class AxisDialog;
}

class AxisDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AxisDialog(QWidget *parent = 0);
    ~AxisDialog();
    int method=-1;
private slots:
    void getMethod();
private:
    Ui::AxisDialog *ui;
};

#endif // AXISDIALOG_H
