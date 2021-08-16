#ifndef TUNNELTYPEDIALOG_H
#define TUNNELTYPEDIALOG_H

#include <QDialog>

namespace Ui {
class TunnelTypeDialog;
}

class TunnelTypeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TunnelTypeDialog(QWidget *parent = nullptr);
    ~TunnelTypeDialog();

private slots:
    void on_pushButton_ok_clicked();

    void on_pushButton_cancel_clicked();

private:
    Ui::TunnelTypeDialog *ui;
};

#endif // TUNNELTYPEDIALOG_H
