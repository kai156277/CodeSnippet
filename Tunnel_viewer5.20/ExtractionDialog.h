#ifndef EXTRACTIONDIALOG_H
#define EXTRACTIONDIALOG_H

#include <QDialog>

namespace Ui {
class ExtractionDialog;
}

class ExtractionDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ExtractionDialog(QWidget *parent = 0);
    ~ExtractionDialog();

    int method=-1;
private slots:
    void getMethod();
private:
    Ui::ExtractionDialog *ui;

};

#endif // EXTRACTIONDIALOG_H
