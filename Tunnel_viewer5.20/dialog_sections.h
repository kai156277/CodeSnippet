#ifndef DIALOG_SECTIONS_H
#define DIALOG_SECTIONS_H

#include <QDialog>

namespace Ui {
class Dialog_Sections;
}

class Dialog_Sections : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog_Sections(QWidget *parent = 0);
    ~Dialog_Sections();
    float MileageInterval;
    QString AxisPath;
    float d=-1;//断面厚度
    QString SaveSectionPath;
private slots:
    void GetParameter();


private:
    Ui::Dialog_Sections *ui;
};

#endif // DIALOG_SECTIONS_H
