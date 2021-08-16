#ifndef DIALOG_SECTION_H
#define DIALOG_SECTION_H

#include <QDialog>

namespace Ui {
class Dialog_Section;
}

class Dialog_Section : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog_Section(QWidget *parent = 0);
    ~Dialog_Section();
    float Mileage;
    QString AxisPath;
    float d=-1;//断面厚度
    QString SaveSectionPath;
private slots:
    void GetParameter();

private:
    Ui::Dialog_Section *ui;
};

#endif // DIALOG_SECTION_H
