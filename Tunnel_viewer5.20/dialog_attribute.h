#ifndef DIALOG_ATTRIBUTE_H
#define DIALOG_ATTRIBUTE_H

#include <QDialog>
#include<vector>
using namespace std;
namespace Ui {
class Dialog_Attribute;
}

class Dialog_Attribute : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog_Attribute(QWidget *parent = 0);
    ~Dialog_Attribute();
    float z0;
    vector<float> parameter;
    bool TopPoint;
private slots:
    void GetParameter();

private:
    Ui::Dialog_Attribute *ui;
};

#endif // DIALOG_ATTRIBUTE_H
