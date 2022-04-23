/*
**********************************************************************************************
* @Copyright(C):青岛秀山移动测量有限公司

* @File Name:ScanOutPage.h

* @Author:朱淑红

* @Version:1.1

* @Date:2012.7.19

* @Description: 该类是数据过滤条件的界面类。
**********************************************************************************************
*/

#ifndef ScanOutPage_H
#define ScanOutPage_H
 
#include <QDialog>
 
#include "DataType.h"
#include "WizardPage.h"
 
namespace Ui {
    class ScanOutPage;
}

class CoordInfoWidget;
class ScanOutPage : public WizardPage
{
    Q_OBJECT

public:
    /*!  构造函数           */
    explicit ScanOutPage(const QString& title, bool bmb, QWidget *parent = 0);

    /*!  析构函数           */
    ~ScanOutPage();
 
    virtual void SetPageParam(xstype::PageParam* param);

    virtual bool IsValid();
    virtual bool OnPageParam(xstype::PageParam* param);

private:
    void setCalibrate(double rphV[3]);//, ScannerType::unit unit
    void setMb(bool mb);
    //void setOver(double paraV[6]);
 
private slots:

    void on_checkBox_outputLASFile_toggled(bool);
    void on_checkBox_outputLinFile_toggled(bool bChecked);
    void on_groupBox_single_toggled(bool bChecked);
    void on_checkBox_q12_toggled(bool bChecked);
    void on_checkBox_scanMb_toggled(bool bChecked);
private:
    Ui::ScanOutPage *ui;                     ///< 界面的指针
    bool m_bMb;
    //bool m_scanMb;
    CoordInfoWidget* m_coordInfo;
};

#endif // ScanOutPage_H
