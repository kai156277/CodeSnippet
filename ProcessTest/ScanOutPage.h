/*
**********************************************************************************************
* @Copyright(C):�ൺ��ɽ�ƶ��������޹�˾

* @File Name:ScanOutPage.h

* @Author:�����

* @Version:1.1

* @Date:2012.7.19

* @Description: ���������ݹ��������Ľ����ࡣ
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
    /*!  ���캯��           */
    explicit ScanOutPage(const QString& title, bool bmb, QWidget *parent = 0);

    /*!  ��������           */
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
    Ui::ScanOutPage *ui;                     ///< �����ָ��
    bool m_bMb;
    //bool m_scanMb;
    CoordInfoWidget* m_coordInfo;
};

#endif // ScanOutPage_H
