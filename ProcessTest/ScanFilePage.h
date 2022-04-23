/*
**********************************************************************************************
* @Copyright(C):青岛秀山移动测量有限公司

* @File Name:ScanTransPage.h

* @Author:朱淑红

* @Version:1.1

* @Date:2012.7.19

* @Description: 该类是数据过滤条件的界面类。
**********************************************************************************************
*/

#ifndef ScanFilePage_H
#define ScanFilePage_H
 
#include <QDialog>
 
#include "DataTypeDefs.h"
#include "WizardPage.h"
#include "MultiTreeView.h"
 
namespace Ui {
    class ScanFilePage;
}

/*!
*@brief 该类是数据过滤条件的界面类
*@note 显示该界面后点击确定发出信号oksignal()，显示下一个界面。
*/
 
class ScanFilePage : public WizardPage
{
    Q_OBJECT

public:
    /*!  构造函数           */
    explicit ScanFilePage(const QString& title, QWidget *parent = 0);

    /*!  析构函数           */
    virtual ~ScanFilePage();
 
    virtual bool IsValid();
    virtual void SetPageParam(xstype::PageParam* param);

    virtual bool OnPageParam(xstype::PageParam* param);
    virtual void GetSelFiles(xstype::ScannerFileMap&);
    virtual void getFiles(const QString& strDatDir);
protected slots:
 
    void on_pushButton_lasDir_clicked();
 
    void onDataDirTextChanged(const QString& text);

protected:
    Ui::ScanFilePage *ui;                     ///< 界面的指针
    xstype::ScannerFileMap m_datFileMap;
 
    MultiTreeView* m_treeFiles;
    QIcon  m_treeIcon1; 
    QIcon  m_treeIcon2;
    QString m_datDir;
};

#endif // ScanTransPage_H
