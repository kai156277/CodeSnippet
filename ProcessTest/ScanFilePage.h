/*
**********************************************************************************************
* @Copyright(C):�ൺ��ɽ�ƶ��������޹�˾

* @File Name:ScanTransPage.h

* @Author:�����

* @Version:1.1

* @Date:2012.7.19

* @Description: ���������ݹ��������Ľ����ࡣ
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
*@brief ���������ݹ��������Ľ�����
*@note ��ʾ�ý������ȷ�������ź�oksignal()����ʾ��һ�����档
*/
 
class ScanFilePage : public WizardPage
{
    Q_OBJECT

public:
    /*!  ���캯��           */
    explicit ScanFilePage(const QString& title, QWidget *parent = 0);

    /*!  ��������           */
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
    Ui::ScanFilePage *ui;                     ///< �����ָ��
    xstype::ScannerFileMap m_datFileMap;
 
    MultiTreeView* m_treeFiles;
    QIcon  m_treeIcon1; 
    QIcon  m_treeIcon2;
    QString m_datDir;
};

#endif // ScanTransPage_H
