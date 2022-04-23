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

#ifndef MultiBeamFilePage_H
#define MultiBeamFilePage_H
 
#include <QDialog>
#include <QRegExpValidator>
 
#include "DataTypeDefs.h"
#include "ScanFilePage.h"
 
 
/*!
*@brief ���������ݹ��������Ľ�����
*@note ��ʾ�ý������ȷ�������ź�oksignal()����ʾ��һ�����档
*/

 
class MultiBeamFilePage : public ScanFilePage
{
 
public:
    /*!  ���캯��           */
    explicit MultiBeamFilePage(const QString& title, QWidget *parent = 0);

    /*!  ��������           */
    virtual ~MultiBeamFilePage();
 
    virtual bool IsValid();
    virtual void SetPageParam(xstype::PageParam* param);
    virtual bool OnPageParam(xstype::PageParam* param);
    virtual void getFiles(const QString& strDatDir);
};

#endif // ScanTransPage_H
