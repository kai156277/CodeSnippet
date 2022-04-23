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

#ifndef MultiBeamFilePage_H
#define MultiBeamFilePage_H
 
#include <QDialog>
#include <QRegExpValidator>
 
#include "DataTypeDefs.h"
#include "ScanFilePage.h"
 
 
/*!
*@brief 该类是数据过滤条件的界面类
*@note 显示该界面后点击确定发出信号oksignal()，显示下一个界面。
*/

 
class MultiBeamFilePage : public ScanFilePage
{
 
public:
    /*!  构造函数           */
    explicit MultiBeamFilePage(const QString& title, QWidget *parent = 0);

    /*!  析构函数           */
    virtual ~MultiBeamFilePage();
 
    virtual bool IsValid();
    virtual void SetPageParam(xstype::PageParam* param);
    virtual bool OnPageParam(xstype::PageParam* param);
    virtual void getFiles(const QString& strDatDir);
};

#endif // ScanTransPage_H
