 
#ifndef ScanCheckPage_H
#define ScanCheckPage_H


#include <QDialog>
#include <vector>
#include <QString>
#include <QStringList>
#include <QListWidgetItem>

#include "DataType.h"
#include "CheckPage.h"
 
class  ScanCheckPage : public CheckPage
{
 
public:
    /*! 构造函数  */
    ScanCheckPage(const QString& title, bool, QWidget *parent = 0);

    /*! 析构函数  */
    ~ScanCheckPage();
 
    virtual void PrintPageParam(xstype::PageParam* param);

protected:
    bool m_bMultiBeam;
};

#endif // CheckPage_H
