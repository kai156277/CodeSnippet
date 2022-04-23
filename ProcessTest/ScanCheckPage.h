
#ifndef ScanCheckPage_H
#define ScanCheckPage_H

#include <QDialog>
#include <QListWidgetItem>
#include <QString>
#include <QStringList>
#include <vector>

#include "CheckPage.h"
#include "DataType.h"

#include <PageParam.h>
class ScanCheckPage : public CheckPage
{

public:
    /*! 构造函数  */
    ScanCheckPage(const QString &title, bool, QWidget *parent = 0);

    /*! 析构函数  */
    ~ScanCheckPage();

    virtual void PrintPageParam(xstype::PageParam *param);

protected:
    bool m_bMultiBeam;
};

#endif   // CheckPage_H
