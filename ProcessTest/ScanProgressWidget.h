
#ifndef ScanProgressWidget_H
#define ScanProgressWidget_H

#include <map>
#include <vector>

#include <QString>

#include "ProgressWidget.h"

#include <common/PageParam.h>

class QProgressBar;
class ScanProgressWidget : public ProgressWidget
{

public:
    /*!  构造函数           */
    explicit ScanProgressWidget(QWidget *parent = 0);

    /*!  析构函数           */
    ~ScanProgressWidget();
    void Init(xstype::PageParam *param);

protected:
    void onBeginFile(const QString &fileName);

    void onEndFile(const QString &fileName, unsigned, unsigned);

    void onEndProcess(int num);

    void on_pushButton_close_clicked();

protected:
    QProgressBar *m_posBar;
    int           m_num;
    QString       posPath;
};

#endif   // ScanProgressWidget_H
