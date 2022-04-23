 
#ifndef MultiBeamProgressWidget_H
#define MultiBeamProgressWidget_H
 
 
#include <vector>
#include <map>
 
#include <QString>
 
#include "ScanProgressWidget.h"
 
class MultiBeamProgressWidget : public ScanProgressWidget
{
 
public:
    /*!  ���캯��           */
    explicit MultiBeamProgressWidget(QWidget *parent = 0);

    /*!  ��������           */
    ~MultiBeamProgressWidget();
    void Init(xstype::PageParam* param);
 
};

#endif // MultiBeamProgressWidget_H
