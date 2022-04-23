 
#ifndef ScanWizard_H
#define ScanWizard_H
 
#include "WizardWidget.h"
#include "DataTypeDefs.h"
#include "Criterion.h"
#include "Constants.h"
namespace xstype {
    struct ScanParam;
}
void initParam(xstype::ScanParam* param, bool bMultiBeam);
class ScanWizard : public WizardWidget
{
 
public:
    /*!  ���캯��           */
    explicit ScanWizard(QWidget *parent, MainWindow* wnd= 0);

    /*!  ��������           */
    virtual ~ScanWizard();
 
    void AfterInit();
     
    void onProjectClosed();
    virtual void onBack(int);
};

#endif // ScanWizard_H
