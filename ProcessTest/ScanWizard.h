
#ifndef ScanWizard_H
#define ScanWizard_H

#include "Constants.h"
#include "Criterion.h"
#include "DataTypeDefs.h"
#include "WizardWidget.h"
namespace xstype {
struct ScanParam;
}
void initParam(xstype::ScanParam *param, bool bMultiBeam);
class ScanWizard : public WizardWidget
{

public:
    /*!  构造函数           */
    explicit ScanWizard(QWidget *parent);

    /*!  析构函数           */
    virtual ~ScanWizard();

    void AfterInit();

    void         onProjectClosed();
    virtual void onBack(int);
};

#endif   // ScanWizard_H
