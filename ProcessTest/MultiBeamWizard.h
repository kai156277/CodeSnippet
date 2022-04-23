 
#ifndef MultiBeamWizard_H
#define MultiBeamWizard_H
 
#include "WizardWidget.h"
#include "DataTypeDefs.h"
#include "Criterion.h"
#include "Constants.h"
 
class MultiBeamWizard : public WizardWidget
{
 
public:
    /*!  ���캯��           */
    explicit MultiBeamWizard(QWidget *parent, MainWindow* wnd= 0);

    /*!  ��������           */
    virtual ~MultiBeamWizard();
 
    void AfterInit();
    void onProjectClosed();
    virtual void onBack(int);
};

#endif // ScanWizard_H
