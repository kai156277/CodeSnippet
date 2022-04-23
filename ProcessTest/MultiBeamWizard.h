 
#ifndef MultiBeamWizard_H
#define MultiBeamWizard_H
 
#include "WizardWidget.h"
#include "DataTypeDefs.h"
#include "Criterion.h"
#include "Constants.h"
 
class MultiBeamWizard : public WizardWidget
{
 
public:
    /*!  构造函数           */
    explicit MultiBeamWizard(QWidget *parent, MainWindow* wnd= 0);

    /*!  析构函数           */
    virtual ~MultiBeamWizard();
 
    void AfterInit();
    void onProjectClosed();
    virtual void onBack(int);
};

#endif // ScanWizard_H
