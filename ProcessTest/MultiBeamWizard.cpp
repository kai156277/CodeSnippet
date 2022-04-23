   
#include "MultiBeamWizard.h"
#include "MultiBeamFilePage.h"
#include "ScanFilterPage.h"
#include "ScanOutPage.h"
 
#include "ScanCheckPage.h"

#include <QFileInfo>

#include <memory>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "cJSON.h"
#include "ui_WizardWidget.h"
#include "ParamInterface.h"
#include "ProjectMgr.h"
#include "ScanParam.h"
#include "ScanWizard.h"

using namespace xstype;
using namespace xsplugin;
MultiBeamWizard::MultiBeamWizard(QWidget *parent, MainWindow*) :
WizardWidget(parent)
{
    m_pageParams = new MultiBeamParam;//!!
 
    QString title = QStringLiteral("多波束点云生成");
    AddWidget(new MultiBeamFilePage(title, ui->stackedWidget));
    ScanFilterPage* fp = new ScanFilterPage(title, true, ui->stackedWidget);
    connect(fp, &ScanFilterPage::openPosSplit, this, &WizardWidget::onPosSplit);
    AddWidget(fp);
 
    AddWidget(new ScanOutPage(title, true, ui->stackedWidget));
    AddWidget(new ScanCheckPage(title,true, ui->stackedWidget));

    AfterInit();
}

 
MultiBeamWizard::~MultiBeamWizard()
{
 
}
void MultiBeamWizard::AfterInit()
{
    WizardWidget::AfterInit();
    MultiBeamParam* param = static_cast<MultiBeamParam*> (m_pageParams);
    initParam(param, true);
     
    param->soundFile = ProjectManage::getInstance()->getSoundFile();;
    param->dStayDistance = 0; 
    m_pages[m_curPage]->OnPageParam(m_pageParams);
 
}

void MultiBeamWizard::onProjectClosed()
{
    WizardWidget::onProjectClosed();
    MultiBeamParam* param = static_cast<MultiBeamParam*> (m_pageParams);
    param->clear(); 
    param->soundFile.clear(); 
    param->dStayDistance = 0; 
    m_pages[0]->OnPageParam(m_pageParams);
}


void MultiBeamWizard::onBack(int)
{
    ((ScanFilterPage*)(m_pages[1]))->on_pushButton_importPbr_clicked();
}
