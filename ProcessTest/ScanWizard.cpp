 
#include "ScanWizard.h"
#include "ScanFilePage.h"
#include "ScanFilterPage.h"
#include "ScanOutPage.h"
//#include "ScanOutPage.h"
#include "ScanCheckPage.h"

#include <QFileInfo>

#include <memory>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "cJSON.h"
#include "ui_WizardWidget.h"
#include "ProjectMgr.h"
#include "ParamMgr.h"
#include "ScanParam.h"
#include "GlobalOption.h"

using namespace xstype;
using namespace xsplugin;
ScanWizard::ScanWizard(QWidget *parent, MainWindow*) :
WizardWidget(parent)
{
    m_pageParams = new ScanParam;
    QString title = QStringLiteral("µãÔÆÉú³É");
    AddWidget(new ScanFilePage(title, ui->stackedWidget));
    ScanFilterPage* fp = new ScanFilterPage(title, false, ui->stackedWidget);
    connect(fp, &ScanFilterPage::openPosSplit, this, &WizardWidget::onPosSplit);
    AddWidget(fp);
    AddWidget(new ScanOutPage(title, false, ui->stackedWidget));
    //AddWidget(new ScanOutPage(title, ui->stackedWidget));
    AddWidget(new ScanCheckPage(title, false, ui->stackedWidget));

    AfterInit();
}

 
ScanWizard::~ScanWizard()
{
 
}
void ScanWizard::AfterInit()
{
    WizardWidget::AfterInit();
    ScanParam* param = static_cast<ScanParam*> (m_pageParams);
    initParam(param, false);

    m_pages[m_curPage]->OnPageParam(m_pageParams);
 
}


void initParam(ScanParam* param, bool bMultiBeam)
{
    param->strDatPath = (bMultiBeam ? ProjectManage::getInstance()->getMBDirPath() : ProjectManage::getInstance()->getDataDirPath());
    param->strSavePath = ProjectManage::getInstance()->getLasDirPath();
    param->strSpanFilePath = ProjectManage::getInstance()->getSpanFilePath();
    param->draft = ProjectManage::getInstance()->getDraft(param->hOff);;
    param->heaveFile = ProjectManage::getInstance()->getHeaveFile();;
    param->tidFile = ProjectManage::getInstance()->getTidFile();;
    if (param->bUseBeijingTime)
    {
        param->dMinTime = ParamInterface::getTimeShift();
        param->dMaxTime = ParamInterface::getTimeShift();
    }
    else
    {
        //param->dMinTime = 0;
        //param->dMaxTime = ONE_DAY_SECOND;
    }
    param->dLongitCenter = 0;
    param->bSingle = ProjectManage::getInstance()->isSingle();
    if (param->bSingle)
    {
        param->dStayDistance = 0;
    }

    {
        param->strTime = ProjectManage::getInstance()->getTimeFile();
    }
    param->absTimeVec.clear();
 
    param->bFaroPrepare = GlobalOption::Parameters().bFaroPrepare;
}

void ScanWizard::onProjectClosed()
{
    WizardWidget::onProjectClosed();
    ScanParam* param = static_cast<ScanParam*> (m_pageParams);
    param->clear();
    m_pages[0]->OnPageParam(m_pageParams);
}

void ScanWizard::onBack(int)
{
    ((ScanFilterPage*)(m_pages[1]))->on_pushButton_importPbr_clicked();
}
