/*
**********************************************************************************************
* @Copyright(C):青岛秀山移动测量有限公司

* @File Name:ScanOutPage.h

* @Author:朱淑红

* @Version:1.1

* @Date:2012.7.19

* @Description: 该类是数据过滤条件的界面类。
**********************************************************************************************
*/

//#include <QPlastiqueStyle>
#include <QLayout>
#include <QMessageBox>

#include "CoordInfoWidget.h"
#include "CoordParam.h"
#include "ParamMgr.h"
#include "PluginMgr.h"
#include "ProjectMgr.h"
#include "ScanOutPage.h"
#include "ScanParam.h"
#include "TransCoords.h"
#include "interfacePrompt.h"
#include "ui_ScanOutPage.h"
#include <QDebug>
#include <QFileDialog>
using namespace xstype;
ScanOutPage::ScanOutPage(const QString &title, bool mb, QWidget *parent)
    : WizardPage(title, parent)
    , m_bMb(mb)
    , ui(new Ui::ScanOutPage)
{
    ui->setupUi(this);

    //ui->groupBox_spanAngle->setStyle(new QPlastiqueStyle);
    //ui->groupBox_6params->setStyle(new QPlastiqueStyle);
    double dVal[3] = {0};
    setCalibrate(dVal);   //, ScannerType::ANGLE
    //double dOverVal[6] = { 0 };
    //setOver(dOverVal);
    //    ui->checkBox_scanMb->setVisible(false);
    //    ui->checkBox_scanMb->setChecked(false);
    if (!m_bMb && !ProjectManage::getInstance()->getMBType().isEmpty())
    {
        //        ui->checkBox_scanMb->setVisible(true);
    }
    //m_scanMb = false;
    //ui->MileIncreaseByBeforeFile->setVisible(false);
    m_coordInfo = new CoordInfoWidget(ParamMgr::getInstance(), &this->m_tempDir);
    //    ui->gridLayout_coord->addWidget(m_coordInfo);
    m_coordInfo->setFixEllipsoid(true);

    setMb(m_bMb);
    if (!PluginManage::getInstance()->isSupport(xsplugin::MainAppInterface::Support_MultiBeam))
    {
        //        ui->checkBox_scanMb->setChecked(false);
        //        ui->checkBox_scanMb->setVisible(false);
    }
}

// 析构函数
ScanOutPage::~ScanOutPage()
{

    delete ui;
}

// 打开窗口时，设置各参数显示为调整前的数值
// 参数：
// rphV[3]：姿态角改正值
// unit：角度单位
void ScanOutPage::setCalibrate(double rphV[3])   //, ScannerType::unit unit
{

    //    ui->RollSpinBox->setValue(rphV[0]);      // 侧滚角
    //    ui->PitchSpinBox->setValue(rphV[1]);     // 俯仰角
    //    ui->HeadingSpinBox->setValue(rphV[2]);   // 航向角
}
//void ScanOutPage::setOver(double paraV[6])
//{
//ui->panXSpinBox->setValue(paraV[0]);
//ui->panYSpinBox->setValue(paraV[1]);
//ui->panZSpinBox->setValue(paraV[2]);
//ui->rotateYSpinBox->setValue(paraV[3]);
//ui->rotateXSpinBox->setValue(paraV[4]);
//ui->rotateZSpinBox->setValue(paraV[5]);
//}
void ScanOutPage::on_checkBox_scanMb_toggled(bool bChecked)
{
    //m_scanMb = bChecked;
    m_coordInfo->setVisible(bChecked);
    m_coordInfo->showMenu();
}
void ScanOutPage::on_groupBox_single_toggled(bool bChecked)
{
    if (bChecked)
    {
        //        ui->groupBox_spanAngle->setChecked(false);
        //        ui->groupBox_spanAngle->setEnabled(false);
        //        ui->groupBox_prec->setEnabled(false);
        //        ui->checkBox_bnz->setChecked(false);
        //        ui->checkBox_bnz->setEnabled(false);
    }
    else
    {
        //        ui->groupBox_spanAngle->setEnabled(true);
        //        ui->groupBox_prec->setEnabled(true);
        //        ui->checkBox_bnz->setEnabled(ui->checkBox_outputLinFile->isChecked());
    }
}
void ScanOutPage::on_checkBox_q12_toggled(bool bChecked)
{
    //    ui->label_q3->setEnabled(bChecked);
    //    ui->doubleSpinBox_q3->setEnabled(bChecked);
}
void ScanOutPage::on_checkBox_outputLASFile_toggled(bool bChecked)
{
    //    ui->checkBox_laz->setEnabled(bChecked);
}
void ScanOutPage::on_checkBox_outputLinFile_toggled(bool bChecked)
{
    //    ui->checkBox_bnz->setEnabled(bChecked);
}
void ScanOutPage::SetPageParam(PageParam *param)
{
    //    ScanParam *dp     = static_cast<ScanParam *>(param);
    //    dp->spanAngleV[0] = ui->RollSpinBox->value();
    //    dp->spanAngleV[1] = ui->PitchSpinBox->value();     // 俯仰角
    //    dp->spanAngleV[2] = ui->HeadingSpinBox->value();   // 航向角

    //    //dp->bUsePrecisePara = ui->checkBox_isUsePrecisePara->isChecked();
    //    dp->scannerSigmaInfo[0] = ui->doubleSpinBox_sigma_distance->value();
    //    dp->scannerSigmaInfo[1] = ui->doubleSpinBox_sigma_hAngle->value();
    //    dp->scannerSigmaInfo[2] = ui->doubleSpinBox_sigma_vAngle->value();

    //    dp->bLaz           = ui->checkBox_laz->isChecked();
    //    dp->bCreateLasFile = ui->checkBox_outputLASFile->isChecked();
    //    dp->bCreateLinFile = ui->checkBox_outputLinFile->isChecked();
    //    dp->bBinAll        = true;
    //    dp->bBnz           = ui->checkBox_bnz->isChecked();
    //    dp->bScanMb        = ui->checkBox_scanMb->isChecked();
    //    if (m_bMb || dp->bScanMb)   //m_coordInfo->isVisible())
    //    {
    //        m_coordInfo->SetPageParam(dp->projPara);
    //    }

    //    //ui->groupBox_single->isChecked();
    //    dp->bQ12    = ui->checkBox_q12->isChecked();
    //    dp->dQ3Time = ui->doubleSpinBox_q3->value();
    //    if (dp->dQ3Time < FLT_EPSILON)
    //    {
    //        dp->dQ3Time = 0;
    //    }
    //    dp->getLasFileMap(m_bMb);   //for bScanMb;
}

bool ScanOutPage::IsValid()
{
    /*
    if (!ui->checkBox_outputLinFile->isChecked() &&
        !ui->checkBox_outputLASFile->isChecked())
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("las和lin请至少选择一个！"));
        return false;
    }
    if (!ProjectManage::getInstance()->isSingle())   //ui->groupBox_single->isChecked()
    {
        if (ProjectManage::getInstance()->getSpanFilePath().isEmpty())
        {
            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("没有POS数据！"));
            return false;
        }
    }
    bool m_scanMb = ui->checkBox_scanMb->isChecked();
    if (m_bMb || m_scanMb)   //m_coordInfo->isVisible())
    {
        if (!m_coordInfo->IsValid(m_titleHint))
        {
            return false;
        }

        PlaneProjPara projPara;
        m_coordInfo->SetPageParam(projPara);
        if (ui->checkBox_outputLinFile->isChecked())   //&& ui->checkBox_bnz->isChecked()
        {                                              //outbin
            if (projPara.projParam.zoneWide != 3)
            {   //for calib
                interfacePrompt::Warn(this, m_titleHint, QStringLiteral("BIN数据必须是高斯3度带！"));
                return false;
            }
        }
    }
*/

    return true;
}

bool ScanOutPage::OnPageParam(PageParam *param)
{
    /*
    if (!m_bMb && !ProjectManage::getInstance()->getMBType().isEmpty())
    {
        ui->checkBox_scanMb->setVisible(true);
    }
    ScanParam *dp = static_cast<ScanParam *>(param);
    setCalibrate(dp->spanAngleV);

    ui->doubleSpinBox_sigma_distance->setValue(dp->scannerSigmaInfo[0]);
    ui->doubleSpinBox_sigma_hAngle->setValue(dp->scannerSigmaInfo[1]);
    ui->doubleSpinBox_sigma_vAngle->setValue(dp->scannerSigmaInfo[2]);

    ui->checkBox_outputLASFile->setChecked(dp->bCreateLasFile);
    ui->checkBox_laz->setEnabled(dp->bCreateLasFile);
    ui->checkBox_laz->setChecked(dp->bLaz);
    ui->checkBox_outputLinFile->setChecked(dp->bCreateLinFile);
    ui->checkBox_bnz->setEnabled(dp->bCreateLinFile);
    ui->checkBox_bnz->setChecked(dp->bBnz);
    ui->checkBox_scanMb->setChecked(dp->bScanMb);

    //if (m_bMb || dp->bScanMb)//m_coordInfo->isVisible())
    {
        if (dp->dLongitCenter == 0)
        {
            dp->dLongitCenter = ProjectManage::getInstance()->getCurJobLong();
            //m_coordInfo->setCenter();
            dp->projPara.projParam.L0 = pcm::TransCoords::longit2center(dp->dLongitCenter);
            pcm::CoordinateTransform::computeProjNo(dp->projPara.projParam);
            m_coordInfo->setInitLong(dp->dLongitCenter);
        }
        m_coordInfo->OnPageParam(dp->projPara);
    }
    //ui->groupBox_single->setChecked(dp->bSingle);

    ui->checkBox_q12->setChecked(dp->bQ12);
    on_checkBox_q12_toggled(dp->bQ12);
    ui->doubleSpinBox_q3->setValue(dp->dQ3Time);
    */
    return true;
}

void ScanOutPage::setMb(bool mb)
{
    m_coordInfo->setVisible(mb);
    m_coordInfo->showMenu();
}
