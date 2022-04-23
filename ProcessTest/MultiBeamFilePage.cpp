/*
**********************************************************************************************
* @Copyright(C):青岛秀山移动测量有限公司

* @File Name:ScanTransPage.h

* @Author:朱淑红

* @Version:1.1

* @Date:2012.7.19

* @Description: 该类是数据过滤条件的界面类。
**********************************************************************************************
*/

//#include <QPlastiqueStyle>
#include <QLayout>
#include <QMessageBox>

#include "MultiBeamFilePage.h"
#include "ParamMgr.h"
#include "PluginMgr.h"
#include "ProjectMgr.h"
#include "ScanParam.h"
#include "interfacePrompt.h"
#include "pubfun.h"
#include "ui_ScanFilePage.h"
#include "uifun.h"
#include <QDebug>
#include <QFileDialog>
using namespace xscommon;
using namespace xstype;

MultiBeamFilePage::MultiBeamFilePage(const QString &title, QWidget *parent)
    : ScanFilePage(title, parent)
{
}

MultiBeamFilePage::~MultiBeamFilePage()
{
}

bool MultiBeamFilePage::IsValid()
{

    //    if (!checkLineEditDir(ui->lineEdit_lasDir, m_titleHint, QStringLiteral("请输入LAS保存路径！")))
    //    {
    //        return false;
    //    }

    if (m_datFileMap.empty())
    {
        //        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("没有原始数据文件！"));
        return false;
    }
    QString                 strScan = ProjectManage::getInstance()->getMBType();
    const MultiBeamParaAll *pMbAll  = ParamMgr::getInstance()->GetMBParaAll(strScan);
    if (strScan.isEmpty() || (pMbAll == NULL))
    {
        //        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("多波束检校参数不存在，请设置！"));
        return false;
    }
    //qDebug() << pMbAll->strPlugin;

    ScannerFileMapIt it = m_datFileMap.begin();
    //for (; it != m_datFileMap.end(); ++it)
    {
        //qDebug() << it->first;
        if (pMbAll->strPlugin != it->first)
        {
            //            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("多波束检校参数%1和原始数据不一致，请重新设置！").arg(strScan));
            return false;
        }
    }

    ScannerFileMap sel;
    GetSelFiles(sel);
    if (sel.empty())
    {
        //        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("请选择文件！"));
        return false;
    }
    return true;
}

void MultiBeamFilePage::SetPageParam(PageParam *param)
{
    ScanParam *dp      = static_cast<ScanParam *>(param);
    QString    strScan = ProjectManage::getInstance()->getMBType();
    dp->scanPara       = ParamMgr::getInstance()->GetMBParaAll(strScan);
    dp->paraMb         = nullptr;

    //if (sp)
    //{
    //    sp->toArray(dp->para);
    //}
    ScannerFileMap sel;
    GetSelFiles(sel);
    dp->datFileResultMap = sel;

    dp->strDatPath      = ProjectManage::getInstance()->getMBDirPath();      //ui->lineEdit_datDir->text();
    dp->strSpanFilePath = ProjectManage::getInstance()->getSpanFilePath();   //ui->lineEdit_span->text();

    //    dp->strSavePath = ui->lineEdit_lasDir->text();
    dp->strSavePath.replace('\\', '/');
    // dp->getLasFileMap(true);
}

bool MultiBeamFilePage::OnPageParam(PageParam *param)
{
    ScanParam *dp = static_cast<ScanParam *>(param);

    //    ui->lineEdit_datDir->setText(dp->strDatPath);
    if (dp->strSavePath.isEmpty())
    {
        dp->strSavePath = ProjectManage::getInstance()->getLasDirPath();
    }
    //    ui->lineEdit_lasDir->setText(dp->strSavePath);
    if (dp->strDatPath.isEmpty())
    {
        dp->strDatPath = ProjectManage::getInstance()->getMBDirPath();
    }
    //    ui->lineEdit_span->setText(dp->strSpanFilePath);
    onDataDirTextChanged(dp->strDatPath);
    return true;
}

void MultiBeamFilePage::getFiles(const QString &strDatDir)
{
    m_datFileMap.clear();
    PluginManage::getInstance()->GetAllSupportMBFiles(strDatDir, m_datFileMap);
}
