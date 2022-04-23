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
#include <QMessageBox>
#include <QLayout>
#include <QStandardItemModel>

#include "ScanFilePage.h"
#include "ui_ScanFilePage.h"
#include <QFileDialog>
#include <QPushButton>

#include <QDebug>
#include "ParamMgr.h"
#include "PluginMgr.h"
#include "pubfun.h"
#include "uifun.h"
#include "ScanParam.h"
#include "ProjectMgr.h"
#include "interfacePrompt.h"
using namespace xscommon;
using namespace xstype;
ScanFilePage::ScanFilePage(const QString& title, QWidget *parent) :
WizardPage(title, parent),
ui(new Ui::ScanFilePage)
{
	ui->setupUi(this);
    INIT_LINEEDIT_BUTTON(lasDir);
    //ui->lineEdit_datDir->setReadOnly(true);
    ui->lineEdit_lasDir->setReadOnly(true);

    //ui->lineEdit_span->setReadOnly(true);
 
    m_treeFiles = new MultiTreeView(ui->groupBox);
    ui->gridLayout_Group->addWidget(m_treeFiles);
 
    m_treeIcon1 = QIcon(QStringLiteral(":/images/scantype.png"));
    m_treeIcon2 = QIcon(QStringLiteral(":/images/dataItem.png"));

    m_treeFiles->getModel()->setHorizontalHeaderLabels(QStringList() << QStringLiteral("文件") << QStringLiteral("大小"));
    m_treeFiles->setColumnWidth(0, 350);

}

 
ScanFilePage::~ScanFilePage()
{
 
	delete ui;
}
 
bool ScanFilePage::IsValid()
{
    //QString datDirPath = ui->lineEdit_datDir->text();
    //if (datDirPath.isEmpty())
    //{
    //    interfacePrompt::Warn(this, m_titleHint, QStringLiteral("请输入原始数据路径！"));
    //    return false;
    //}

    if (!checkLineEditDir(ui->lineEdit_lasDir, m_titleHint, QStringLiteral("请输入LAS保存路径！")))
    {
        return false;
    }
 
    //QString spanFilePath = ui->lineEdit_span->text();
    //if (spanFilePath.isEmpty())
    //{
    //    interfacePrompt::Warn(this, m_titleHint, QStringLiteral("请输入POS数据路径！"));
    //    return false;
    //}


    if (m_datFileMap.empty())
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("没有原始数据文件！"));
        return false;
    }
    QString strScan = ProjectManage::getInstance()->getScanType();
    const ScannerParaAll* pScanAll = ParamMgr::getInstance()->GetScannerParaAll(strScan);
    if (strScan.isEmpty() || (pScanAll == NULL))
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("扫描仪检校参数不存在，请设置！"));
        return false;
    }
    ScannerFileMapIt it = m_datFileMap.begin();
    //for (; it != m_datFileMap.end(); ++it)
    {
        if (pScanAll->strPlugin != it->first)
        {
            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("扫描仪检校参数%1和原始数据不一致，请重新设置！").arg(strScan));
            return false;
        }
    }

    ScannerFileMap sel;
    GetSelFiles(sel);
    if (sel.empty())
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("请选择文件！"));
        return false;
    }
    return true;
}

void ScanFilePage::SetPageParam(PageParam* param)
{
    ScanParam* dp = static_cast<ScanParam*>(param);
    QString strScan = ProjectManage::getInstance()->getScanType();
    dp->scanPara = ParamMgr::getInstance()->GetScannerParaAll(strScan);

    QString strMb = ProjectManage::getInstance()->getMBType();
    if (!strMb.isEmpty())
    {  
        dp->paraMb = ParamMgr::getInstance()->GetMBPara(strMb);
    }
    else
    {
        dp->paraMb = nullptr;
    }

    //if (sp)
    //{
    //    sp->toArray(dp->para);
    //}
    ScannerFileMap sel;
    GetSelFiles(sel);
    dp->datFileResultMap = sel;
 
    dp->strDatPath = ProjectManage::getInstance()->getDataDirPath();//ui->lineEdit_datDir->text();
    dp->strSpanFilePath = ProjectManage::getInstance()->getSpanFilePath();//ui->lineEdit_span->text();

    dp->strSavePath = ui->lineEdit_lasDir->text();
    dp->strSavePath.replace('\\', '/');
    ///dp->getLasFileMap(false);

}

void ScanFilePage::onDataDirTextChanged(const QString& strDatDir)
{
    if (strDatDir.isEmpty())
    {//关闭工程
        m_datDir.clear();
        m_datFileMap.clear();
        m_treeFiles->getModel()->removeRows(0, m_treeFiles->getModel()->rowCount());
        return;
    }
    //qDebug() << strDatDir;
    //qDebug() << m_datDir;
    if (m_datDir.isEmpty())
    {
        m_datDir = strDatDir;
        m_datDir.replace('\\', '/');
    }
    else 
    {
        QString strDir = strDatDir;
        strDir.replace('\\', '/');
        if (m_datDir == strDir)
        {  
            return;
        }
        m_datDir = strDir;
    }
    getFiles(strDatDir);
 
    // m_treeFiles->reset();
    m_treeFiles->getModel()->removeRows(0, m_treeFiles->getModel()->rowCount());
 
    for (ScannerFileMapIt it = m_datFileMap.begin(); it != m_datFileMap.end(); ++it)
    { 
 
        QStandardItem* scanRoot = new QStandardItem(m_treeIcon1, it->first);
        scanRoot->setCheckable(true);
        scanRoot->setTristate(true);
        scanRoot->setCheckState(Qt::Checked);
        scanRoot->setEditable(false);
        m_treeFiles->getModel()->appendRow(scanRoot);
        m_treeFiles->getModel()->setItem(m_treeFiles->getModel()->indexFromItem(scanRoot).row(), 1, new QStandardItem(QStringLiteral("")));
        const FileMap& fileList = it->second;
        int idx = 0;
        for (FileMapIt itf = fileList.begin(); itf != fileList.end(); ++itf)
        {
            QFileInfo fileInfo(itf->first);
 
            QStandardItem* scanFile = new QStandardItem(m_treeIcon2, fileInfo.fileName());
            scanFile->setCheckable(true);
            scanFile->setCheckState(Qt::Checked);
            scanFile->setData(idx++);//!!
            scanFile->setEditable(false);
            scanRoot->appendRow(scanFile);
            QStandardItem* scanFileSz = new QStandardItem(QStringLiteral("%1M").arg(itf->second / (1024.0*1024.0), 0, 'f', 2));
            scanFileSz->setEditable(false);

            scanRoot->setChild(scanFile->index().row(), 1, scanFileSz);
        }
 
    }
    m_treeFiles->expandAll();
}
 

void ScanFilePage::getFiles(const QString& strDatDir)
{
    m_datFileMap.clear();
    PluginManage::getInstance()->GetAllSupportFiles(strDatDir, m_datFileMap);
}

void ScanFilePage::on_pushButton_lasDir_clicked()
{
    setEditDirPath(ui->lineEdit_lasDir, QStringLiteral("请选择las保存路径"));
}

 

void ScanFilePage::GetSelFiles(ScannerFileMap& selFiles)
{ 
    QStandardItem* item = m_treeFiles->getModel()->invisibleRootItem();
    int rowCount = item->rowCount();

    for (int i = 0; i < rowCount; ++i)
    {
        QStandardItem* scanItems = item->child(i);
        QString strName = scanItems->text();
        ScannerFileMapIt it = m_datFileMap.find(strName);
        FileMap selFileList;
        if (it != m_datFileMap.end())
        {
            int fileCount = scanItems->rowCount();
            const FileMap& fileList = it->second;
            for (int j = 0; j < fileCount; ++j)
            {
                QStandardItem* fileItems = scanItems->child(j);
                if (fileItems->checkState() == Qt::Checked)
                {
                    int idx = fileItems->data().toInt();
                    //FileMapIt itf = fileList.find(strFile);
                    //if (itf != fileList.end())
                    {//有序
                        selFileList.push_back(fileList[idx]);
                    }
                }

            }

        }
        if (selFileList.size() != 0)
        {
            ScannerFileMap::iterator itf = selFiles.find(strName);
            if (itf != selFiles.end())
            {
                itf->second = selFileList ;
            }
            else
            {  
                selFiles.insert(std::make_pair(strName, selFileList));
            }

        }
 

    }
}

bool ScanFilePage::OnPageParam(PageParam* param)
{
    ScanParam* dp = static_cast<ScanParam*>(param);

    //    ui->lineEdit_datDir->setText(dp->strDatPath);
    if (dp->strSavePath.isEmpty())
    {
        dp->strSavePath = ProjectManage::getInstance()->getLasDirPath();
    }
    ui->lineEdit_lasDir->setText(dp->strSavePath);
    if (dp->strDatPath.isEmpty())
    {
        dp->strDatPath = ProjectManage::getInstance()->getDataDirPath();
    }
    //    ui->lineEdit_span->setText(dp->strSpanFilePath);
    onDataDirTextChanged(dp->strDatPath);
    return true;
}
