 
//#include <QPlastiqueStyle>
#include <QLayout>
#include <QMessageBox>
#include <QDebug>
#include <QDir>
#include <memory>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "ScanCheckPage.h"
#include "ui_CheckPage.h"
#include "interfacePrompt.h"
#include "PageParam.h"
#include "pubfun.h"
#include "ScanParam.h"
#include "PluginMgr.h"
#include "ParamMgr.h"
#include "cJSON.h"
#include "ProjectMgr.h"
#include "GlobalOption.h"
#include "ProcessObject.h"
using namespace xscommon;
using namespace xstype;
using namespace xsproc; 

void ScanCheckPage::PrintPageParam(PageParam* param)
{
    m_strContent.clear();
    ScanParam* dp = static_cast<ScanParam*>(param);
    dp->bFaroPrepare = GlobalOption::Parameters().bFaroPrepare;
    QString tmpStr = dp->printParaNoFile(m_bMultiBeam);

    m_strContent.append(tmpStr);
    ui->textEdit->append(tmpStr);

    ScannerFileMapIt it;

    for (it = dp->datFileResultMap.begin(); it != dp->datFileResultMap.end(); it++) //依次处理每个扫描仪
    {
        QString scannerName = it->first;                           // 获取扫描仪类型
        const FileMap& datFileList = it->second;             // 获取扫描仪dat文件列表
        const QStringList& lasFileList = dp->lasFileMap[it->first]; // 获取las结果文件列表
        QString strScan = m_bMultiBeam ? ProjectManage::getInstance()->getMBType() : ProjectManage::getInstance()->getScanType();
        tmpStr = "<table border=\"1\"  width=\"100%\">";
        if (!dp->bSingle)
        {
            tmpStr.append(QStringLiteral("<tr><th >%1参数</th> <th >%2</th> </tr>").
                arg(m_bMultiBeam ? QStringLiteral("多波束"):QStringLiteral("扫描仪")).arg(strScan));
 
            const ScannerPara* pValues = m_bMultiBeam ? ParamMgr::getInstance()->GetMBPara(strScan) : ParamMgr::getInstance()->GetScannerPara(strScan);
            pValues->printScannerPara(tmpStr, m_bMultiBeam);
	
        }
        tmpStr.append(QStringLiteral("<tr><th >原始数据</th> <th >结果文件</th> </tr>"));
        //tmpStr.append(QStringLiteral("<tr><td width=\"50%\"><table border=\"0\" >"));

        IPlugin* plugin = NULL;
        int iFile = 0;
        QString datDirName = dp->strDatPath;// + "/" + ;
        QString lasDirName = dp->strSavePath + "/" + dp->getSubDir(m_bMultiBeam);

        QString allFileName;
        QString allLasName;

        tmpStr.append(QStringLiteral("<tr><td  width=\"50%\">%1</td><td>%2</td></tr>").arg(datDirName).arg(lasDirName));
        for (FileMapIt itf = datFileList.begin(); itf != datFileList.end(); ++itf, ++iFile)      // 遍历每个扫描仪的文件
        {

            QString sickFileName = itf->first;
            QFileInfo fileinfo(sickFileName);
            QString fileExt = fileinfo.suffix().toLower();
            plugin = m_bMultiBeam ? PluginManage::getInstance()->GetMBPluginByExt(fileExt) : PluginManage::getInstance()->GetScanPluginByExt(fileExt);
            if (!plugin)
            {
                continue;
            }
            uint32_t pointNum1 = itf->second * 0.01 * plugin->pGetPointByteFunc(fileExt.toStdWString().c_str());
            bool bNeedPart = (plugin->pIsNeedPartFunc() == 1) || (pointNum1 * 100 > ProcessObject::PACKAGE_ONE_MAX_SIZE); // 1.9/2 亿
            QString las84File = lasFileList.at(iFile);
            las84File.remove(0, lasDirName.length() + 1);
            if (dp->bLaz)
            {
                las84File[las84File.size() - 1] = 'z';
            }
            QString linFile = las84File;
            if (dp->bCreateLinFile)
            {
                linFile.remove(linFile.size() - 3, 3);
                linFile.append(dp->bBnz ? "bnz" : "bin");//  "lin"
            }

            QString mileFile = las84File;
            mileFile.remove(mileFile.size() - 4, 4);

            if (bNeedPart)
            {
                las84File.insert(las84File.length() - 4, "_000");    
                linFile.insert(linFile.length() - 4, "_000");
                mileFile.append("_000");
            }
            mileFile.append("mile.txt");

            if (dp->bCreateLasFile && dp->bCreateLinFile)
            {//las 和 lin
                las84File.append(" <br/>");
                las84File.append(linFile);
 
            }
            else if (!dp->bCreateLasFile)
            {//只有lin
                las84File = linFile;
            }

            las84File.append(" <br/>");
            las84File.append(mileFile);
            if (bNeedPart)
            {
                las84File.append(" <br/>...<br/>");
            }
            sickFileName.replace('\\', '/');
            sickFileName.remove(0, datDirName.length() + 1);
 
            allFileName.append(sickFileName).append(" <br/>");
            allLasName.append(las84File).append(" <br/>");

        }  
        tmpStr.append(QStringLiteral("<tr><td >%1</td><td>%2</td></tr>").arg(allFileName).arg(allLasName));

        tmpStr.append("</table>");

        ui->textEdit->append(tmpStr);
        m_strContent.append(tmpStr);
    }
    float allSize = dp->getScanSize(m_bMultiBeam);;
    tmpStr = "<table border=\"1\"  width=\"100%\">";
    if (allSize > 1024.0f)
    {
        tmpStr.append(QStringLiteral("<tr><td  width=\"50%\">结果文件大小估计</td><td>%1GB</td></tr>").arg(allSize / 1024.0f, 0, 'f', 3));
    }
    else
    {
        tmpStr.append(QStringLiteral("<tr><td  width=\"50%\">结果文件大小估计</td><td>%1MB</td></tr>").arg(allSize, 0, 'f', 0));
    }
    tmpStr.append("</table>");

    ui->textEdit->append(tmpStr);
    m_strContent.append(tmpStr);

    float disk = getFreeSpace(dp->strSavePath[0].toLatin1());
    if (disk < allSize)
    {
        tmpStr = QStringLiteral("【Warn】结果目录磁盘剩余空间可能不足！");
        m_bNoSpace = true;
        ui->textEdit->append(tmpStr);
        m_strContent.append(tmpStr);
    }

}

ScanCheckPage::ScanCheckPage(const QString& title, bool mb, QWidget *parent /*= 0*/) :CheckPage(title, parent)
{
    m_bMultiBeam = mb;

}

ScanCheckPage::~ScanCheckPage()
{

}


