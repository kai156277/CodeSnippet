#include <QApplication>
#include <QDebug>

#include <GlobalOption.h>
#include <OpenCLHelper.h>
#include <ParamMgr.h>
#include <PluginMgr.h>

#include "WizardWidget.h"

#include "ProjectMgr.h"

#include <DataTypeDefs.h>
#include <MultiBeamTransThread.h>
#include <ScanParam.h>
#include <ScanTransThread.h>

using xstype::ScanParam;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ParamMgr::getInstance()->setAppName("ProcessTest");
    OpenCLHelper::check();
    PluginManage::getInstance()->LoadConfig();
    const GlobalOption::ParamStruct &param = GlobalOption::Parameters(ParamMgr::getInstance());
    PluginManage::getInstance()->LoadPlugin(GlobalOption::Parameters().bRxpNew);

    QString param_dir = QApplication::applicationDirPath();
    qDebug() << param_dir;
    qDebug() << ParamMgr::getInstance()->LoadPara(param_dir);

    //    qDebug() << ParamMgr::getInstance()->LoadPara(QApplication::applicationDirPath() + "/scanner.para");
    //    qDebug() << ParamMgr::getInstance()->LoadPara(QApplication::applicationDirPath() + "/multibeam.para");

    qDebug() << "app name:" << ParamMgr::getInstance()->getAppName();
    qDebug() << "MB plugin: " << PluginManage::getInstance()->getMBPluginList();
    qDebug() << "Scan plugin: " << PluginManage::getInstance()->getScanPluginList();
    qDebug() << "Photo plugin: " << PluginManage::getInstance()->getPhotoPluginList();
    qDebug() << "scan: " << ParamMgr::getInstance()->GetScannerList();
    qDebug() << "mb: " << ParamMgr::getInstance()->GetMBList();

    //    ScanTransThread thread;
    ScanParam *dp = new ScanParam;
    //    ProjectManage::getInstance()->getScanType();
    QString strScan = "VSurs-W-VZ2000-20190316";
    dp->scanPara    = ParamMgr::getInstance()->GetScannerParaAll(strScan);
    qDebug() << "x:" << dp->scanPara->scannerOne.lx;

    QString strMb = "mb-0320";
    if (!strMb.isEmpty())
    {
        dp->paraMb = ParamMgr::getInstance()->GetMBPara(strMb);
    }
    else
    {
        dp->paraMb = nullptr;
    }

    if (sp)
    {
        sp->toArray(dp->para);
    }
    //    ScannerFileMap sel;
    //    GetSelFiles(sel);
    //    dp->datFileResultMap = sel;

    //    dp->strDatPath      = ProjectManage::getInstance()->getDataDirPath();    //ui->lineEdit_datDir->text();
    //    dp->strSpanFilePath = ProjectManage::getInstance()->getSpanFilePath();   //ui->lineEdit_span->text();

    //    //    dp->strSavePath = ui->lineEdit_lasDir->text();
    //    dp->strSavePath.replace('\\', '/');
    //    ///dp->getLasFileMap(false);

    //    ScanParam &dp = *static_cast<ScanParam *>(param);

    //    unsigned numf = 0;
    //    for (auto it = dp.datFileResultMap.begin(); it != dp.datFileResultMap.end(); it++)   //依次处理每个扫描仪
    //    {
    //        QString                scannerName = it->first;    // 获取扫描仪类型
    //        const xstype::FileMap &datFileList = it->second;   // 获取扫描仪dat文件列表
    //        //const QStringList& lasFileList = param.lasFileMap[it.key()]; // 获取las结果文件列表

    //        for (auto itf = datFileList.begin(); itf != datFileList.end(); ++itf)   // 遍历每个扫描仪的文件
    //        {
    //            numf++;
    //            //            createBar(itf->first);
    //        }
    //    }

    //    QStringList posFile;
    //    posFile << dp.strSpanFilePath;
    //    numf++;
    //    m_num = 100;
    //    if (numf > 60)
    //    {
    //        m_num = int((float(numf) / 60.0f) * 100.0f);
    //    }
    //    const GlobalOption::ParamStruct &opt = GlobalOption::Parameters();
    //    if (opt.bLasGpu)
    //    {
    //        m_num *= 2;
    //    }
    //    if (dp.bCreateLinFile && !dp.bCreateLasFile)
    //    {
    //        m_num *= 2;
    //    }
    //    if (!dp.bSingle)
    //    {
    //        uint32_t flag = 0;
    //        double   l0   = pcm::INVALID_L0;
    //        if (dp.bScanMb)
    //        {
    //            flag                 = FieldPara::ProjFlag;
    //            ProjParams4Json &ppm = dp.projPara.projParam;
    //            if (ppm.projMethod == Project::UTM)
    //            {
    //                flag |= FieldPara::UtmFlag;
    //            }

    //            if (ppm.hemisphere == Position::ES || ppm.hemisphere == Position::WS)
    //            {
    //                flag |= FieldPara::EarthSFlag;
    //            }

    //            if (ppm.hemisphere == Position::ES || ppm.hemisphere == Position::EN)
    //            {
    //                flag |= FieldPara::EarthEFlag;
    //            }
    //            l0 = ppm.L0 * DEG_TO_RAD;
    //        }
    //        //??
    //        ((SimpleViewer *) m_posWidget)->addToDB(posFile, m_num, false, true, true, flag, l0);   //时间段??
    //    }

    //    if (dp.dStayDistance < 0.01)
    //    {
    //        m_num *= 5;
    //    }

    //    if (dp.bSingle)
    //    {
    //        m_thread = new ScanSingleTransThread();
    //    }
    //    else if (dp.bScanMb)
    //    {
    //        if (opt.bLasGpu)
    //        {
    //            m_thread = new ScanMbTransThread();
    //        }
    //        else
    //        {
    //            m_thread = new ScanMbTransMPThread();
    //        }
    //    }
    //    else
    //    {
    //        if (opt.bLasGpu)
    //        {
    //            m_thread = new ScanTransThread();
    //        }
    //        else
    //        {
    //            m_thread = new ScanTransMPThread();
    //        }
    //    }

    //    InitThread(param);

    //    m_thread->start();

    //    WizardWidget wizard;
    //    wizard.show();

    //    return a.exec();
    return 0;
}
