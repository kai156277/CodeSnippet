#include <QApplication>
#include <QDebug>
#include <QString>

#include <GlobalOption.h>
#include <OpenCLHelper.h>
#include <ParamMgr.h>
#include <PluginMgr.h>

#include "WizardWidget.h"

#include "ProjectMgr.h"

#include <DataTypeDefs.h>
#include <MultiBeamTransThread.h>
#include <ScanMbTransThread.h>
#include <ScanParam.h>
#include <ScanSingleTransThread.h>
#include <ScanTransThread.h>

#include <map>
#include <utility>

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

    ProcessThread *m_thread = nullptr;
    //    ScanTransThread thread;
    xstype::ScanParam *dp = new xstype::ScanParam;
    qDebug() << "build scan param";
    ProjectManage::getInstance()->getScanType();
    QString strScan = "VSurs-W-VZ2000-20190316";
    dp->bSingle     = false;
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

    xstype::ScannerFileMap sel;
    xstype::FileMap        file_map;
    file_map.push_back(std::make_pair("C:\\Users\\zhao\\Documents\\Data\\seabat7125\\8.7_rxp_data\\20210807_0918.rxp", 411.55 * 1024 * 1024));
    sel["RIEGL"]         = file_map;
    dp->datFileResultMap = sel;

    dp->strDatPath      = "C:\\Users\\zhao\\Documents\\Data\\seabat7125\\8.7_rxp_data";                    //ui->lineEdit_datDir->text();
    dp->strSpanFilePath = "C:\\Users\\zhao\\Documents\\Data\\seabat7125\\hyrdrins\\ie_20210807_000.txt";   //ui->lineEdit_span->text();

    dp->strSavePath = "C:\\Users\\zhao\\Documents\\Data\\seabat7125";
    dp->strSavePath.replace('\\', '/');
    dp->getLasFileMap(true);
    for (auto file : dp->lasFileMap)
    {
        qDebug() << file.first;
        qDebug() << file.second;
    }

    QStringList posFile;
    posFile << dp->strSpanFilePath;
    const GlobalOption::ParamStruct &opt = GlobalOption::Parameters();

    dp->bUseBeijingTime = false;
    dp->dMaxAngle       = 360 * DEG_TO_RAD;
    dp->dMinAngle       = 0 * DEG_TO_RAD;
    dp->dMaxRadius      = 200;   // 极径过滤
    dp->dMinRadius      = 0;
    dp->uMaxIntensity   = 255;
    dp->uMinIntensity   = 0;
    dp->dMaxTime        = 86400;   // s
    dp->dMinTime        = 0;

    dp->bSkipPause = false;
    dp->speedPause = 0.01;
    if (dp->speedPause == 0.0)
    {
        dp->speedPause = FLT_EPSILON;
    }
    //    dp->dMaxHorRadius      = GetMaxHorDistance();
    //    dp->dMinHorRadius      = GetMinHorDistance();
    //    dp->bHorDist           = ui->checkBox_horDist->isChecked();
    dp->nLineThinFrequency = 1;      // 扫描线抽稀条数
    dp->dStayDistance      = 0.05;   // 抽稀点间距
    //dp->nStaySearchPointNum = ui->spinBox_lasStayPointSearchNum->value();

    dp->filterSetting.bBadLineCheck = true;   // 坏线过滤
    dp->filterSetting.bEdgeCheck    = true;   // 边缘过滤
    if (true)
    {
        dp->filterSetting.bSingleCheck = true;    // 单点过滤
        dp->filterSetting.singleDist   = 10.00;   // 极径因子
        dp->filterSetting.singleNum    = 2;       // 点数
    }
    else
    {
        dp->filterSetting.bSingleCheck = false;
    }

    if (true)
    {
        dp->filterSetting.bRefCheck = true;
        dp->filterSetting.minRef    = -20;
        dp->filterSetting.maxRef    = 100;
    }
    else
    {
        dp->filterSetting.bRefCheck = false;
    }

    if (dp->bSingle)
    {
        m_thread = new ScanSingleTransThread();
    }
    else if (dp->bScanMb)
    {
        if (opt.bLasGpu)
        {
            m_thread = new ScanMbTransThread();
        }
        else
        {
            m_thread = new ScanMbTransMPThread();
        }
    }
    else
    {
        if (opt.bLasGpu)
        {
            m_thread = new ScanTransThread();
        }
        else
        {
            m_thread = new ScanTransMPThread();
        }
    }

    QObject::connect(m_thread, &ScanTransThread::updateValue, [](int value) {
        qDebug() << "process value: " << value;
    });

    QObject::connect(m_thread, &ProcessThread::endFile, [](QString file_name, unsigned a, unsigned b) {
        qDebug() << "end file: " << file_name << "a: " << a << "b: " << b;
    });

    QObject::connect(m_thread, &ProcessThread::beginFile, [](QString file_name) {
        qDebug() << "begin file: " << file_name;
    });

    QObject::connect(m_thread, &ProcessThread::messageBox, [](const QString &text, int err, const QString &a) {
        qDebug() << "text: " << text << "err: " << err << "a: " << a;
    });

    m_thread->Init(dp);
    m_thread->start();

    WizardWidget wizard;
    wizard.show();

    return a.exec();
}
