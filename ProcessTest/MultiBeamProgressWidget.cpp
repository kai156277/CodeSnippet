
//#include <QPlastiqueStyle>
#include <QLayout>
#include <QMessageBox>

#include "MultiBeamProgressWidget.h"
#include "ui_ProgressWidget.h"
#include <QDebug>
#include <QFileDialog>
#include <QLabel>

#include <QProgressBar>

#include "PluginMgr.h"

#include "ProjectMgr.h"

#include "pubfun.h"

#include "GlobalOption.h"
#include "MultiBeamTransThread.h"
#include "ScanParam.h"
#include "interfacePrompt.h"
#include "para.h"

#include "SimpleViewer.h"
using namespace xscommon;
using namespace pcm;
using namespace xstype;

MultiBeamProgressWidget::MultiBeamProgressWidget(QWidget *parent)
    : ScanProgressWidget(parent)
{
    m_titleHint = QStringLiteral("多波束点云生成");
}

// 析构函数
MultiBeamProgressWidget::~MultiBeamProgressWidget()
{
}

void MultiBeamProgressWidget::Init(PageParam *param)
{
    //    m_bars.clear();
    ScanParam &      dp = *static_cast<ScanParam *>(param);
    ScannerFileMapIt it;
    //    createBar(dp.strSpanFilePath);
    posPath       = dp.strSpanFilePath;
    unsigned numf = 0;
    for (it = dp.datFileResultMap.begin(); it != dp.datFileResultMap.end(); it++)   //依次处理每个扫描仪
    {
        QString        scannerName = it->first;    // 获取扫描仪类型
        const FileMap &datFileList = it->second;   // 获取扫描仪dat文件列表
        //const QStringList& lasFileList = param.lasFileMap[it.key()]; // 获取las结果文件列表

        for (FileMapIt itf = datFileList.begin(); itf != datFileList.end(); ++itf)   // 遍历每个扫描仪的文件
        {
            numf++;
            //            createBar(itf->first);
        }
    }

    QStringList posFile;
    posFile << dp.strSpanFilePath;
    numf++;
    m_num = 100;
    if (numf > 60)
    {
        m_num = int((float(numf) / 60.0f) * 100.0f);
    }
    const GlobalOption::ParamStruct &opt = GlobalOption::Parameters();
    if (opt.bLasGpu)
    {
        m_num *= 2;
    }

    uint32_t         flag = FieldPara::ProjFlag;
    ProjParams4Json &ppm  = dp.projPara.projParam;
    if (ppm.projMethod == Project::UTM)
    {
        flag |= FieldPara::UtmFlag;
    }

    if (ppm.hemisphere == Position::ES || ppm.hemisphere == Position::WS)
    {
        flag |= FieldPara::EarthSFlag;
    }

    if (ppm.hemisphere == Position::ES || ppm.hemisphere == Position::EN)
    {
        flag |= FieldPara::EarthEFlag;
    }
    //??
    //    ((SimpleViewer *) m_posWidget)->addToDB(posFile, m_num, false, true, true, flag, ppm.L0 * DEG_TO_RAD);   //时间段??

    if (opt.bLasGpu)
    {
        m_thread = new MultiBeamTransThread();
    }
    else
    {
        m_thread = new MultiBeamTransMPThread();
    }

    InitThread(param);

    m_thread->start();
}
