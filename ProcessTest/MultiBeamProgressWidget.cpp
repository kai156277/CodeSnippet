 
//#include <QPlastiqueStyle>
#include <QMessageBox>
#include <QLayout>

#include "MultiBeamProgressWidget.h"
#include "ui_ProgressWidget.h"
#include <QFileDialog>
#include <QDebug>
#include <QLabel>

#include <QProgressBar>

#include "PluginMgr.h"
 
#include "ProjectMgr.h"

#include "pubfun.h"
 
#include "para.h"
#include "MultiBeamTransThread.h"
#include "ScanParam.h"
#include "interfacePrompt.h"
#include "GlobalOption.h"
 
#include "SimpleViewer.h"
using namespace xscommon;
using namespace pcm;
using namespace xstype;

MultiBeamProgressWidget::MultiBeamProgressWidget(QWidget *parent) :
ScanProgressWidget(parent)
{
    m_titleHint = QStringLiteral("�ನ����������");
}


// ��������
MultiBeamProgressWidget::~MultiBeamProgressWidget()
{
 
}
 

void MultiBeamProgressWidget::Init(PageParam* param)
{
    m_bars.clear();
    ScanParam& dp = *static_cast<ScanParam*>(param);
    ScannerFileMapIt it;
    createBar(dp.strSpanFilePath);
    posPath = dp.strSpanFilePath;
    unsigned numf = 0;
    for (it = dp.datFileResultMap.begin(); it != dp.datFileResultMap.end(); it++) //���δ���ÿ��ɨ����
    {
        QString scannerName = it->first;                           // ��ȡɨ��������
        const FileMap& datFileList = it->second;             // ��ȡɨ����dat�ļ��б�
        //const QStringList& lasFileList = param.lasFileMap[it.key()]; // ��ȡlas����ļ��б�

        for (FileMapIt itf = datFileList.begin(); itf != datFileList.end(); ++itf)      // ����ÿ��ɨ���ǵ��ļ�
        {
            numf++;
            createBar(itf->first);
        }
    }

    QStringList posFile;
    posFile << dp.strSpanFilePath;
    numf++;
    m_num = 100;
    if (numf > 60)
    {
        m_num = int((float(numf) / 60.0f)*100.0f);
    }
    const GlobalOption::ParamStruct& opt = GlobalOption::Parameters();
    if (opt.bLasGpu)
    {
        m_num *= 2;
    }
 
    uint32_t flag = FieldPara::ProjFlag;
    ProjParams4Json& ppm = dp.projPara.projParam;
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
    ((SimpleViewer*)m_posWidget)->addToDB(posFile, m_num, false, true, true, flag, ppm.L0*DEG_TO_RAD);//ʱ���??
  
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
 