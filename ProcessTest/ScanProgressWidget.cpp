 
//#include <QPlastiqueStyle>
#include <QMessageBox>
#include <QLayout>

#include "ScanProgressWidget.h"
#include "ui_ProgressWidget.h"
#include <QFileDialog>
#include <QDebug>
#include <QLabel>

#include <QProgressBar>

#include "PluginMgr.h"
 
#include "ProjectMgr.h"
#include "ParamMgr.h"
#include "pubfun.h"
 
#include "para.h"
#include "ScanSingleTransThread.h"

#include "ScanTransMPThread.h"
#include "ScanMbTransThread.h"
#include "ScanParam.h"
#include "interfacePrompt.h"
#include "GlobalOption.h"
 
#include "SimpleViewer.h"
using namespace xscommon;
using namespace pcm;
using namespace xstype;
ScanProgressWidget::ScanProgressWidget(QWidget *parent) :
ProgressWidget(parent), m_posBar(NULL)
{
    m_titleHint = QStringLiteral("��������");
   
}


// ��������
ScanProgressWidget::~ScanProgressWidget()
{
 
}
 

void ScanProgressWidget::Init(PageParam* param)
{
    m_bars.clear();
    ScanParam& dp = *static_cast<ScanParam*>(param);
    ScannerFileMapIt it;
    if (!dp.bSingle)
    {
        posPath = dp.strSpanFilePath;
        createBar(posPath);
    }

    unsigned numf = 0;
    for (it = dp.datFileResultMap.begin(); it != dp.datFileResultMap.end(); it++) //���δ���ÿ��ɨ����
    {
        QString scannerName = it->first;                           // ��ȡɨ��������
        const FileMap& datFileList = it->second;             // ��ȡɨ����dat�ļ��б�
        //const QStringList& lasFileList = param.lasFileMap[it.key()]; // ��ȡlas����ļ��б�
 
        for (FileMapIt itf = datFileList.begin(); itf != datFileList.end(); ++itf )      // ����ÿ��ɨ���ǵ��ļ�
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
    if (dp.bCreateLinFile && !dp.bCreateLasFile)
    {
        m_num *= 2;
    }
    if (!dp.bSingle)
    {
        uint32_t flag = 0;
        double l0 = pcm::INVALID_L0;
        if (dp.bScanMb)
        {
            flag = FieldPara::ProjFlag;
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
            l0 = ppm.L0*DEG_TO_RAD;
        }
        //??
        ((SimpleViewer*)m_posWidget)->addToDB(posFile, m_num, false, true, true, flag, l0);//ʱ���??
    }

    if (dp.dStayDistance < 0.01)
    {
        m_num *= 5;
    }
 

    if (dp.bSingle)
    {
        m_thread = new ScanSingleTransThread();
    }
    else if (dp.bScanMb)
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
     
    InitThread(param);
 
    m_thread->start();
 
}
 
void ScanProgressWidget::onEndFile(const QString& fileName, unsigned nRead, unsigned nWrite)
{
    QString curJob = ProjectManage::getInstance()->getCurJob();
 
    if (fileName.isEmpty())
    {
        if (nRead > 0)
        {   
            emit updateLog(LOG_INFO, QStringLiteral("POS���ݶ�ȡ���"), curJob);
        }
        else
        {
            emit updateLog(LOG_INFO, QStringLiteral("POS���ݶ�ȡ��ֹͣ"), curJob);
        }
        return;
    }
    emit updateLog(LOG_INFO, QStringLiteral("%1���ɽ���������%2���㣬д��%3����").arg(fileName).arg(nRead, 0, 10).arg(nWrite, 0, 10), curJob); 
    if (GlobalOption::Parameters().bProcShowCloud)
    {
        if (nWrite > 0)
        {
            QStringList posFile;
            posFile << fileName;
            //qDebug() << fileName;
            ((SimpleViewer*)m_posWidget)->addToDB(posFile, m_num, false);
        }

    }
}
void ScanProgressWidget::onBeginFile(const QString& fileName)
{
    QString curJob = ProjectManage::getInstance()->getCurJob();
    //qDebug() << fileName;
    getBar(fileName);
    if (m_curBar == nullptr)
    {
        getBar(posPath);
    }
    QString strf = fileName.right(4); strf.prepend("*");//*.txt *.out
    QStringList filters = PageParam::getPosFilters();
    if (!posPath.isEmpty() && (filters.contains(strf)))
    {//txt
        emit updateLog(LOG_INFO, QStringLiteral("��ʼ��ȡPOS���ݣ�") + fileName, curJob);
        return;
    }
    emit updateLog(LOG_INFO, QStringLiteral("��ʼ%1��").arg(m_titleHint) + fileName, curJob);
}
 

void ScanProgressWidget::onEndProcess(int num )
{
    QString curJob = ProjectManage::getInstance()->getCurJob();
    if (num > 0)
    {  
        emit updateLog(LOG_INFO, QStringLiteral("%1����ɣ�һ������%2������").arg(m_titleHint).arg(num), curJob);
    }
    else
    {
        emit updateLog(LOG_INFO, QStringLiteral("%1��ֹͣ").arg(m_titleHint), curJob);
    }

    JobInfo* job = ProjectManage::getInstance()->getJob(curJob);

    if (job && job->bScanTime)
    {//20181120
        QString strtmf = job->getTimeFile();
        if ((job->strAbsSynFilePath != strtmf)
            && QFile::exists(strtmf))
        {
            ProjectManage::getInstance()->setTimeSynFile(strtmf);
        }
    }
    ProgressWidget::onEndProcess(num);
 
}
void ScanProgressWidget::on_pushButton_close_clicked()
{
    if (m_thread && !m_thread->isFinished())
    {
        interfacePrompt::Warn(NULL, m_titleHint, QStringLiteral("����ֹͣ��"));
        return;
    }
 
    ((SimpleViewer*)m_posWidget)->removeAll( );
 
    ProgressWidget::on_pushButton_close_clicked();
}
 