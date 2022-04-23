/*
**********************************************************************************************
* @Copyright(C):青岛秀山移动测量有限公司

* @File Name:ScanFilterPage.h

* @Author:朱淑红

* @Version:1.1

* @Date:2012.7.19

* @Description: 该类是数据过滤条件的界面类。
**********************************************************************************************
*/

//#include <QPlastiqueStyle>
#include <QMessageBox>
#include <QLayout>
#include <assert.h>

#include "ScanFilterPage.h"
#include "ui_ScanFilterPage.h"
#include <QFileDialog>
#include <QDebug>
#include "ScanParam.h"
#include "pubfun.h"
#include "ProjectMgr.h"
#include "interfacePrompt.h"
#include "PluginMgr.h"
#include "ParamMgr.h"
#include "PbrFile.h"
using namespace xscommon;
using namespace xstype;
using namespace xsplugin;
using namespace pcm;
ScanFilterPage::ScanFilterPage(const QString& title, bool bmb, QWidget *parent) :
WizardPage(title, parent), m_bMb(bmb),
ui(new Ui::ScanFilterPage)
{
    ui->setupUi(this);
    ui->label_lasStayPointSearchNum->setVisible(false);
    ui->spinBox_lasStayPointSearchNum->setVisible(false);

   
    for (int i = 0; i < 3; i++)
    {
        m_validator[i] = new QRegExpValidator(this);
    }

    QRegExp rx;
    rx.setPattern(getPositiveNumberPattern(5));                // 输入格式为：...xxx.xxx...
    m_validator[0]->setRegExp(rx);
    //ui->minRangeOfMILE->setValidator(m_validator[0]);
    //ui->maxRangeOfMILE->setValidator(m_validator[0]);
    ui->minRangeOfDist->setValidator(m_validator[0]);//{1,4}
    ui->maxRangeOfDist->setValidator(m_validator[0]);//{1,4}
    ui->minRangeOfTime->setValidator(m_validator[0]);
    ui->maxRangeOfTime->setValidator(m_validator[0]);

    rx.setPattern(getNumberPattern(3));       // 输入格式为：(-)xxx.xxx...
    m_validator[1]->setRegExp(rx);
    ui->minRangeOfAngle->setValidator(m_validator[1]);
    ui->maxRangeOfAngle->setValidator(m_validator[1]);
    ui->lineEdit_minRef->setValidator(m_validator[1]);
    ui->lineEdit_maxRef->setValidator(m_validator[1]);

    rx.setPattern("[0-9]{1,3}");                                // 输入格式为：xxx
    m_validator[2]->setRegExp(rx);
    ui->minRangeOfIntensity->setValidator(m_validator[2]);
    ui->maxRangeOfIntensity->setValidator(m_validator[2]);
    
    ui->radio_RelativeTime->setChecked(true);
    onRadioToggled(true);
    connect(ui->radio_BeijingTime, &QAbstractButton::toggled,
        this, &ScanFilterPage::onRadioToggled);      // 
    connect(ui->radio_RelativeTime, &QAbstractButton::toggled,
        this, &ScanFilterPage::onRadioToggled);      // 

    //on_checkBox_skipPause_toggled(false);
}


// 析构函数
ScanFilterPage::~ScanFilterPage()
{
    for (int i = 0; i < 3; i++)
    {
        delete m_validator[i];
    }
    delete ui;
}
 

// 检查里程参数
// 返回值：true，设置正确；false，设置错误
bool ScanFilterPage::checkRef()
{
    if (ui->checkBox_ref->isChecked())
    {

        if (ui->lineEdit_minRef->text().isEmpty() ||
            ui->lineEdit_maxRef->text().isEmpty())
        {
            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("请设置反射系数过滤范围！"));
            return false;
        }

        if (GetMinRef() > GetMaxRef())
        {
            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("反射系数过滤条件不正确，请重新设置！"));
            return false;
        }
    }
    return true;
}


// 检查极径参数
// 返回值：true，设置正确；false，设置错误
bool ScanFilterPage::checkDistance()
{
    if (ui->minRangeOfDist->text().isEmpty() ||
        ui->maxRangeOfDist->text().isEmpty())
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("请设置极径过滤范围"));
        return false;
    }

    if (GetMinDistance() > GetMaxDistance())
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("极径过滤条件不正确，请重新设置"));
        return false;
    }
    if (ui->checkBox_horDist->isChecked() && GetMinHorDistance() > GetMaxHorDistance())
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("水平极径过滤条件不正确，请重新设置"));
        return false;
    }

    return true;
}


// 检查极角参数
// 返回值：true，设置正确；false，设置错误
bool ScanFilterPage::checkAngle()
{
    if (ui->minRangeOfAngle->text().isEmpty() ||
        ui->maxRangeOfAngle->text().isEmpty())
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("请设置极角过滤范围"));
        return false;
    }

    if (GetMinAngle() > GetMaxAngle())
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("极角过滤条件不正确，请重新设置"));
        return false;
    }

    return true;
}

// 检查反射强度参数
// 返回值：true，设置正确；false，设置错误
bool ScanFilterPage::checkIntensity()
{
    if (ui->minRangeOfIntensity->text().isEmpty() ||
        ui->maxRangeOfIntensity->text().isEmpty())
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("请设置反射强度过滤范围"));
        return false;
    }

    if (GetMinIntensity() > GetMaxIntensity())
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("反射强度过滤条件不正确，请重新设置"));
        return false;
    }

    return true;
}
bool ScanFilterPage::checkTime()
{

    if (ui->radio_BeijingTime->isChecked())                            // 判断时间过滤类型
    {
        assert(m_absTimes.size() == ui->tableWidget->rowCount());
        //m_absTimes.clear();
        if ( ui->tableWidget->rowCount() != 0 )
        {//导入配置
            //for (int row = 0; row < ui->tableWidget->rowCount(); ++row)
            //{
            //    CriterionData cd;
            //    cd.dMin = ui->tableWidget->item(row, 0)->data(Qt::UserRole + 1).toDouble();
            //    cd.dMax = ui->tableWidget->item(row, 2)->data(Qt::UserRole + 1).toDouble();
            //    m_absTimes.push_back(cd);      
            //}
        }
 

        if (m_absTimes.empty())
        {
            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("请添加绝对时间过滤范围"));
            return false;
        }
    }
    else
    {
        if (ui->minRangeOfTime->text().isEmpty() ||
            ui->maxRangeOfTime->text().isEmpty())
        {
            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("请设置时间过滤范围"));
            return false;
        }
        double dmin = GetMinTime();
        if (dmin >= GetMaxTime())
        {
            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("相对时间范围错误！"));
            return false;
        }
        double mintm, maxtm;
        //if (!dp->bUseBeijingTime)
        {
            ProjectManage::getInstance()->getCurJobTime(mintm, maxtm);
            maxtm = ceil(maxtm);
            mintm = floor(mintm);
        }
        if (dmin > 0 && mintm > 0 && maxtm > 0)
        {
            if (dmin >= (maxtm - mintm))
            {
                interfacePrompt::Warn(this, m_titleHint, QStringLiteral("相对时间起始值大于POS时间范围！"));
                return false;
            }
        }

    }

    return true;


}


// 根据时间过滤条件，获取最小时间
double ScanFilterPage::GetMinTime()
{
    double minTime(0.0);

    if (IsUseBeijingTime())                                                   // 北京时间h,m,s
    {//
        QTime mtime = ui->timeEdit_minTime->time();
        minTime = mtime.hour() * 3600.0 + mtime.minute() * 60.0 + mtime.second() + mtime.msec() / 1000.0;
    }
    else                                                                  // 扫描仪时间
    {
        minTime = ui->minRangeOfTime->text().toDouble();
    }

    return minTime;
}


// 根据时间过滤条件，获取最大时间
double ScanFilterPage::GetMaxTime()
{
    double maxTime(0.0);

    if (IsUseBeijingTime())                                                   // 绝对时间 北京时间
    {//
        QTime mtime = ui->timeEdit_maxTime->time();
        maxTime = mtime.hour() * 3600.0 + mtime.minute() * 60.0 + mtime.second() + mtime.msec() / 1000.0;
    }
    else                                                                  // 相对时间 扫描仪时间 
    {
        maxTime = ui->maxRangeOfTime->text().toDouble();
    }

    return maxTime;
}



// 获取最小极径
double ScanFilterPage::GetMinDistance()
{
    return ui->minRangeOfDist->text().toDouble();
}


// 获取最大极径
double ScanFilterPage::GetMaxDistance()
{
    return ui->maxRangeOfDist->text().toDouble();
}

// 获取最小极径
double ScanFilterPage::GetMinHorDistance()
{
    return ui->minRangeOfHorDist->text().toDouble();
}


// 获取最大极径
double ScanFilterPage::GetMaxHorDistance()
{
    return ui->maxRangeOfHorDist->text().toDouble();
}

// 获取最小极角,刘如飞修改
double ScanFilterPage::GetMinAngle()
{
    double mina = ui->minRangeOfAngle->text().toDouble();
    return mina*DEG_TO_RAD;
}


// 获取最大极角
double ScanFilterPage::GetMaxAngle()
{
    double maxa = ui->maxRangeOfAngle->text().toDouble();
    return maxa*DEG_TO_RAD;
}


// 获取最小反射强度
ushort ScanFilterPage::GetMinIntensity()
{
    return ui->minRangeOfIntensity->text().toUShort();
}


// 获取最大反射强度
ushort ScanFilterPage::GetMaxIntensity()
{
    return ui->maxRangeOfIntensity->text().toUShort();
}


// 获取最小里程
double ScanFilterPage::GetMinRef()
{
    return ui->lineEdit_minRef->text().toDouble();
}


// 获取最大里程
double ScanFilterPage::GetMaxRef()
{
    return ui->lineEdit_maxRef->text().toDouble();
}


// 判断是否采用GPS时间
// 返回值：true，采用GPS时间；false，采用扫描仪时间
bool ScanFilterPage::IsUseBeijingTime()
{
    return ui->radio_BeijingTime->isChecked();
}
void ScanFilterPage::on_checkBox_skipPause_toggled(bool bChecked)
{
    ui->doubleSpinBox_speed->setEnabled(bChecked);
}
void ScanFilterPage::on_checkBox_horDist_toggled(bool bChecked)
{
    ui->minRangeOfHorDist->setEnabled(bChecked);
    ui->maxRangeOfHorDist->setEnabled(bChecked);
}
void ScanFilterPage::on_checkBox_single_toggled(bool bChecked)
{
 
    //ui->label_singleN->setEnabled(bChecked);
    ui->spinBox_singleN->setEnabled(bChecked);
 
    //ui->label_singleD->setEnabled(bChecked);
    ui->doubleSpinBox_singleD->setEnabled(bChecked);

}
void ScanFilterPage::on_checkBox_ref_toggled(bool bChecked)
{

    ui->lineEdit_minRef->setEnabled(bChecked);
    ui->lineEdit_maxRef->setEnabled(bChecked);

    //ui->label_refl->setEnabled(bChecked);
    //ui->label_refn->setEnabled(bChecked);
    //ui->label_refr->setEnabled(bChecked);
    //ui->label_refu->setEnabled(bChecked);


}
void ScanFilterPage::SetPageParam(PageParam* param)
{
    ScanParam* dp = static_cast<ScanParam*>(param);
    dp->bUseBeijingTime = IsUseBeijingTime();
    dp->dMaxAngle = GetMaxAngle();
    dp->dMinAngle = GetMinAngle();
    dp->dMaxRadius = GetMaxDistance();
    dp->dMinRadius = GetMinDistance();
    dp->uMaxIntensity = GetMaxIntensity();
    dp->uMinIntensity = GetMinIntensity();
    dp->dMaxTime = GetMaxTime();
    dp->dMinTime = GetMinTime();


        double mintm, maxtm;
        //if (!dp->bUseBeijingTime)
        {
            ProjectManage::getInstance()->getCurJobTime(mintm, maxtm);
            maxtm = ceil(maxtm);
            mintm = floor(mintm);
        }

    if (dp->bUseBeijingTime)
    {   
        dp->bTimeSplit = ui->checkBox_timeSplit->isChecked();
        std::sort(m_absTimes.begin(), m_absTimes.end());
        dp->absTimeVec = m_absTimes;
        dp->dMinTime += ParamInterface::getTimeShift();
        dp->dMaxTime += ParamInterface::getTimeShift();
        if (dp->dMinTime < 0)
        {
            dp->dMinTime += ONE_DAY_SECOND;
            dp->dMaxTime += ONE_DAY_SECOND;
        }
 
        if (dp->dMaxTime < mintm)
        {
            dp->dMinTime += ONE_DAY_SECOND;
            dp->dMaxTime += ONE_DAY_SECOND;
        }
    }
    else
    {

    }
    if (!dp->strJsonFilter.empty() && dp->strFilterCheck.isEmpty())
    {
        char strJson[1024];
        int len = 1024;
        wchar_t strReadable[1024];
        int nReadable = 1024;
        IPlugin* plugin = nullptr;
        if (m_bMb)
        {
            const MultiBeamParaAll* scanAll = ParamMgr::getInstance()->GetMBParaAll(ProjectManage::getInstance()->getMBType());
            if (scanAll)//mb??
            {
                plugin = PluginManage::getInstance()->GetMBPlugin(scanAll->strPlugin);
            }
        }
        else
        {
           const ScannerParaAll* scanAll = ParamMgr::getInstance()->GetScannerParaAll(ProjectManage::getInstance()->getScanType());
            if (scanAll)//mb??
            {
                plugin = PluginManage::getInstance()->GetScanPlugin(scanAll->strPlugin);
            }
        }
 
            if (plugin)
            {
                strcpy_s(strJson, 1024, dp->strJsonFilter.c_str());
                int res = plugin->pGetFilterSettingFunc(NULL, strJson, &len, strReadable, &nReadable);
                if (res == QDialog::Accepted)
                {
                    dp->strFilterCheck = QString::fromWCharArray(strReadable);
                }
            }
        
    }

    dp->bSkipPause = ui->checkBox_skipPause->isChecked();
    dp->speedPause = ui->doubleSpinBox_speed->value();
    if (dp->speedPause == 0.0)
    {
        dp->speedPause = FLT_EPSILON;
    }
    dp->dMaxHorRadius = GetMaxHorDistance();
    dp->dMinHorRadius = GetMinHorDistance();
    dp->bHorDist = ui->checkBox_horDist->isChecked();
    dp->nLineThinFrequency = ui->spinBox_lasStayLineNum->value();
    dp->dStayDistance = ui->doubleSpinBox_lasPointStayDistance->value();
    //dp->nStaySearchPointNum = ui->spinBox_lasStayPointSearchNum->value();

    dp->filterSetting.bBadLineCheck = ui->checkBox_badLine->isChecked();
    dp->filterSetting.bEdgeCheck = ui->checkBox_edge->isChecked();
    if (ui->checkBox_single->isChecked())
    {
        dp->filterSetting.bSingleCheck = true;
        dp->filterSetting.singleDist = ui->doubleSpinBox_singleD->value();
        dp->filterSetting.singleNum = ui->spinBox_singleN->value();
 
    }
    else
    {
        dp->filterSetting.bSingleCheck = false;
    }

    if (ui->checkBox_ref->isChecked())
    {
        dp->filterSetting.bRefCheck = true;
        dp->filterSetting.minRef = GetMinRef();
        dp->filterSetting.maxRef = GetMaxRef();

    }
    else
    {
        dp->filterSetting.bRefCheck = false;
    }
}

bool ScanFilterPage::OnPageParam(PageParam* param)
{
    m_param = param;
    ScanParam* dp = static_cast<ScanParam*>(param);
    ui->radio_BeijingTime->setChecked(dp->bUseBeijingTime);
    ui->maxRangeOfAngle->setText(QString::number(dp->dMaxAngle * RAD_TO_DEG));
    ui->minRangeOfAngle->setText(QString::number(dp->dMinAngle * RAD_TO_DEG));

    ui->maxRangeOfDist->setText(QString::number(dp->dMaxRadius));
    ui->minRangeOfDist->setText(QString::number(dp->dMinRadius));
    ui->maxRangeOfIntensity->setText(QString::number(dp->uMaxIntensity));
    ui->minRangeOfIntensity->setText(QString::number(dp->uMinIntensity));
    double mintm, maxtm;
    //if (!dp->bUseBeijingTime)
    {
        ProjectManage::getInstance()->getCurJobTime(mintm, maxtm);
        maxtm = ceil(maxtm);
        mintm = floor(mintm);
    }

    if (dp->bUseBeijingTime)
    {

        if (dp->dMaxTime < mintm || dp->dMinTime > maxtm)
        {
            dp->dMinTime = mintm;
            dp->dMaxTime = maxtm;
        }
    }
    else
    {
 
    }
    m_absTimes = dp->absTimeVec;
    if (dp->bUseBeijingTime)                                                   // 绝对时间 北京时间
    {//
        ui->tableWidget->clearContents();
        ui->tableWidget->setRowCount(0);
        for (int i = 0; i < m_absTimes.size(); ++i)
        {
            addRow(m_absTimes[i]);
        }
        ui->timeEdit_maxTime->setTime(DoubleToTime(dp->dMaxTime, ParamInterface::getHourShift()));
        ui->timeEdit_minTime->setTime(DoubleToTime(dp->dMinTime, ParamInterface::getHourShift()));
    }
    else                                                                  // 相对时间 扫描仪时间 
    {
        if (m_absTimes.empty())
        {
            ui->tableWidget->clearContents();
            ui->tableWidget->setRowCount(0);
        }
        ui->maxRangeOfTime->setText(QString::number(dp->dMaxTime));
        ui->timeEdit_maxTime->setTime(DoubleToTime(maxtm, ParamInterface::getHourShift()));
        ui->minRangeOfTime->setText(QString::number(dp->dMinTime));
        ui->timeEdit_minTime->setTime(DoubleToTime(mintm, ParamInterface::getHourShift()));
    }

    //ui->spinBox_lasStayPointSearchNum->setValue(dp->nStaySearchPointNum);
    ui->spinBox_lasStayLineNum->setValue(dp->nLineThinFrequency);
    ui->doubleSpinBox_lasPointStayDistance->setValue(dp->dStayDistance);
    
    IPlugin* plugin = nullptr;
    if (m_bMb)
    {
        const MultiBeamParaAll* scanAll = ParamMgr::getInstance()->GetMBParaAll(ProjectManage::getInstance()->getMBType());
        if (scanAll)//mb??
        {
            plugin = PluginManage::getInstance()->GetMBPlugin(scanAll->strPlugin);
        }
    }
    else
    {
        const ScannerParaAll* scanAll = ParamMgr::getInstance()->GetScannerParaAll(ProjectManage::getInstance()->getScanType());
        if (scanAll)//mb??
        {
            plugin = PluginManage::getInstance()->GetScanPlugin(scanAll->strPlugin);
        }
    }

    if (plugin && plugin->pHasFilterDialogFunc() == 1)
    {
        ui->pushButton_filterPlugin->setVisible(true);
    }
    else
    {
        ui->pushButton_filterPlugin->setVisible(false);
    }

 
    {
        ui->checkBox_badLine->setChecked(dp->filterSetting.bBadLineCheck);
        ui->checkBox_edge->setChecked(dp->filterSetting.bEdgeCheck);
        ui->checkBox_single->setChecked(dp->filterSetting.bSingleCheck);
        ui->doubleSpinBox_singleD->setValue(dp->filterSetting.singleDist);
        ui->spinBox_singleN->setValue(dp->filterSetting.singleNum);
        ui->checkBox_ref->setChecked(dp->filterSetting.bRefCheck);
        ui->lineEdit_minRef->setText(QString::number(dp->filterSetting.minRef));
        ui->lineEdit_maxRef->setText(QString::number(dp->filterSetting.maxRef));

        ui->checkBox_skipPause->setChecked(dp->bSkipPause);
        ui->doubleSpinBox_speed->setValue(dp->speedPause);
        on_checkBox_single_toggled(dp->filterSetting.bSingleCheck);
        on_checkBox_ref_toggled(dp->filterSetting.bRefCheck);
        on_checkBox_skipPause_toggled(dp->bSkipPause);

        ui->checkBox_horDist->setChecked(dp->bHorDist);
        ui->minRangeOfHorDist->setText(QString::number(dp->dMinHorRadius));
        ui->maxRangeOfHorDist->setText(QString::number(dp->dMaxHorRadius));
        on_checkBox_horDist_toggled(dp->bHorDist);

    }
    return true;
}

bool ScanFilterPage::IsValid()
{
    return checkTime() && checkDistance() &&
        checkAngle() && checkIntensity() && checkRef();

}
void ScanFilterPage::on_pushButton_modifyPbr_clicked()
{
    emit openPosSplit(0);
}
void ScanFilterPage::on_pushButton_importPbr_clicked()
{
    QString strPos = ProjectManage::getInstance()->getSpanFilePath();//param->strSpanFilePath
	std::string strbr = PbrFile::getFileName(strPos.toLocal8Bit().constData(), m_bMb);
    if (!existFile(strbr))
    {
        strbr = PbrFile::getFileName(strPos.toLocal8Bit().constData(), !m_bMb);
        if (!existFile(strbr))
        {
            strbr = PbrFile::getSmoothFileName(strPos.toLocal8Bit().constData(), m_bMb);

        }
    }
    if (existFile(strbr))
    {
        xs3d::TimeColorMap tcmap;
        if (PbrFile::read(strbr, tcmap))
        {  
            on_pushButton_clear_clicked();
            xs3d::TimeColorMap::const_iterator tit = tcmap.begin();  
            for (; tit != tcmap.end(); )
            {
                if (xs3d::TimeColor::isEnd(tit->second.clr))
                {
                    ++tit;
                    continue;
                }
                CriterionData cd;
                cd.dMin = tit->first;
                ++tit;
                if (tit == tcmap.end())
                {
                    break;
                }
                cd.dMax = tit->first;
                m_absTimes.push_back(cd);
                ++tit;
                addRow(cd);
            }
            ui->checkBox_timeSplit->setChecked(true);
        }
        else
        {
            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("无法打开分段文件！"));
            return;
        }
    } 
    else
    {
        bool bres = interfacePrompt::Question(this, m_titleHint, QStringLiteral("分段文件不存在，是否打开POS分段？"));
        if (bres)
        {
            emit openPosSplit(0);
        }
        return;
    }
}
void ScanFilterPage::on_pushButton_filterPlugin_clicked()
{
    IPlugin* plugin = nullptr;
    if (m_bMb)
    {
        const MultiBeamParaAll* scanAll = ParamMgr::getInstance()->GetMBParaAll(ProjectManage::getInstance()->getMBType());
        if (scanAll)//mb??
        {
            plugin = PluginManage::getInstance()->GetMBPlugin(scanAll->strPlugin);
        }
    }
    else
    {
        const ScannerParaAll* scanAll = ParamMgr::getInstance()->GetScannerParaAll(ProjectManage::getInstance()->getScanType());
        if (scanAll)//mb??
        {
            plugin = PluginManage::getInstance()->GetScanPlugin(scanAll->strPlugin);
        }
    }

    if (plugin)
    {
        ScanParam* dp = static_cast<ScanParam*>(m_param);//MultiBeamParam
        QDialog* dlg = plugin->pCreateFilterDialogFunc(dp->strJsonFilter.c_str());
        if (dlg == NULL)
        {
            return;
        }
        dlg->exec();
        char strJson[1024];
        int len = 1024;
        wchar_t strReadable[1024];
        int nReadable = 1024;
        int res = plugin->pGetFilterSettingFunc(dlg, strJson, &len, strReadable, &nReadable);
        if (res == QDialog::Accepted)
        {
            dp->strJsonFilter = std::string(strJson, len);
            dp->strFilterCheck = QString::fromWCharArray(strReadable);
        }

    }


}
void ScanFilterPage::on_pushButton_time_clicked()
{
    CriterionData cd;
 
    cd.dMax = GetMaxTime();
    cd.dMin = GetMinTime();
    cd.dMin += ParamInterface::getTimeShift();//beijing -> utc
    cd.dMax += ParamInterface::getTimeShift();
 
    if (cd.dMin < 0)
    {
 
        cd.dMin += ONE_DAY_SECOND;
        cd.dMax += ONE_DAY_SECOND;
    }

    if (cd.dMin >= cd.dMax)
    {
        interfacePrompt::Warn(this, m_titleHint, QStringLiteral("时间范围不正确！"));
        return;
    }


    double mintm, maxtm;
    ProjectManage::getInstance()->getCurJobTime(mintm, maxtm);
    mintm = floor(mintm);
    maxtm = ceil(maxtm);
 
    if (cd.dMin > maxtm || cd.dMax < mintm)
    {
        if (cd.dMax < mintm)
        {//pos跨天， 输入没有
            cd.dMin += ONE_DAY_SECOND;
            cd.dMax += ONE_DAY_SECOND;
        }

        if (cd.dMin > maxtm || cd.dMax < mintm)
        {
            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("时间范围和POS不一致\nPOS时间为%1～%2")
                .arg(SEC2HMS(mintm, ParamInterface::getHourShift())).arg(SEC2HMS(maxtm, ParamInterface::getHourShift())));
            return ;
        }

    }

    CritDataIt itv = m_absTimes.begin();
    for (; itv != m_absTimes.end(); ++itv)
    {
        if ((*itv).isIntersect(cd))
        {
            interfacePrompt::Warn(this, m_titleHint, QStringLiteral("时间取值范围不允许重合"));
            return;
        }
    }
    m_absTimes.push_back(cd);
    addRow(cd);

 
}

void ScanFilterPage::on_pushButton_delLine_clicked()
{
    int row = ui->tableWidget->currentRow();
    if (row < 0)
    {
        return;
    }

    double minVal = ui->tableWidget->item(row, 0)->data(Qt::UserRole + 1).toDouble();
    double maxVal = ui->tableWidget->item(row, 2)->data(Qt::UserRole + 1).toDouble();

    CritDataIt itv = m_absTimes.begin();
    for (; itv != m_absTimes.end(); ++itv)
    {
        if ((*itv).dMin == minVal && (*itv).dMax == maxVal)
        {
            m_absTimes.erase(itv);
            break;
        }
    }

    ui->tableWidget->removeRow(row);
}

void ScanFilterPage::on_pushButton_clear_clicked()
{
    ui->tableWidget->clearContents();
    ui->tableWidget->setRowCount(0);
    m_absTimes.clear();
}

void ScanFilterPage::onRadioToggled(bool bChecked)
{// abs / ref time
    if (bChecked)
    {
        if (ui->radio_BeijingTime->isChecked())
        {
            ui->pushButton_time->setVisible(true);
            ui->pushButton_importPbr->setVisible(true);
            ui->pushButton_modifyPbr->setVisible(true);
            ui->checkBox_timeSplit->setVisible(true);
            ui->groupBox_time->setVisible(true);
        }
        else
        {
            ui->pushButton_time->setVisible(false);
            ui->pushButton_importPbr->setVisible(false);
            ui->pushButton_modifyPbr->setVisible(false);
            ui->checkBox_timeSplit->setVisible(false);
            ui->groupBox_time->setVisible(false);
        }
    }
}

void ScanFilterPage::addRow(const CriterionData &cd)
{
    int row = ui->tableWidget->rowCount() + 1;
    ui->tableWidget->setRowCount(row);
    row--;
    QTableWidgetItem* item0 = new QTableWidgetItem(SEC2HMSZ(cd.dMin, ParamInterface::getHourShift()));
    item0->setData(Qt::UserRole + 1, cd.dMin);
    ui->tableWidget->setItem(row, 0, item0);

    ui->tableWidget->setItem(row, 1, new QTableWidgetItem(QStringLiteral(" — ")));
    ui->tableWidget->item(row, 1)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
    QTableWidgetItem* item4 = new QTableWidgetItem(SEC2HMSZ(cd.dMax, ParamInterface::getHourShift()));

    item4->setData(Qt::UserRole + 1, cd.dMax);
    ui->tableWidget->setItem(row, 2, item4);
}

void ScanFilterPage::on_minRangeOfIntensity_textChanged(const QString& str)
{//当前是0-255
    if (str.size() == 3)
    {
        if (str > QLatin1String("255"))
        {
            ui->minRangeOfIntensity->setText("255");
        }

    }
}

void ScanFilterPage::on_maxRangeOfIntensity_textChanged(const QString& str)
{//当前是0-255
    if (str.size() == 3)
    {
        if (str > QLatin1String("255"))
        {
            ui->maxRangeOfIntensity->setText("255");
        }

    }
}
