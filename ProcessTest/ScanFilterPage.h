
#ifndef ScanFilterPage_H
#define ScanFilterPage_H

#include <QDialog>

#include "DataType.h"

#include "Criterion.h"
#include "WizardPage.h"

#include <PageParam.h>

class QListWidgetItem;
class QRegExpValidator;
namespace Ui {
class ScanFilterPage;
}

class ScanFilterPage : public WizardPage
{
    Q_OBJECT

public:
    /*!  构造函数           */
    explicit ScanFilterPage(const QString &title, bool bmb, QWidget *parent = 0);

    /*!  析构函数           */
    virtual ~ScanFilterPage();
    virtual bool IsValid();

    virtual bool OnPageParam(xstype::PageParam *param);

    virtual void SetPageParam(xstype::PageParam *param);

private:
    /*! 获取最小极径         */
    double GetMinRef();

    /*! 获取最大极径         */
    double GetMaxRef();
    /*! 获取最小极径         */
    double GetMinDistance();

    /*! 获取最大极径         */
    double GetMaxDistance();

    double GetMinHorDistance();
    double GetMaxHorDistance();
    /*! 获取最小极角         */
    double GetMinAngle();

    /*! 获取最大极角         */
    double GetMaxAngle();

    /*! 获取最小反射强度      */
    ushort GetMinIntensity();

    /*! 获取最大反射强度      */
    ushort GetMaxIntensity();

    /*! 判断是否采用GPS时间\n
    返回值：true，采用GPS时间；false，采用扫描仪时间*/
    bool IsUseBeijingTime();
    /*! 根据时间过滤条件，获取最小时间       */
    double GetMinTime();

    /*! 根据时间过滤条件，获取最大时间       */
    double GetMaxTime();

    /*! 检查极径参数\n
    返回值：true，设置正确；false，设置错误  */
    bool checkDistance();

    /*! 检查极角参数\n
    返回值：true，设置正确；false，设置错误  */
    bool checkAngle();

    /*! 检查反射强度参数\n
    返回值：true，设置正确；false，设置错误  */
    bool checkIntensity();

    /*! 检查里程参数\n
    返回值：true，设置正确；false，设置错误  */
    bool checkRef();

    /*! 检查时间参数\n
    返回值：true，设置正确；false，设置错误  */
    bool checkTime();

    void addRow(const xstype::CriterionData &cd);
signals:
    void openPosSplit(int);
public slots:
    void on_pushButton_importPbr_clicked();

    void on_pushButton_modifyPbr_clicked();
private slots:
    void on_pushButton_filterPlugin_clicked();
    void on_pushButton_time_clicked();

    void on_pushButton_delLine_clicked();
    void on_pushButton_clear_clicked();
    void onRadioToggled(bool bChecked);
    void on_minRangeOfIntensity_textChanged(const QString &);
    void on_maxRangeOfIntensity_textChanged(const QString &);

    void on_checkBox_single_toggled(bool bChecked);
    void on_checkBox_ref_toggled(bool bChecked);

    void on_checkBox_skipPause_toggled(bool bChecked);
    void on_checkBox_horDist_toggled(bool bChecked);

private:
    Ui::ScanFilterPage *ui;   ///< 界面的指针

    QRegExpValidator *  m_validator[3];   ///< 设置正则表达式，用于参数设置
    bool                m_bMb;
    xstype::PageParam * m_param;
    xstype::CritDataVec m_absTimes;
};

#endif   // ScanFilterPage_H
