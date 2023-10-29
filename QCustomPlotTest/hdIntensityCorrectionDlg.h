#pragma once

//#include "CustomChartView.h"
#include "koDataset/koDatasetBase.h"
#include "koFramework/koViewWidget.h"
#include "koLayers/koDrawablesLayer.h"
#include "koLayers/koPtCloudLayer.h"
#include "koLidarDriver/koLidarPointType.h"
#include "koRenderEngine/koLayerBase.h"
#include "pointCloudAlg.h"
#include "qcustomplot.h"
#include <QDialog>
#include <QtCharts/qchart.h>
//#include <QtCharts>
//#include <qchartview.h>
#include <qfiledialog.h>
#include <qgraphicsitem.h>
#include <qgraphicsview.h>
#include <qlineseries.h>
#include <qmouseeventtransition.h>
namespace Ui {
class hdIntensityCorrectionDlgClass;
}   // namespace Ui

class hdIntensityCorrectionDlg : public QDialog
{
    Q_OBJECT

public:
    hdIntensityCorrectionDlg(QWidget *parent = Q_NULLPTR);
    ~hdIntensityCorrectionDlg();

public:
    Ui::hdIntensityCorrectionDlgClass *ui;
    void                               initCurve();
    void                               initKB();
    void                               initRoad();
    void                               initDisplayRatio();
    void                               setCurve();
    void                               setKB();
    void                               setRoad();
    void                               setDisplayRatio();

    void saveRawIntensity(kyle_optics::engine::koLayerBase *layer, std::string path);
    void setClassification(kyle_optics::engine::koLayerBase *layer);
    void checkCurveCorrectionButton();
    void checkkbCorrectionButton();
    void checkRawIntensityButton();
    void checkCalibrationDataButton();
    void initChart();
    void checkCurveFitButton();
    void initKOParam();
    void myProgressCallback(void *context, std::string lasFile, int progress);
    void checkSelectKOFilesButton();
    void checkSelectGPSFilesButton();
    void checkFuseButton();
    void checkReadSelectPoints();
    void checkSavePoints();
    void checkSavePointCloud();
    void checkReductionIntensity();
    void handleSelectedItem(QListWidgetItem *item);

    QString paramConfig();
public slots:
    void addItemToList(QString path);
    void updataProgress(int progress);
signals:
    void sendLasFile(QString path);
    void loadLasFile(QString &path);
    void loadProgress(int progress);

private:
    // qcustomplot 曲线绘图功能
    void initCustomPlot();
    void horzScrollBarChanged(int value);
    void vertScrollBarChanged(int value);
    void xAxisChanged(QCPRange range);
    void yAxisChanged(QCPRange range);
    void mousePress(QMouseEvent *e);
    void contextMenuRequest(QPoint pos);
    void addPoint();
    void removeSelectedPoint();

    void   scatterPlot();   //
    void   curvePlot();
    QPoint mPos;

private:
    std::map<std::shared_ptr<kyle_optics::ptcloud::Block>, std::vector<kyle_optics::ptclouddriver::koPointIntensity>> m_blocksIntensityMap;
    kyle_optics::engine::koLayerBase *                                                                                m_layer;
    QChart *                                                                                                          m_chart;

    QCPGraphDataContainer m_scatter;
    QVector<QPointF>      m_dataPoints;
    QVector<QPointF>      m_curvePoints;
    QScatterSeries *      m_scatterSeries;
    QSplineSeries *       m_splineSeries;

    QVector<QPointF> m_firstPoints;   //第一段数据
    QScatterSeries * m_firstScatter;
    QVector<QPointF> m_firstCurve;
    QSplineSeries *  m_firstSpline;

    QVector<QPointF> m_secondPoints;   // 第二段数据
    QScatterSeries * m_secondScatter;
    QVector<QPointF> m_secondCurve;
    QSplineSeries *  m_secondSpline;

    std::vector<std::string> m_koFiles;
    std::string              m_gpsFile;

    //雷达安置误差
    float m_deltaX;
    float m_deltaY;
    float m_deltaZ;
    float m_yaw;
    float m_pitch;
    float m_roll;
    //雷达转导航坐标系
    float  m_coordTransX;
    float  m_coordTransY;
    float  m_coordTransZ;
    float  m_coordRotateX;
    float  m_coordRotateY;
    float  m_coordRotateZ;
    int    m_progress;
    double m_splitValue;

    correctionParam m_correctionParam;
    QPointF         m_lastMousePos;   // 用于保存上一个鼠标位置
    std::string     m_path;
    bool            m_angleCorrection;
    bool            m_roadEnhance;
    double          m_intensityMax;
    double          m_smooth;
    double          m_contrast;   // 对比度
    double          m_brightness;
    double          m_intensityThreshold;
    int             m_headIntensity;
    QString         m_config;
    float           m_ratio;
    std::string     m_curvePath;

    QString     m_filePath;
    std::string m_fileName;
};
