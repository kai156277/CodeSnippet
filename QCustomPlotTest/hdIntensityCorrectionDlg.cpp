#include "hdIntensityCorrectionDlg.h"
#include "ui_hdIntensityCorrectionDlg.h"
#include <koFuse/KOFuse.h>
#include <qjsonarray.h>
#include <qjsondocument.h>
#include <qjsonobject.h>
#include <qmessagebox.h>
#include <qprogressbar.h>
#include <qsettings.h>
#include <qtextstream.h>

#include "qcustomplot.h"

using namespace kyle_optics::framework;
using namespace kyle_optics::engine;
using namespace kyle_optics::ptcloud;
using namespace kyle_optics::layers;
using namespace kyle_optics::ptclouddriver;
using namespace kyle_optics::dataset;
using namespace kyle_optics::kernel;
using namespace std::placeholders;
using namespace std;

QT_CHARTS_USE_NAMESPACE

enum method
{
    MODEL = 0,
    KB,
    ANGLE,
    ROAD
};

hdIntensityCorrectionDlg::hdIntensityCorrectionDlg(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::hdIntensityCorrectionDlgClass())
{
    ui->setupUi(this);
    initCustomPlot();
    initChart();
    initCurve();
    initKB();
    initRoad();

    connect(ui->curveCorrection, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkCurveCorrectionButton);
    connect(ui->kbCorrection, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkkbCorrectionButton);
    connect(ui->rawIntensity, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkRawIntensityButton);
    connect(ui->calibrationData, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkCalibrationDataButton);
    connect(ui->curveFit, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkCurveFitButton);
    connect(ui->selectkoFiles, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkSelectKOFilesButton);
    connect(ui->selectGPSFiles, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkSelectGPSFilesButton);
    connect(ui->fuse, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkFuseButton);
    connect(ui->readSelectPoint, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkReadSelectPoints);
    //    connect(ui->zoomPushButton, &QPushButton::toggled, ui->chartView, &CustomChartView::zoomOrMenu);
    connect(ui->savePoints, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkSavePoints);
    connect(ui->savePointCloud, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkSavePointCloud);
    connect(ui->reductionIntensity, &QPushButton::clicked, this, &hdIntensityCorrectionDlg::checkReductionIntensity);
    connect(ui->lasFilesListWidget, &QListWidget::itemClicked, this, &hdIntensityCorrectionDlg::handleSelectedItem);
    connect(this, SIGNAL(sendLasFile(QString)), this, SLOT(addItemToList(QString)));
    connect(this, SIGNAL(loadProgress(int)), this, SLOT(updataProgress(int)));

    paramConfig();
}

hdIntensityCorrectionDlg::~hdIntensityCorrectionDlg()
{
    delete ui;
}

void hdIntensityCorrectionDlg::initCurve()
{
    m_correctionParam.d1 = 30;
    m_correctionParam.d2 = 30;
    m_correctionParam.k  = 0.3;
    m_correctionParam.b  = 800;
}

void hdIntensityCorrectionDlg::initKB()
{
    m_correctionParam.k1 = 0;
    m_correctionParam.b1 = 0;
    m_correctionParam.k2 = 2000;
    m_correctionParam.b2 = 0;
}

void hdIntensityCorrectionDlg::initRoad()
{
    m_intensityMax       = 65535;
    m_intensityThreshold = 5000;
    m_smooth             = 0.5;
    m_contrast           = 2.5;
    m_brightness         = 1;

    m_angleCorrection = false;
    m_roadEnhance     = false;
}

void hdIntensityCorrectionDlg::initDisplayRatio()
{
    m_ratio = 0.01;
}

void hdIntensityCorrectionDlg::setCurve()
{
    m_splitValue         = ui->splitSpinBox->value();
    m_correctionParam.d1 = ui->d1SpinBox->value();
    m_correctionParam.d2 = ui->d2SpinBox->value();
    m_correctionParam.k  = ui->kSpinBox->value();
    m_correctionParam.b  = ui->bSpinBox_2->value();
}

void hdIntensityCorrectionDlg::setKB()
{
    m_correctionParam.k1 = ui->k1SpinBox->value();
    m_correctionParam.b1 = ui->b1SpinBox->value();
    m_correctionParam.k2 = ui->k2SpinBox->value();
    m_correctionParam.b2 = ui->b2SpinBox->value();
}

void hdIntensityCorrectionDlg::setRoad()
{
    m_intensityMax       = ui->intensityMaxSpinBox->value();
    m_intensityThreshold = ui->intensityThresholdSpinBox->value();
    m_smooth             = ui->smoothSpinBox->value();
    m_contrast           = ui->contrastSpinBox->value();
    m_brightness         = ui->brightnessSpinBox->value();

    if (ui->angleCheckBox->isChecked())
    {
        m_angleCorrection = true;
    }
    else
    {
        m_angleCorrection = false;
    }

    if (ui->roadEnhanceBox->isChecked())
    {
        m_roadEnhance = true;
    }
    else
    {
        m_roadEnhance = false;
    }
}

void hdIntensityCorrectionDlg::setDisplayRatio()
{
    m_ratio = ui->ratioSpinBox->value();
}

void hdIntensityCorrectionDlg::saveRawIntensity(kyle_optics::engine::koLayerBase *layer, std::string path)
{
    using namespace std;
    auto ptCloudLayer = dynamic_cast<kyle_optics::layers::koPtCloudLayer *>(layer);
    if (nullptr == ptCloudLayer)
    {
        return;
    }
    kyle_optics::dataset::koDatasetBasePtr data_base   = (kyle_optics::dataset::koDatasetBasePtr)(ptCloudLayer->getDataset());
    auto                                   point_cloud = osg::dynamic_pointer_cast<koPtCloudDataset>(data_base);
    if (point_cloud == nullptr)
    {
        return;
    }
    std::unique_lock<std::recursive_mutex>                                                                            locker(point_cloud->getLock());
    std::map<std::shared_ptr<kyle_optics::ptcloud::Block>, std::vector<kyle_optics::ptclouddriver::koPointIntensity>> blocksIntensityMap;
    for (kyle_optics::ptcloud::BlockCache::Iterator block_iter = point_cloud->begin(); block_iter != point_cloud->end(); block_iter++)
    {
        //强度
        std::vector<koPointIntensity>                intensity_raw = (*block_iter)->getIntensity();
        std::shared_ptr<kyle_optics::ptcloud::Block> block         = point_cloud->getBlock((*block_iter)->getUID());
        blocksIntensityMap.insert(std::make_pair(block, intensity_raw));
    }
    m_blocksIntensityMap.swap(blocksIntensityMap);
    m_layer = layer;
    m_path  = path;
}

void hdIntensityCorrectionDlg::setClassification(kyle_optics::engine::koLayerBase *layer)
{
    resetClassification(layer);
    // saveIntensity(m_layer);
}

void hdIntensityCorrectionDlg::checkCurveCorrectionButton()
{
    setCurve();
    setKB();
    setRoad();
    setDisplayRatio();

    if (m_splitValue > 0)
    {
        segmentalCurveCorrection(m_layer, m_correctionParam, m_splitValue, MODEL, m_angleCorrection, m_roadEnhance, m_intensityMax, m_smooth,
                                 m_contrast,   // 对比度
                                 m_brightness,
                                 m_intensityThreshold,
                                 m_ratio);
    }
    else
    {
        curveCorrection(m_layer, m_correctionParam, MODEL, m_angleCorrection, m_roadEnhance, m_intensityMax, m_smooth,
                        m_contrast,   // 对比度
                        m_brightness,
                        m_intensityThreshold,
                        m_ratio);
    }
}

void hdIntensityCorrectionDlg::checkkbCorrectionButton()
{
    setKB();
    setRoad();
    setDisplayRatio();

    curveCorrection(m_layer, m_correctionParam, KB, m_angleCorrection, m_roadEnhance, m_intensityMax, m_smooth,
                    m_contrast,   // 对比度
                    m_brightness,
                    m_intensityThreshold,
                    m_ratio);
}

void hdIntensityCorrectionDlg::checkRawIntensityButton()
{
    if (m_layer == nullptr)
    {
        return;
    }
    if (m_headIntensity > 0)
    {
        ui->intensityReductionSpinBox->setValue(m_headIntensity);
    }
    else
    {
        m_headIntensity = ui->intensityReductionSpinBox->value();
    }
    setDisplayRatio();

    resetIntensity(m_layer, m_blocksIntensityMap, m_ratio);
}

void hdIntensityCorrectionDlg::checkCalibrationDataButton()
{
    QString filePath = QFileDialog::getOpenFileName(this, "选择文本文件", "", "文本文件 (*.txt)");
    m_curvePath      = filePath.toStdString();
    if (!filePath.isEmpty())
    {
        m_dataPoints.clear();
        m_curvePoints.clear();
        m_scatterSeries->replace(m_dataPoints);
        m_splineSeries->replace(m_curvePoints);

        m_firstPoints.clear();
        m_secondPoints.clear();
        m_firstScatter->replace(m_firstPoints);
        m_secondScatter->replace(m_secondPoints);

        m_firstCurve.clear();
        m_secondCurve.clear();
        m_firstSpline->replace(m_firstCurve);
        m_secondSpline->replace(m_secondCurve);
    }

    QFile file(filePath);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QTextStream in(&file);
        while (!in.atEnd())
        {
            QString     line  = in.readLine();
            QStringList parts = line.split(",");
            if (parts.size() == 2)
            {
                double x = parts[0].toDouble();
                double y = parts[1].toDouble();
                m_dataPoints.append(QPointF(x, y));
            }
        }
        file.close();
    }
    std::sort(m_dataPoints.begin(), m_dataPoints.end(), [](const QPointF &a, const QPointF &b) {
        return a.x() < b.x();
    });
    m_scatterSeries->replace(m_dataPoints);
    m_firstScatter->replace(m_firstPoints);
    m_secondScatter->replace(m_secondPoints);
}

void hdIntensityCorrectionDlg::initChart()
{
    ui->fuseProgressBar->setValue(0);
    m_chart         = new QChart();
    m_scatterSeries = new QScatterSeries(m_chart);
    m_scatterSeries->setUseOpenGL(true);                                     //启用OpenGL，否则可能会很卡顿
    m_scatterSeries->setMarkerShape(QScatterSeries::MarkerShapeRectangle);   //设置散点样式
    m_scatterSeries->setMarkerSize(4);                                       //设置散点大小
    m_scatterSeries->setColor(QColor(0, 0, 255));                            //设置散点颜色

    //第一段散点
    m_firstScatter = new QScatterSeries(m_chart);
    m_firstScatter->setUseOpenGL(true);                                     //启用OpenGL，否则可能会很卡顿
    m_firstScatter->setMarkerShape(QScatterSeries::MarkerShapeRectangle);   //设置散点样式
    m_firstScatter->setMarkerSize(4);                                       //设置散点大小
    m_firstScatter->setColor(QColor(0, 255, 255));                          //设置散点颜色
    //第二段散点
    m_secondScatter = new QScatterSeries(m_chart);
    m_secondScatter->setUseOpenGL(true);                                     //启用OpenGL，否则可能会很卡顿
    m_secondScatter->setMarkerShape(QScatterSeries::MarkerShapeRectangle);   //设置散点样式
    m_secondScatter->setMarkerSize(4);                                       //设置散点大小
    m_secondScatter->setColor(QColor(0, 255, 255));                          //设置散点颜色

    m_chart->addSeries(m_scatterSeries);
    m_chart->addSeries(m_firstScatter);
    m_chart->addSeries(m_secondScatter);

    m_splineSeries = new QSplineSeries(m_chart);
    m_firstSpline  = new QSplineSeries(m_chart);
    m_secondSpline = new QSplineSeries(m_chart);
    m_chart->createDefaultAxes();

    m_chart->axes(Qt::Horizontal).first()->setRange(0, 100);   //设置水平坐标范围
    m_chart->axes(Qt::Vertical).first()->setRange(0, 65535);   //设置垂直坐标范围
    m_chart->legend()->hide();                                 //隐藏图例

    ui->chartView->setChart(m_chart);

    ui->chartView->chart()->setAnimationOptions(QChart::AllAnimations);
    ui->chartView->viewport()->setCursor(Qt::CrossCursor);   // 设置鼠标样式为十字光标
    ui->chartView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    ui->chartView->setRenderHint(QPainter::Antialiasing);
    ui->chartView->setInteractive(true);   // 启用交互操作
}

void hdIntensityCorrectionDlg::checkCurveFitButton()
{
    m_firstCurve.clear();
    m_firstSpline->replace(m_firstCurve);

    m_secondCurve.clear();
    m_secondSpline->replace(m_secondCurve);

    m_curvePoints.clear();
    m_splineSeries->replace(m_curvePoints);

    m_firstPoints.clear();
    m_secondPoints.clear();

    m_splitValue = ui->splitSpinBox->value();
    cv::Mat matK;
    cv::Mat matK1;
    if (m_splitValue > 0)
    {
        for (size_t i = 0; i < m_dataPoints.size(); i++)
        {
            if (m_dataPoints[i].x() >= 0.0 && m_dataPoints[i].x() <= m_splitValue)
            {
                m_firstPoints.push_back(m_dataPoints[i]);
            }
            else if (m_dataPoints[i].x() > m_splitValue)
            {
                m_secondPoints.push_back(m_dataPoints[i]);
            }
        }

        int degree  = ui->degreeSpinBox->value();
        int degree1 = ui->degree1SpinBox->value();

        curveFit(m_firstPoints, degree, m_firstCurve, matK);
        curveFit(m_secondPoints, degree1, m_secondCurve, matK1);
        for (int i = 0; i < m_firstCurve.size(); i++)
        {
            m_firstSpline->append(m_firstCurve[i]);
        }
        m_chart->addSeries(m_firstSpline);

        for (int i = 0; i < m_secondCurve.size(); i++)
        {
            m_secondSpline->append(m_secondCurve[i]);
        }
        m_chart->addSeries(m_secondSpline);
        m_correctionParam.curve1.clear();
        m_correctionParam.curve2.clear();

        for (int i = 0; i < matK.rows; i++)
        {
            double param = 0.0;
            param        = matK.at<double>(i, 0);
            m_correctionParam.curve1.push_back(param);
        }
        for (int i = 0; i < matK1.rows; i++)
        {
            double param = 0.0;
            param        = matK1.at<double>(i, 0);
            m_correctionParam.curve2.push_back(param);
        }
    }
    else
    {
        int degree = ui->degreeSpinBox->value();
        curveFit(m_dataPoints, degree, m_curvePoints, matK);
        for (int i = 0; i < m_curvePoints.size(); i++)
        {
            m_splineSeries->append(m_curvePoints[i]);
        }
        m_chart->addSeries(m_splineSeries);
        m_correctionParam.curve1.clear();
        for (int i = 0; i < matK.rows; i++)
        {
            double param = 0.0;
            param        = matK.at<double>(i, 0);
            //std::cout<<"param: "<< i << " : "<< matK.at<double>(i, 0);
            m_correctionParam.curve1.push_back(param);
        }
    }

    std::string file_dir;
    std::string file_name_no_ext;
    getPath(m_curvePath, file_dir, file_name_no_ext);

    std::string   out_path = file_dir + file_name_no_ext + "curveParam.txt";
    std::ofstream outputFile(out_path);
    if (outputFile.is_open())
    {
        if (m_splitValue > 0.0)
        {
            outputFile << m_splitValue << "\n";
            outputFile << m_correctionParam.curve1.size() - 1 << "\n";

            for (const double &value : m_correctionParam.curve1)
            {
                outputFile << value << ",";
            }
            outputFile << "\n";

            outputFile << m_correctionParam.curve2.size() - 1 << "\n";

            for (const double &value : m_correctionParam.curve2)
            {
                outputFile << value << ",";
            }
            outputFile << "\n";
        }
        else
        {
            outputFile << m_correctionParam.curve1.size() - 1 << "\n";

            for (const double &value : m_correctionParam.curve1)
            {
                outputFile << value << ",";
            }
        }

        outputFile.close();
        std::cout << "Curve has been written to file." << std::endl;
    }
    else
    {
        std::cerr << "Failed to open file for writing." << std::endl;
    }

    m_chart->createDefaultAxes();
}

void hdIntensityCorrectionDlg::initKOParam()
{
    ui->fuseProgressBar->setRange(0, 100);

    ui->fuseProgressBar->setOrientation(Qt::Horizontal);
    //雷达安置误差
    m_deltaX = ui->errorXSpinBox->value();
    m_deltaY = ui->errorYSpinBox->value();
    m_deltaZ = ui->errorZSpinBox->value();
    m_yaw    = ui->errorHeadSpinBox->value();
    m_pitch  = ui->errorPitchSpinBox->value();
    m_roll   = ui->errorRollSpinBox->value();
    //雷达转导航坐标系
    m_coordTransX  = ui->offsetXDoubleSpinBox->value();
    m_coordTransY  = ui->offsetYDoubleSpinBox->value();
    m_coordTransZ  = ui->offsetZDoubleSpinBox->value();
    m_coordRotateX = ui->rotationXDoubleSpinBox->value();
    m_coordRotateY = ui->rotationYDoubleSpinBox->value();
    m_coordRotateZ = ui->rotationZDoubleSpinBox->value();
}

void hdIntensityCorrectionDlg::checkSelectKOFilesButton()
{
    QString lastOpenPath = m_filePath;

    QStringList filePath = QFileDialog::getOpenFileNames(this, tr("选择ko文件"), lastOpenPath, tr("ko文件(*.ko);;所有文件(*.*)"));

    if (filePath.isEmpty())
    {
        return;
    }
    else
    {
        QString strs;
        for (int i = 0; i < filePath.size(); i++)
        {
            QString str_path = filePath[i];
            // 单个文件路径
            QFileInfo file = QFileInfo(str_path);
            // 获得文件名
            QString file_name     = file.fileName();
            QString file_path     = file.filePath();
            m_filePath            = file.path() + "/";
            std::string ko_string = file_path.toLocal8Bit().constData();
            m_koFiles.push_back(ko_string);
            strs.append(file_name);
            strs += "\n";
            ui->kotextBrowser->setText(strs);

            // 记录本次打开文件对话框的路径
            QSettings().setValue("LastOpenPath", file_path);
        }
    }
}

void hdIntensityCorrectionDlg::checkSelectGPSFilesButton()
{
    QString lastOpenPath = QSettings().value("LastGPSOpenPath", QDir::homePath()).toString();

    QString filePath = QFileDialog::getOpenFileName(this, tr("选择GPS文件"), lastOpenPath, tr("GPS(*.txt);;所有文件(*.*)"));

    if (filePath.isEmpty())
    {
        return;
    }
    else
    {
        QFileInfo file_info = QFileInfo(filePath);
        QString   file_name = file_info.fileName();
        QString   file_path = file_info.filePath();
        m_gpsFile           = file_path.toLocal8Bit().constData();
        ui->GPSlineEdit->setText(file_name);

        QSettings().setValue("LastGPSOpenPath", file_path);
    }
}

void hdIntensityCorrectionDlg::myProgressCallback(void *context, std::string lasFile, int progress)
{
    emit loadProgress(progress);
    if (progress == -1)
    {
        QString lasFileName = QString::fromLocal8Bit(lasFile.c_str());
        qDebug() << lasFileName;
        emit sendLasFile(lasFileName);
    }
}

CKOFuse ko;
void    hdIntensityCorrectionDlg::checkFuseButton()
{
    //initKOParam();
    PFuseCallFun fun = std::bind(&hdIntensityCorrectionDlg::myProgressCallback, this, _1, _2, _3);
    //ko.setParam(m_roll, m_pitch, m_yaw, m_deltaX, m_deltaY, m_deltaZ,m_coordTransX,m_coordTransY,m_coordTransZ,m_coordRotateX,m_coordRotateY,m_coordRotateZ);
    ko.setParam("config.json");
    bool fuseResult = ko.fuse(m_koFiles, m_gpsFile, fun, this);

    m_headIntensity = ko.radarIntensityThresholdIn5M();

    std::cout << "m_headIntensity:" << m_headIntensity << std::endl;
    if (fuseResult)
    {
        std::cout << "success" << std::endl;
    }
    else
    {
        std::cout << "fail" << std::endl;
    }
}

void hdIntensityCorrectionDlg::checkReadSelectPoints()
{
    m_dataPoints.clear();
    m_curvePoints.clear();
    m_scatterSeries->replace(m_dataPoints);
    m_splineSeries->replace(m_curvePoints);

    m_firstPoints.clear();
    m_secondPoints.clear();
    m_firstCurve.clear();
    m_secondCurve.clear();
    m_firstScatter->replace(m_firstPoints);
    m_secondScatter->replace(m_secondPoints);
    m_firstSpline->replace(m_firstCurve);
    m_secondSpline->replace(m_secondCurve);

    QString filePath = QFileDialog::getOpenFileName(this, "Select CSV File", "", "CSV Files (*.csv)");
    if (filePath.isEmpty())
    {
        return;
    }
    m_curvePath = filePath.toStdString();

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qWarning() << "Unable to open the file.";
        return;
    }

    QTextStream in(&file);
    QStringList header;
    QStringList data;
    bool        isHeaderLine = true;

    while (!in.atEnd())
    {
        QString line = in.readLine();

        if (isHeaderLine)
        {
            header       = line.split(',');
            isHeaderLine = false;
        }
        else if (line.isEmpty())
        {
            //跳过空行
        }
        else
        {
            data = line.split(',');
            if (data.size() >= 10)
            {
                double x = data[4].toDouble();
                double y = data[9].toDouble() / 1000;
                m_dataPoints.append(QPointF(y, x));
            }
        }
        file.close();
    }
    std::sort(m_dataPoints.begin(), m_dataPoints.end(), [](const QPointF &a, const QPointF &b) {
        return a.x() < b.x();
    });
    m_scatterSeries->replace(m_dataPoints);
    m_firstScatter->replace(m_firstPoints);
    m_secondScatter->replace(m_secondPoints);
}

void hdIntensityCorrectionDlg::checkSavePoints()
{
    std::cout << "before:" << m_dataPoints.size() << std::endl;
    QVector<QPointF> points_new = m_scatterSeries->pointsVector();
    m_dataPoints.clear();
    for (size_t i = 0; i < points_new.size(); i++)
    {
        m_dataPoints.push_back(points_new[i]);
    }
    std::sort(m_dataPoints.begin(), m_dataPoints.end(), [](const QPointF &a, const QPointF &b) {
        return a.x() < b.x();
    });

    std::cout << "after:" << m_dataPoints.size() << std::endl;
    std::string file_dir;
    std::string file_name_no_ext;
    getPath(m_curvePath, file_dir, file_name_no_ext);

    std::string out_path = file_dir + file_name_no_ext + "_new.txt";

    FILE *fp = fopen(out_path.c_str(), "w");

    for (size_t i = 0; i < points_new.size(); i++)
    {
        char szTemp[128] = {0};
        sprintf(szTemp, "%.2f,%.0f\n", m_dataPoints[i].x(), m_dataPoints[i].y());
        fwrite(szTemp, 1, strlen(szTemp), fp);
    }
    fclose(fp);
}

void hdIntensityCorrectionDlg::checkSavePointCloud()
{
    std::string file_dir;
    std::string file_name_no_ext;
    getPath(m_path, file_dir, file_name_no_ext);

    std::string out_path = file_dir + file_name_no_ext + "_afetr.las";
    savePointcCloud(m_layer, out_path);
}

void hdIntensityCorrectionDlg::checkReductionIntensity()
{
    m_ratio = ui->ratioSpinBox->value();
    resetIntensity(m_layer, m_blocksIntensityMap, m_headIntensity, m_ratio);
}

void hdIntensityCorrectionDlg::handleSelectedItem(QListWidgetItem *item)
{
    QString name     = item->data(Qt::UserRole).toString();
    QString las_path = m_filePath + name;
    qDebug() << m_filePath;
    qDebug() << name;
    qDebug() << las_path;

    //m_path = path.toLocal8Bit().toStdString();
    emit loadLasFile(las_path);
}

void hdIntensityCorrectionDlg::addItemToList(QString path)
{

    std::vector<QString> lasFilePaths;
    lasFilePaths.push_back(path);
    for (const QString &lasFilePath : lasFilePaths)
    {
        QString          lasFileName = QFileInfo(lasFilePath).fileName();
        QListWidgetItem *item        = new QListWidgetItem(lasFileName);
        item->setData(Qt::UserRole, QVariant(lasFileName));
        ui->lasFilesListWidget->addItem(item);
    }
}

QString hdIntensityCorrectionDlg::paramConfig()
{
    QJsonObject configObject;

    configObject["heading"] = -0.26;
    configObject["pitch"]   = 0.023;
    configObject["roll"]    = 0.0;
    configObject["deltaX"]  = -0.19;
    configObject["deltaY"]  = 0.0;
    configObject["deltaZ"]  = 0.0;

    configObject["coordTransX"]  = 0.0;
    configObject["coordTransY"]  = 0.0;
    configObject["coordTransZ"]  = 0.0;
    configObject["coordRotateX"] = -120.0;
    configObject["coordRotateY"] = 0.0;
    configObject["coordRotateZ"] = -90.0;

    configObject["lasFileCountMax"]   = 11000000;
    configObject["disLimit"]          = true;
    configObject["disLimitMax"]       = 500000;
    configObject["disLimitMin"]       = 500;
    configObject["cutOfStart"]        = 0;
    configObject["cutOfEnd"]          = 360.0;
    configObject["weekSecondsEnable"] = true;
    configObject["centerLongitude"]   = 114.0;
    configObject["laz"]               = false;
    /*configObject["right"] = false;
	configObject["left"] = true;*/

    QJsonDocument configDocument(configObject);
    QString       strjson = configDocument.toJson(QJsonDocument::Indented);
    qDebug() << strjson;
    QString configFilePath = "config.json";
    QFile   configFile(configFilePath);
    if (configFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&configFile);
        out << strjson;
        configFile.close();
    }

    return strjson;
}

void hdIntensityCorrectionDlg::updataProgress(int progress)
{
    ui->fuseProgressBar->setValue(progress);
}

void hdIntensityCorrectionDlg::initCustomPlot()
{
    ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectItems | QCP::iSelectPlottables | QCP::iMultiSelect);
    ui->plot->setSelectionRectMode(QCP::srmNone);
    //    setupPlot();
    scatterPlot();
    curvePlot();

    // configure scroll bars:
    // Since scroll bars only support integer values, we'll set a high default range of -500..500 and
    // divide scroll bar position values by 100 to provide a scroll range -5..5 in floating point
    // axis coordinates. if you want to dynamically grow the range accessible with the scroll bar,
    // just increase the minimum/maximum values of the scroll bars as needed.
    ui->horizontalScrollBar->setRange(-500, 500);
    ui->verticalScrollBar->setRange(-500, 500);

    // create connection between axes and scroll bars:

    connect(ui->plot, SIGNAL(mousePress(QMouseEvent *)), this, SLOT(mousePress(QMouseEvent *)));
    connect(ui->horizontalScrollBar, SIGNAL(valueChanged(int)), this, SLOT(horzScrollBarChanged(int)));
    connect(ui->verticalScrollBar, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBarChanged(int)));
    connect(ui->plot->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxisChanged(QCPRange)));
    connect(ui->plot->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxisChanged(QCPRange)));

    //    connect(ui->customPlot, SIGNAL(mousePress(QMouseEvent *)), this, SLOT(mousePress(QMouseEvent *)));

    // initialize axis range (and scroll bar positions via signals we just connected):
    ui->plot->xAxis->setRange(0, 6, Qt::AlignCenter);
    ui->plot->yAxis->setRange(0, 10, Qt::AlignCenter);

    // setup policy and connect slot for context menu popup:
    ui->plot->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->plot, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));
}

void hdIntensityCorrectionDlg::horzScrollBarChanged(int value)
{
    if (qAbs(ui->plot->xAxis->range().center() - value / 100.0) > 0.01)   // if user is dragging plot, we don't want to replot twice
    {
        ui->plot->xAxis->setRange(value / 100.0, ui->plot->xAxis->range().size(), Qt::AlignCenter);
        ui->plot->replot();
    }
}

void hdIntensityCorrectionDlg::vertScrollBarChanged(int value)
{
    if (qAbs(ui->plot->yAxis->range().center() + value / 100.0) > 0.01)   // if user is dragging plot, we don't want to replot twice
    {
        ui->plot->yAxis->setRange(-value / 100.0, ui->plot->yAxis->range().size(), Qt::AlignCenter);
        ui->plot->replot();
    }
}

void hdIntensityCorrectionDlg::xAxisChanged(QCPRange range)
{
    ui->horizontalScrollBar->setValue(qRound(range.center() * 100.0));    // adjust position of scroll bar slider
    ui->horizontalScrollBar->setPageStep(qRound(range.size() * 100.0));   // adjust size of scroll bar slider
}

void hdIntensityCorrectionDlg::yAxisChanged(QCPRange range)
{

    ui->verticalScrollBar->setValue(qRound(-range.center() * 100.0));   // adjust position of scroll bar slider
    ui->verticalScrollBar->setPageStep(qRound(range.size() * 100.0));   // adjust size of scroll bar slider
}

void hdIntensityCorrectionDlg::mousePress(QMouseEvent *e)
{
    // if an axis is selected, only allow the direction of that axis to be dragged
    // if no axis is selected, both directions may be dragged
    auto key = e->modifiers();
    if (key & Qt::ControlModifier)
        ui->plot->setSelectionRectMode(QCP::srmSelect);
    else
        ui->plot->setSelectionRectMode(QCP::srmNone);

    if (ui->plot->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
        ui->plot->axisRect()->setRangeDrag(ui->plot->xAxis->orientation());
    else if (ui->plot->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
        ui->plot->axisRect()->setRangeDrag(ui->plot->yAxis->orientation());
    else
        ui->plot->axisRect()->setRangeDrag(Qt::Horizontal | Qt::Vertical);
}

void hdIntensityCorrectionDlg::contextMenuRequest(QPoint pos)
{
    QMenu *menu = new QMenu(this);
    menu->setAttribute(Qt::WA_DeleteOnClose);

    menu->addAction("Add point", this, SLOT(addPoint()));
    if (ui->plot->selectedPlottables().size() > 0)
        menu->addAction("Remove selected point", this, SLOT(removeSelectedPoint()));

    menu->popup(ui->plot->mapToGlobal(pos));
    mPos = pos;
}

void hdIntensityCorrectionDlg::scatterPlot()
{
    QCPGraphDataContainer mScatter;
    ui->plot->addGraph();
    ui->plot->graph()->setPen(QPen(Qt::red));
    QVector<double> x(500), y0(500), y1(500);
    for (int i = 0; i < 500; ++i)
    {
        x[i]  = (i / 499.0 - 0.5) * 10;
        y0[i] = qExp(-x[i] * x[i] * 0.25) * qSin(x[i] * 5) * 5;
        mScatter.add(x[i], y[0]);
        y1[i] = qExp(-x[i] * x[i] * 0.25) * 5;
    }
    ui->plot->graph()->setData(mScatter);
    ui->plot->graph()->setLineStyle(QCPGraph::lsLine);                                              // 设置数据点之间的连线
    ui->plot->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape::ssDisc)));   // 设置数据点的样式

    ui->plot->axisRect()->setupFullAxesBox(true);
    //    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
    //                                    QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->plot->plottable()->setSelectable(QCP::stMultipleDataRanges);
    ui->plot->replot();
}

void hdIntensityCorrectionDlg::curvePlot()
{
    ui->plot->addGraph();
    ui->plot->graph()->setPen(QPen(Qt::green));
    QVector<double> x(500), y0(500), y1(500);
    for (int i = 0; i < 500; ++i)
    {
        x[i]  = (i / 499.0 - 0.5) * 10;
        y0[i] = qExp(-x[i] * x[i] * 0.25) * qSin(x[i] * 5) * 5;
        y1[i] = qExp(-x[i] * x[i] * 0.25) * 5;
    }
    ui->plot->graph()->setData(x, y1);
    ui->plot->graph()->setLineStyle(QCPGraph::lsLine);
    ui->plot->graph()->setSelectable(QCP::stNone);
    //    ui->plot->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape::ssDisc)));
    ui->plot->axisRect()->setupFullAxesBox(true);

    //    ui->plot->plottable()->setSelectable(QCP::stMultipleDataRanges);
    ui->plot->replot();
}

void hdIntensityCorrectionDlg::addPoint()
{
    double pos_x = ui->plot->xAxis->pixelToCoord(mPos.x());
    double pos_y = ui->plot->yAxis->pixelToCoord(mPos.y());
    ui->plot->graph(0)->addData(pos_x, pos_y);
    ui->plot->replot();
}

void hdIntensityCorrectionDlg::removeSelectedPoint()
{
    QCPGraph *       graph     = ui->plot->graph(0);
    QCPDataSelection selection = graph->selection();

    qDebug() << graph->dataCount();
    //    qDebug() << graph->data().data()->size();
    QList<QCPDataRange> dataRanges = selection.dataRanges();

    for (int i = dataRanges.size() - 1; i >= 0; i--)
    {
        qDebug() << dataRanges[i].begin() << "-" << dataRanges[i].end();
        auto begin_index = graph->data()->at(dataRanges[i].begin());
        auto end_index   = graph->data()->at(dataRanges[i].end());
        graph->data().data()->remove(begin_index->sortKey(), end_index->sortKey());
    }
    ui->plot->deselectAll();
    ui->plot->replot();
    qDebug() << graph->dataCount();
}
