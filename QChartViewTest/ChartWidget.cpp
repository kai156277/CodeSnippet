#include "ChartWidget.h"
#include "ui_ChartWidget.h"
#include <QDebug>
#include <QTime>
#include <QXYSeries>

QT_CHARTS_USE_NAMESPACE

ChartWidget::ChartWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ChartWidget)
    , mSeries(new QSplineSeries)
    , mScatterSeries(new QScatterSeries)
    , mChart(new QChart)
{
    ui->setupUi(this);

    ui->chartView->setChart(mChart);
    on_importPushButton_clicked();
    connect(ui->zoomPushButton, &QPushButton::toggled, ui->chartView, &CustomChartView::zoomOrMenu);
}

ChartWidget::~ChartWidget()
{
    delete ui;
}

void ChartWidget::on_importPushButton_clicked()
{
    QFile point_file("E:\\1_175952_00001-cut.txt");
    if (!point_file.open(QIODevice::ReadOnly))
    {
        qDebug() << "Can`t open file";
        return;
    }

    QList<QPointF> points;
    while (!point_file.atEnd())
    {
        QString     readline = point_file.readLine();
        QStringList items    = readline.split(",");

        points.append({items[0].toDouble(), items[1].toDouble()});
    }

    std::sort(points.begin(), points.end(), [](const QPointF &a, const QPointF &b) { return a.x() < b.x(); });

    mSeries->clear();
    mSeries->append(points);
    mScatterSeries->clear();
    mScatterSeries->append(points);

    connect(mScatterSeries, &QXYSeries::clicked, this, &ChartWidget::selectScatterSeries);

    mChart->addSeries(mSeries);
    mSeries->setUseOpenGL(true);   //启用OpenGL，否则可能会很卡顿
    mChart->addSeries(mScatterSeries);
    mScatterSeries->setUseOpenGL(true);   //启用OpenGL，否则可能会很卡顿

    updateChart();
}

void ChartWidget::on_savePushButton_clicked()
{
    qDebug() << QTime::currentTime().toString() << mScatterSeries->pointsVector();
    qDebug() << QTime::currentTime().toString() << mSeries->pointsVector();
}

void ChartWidget::selectScatterSeries(const QPointF &pos)
{
}

void ChartWidget::updateChart()
{

    mChart->createDefaultAxes();
    //    ui->chartView->setDragMode(QChartView::ScrollHandDrag);
    //    ui->chartView->setRubberBand(QChartView::RectangleRubberBand);   //拉伸效果
    ui->chartView->chart()->setAnimationOptions(QChart::AllAnimations);

    ui->chartView->viewport()->setCursor(Qt::CrossCursor);   // 设置鼠标样式为十字光标
    ui->chartView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    ui->chartView->setRenderHint(QPainter::Antialiasing);
    ui->chartView->setInteractive(true);   // 启用交互操作
}
