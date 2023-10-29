#pragma once

#include <QWidget>

#include <QChart>
#include <QLineSeries>
#include <QScatterSeries>
#include <QSplineSeries>

QT_BEGIN_NAMESPACE
namespace Ui {
class ChartWidget;
}
QT_END_NAMESPACE

class ChartWidget : public QWidget
{
    Q_OBJECT

public:
    ChartWidget(QWidget *parent = nullptr);
    ~ChartWidget();

private slots:
    void on_importPushButton_clicked();

    void on_savePushButton_clicked();

    void selectScatterSeries(const QPointF &pos);

private:
    void             updateChart();
    Ui::ChartWidget *ui;

    QT_CHARTS_NAMESPACE::QScatterSeries *mScatterSeries;
    QT_CHARTS_NAMESPACE::QSplineSeries * mSeries;
    QT_CHARTS_NAMESPACE::QChart *        mChart;
};
