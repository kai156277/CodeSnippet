#pragma once

#include <QWidget>

#include "qcustomplot.h"

#include <QMouseEvent>

QT_BEGIN_NAMESPACE
namespace Ui {
class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

    void setupPlot();

private slots:
    void horzScrollBarChanged(int value);
    void vertScrollBarChanged(int value);
    void xAxisChanged(QCPRange range);
    void yAxisChanged(QCPRange range);

    void mousePress(QMouseEvent *e);

    void scatterPlot();
    void curvePlot();

    void contextMenuRequest(QPoint pos);

    void addPoint();

    void removeSelectedPoint();

private:
    Ui::Widget *ui;
    QPoint      mPos;

    QSharedPointer<QCPGraphDataContainer> mData;
};
