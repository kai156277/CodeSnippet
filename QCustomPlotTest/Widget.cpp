#include "Widget.h"
#include "ui_Widget.h"

#include "qcustomplot.h"

#include <QDebug>
#include <QKeyEvent>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , mData(new QCPGraphDataContainer)
{
    ui->setupUi(this);

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

Widget::~Widget()
{
    delete ui;
}

void Widget::setupPlot()
{
    // The following plot setup is mostly taken from the plot demos:
    ui->plot->addGraph();
    ui->plot->graph()->setPen(QPen(Qt::yellow));
    ui->plot->graph()->setBrush(QBrush(QColor(0, 0, 255, 20)));
    ui->plot->addGraph();
    ui->plot->graph()->setPen(QPen(Qt::red));
    QVector<double> x(500), y0(500), y1(500);
    for (int i = 0; i < 500; ++i)
    {
        x[i]  = (i / 499.0 - 0.5) * 10;
        y0[i] = qExp(-x[i] * x[i] * 0.25) * qSin(x[i] * 5) * 5;
        y1[i] = qExp(-x[i] * x[i] * 0.25) * 5;
    }
    ui->plot->graph(0)->setData(x, y0);
    ui->plot->graph(0)->setLineStyle(QCPGraph::lsNone);                                              // 设置数据点之间的连线
    ui->plot->graph(0)->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape::ssDisc)));   // 设置数据点的样式
    ui->plot->graph(1)->setData(x, y1);
    ui->plot->axisRect()->setupFullAxesBox(true);
    ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectItems | QCP::iSelectPlottables | QCP::iMultiSelect);
    ui->plot->setSelectionRectMode(QCP::srmNone);
    //    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
    //                                    QCP::iSelectLegend | QCP::iSelectPlottables);

    QFont legendFont = font();
    legendFont.setPointSize(10);
    ui->plot->legend->setFont(legendFont);
    ui->plot->legend->setSelectedFont(legendFont);
    ui->plot->legend->setSelectableParts(QCPLegend::spItems);   // legend box shall not be selectable, only legend items
    ui->plot->plottable(0)->setSelectable(QCP::stMultipleDataRanges);
    ui->plot->plottable(1)->setSelectable(QCP::stMultipleDataRanges);
}

void Widget::horzScrollBarChanged(int value)
{
    if (qAbs(ui->plot->xAxis->range().center() - value / 100.0) > 0.01)   // if user is dragging plot, we don't want to replot twice
    {
        ui->plot->xAxis->setRange(value / 100.0, ui->plot->xAxis->range().size(), Qt::AlignCenter);
        ui->plot->replot();
    }
}

void Widget::vertScrollBarChanged(int value)
{
    if (qAbs(ui->plot->yAxis->range().center() + value / 100.0) > 0.01)   // if user is dragging plot, we don't want to replot twice
    {
        ui->plot->yAxis->setRange(-value / 100.0, ui->plot->yAxis->range().size(), Qt::AlignCenter);
        ui->plot->replot();
    }
}

void Widget::xAxisChanged(QCPRange range)
{
    ui->horizontalScrollBar->setValue(qRound(range.center() * 100.0));    // adjust position of scroll bar slider
    ui->horizontalScrollBar->setPageStep(qRound(range.size() * 100.0));   // adjust size of scroll bar slider
}

void Widget::yAxisChanged(QCPRange range)
{
    ui->verticalScrollBar->setValue(qRound(-range.center() * 100.0));   // adjust position of scroll bar slider
    ui->verticalScrollBar->setPageStep(qRound(range.size() * 100.0));   // adjust size of scroll bar slider
}

void Widget::mousePress(QMouseEvent *e)
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

void Widget::scatterPlot()
{
    ui->plot->addGraph();
    ui->plot->graph()->setPen(QPen(Qt::red));
    //    QVector<double> x(500), y0(500), y1(500);
    for (int i = 0; i < 500; ++i)
    {
        double x = (i / 499.0 - 0.5) * 10;
        double y = qExp(-x * x * 0.25) * qSin(x * 5) * 5;
        //        y = qExp(-x[i] * x[i] * 0.25) * 5;

        mData->add(QCPGraphData(x, y));
    }
    ui->plot->graph()->setData(mData);
    ui->plot->graph()->setLineStyle(QCPGraph::lsLine);                                              // 设置数据点之间的连线
    ui->plot->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape::ssDisc)));   // 设置数据点的样式

    ui->plot->axisRect()->setupFullAxesBox(true);
    //    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
    //                                    QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->plot->plottable()->setSelectable(QCP::stMultipleDataRanges);
    ui->plot->replot();
}

void Widget::curvePlot()
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

void Widget::contextMenuRequest(QPoint pos)
{

    QMenu *menu = new QMenu(this);
    menu->setAttribute(Qt::WA_DeleteOnClose);

    menu->addAction("Add point", this, SLOT(addPoint()));
    if (ui->plot->selectedPlottables().size() > 0)
        menu->addAction("Remove selected point", this, SLOT(removeSelectedPoint()));

    menu->popup(ui->plot->mapToGlobal(pos));
    mPos = pos;
}

void Widget::addPoint()
{
    qDebug() << "old:" << mData->size();
    double pos_x = ui->plot->xAxis->pixelToCoord(mPos.x());
    double pos_y = ui->plot->yAxis->pixelToCoord(mPos.y());
    ui->plot->graph(0)->addData(pos_x, pos_y);
    ui->plot->replot();
    qDebug() << "new:" << mData->size();
}

void Widget::removeSelectedPoint()
{
    qDebug() << "old:" << mData->size();
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
        mData->remove(begin_index->sortKey(), end_index->sortKey());
        qDebug() << begin_index->sortKey() << "-" << end_index->sortKey();
        //        graph->data().data()->remove(begin_index->sortKey(), end_index->sortKey());
    }
    ui->plot->deselectAll();
    ui->plot->replot();
    qDebug() << graph->dataCount();
    qDebug() << "new:" << mData->size();
}
