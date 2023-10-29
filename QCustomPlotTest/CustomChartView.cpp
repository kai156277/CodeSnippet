#include "CustomChartView.h"
#include <QMenu>
#include <QInputDialog>
#include <QMenu>
#include <QMessageBox>
#include <QXYSeries>

QT_CHARTS_USE_NAMESPACE

CustomChartView::CustomChartView(QWidget* parent)
    : QChartView(parent)
    , m_addPoint(new QAction("add point"))
    , m_deletePoint(new QAction("delte point"))
    , m_movePoint(new QAction("move point"))
{
    zoomOrMenu(false);   // Menu
    connect(this, &QWidget::customContextMenuRequested, this, &CustomChartView::CustomMenu);
}

void CustomChartView::CustomMenu(const QPoint& pos)
{
    m_cursorPos = pos;
    QMenu menu;
    menu.addAction("Add Point", this, &CustomChartView::addNewPoint);

    // 找散点序列
    auto series = dynamic_cast<QXYSeries*>(chart()->series()[0]);
    if (series == nullptr)
        return;
    QPointF series_value = chart()->mapToValue(m_cursorPos, series);
    // 距离小于0.5的认为点在了点上
    QVector<QPointF> pv = series->pointsVector();
    int              index = -1;
    for (int i = 0; i < pv.size(); i++)
    {
        auto   dp = pv[i] - series_value;
        double dis = sqrt(dp.manhattanLength());
        if (dis < 0.5)
        {
            index = i;
            break;
        }
    }
    if (index != -1)
    {
        menu.addAction("Delete Point", [this, &index, &series]() {
            series->remove(index);
        });
    }

    if (m_mode == Click)
    {
        menu.addAction("Drag", [this]() {
            auto series = dynamic_cast<QXYSeries*>(chart()->series()[0]);
            if (series == nullptr)
                return;
            QPointF series_value = chart()->mapToValue(m_cursorPos, series);
            // 判断区间，确定插入序号
            QVector<QPointF> pv = series->pointsVector();
            m_dragIndex = -1;
            for (int i = 0; i < pv.size(); i++)
            {
                if (series_value.x() < pv[i].x())
                {
                    m_dragIndex = i;
                    break;
                }
            }
            if (m_dragIndex == -1)   // 在最后加的
                m_dragIndex = pv.size();

            series->insert(m_dragIndex, QPointF(series_value.x(), series_value.y()));
            m_mode = Drag;
            viewport()->setCursor(Qt::ClosedHandCursor);   // 设置鼠标样式为十字光标
        });
    }
    if (m_mode == Drag)
    {
        menu.addAction("End Drag", [this]() {
            m_mode = Click;
            m_dragIndex = -1;
            viewport()->setCursor(Qt::CrossCursor);   // 设置鼠标样式为十字光标
        });
    }

    menu.exec(mapToGlobal(pos));
}


void CustomChartView::addNewPoint()
{
    auto series = dynamic_cast<QXYSeries*>(chart()->series()[0]);
    if (series == nullptr)
        return;
    QPointF series_value = chart()->mapToValue(m_cursorPos, series);
    // 判断区间，确定插入序号
    QVector<QPointF> pv = series->pointsVector();
    int              index = 0;
    for (int i = 0; i < pv.size(); i++)
    {
        if (series_value.x() < pv[i].x())
        {
            index = i;
            break;
        }
    }
    bool   ok = true;
    double value = QInputDialog::getDouble(this, "input y value", "value:", series_value.y(), 0,65535, 3, &ok);
    if (ok)
        series->insert(index, QPointF(series_value.x(), value));
}

void CustomChartView::deletePoint()
{
    auto series = dynamic_cast<QXYSeries*>(chart()->series()[0]);
    if (series == nullptr)
        return;
    QPointF          series_value = chart()->mapToValue(m_cursorPos, series);
    QVector<QPointF> pv = series->pointsVector();
    int              index = 0;
    for (int i = 0; i < pv.size(); i++)
    {
        if (series_value.x() < pv[i].x())
        {
            index = i;
            break;
        }
    }
    series->remove(index);
}



void CustomChartView::movePoint()
{

}

void CustomChartView::selectSeries(const QPointF& point)
{

}

void CustomChartView::mouseMoveEvent(QMouseEvent* event)
{
    if (m_mode == Drag && m_dragIndex != -1)
    {
        // 曲线序列
        auto series = dynamic_cast<QXYSeries*>(chart()->series()[0]);
        if (series == nullptr)
            return;
        QPointF series_value = chart()->mapToValue(event->pos());
        // 判断区间，确定插入序号
        series->replace(m_dragIndex, series_value);
        qDebug() << "index: " << m_dragIndex << series_value;
    }
    QChartView::mouseMoveEvent(event);
}

void CustomChartView::zoomOrMenu(bool flag)
{
    if (flag)
    {
        setRubberBand(QChartView::RectangleRubberBand);   //拉伸效果
        setContextMenuPolicy(Qt::NoContextMenu);
    }
    else
    {
        setRubberBand(QChartView::NoRubberBand);
        setContextMenuPolicy(Qt::CustomContextMenu);
    }
}