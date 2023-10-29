#include "CustomChartView.h"
#include <QMenu>

#include <QDebug>
#include <QInputDialog>
#include <QMenu>
#include <QMessageBox>
#include <QXYSeries>

QT_CHARTS_USE_NAMESPACE

CustomChartView::CustomChartView(QWidget *parent)
    : QChartView(parent)
    , mAddPoint(new QAction("add point"))
    , mDeletePoint(new QAction("delte point"))
    , mMovePoint(new QAction("move point"))
{
    zoomOrMenu(false);   // Menu
    connect(this, &QWidget::customContextMenuRequested, this, &CustomChartView::CustomMenu);
}

void CustomChartView::CustomMenu(const QPoint &pos)
{
    mCursorPos = pos;
    QMenu menu;
    menu.addAction("Add Point", this, &CustomChartView::addNewPoint);

    // 找散点序列
    auto series = dynamic_cast<QXYSeries *>(chart()->series()[1]);
    if (series == nullptr)
        return;
    QPointF series_value = chart()->mapToValue(mCursorPos, series);
    // 距离小于0.5的认为点在了点上
    QVector<QPointF> pv    = series->pointsVector();
    int              index = -1;
    for (int i = 0; i < pv.size(); i++)
    {
        auto   dp  = pv[i] - series_value;
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

    if (mMode == Click)
    {
        menu.addAction("Drag", [this]() {
            auto series = dynamic_cast<QXYSeries *>(chart()->series()[0]);
            if (series == nullptr)
                return;
            QPointF series_value = chart()->mapToValue(mCursorPos, series);
            // 判断区间，确定插入序号
            QVector<QPointF> pv = series->pointsVector();
            mDragIndex          = -1;
            for (int i = 0; i < pv.size(); i++)
            {
                if (series_value.x() < pv[i].x())
                {
                    mDragIndex = i;
                    break;
                }
            }
            if (mDragIndex == -1)   // 在最后加的
                mDragIndex = pv.size();

            series->insert(mDragIndex, QPointF(series_value.x(), series_value.y()));
            mMode = Drag;
            viewport()->setCursor(Qt::ClosedHandCursor);   // 设置鼠标样式为十字光标
        });
    }
    if (mMode == Drag)
    {
        menu.addAction("End Drag", [this]() {
            mMode      = Click;
            mDragIndex = -1;
            viewport()->setCursor(Qt::CrossCursor);   // 设置鼠标样式为十字光标
        });
    }

    menu.exec(mapToGlobal(pos));
}

//void CustomChartView::mousePressEvent(QMouseEvent *event)
//{
//    if (mMode == Drag)
//    {
//        // 曲线序列
//        auto series = dynamic_cast<QXYSeries *>(chart()->series()[0]);
//        if (series == nullptr)
//            return;
//        QPointF series_value = chart()->mapToValue(mCursorPos, series);
//        // 判断区间，确定插入序号
//        QVector<QPointF> pv = series->pointsVector();
//        mDragIndex          = -1;
//        for (int i = 0; i < pv.size(); i++)
//        {
//            if (series_value.x() < pv[i].x())
//            {
//                mDragIndex = i;
//                break;
//            }
//        }
//        mDragIndex = pv.size();
//        series->insert(mDragIndex, QPointF(series_value.x(), series_value.y()));
//    }
//    QChartView::mousePressEvent(event);
//}
void CustomChartView::mouseMoveEvent(QMouseEvent *event)
{
    if (mMode == Drag && mDragIndex != -1)
    {
        // 曲线序列
        auto series = dynamic_cast<QXYSeries *>(chart()->series()[0]);
        if (series == nullptr)
            return;
        QPointF series_value = chart()->mapToValue(event->pos());
        // 判断区间，确定插入序号
        series->replace(mDragIndex, series_value);
        qDebug() << "index: " << mDragIndex << series_value;
    }
    QChartView::mouseMoveEvent(event);
}

void CustomChartView::wheelEvent(QWheelEvent *event)
{
#if (QT_VERSION <= QT_VERSION_CHECK(6, 0, 0))
    const auto pos      = QPointF(event->pos());
    const auto isZoomIn = event->delta() > 0;
#else
    const auto pos      = event->position();
    const auto isZoomIn = event->angleDelta().y() > 0;
#endif
    const QPointF curVal = chart()->mapToValue(pos);

    if (!alreadySaveRange_)
    {
        saveAxisRange();
        alreadySaveRange_ = true;
    }

    auto zoom = [](QValueAxis *axis, double centre, bool zoomIn) {
        constexpr auto scaling {1.5};

        const double down = axis->min();
        const double up   = axis->max();

        double downOffset {};
        double upOffset {};
        if (zoomIn)
        {
            downOffset = (centre - down) / scaling;
            upOffset   = (up - centre) / scaling;
        }
        else
        {
            downOffset = (centre - down) * scaling;
            upOffset   = (up - centre) * scaling;
        }

        axis->setRange(centre - downOffset, centre + upOffset);
    };

    if (ctrlPressed_)
    {
        auto axis = qobject_cast<QValueAxis *>(chart()->axisY());
        zoom(axis, curVal.y(), isZoomIn);
    }
    else
    {
        auto axis = qobject_cast<QValueAxis *>(chart()->axisX());
        zoom(axis, curVal.x(), isZoomIn);
    }
}
//void CustomChartView::mouseReleaseEvent(QMouseEvent *event)
//{
//    if (mMode == Drag)
//    {
//        mMode      = Click;
//        mDragIndex = -1;
//    }
//    QChartView::mouseReleaseEvent(event);
//}

void CustomChartView::addNewPoint()
{
    // 这个地方要判断是拿到了第几个序列, QXYSeries 要比 Spline 抽象程度高
    auto series = dynamic_cast<QXYSeries *>(chart()->series()[1]);
    if (series == nullptr)
        return;
    QPointF series_value = chart()->mapToValue(mCursorPos, series);
    // 判断区间，确定插入序号
    QVector<QPointF> pv    = series->pointsVector();
    int              index = 0;
    for (int i = 0; i < pv.size(); i++)
    {
        if (series_value.x() < pv[i].x())
        {
            index = i;
            break;
        }
    }
    bool   ok    = true;
    double value = QInputDialog::getDouble(this, "input y value", "value:", series_value.y(), -999.0, 999.0, 3, &ok);
    if (ok)
        series->insert(index, QPointF(series_value.x(), value));
    //    QMessageBox::information(this, "add point", "no");
}

void CustomChartView::deletePoint()
{
    auto series = dynamic_cast<QXYSeries *>(chart()->series()[1]);
    if (series == nullptr)
        return;
    QPointF          series_value = chart()->mapToValue(mCursorPos, series);
    QVector<QPointF> pv           = series->pointsVector();
    int              index        = 0;
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

void CustomChartView::selectSeries(const QPointF &point)
{
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
