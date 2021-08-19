#include "DisplayGraphicsView.h"

#include <QMouseEvent>

DisplayGraphicsView::DisplayGraphicsView(QGraphicsScene *scene, QWidget *parent)
    : QGraphicsView(scene, parent)
{
    setMouseTracking(true);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setContextMenuPolicy(Qt::NoContextMenu);
}

void DisplayGraphicsView::wheelEvent(QWheelEvent *event)
{
    double numDegrees = event->delta() / 8.0;
    // TODO: 界面配置
    if (numDegrees < 0 && 0.09 > transform().m11())
        return;

    // 6m x 6m
    if (numDegrees > 0 && transform().m11() > 5)
        return;

    double numSteps = numDegrees / 10.0;
    double factor   = std::pow(1.25, numSteps);
    scale(factor, factor);
}

void DisplayGraphicsView::mousePressEvent(QMouseEvent *ev)
{
    QGraphicsView::mousePressEvent(ev);
    emit mousePressPos(ev->pos());
}

void DisplayGraphicsView::mouseMoveEvent(QMouseEvent *ev)
{
    QGraphicsView::mouseMoveEvent(ev);
    emit mouseMovePos(ev->pos());
}
