#pragma once

#include <QGraphicsView>

class DisplayGraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    DisplayGraphicsView(QGraphicsScene *scene, QWidget *parent = 0);

signals:
    void mouseMovePos(const QPoint &pos);
    void mousePressPos(const QPoint &pos);

protected:
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *ev) override;
    void mouseMoveEvent(QMouseEvent *ev) override;
};
