#pragma once

#include <QGraphicsScene>

class DisplayGraphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    DisplayGraphicsScene(QObject *parent = 0);

signals:
    void mouseMovePos(const QPointF &pos);
    void mousePressPos(const QPointF &pos);

protected:
    void mouseMoveEvent(QGraphicsSceneMouseEvent *moveEvent) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *moveEvent) override;
};
