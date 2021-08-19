#pragma once

#include <QGraphicsPixmapItem>
#include <QObject>

class DisplayGraphicsPixmapItem : public QObject
    , public QGraphicsPixmapItem
{
    Q_OBJECT
public:
    DisplayGraphicsPixmapItem(QGraphicsItem *parent = nullptr);

    void setTiffPath(const QString &file);
    void setImage(const QImage &image);
signals:

    void mousePressItemPos(const QPointF &pos);
    void mousePressScenePos(const QPointF &pos);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *moveEvent) override;
};
