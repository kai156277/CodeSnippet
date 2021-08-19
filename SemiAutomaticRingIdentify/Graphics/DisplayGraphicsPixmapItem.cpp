#include "DisplayGraphicsPixmapItem.h"

#include <QDebug>
#include <QGraphicsSceneMouseEvent>

DisplayGraphicsPixmapItem::DisplayGraphicsPixmapItem(QGraphicsItem *parent)
    : QGraphicsPixmapItem(parent)
{
}

void DisplayGraphicsPixmapItem::setTiffPath(const QString &file)
{
    QTransform trans = transform();
    // trans.scale(0.1, 0.1);
    setTransform(trans);
    QPixmap image_tif = QPixmap::fromImage(QImage(file));
    setPixmap(image_tif);
}

void DisplayGraphicsPixmapItem::setImage(const QImage &image)
{
    setPixmap(QPixmap::fromImage(image));
}

void DisplayGraphicsPixmapItem::mousePressEvent(QGraphicsSceneMouseEvent *moveEvent)
{
    emit mousePressScenePos(moveEvent->scenePos());
    emit mousePressItemPos(moveEvent->pos());

    QGraphicsPixmapItem::mousePressEvent(moveEvent);
}
