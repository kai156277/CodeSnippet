#include "DisplayGraphicsScene.h"

#include <QGraphicsSceneMouseEvent>

DisplayGraphicsScene::DisplayGraphicsScene(QObject *parent)
    : QGraphicsScene(parent)
{
}

void DisplayGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *moveEvent)
{
    emit mouseMovePos(moveEvent->scenePos());

    QGraphicsScene::mouseMoveEvent(moveEvent);
}

void DisplayGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *moveEvent)
{
    emit mousePressPos(moveEvent->scenePos());

    QGraphicsScene::mousePressEvent(moveEvent);
}
