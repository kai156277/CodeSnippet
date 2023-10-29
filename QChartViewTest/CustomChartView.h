#pragma once

#include <QAction>
#include <QChartView>
#include <QMouseEvent>
#include <QXYSeries>

class CustomChartView : public QT_CHARTS_NAMESPACE::QChartView
{
    Q_OBJECT
public:
    CustomChartView(QWidget *parent = nullptr);

    enum WorkMode
    {
        Click = 0,
        Drag
    };

public slots:
    // 设置缩放模式和右键菜单不是同时触发
    void zoomOrMenu(bool flag);
signals:
    void addPoint(int index, QPointF p);
    void removePoint(int index, QPointF p);
    void movePoint(int index, QPointF old_point, QPointF new_point);

private slots:
    void CustomMenu(const QPoint &pos);
    void addNewPoint();
    void deletePoint();
    void movePoint();

    void selectSeries(const QPointF &point);

private:
    //    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    //    void mouseReleaseEvent(QMouseEvent *event) override;

    void wheelEvent(QWheelEvent *event) override;

    QAction *mAddPoint    = nullptr;
    QAction *mDeletePoint = nullptr;

    QAction *mMovePoint = nullptr;
    QPoint   mCursorPos;

    WorkMode mMode      = Click;
    int      mDragIndex = -1;
};
