#pragma once

#include <QtCharts>
#include <QtCharts/qchart.h>
#include <qchartview.h>
#include <qlineseries.h>
#include <qmouseeventtransition.h>

class CustomChartView : public QT_CHARTS_NAMESPACE::QChartView
{
    Q_OBJECT
public:
    CustomChartView(QWidget* parent = nullptr);
    enum WorkMode
    {
        Click = 0,
        Drag
    };

public slots:
    // 设置缩放模式和右键菜单不是同时触发
    void zoomOrMenu(bool flag);

signals:
    void addPoints(int index, QPointF p);
    void removePoint(int index, QPointF p);
    void movePoint(int index, QPointF old_point, QPointF new_point);
   
private slots:
    void CustomMenu(const QPoint& pos);
    void addNewPoint();
    void deletePoint();
    void selectSeries(const QPointF& point);
    void movePoint();


private:
    void mouseMoveEvent(QMouseEvent* event) override;
    QAction* m_addPoint = nullptr;
    QAction* m_deletePoint = nullptr;

    QAction* m_movePoint = nullptr;
    QPoint   m_cursorPos;
    WorkMode m_mode = Click;
    int      m_dragIndex = -1;
};


