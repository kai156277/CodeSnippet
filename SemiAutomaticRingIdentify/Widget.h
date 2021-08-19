#pragma once

#include <QAction>
#include <QSettings>
#include <QSpinBox>
#include <QWidget>

#include <opencv2/core.hpp>

#include "ActionButton.h"
#include "Graphics/DisplayGraphicsPixmapItem.h"
#include "Graphics/DisplayGraphicsScene.h"
#include "Graphics/DisplayGraphicsView.h"

#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui {
class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void when_openImageAction_triggered();
    void when_autoRingIdentifyAction_triggered();

private:
    Ui::Widget *ui;

    DisplayGraphicsView *      mImageView  = nullptr;
    DisplayGraphicsScene *     mImageScene = nullptr;
    DisplayGraphicsPixmapItem *mImageItem  = nullptr;

    QAction *mOpenImageAction        = nullptr;
    QAction *mAutoRingIdentifyAction = nullptr;

    QSpinBox *mCannyThreshold1 = nullptr;
    QSpinBox *mCannyThreshold2 = nullptr;

    QSpinBox *      mHoughThreshold     = nullptr;
    QDoubleSpinBox *mHoughMinLineLength = nullptr;
    QDoubleSpinBox *mHoughMaxLineGap    = nullptr;

    QSettings mConfig;

    cv::Mat mImage;
    cv::Mat mEdgeImage;
    cv::Mat mLineImage;

    std::vector<cv::Vec4f> mLines;
};
