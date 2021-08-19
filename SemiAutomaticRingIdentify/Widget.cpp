#include "Widget.h"
#include "ui_Widget.h"

#include <QFileDialog>
#include <QVBoxLayout>
#include <QVector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace cv::dnn;

static QString key_open_image_path = "open_image_path";

struct LineData
{
    double       average;
    QVector<int> datas;
};

void addPointToLine(QVector<LineData> &datas, int value, int threshold);

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , mConfig(qAppName() + ".ini", QSettings::IniFormat)
{
    ui->setupUi(this);

    mImageScene = new DisplayGraphicsScene;
    mImageView  = new DisplayGraphicsView(mImageScene);
    mImageItem  = new DisplayGraphicsPixmapItem;
    mImageItem->setPos(0, 0);
    mImageScene->addItem(mImageItem);

    QVBoxLayout *view_vlayout = new QVBoxLayout();
    view_vlayout->addWidget(mImageView);
    ui->groupBox_ImageView->setLayout(view_vlayout);

    mOpenImageAction        = new QAction("打开图像");
    mAutoRingIdentifyAction = new QAction("自动识别环缝");

    QVBoxLayout *action_vlayout = new QVBoxLayout();

    mCannyThreshold1 = new QSpinBox();
    mCannyThreshold1->setRange(0, 300);
    mCannyThreshold1->setValue(15);

    mCannyThreshold2 = new QSpinBox();
    mCannyThreshold2->setRange(0, 300);
    mCannyThreshold2->setValue(30);

    mHoughThreshold = new QSpinBox();
    mHoughThreshold->setRange(0, 1000);
    mHoughThreshold->setValue(250);

    mHoughMinLineLength = new QDoubleSpinBox();
    mHoughMinLineLength->setRange(0, 10000);
    mHoughMinLineLength->setValue(1300);

    mHoughMaxLineGap = new QDoubleSpinBox();
    mHoughMaxLineGap->setRange(0, 10000);
    mHoughMaxLineGap->setValue(800);

    action_vlayout->addWidget(new ActionButton(mOpenImageAction));
    action_vlayout->addWidget(mCannyThreshold1);
    action_vlayout->addWidget(mCannyThreshold2);
    action_vlayout->addWidget(mHoughThreshold);
    action_vlayout->addWidget(mHoughMinLineLength);
    action_vlayout->addWidget(mHoughMaxLineGap);
    action_vlayout->addWidget(new ActionButton(mAutoRingIdentifyAction));
    action_vlayout->addStretch();

    ui->scrollArea_operationAction->setLayout(action_vlayout);
    connect(mOpenImageAction, &QAction::triggered, this, &Widget::when_openImageAction_triggered);
    connect(mAutoRingIdentifyAction, &QAction::triggered, this, &Widget::when_autoRingIdentifyAction_triggered);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::when_openImageAction_triggered()
{
    QString def_path   = mConfig.value(key_open_image_path).toString();
    QString image_file = QFileDialog::getOpenFileName(this, "打开图像文件", def_path, tr("tiff (*.tiff *.gtiff *.geotiff)"));
    if (!image_file.isEmpty())
    {
        mConfig.setValue(key_open_image_path, image_file);

        mImageItem->setTiffPath(image_file);
        mImage = cv::imread(image_file.toUtf8().toStdString());
    }
}

void Widget::when_autoRingIdentifyAction_triggered()
{
    {
        cv::Mat gray_src;
        cv::cvtColor(mImage, gray_src, CV_BGR2GRAY);

        cv::blur(gray_src, gray_src, cv::Size(9, 9), cv::Point(-1, -1), cv::BORDER_DEFAULT);

        cv::GaussianBlur(gray_src, gray_src, cv::Size(3, 3), 5);

        int threshold1 = mCannyThreshold1->value();
        int threshold2 = mCannyThreshold2->value();
        cv::Canny(gray_src, mEdgeImage, threshold1, threshold2, 3, false);   //false表示用默认L1归一化
        // imshow("canny_image", mEdgeImage);
    }

    {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 1));
        morphologyEx(mEdgeImage, mEdgeImage, MORPH_CLOSE, kernel);

        kernel = getStructuringElement(MORPH_RECT, Size(1, 10));
        morphologyEx(mEdgeImage, mEdgeImage, MORPH_OPEN, kernel);
    }

    {
        Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));

        dilate(mEdgeImage, mEdgeImage, element);
    }
    {

        mLineImage = mImage.clone();
        // InputArray src,
        // 输入图像，必须8-bit的灰度图像；OutputArray lines,
        // 输出的极坐标来表示直线；double rho,
        // 生成极坐标时候的像素扫描步长；double theta,
        // 生成极坐标时候的角度步长，一般取值CV_PI/180；int threshold,
        // 阈值，只有获得足够交点的极坐标点才被看成是直线；double minLineLength = 0;
        // 最小直线长度；double maxLineGap = 0;
        // 最大间隔

        int    threshold     = mHoughThreshold->value();
        double minLineLength = mHoughMinLineLength->value();
        double maxLineGap    = mHoughMaxLineGap->value();
        HoughLinesP(mEdgeImage, mLines, 1, CV_PI / 180, threshold, minLineLength, maxLineGap);

        // 标记出直线
        for (size_t i = 0; i < mLines.size(); i++)
        {
            Vec4f point1 = mLines[i];
            line(mLineImage, Point(point1[0], point1[1]), Point(point1[2], point1[3]), Scalar(255, 0, 255), 1, LINE_AA);
        }
    }

    QVector<int> ring_set;
    ring_set.reserve(300);
    for (size_t i = 0; i < mLines.size(); i++)
    {
        ring_set.push_back((mLines[i][0] + mLines[i][2]) / 2);
    }

    QVector<LineData> lines;

    for (int i = 0; i < ring_set.size(); ++i)
    {
        addPointToLine(lines, ring_set[i], 30);
    }

    std::sort(lines.begin(), lines.end(), [](const LineData &a, const LineData &b) {
        return a.average < b.average;
    });
    for (int i = 0; i < lines.size(); ++i)
    {
        std::cout << "Line:" << i << "    " << int(lines[i].average) << std::endl;
    }
    for (size_t i = 0; i < lines.size(); i++)
    {
        int x = lines[i].average;
        line(mLineImage, Point(x, 0), Point(x, mLineImage.rows - 1), Scalar(255, 255, 255), 1, LINE_AA);
    }

    auto img = QImage(mLineImage.data, mLineImage.cols, mLineImage.rows, mLineImage.step, QImage::Format_RGB888);
    mImageItem->setImage(img);
}

void addPointToLine(QVector<LineData> &datas, int value, int threshold)
{
    bool in_current_line = false;
    for (int i = 0; i < datas.size(); ++i)
    {
        if (abs(datas[i].average - value) < threshold)
        {
            const int data_num = datas[i].datas.size();
            datas[i].average   = (datas[i].average * data_num + value) / (data_num + 1);
            datas[i].datas.push_back(value);
            in_current_line = true;
            break;
        }
    }
    if (!in_current_line)
    {
        LineData tmp;
        tmp.average = value;
        tmp.datas.push_back(value);
        datas.push_back(tmp);
    }
}
