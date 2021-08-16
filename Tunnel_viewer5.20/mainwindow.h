#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "dialog_about.h"
#include "dialog_attribute.h"
#include "dialog_section.h"
#include "dialog_sections.h"

#include <QGLViewer/qglviewer.h>
#include <QMainWindow>
#include <QString>
#include <boost/thread/thread.hpp>
#include <ctime>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/features/boundary.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <qgl.h>
#include <sstream>
#include <string>
#include <vector>
#include <windows.h>   //计算运算时间
#include<QColor>
using namespace std;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    pcl::PointCloud<pcl::PointXYZ>     TargetCloud;//特征点
    pcl::PointCloud<pcl::PointXYZ>     object_cloud;
    pcl::PointCloud<pcl::PointXYZ>     display_cloud;
    QString                            LoadPath;
    QString                            SavePath;
    pcl::PointCloud<pcl::Normal>       object_normals;
    pcl::search::KdTree<pcl::PointXYZ> object_tree;
    int                                MARK            = -1;   //1表示走向沿X轴，0表示走向沿Y轴
    float                              everagedistance = -1;   //平均点密度
    vector<float>                      tunnel_direction;       //隧道精确走向x,y,z
    void                               copyPointCloud(pcl::PointCloud<pcl::PointXYZ> input_cloud);
    QColor PointColor=QColor(0,0,0);
    QColor BackColor=QColor(255,255,255);
    int PointSize=5;

    pcl::PointXYZ minPt, maxPt;
    bool          IsCloud = false;
    void          display(pcl::PointCloud<pcl::PointXYZ> display_cloud);
    void          DrawCube();
/*
    void GetAxisFromProjection();
    void GetAxisFromNormals();
    void GetSection_action();
    void GetSections_action();*/

protected Q_SLOTS:
    void selectTunnelType();
    void sectionFitting();

    void draw();

    void ReadData();
    void SaveData();
    void GetAxis();
    void ExtractSection();

    void ShowAbout();
    void GetAttributePoints();
    void deformation();
    void setting();



    void GetAxisFromProjection();
    void GetAxisFromNormals();
    void GetSection_action();
    void GetSections_action();

private slots:
    void on_actionspline_triggered();

    void on_actionbezie_triggered();

private:
    Ui::MainWindow *ui;
};

#endif   // MAINWINDOW_H
