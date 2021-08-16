#include "mainwindow.h"

#include "ui_mainwindow.h"

#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <fstream>
#include <iostream>
#include <vector>
//QGLViewer
#include <QGLViewer/manipulatedCameraFrame.h>
#include <QGLViewer/manipulatedFrame.h>

// user
#include "SectionFitting.h"
#include "SectionFittingDialog.h"
#include "TunnelDefType.h"
#include "TunnelTypeDialog.h"
#include"ExtractionDialog.h"
#include"AxisDialog.h"
#include"palette.h"
//Opengl
#include <gl/GLU.h>
#include <qtextcodec.h>
//excel
#include"excelhelper.h"
#pragma execution_character_set("utf-8")

using namespace std;
using namespace qglviewer;
void GetXOYDirection(pcl::PointCloud<pcl::PointXYZ> object_cloud, vector<float> &direction, pcl::PointCloud<pcl::Normal> object_normals, pcl::search::KdTree<pcl::PointXYZ> object_tree);
void GetMARKDirection(pcl::PointCloud<pcl::PointXYZ> object_cloud, vector<float> &direction, pcl::PointCloud<pcl::Normal> object_normals, pcl::search::KdTree<pcl::PointXYZ> object_tree, int MARK);
void GetTunnelDirectionFromNormals(vector<float> &approximate_direction, pcl::PointCloud<pcl::Normal> object_normals);
void GetSection(pcl::PointCloud<pcl::PointXYZ> object_cloud, vector<float> direction, pcl::PointXYZ Point, float d, pcl::PointCloud<pcl::PointXYZ> &section);
void SplitString(const string &s, vector<string> &v, const string &c);
void ReadLine(QString datapath, vector<float> &axis);

//void MainWindow::copyPointCloud(pcl::PointCloud<pcl::PointXYZ> input_cloud);
//void display(pcl::PointCloud<pcl::PointXYZ> display_cloud);
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    ui->statusBar->showMessage(QString::fromUtf8("欢迎!"));
    connect(this->ui->actionOpen, SIGNAL(triggered(bool)), this, SLOT(ReadData()));
    connect(this->ui->actionSave, SIGNAL(triggered(bool)), this, SLOT(SaveData()));

    /*
    connect(this->ui->actionProjection_extract, SIGNAL(triggered(bool)), this, SLOT(GetAxisFromProjection()));
    connect(this->ui->actionNormals_extract, SIGNAL(triggered(bool)), this, SLOT(GetAxisFromNormals()));
    connect(this->ui->actionExtract_individually, SIGNAL(triggered(bool)), this, SLOT(GetSection_action()));
    connect(this->ui->actionExtract_iteratively, SIGNAL(triggered(bool)), this, SLOT(GetSections_action()));
    */

    connect(this->ui->actionset,SIGNAL(triggered(bool)),this,SLOT(setting()));
    connect(this->ui->actionAbout, SIGNAL(triggered(bool)), this, SLOT(ShowAbout()));
    connect(this->ui->viewer, SIGNAL(drawNeeded()), this, SLOT(draw()));
    connect(this->ui->action_tunnel_type_, SIGNAL(triggered(bool)), this, SLOT(selectTunnelType()));





    connect(this->ui->actionprojection,SIGNAL(triggered(bool)),this,SLOT(GetAxisFromProjection()));
    connect(this->ui->actionnormal,SIGNAL(triggered(bool)),this,SLOT(GetAxisFromNormals()));
    connect(this->ui->actionindividual,SIGNAL(triggered(bool)),this,SLOT(GetSection_action()));
    connect(this->ui->actioncontinues,SIGNAL(triggered(bool)),this,SLOT(GetSections_action()));
    connect(this->ui->actionspline,SIGNAL(triggered(bool)),this,SLOT());
    connect(this->ui->actionbezie,SIGNAL(triggered(bool)),this,SLOT());
    connect(this->ui->actionfeature,SIGNAL(triggered(bool)),this,SLOT(GetAttributePoints()));
    connect(this->ui->actionexcel,SIGNAL(triggered(bool)),this,SLOT(deformation()));





    connect(this->ui->action_section_fitting_, SIGNAL(triggered(bool)), this, SLOT(sectionFitting()));
    connect(this->ui->action_attribute_point_extraction_, SIGNAL(triggered(bool)), this, SLOT(GetAttributePoints()));
    connect(this->ui->action_deformation_analysis_,SIGNAL(triggered(bool)),this,SLOT(deformation()));
    connect(this->ui->action_section_extract_,SIGNAL(triggered(bool)),this,SLOT(ExtractSection()));
    connect(this->ui->Axis_extract,SIGNAL(triggered(bool)),this,SLOT(GetAxis()));
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::ShowAbout()
{
    Dialog_About a;
    a.exec();
}

void MainWindow::ReadData()
{
    LoadPath = QFileDialog::getOpenFileName(
        this, "LoadPath", "F:\\data\\TunnelViewer_data", "PointCloud files (*.pcd *.ply);;All files (*.*)");
    QTextCodec *code = QTextCodec::codecForName("GB2312");   //解决中文路径问题
    std::string name = code->fromUnicode(LoadPath).data();
    if (!LoadPath.isEmpty())
    {
        ui->statusBar->showMessage("开始加载文件，请稍等..");
        QFileInfo fileinfo;
        fileinfo = QFileInfo(LoadPath);
        //文件名
        //file_name = fileinfo.fileName();
        //文件后缀
        QString file_suffix = fileinfo.suffix();
        QString error_title = QString::fromUtf8("加载数据");
        QString error_msg   = QString::fromUtf8("无法读取此文件！");
        if (file_suffix == "pcd")
        {

            if (pcl::io::loadPCDFile<pcl::PointXYZ>(name, object_cloud) == -1)
            {
                QMessageBox::critical(nullptr, error_title, error_msg);
                return;
            }
        }
        else if (file_suffix == "ply")
        {
            if (pcl::io::loadPLYFile<pcl::PointXYZ>(name, object_cloud) == -1)
            {
                QMessageBox::critical(nullptr, error_title, error_msg);
                return;
            }
        }

        pcl::getMinMax3D(object_cloud, minPt, maxPt);
        qDebug()<<(maxPt.x + minPt.x) / 2<<endl;
        qDebug()<<(maxPt.y + minPt.y) / 2<<endl;
        qDebug()<<(maxPt.z + minPt.z) / 2<<endl;
        int     number = object_cloud.points.size();
        QString points = QString::number(number, 10);
        display(object_cloud);
        ui->statusBar->showMessage(QString::fromUtf8("加载点量: ") + points);
    }
}
void MainWindow::SaveData()
{
    QString error_title = QString::fromUtf8("保存数据");
    if (object_cloud.points.size() == 0)
    {
        QMessageBox::critical(nullptr, error_title, QString::fromUtf8("没有点被保存！"));
        return;
    }
    object_cloud.width  = object_cloud.points.size();
    object_cloud.height = 1;
    SavePath            = QFileDialog::getSaveFileName(this, tr("SavePath"), "D:\\qt_data\\tunnel\\processed\\SaveFile.pcd", tr("*.pcd *.ply"));
    QFileInfo fileinfo;
    fileinfo = QFileInfo(SavePath);
    //文件名
    //file_name = fileinfo.fileName();
    //文件后缀
    QString file_suffix = fileinfo.suffix();
    if (!SavePath.isNull())
    {
        if (object_cloud.points.size() != 0)
        {

            QTextCodec *code = QTextCodec::codecForName("GB2312");   //解决中文路径问题
            std::string name = code->fromUnicode(SavePath).data();

            if (file_suffix == "pcd")
            {
                pcl::io::savePCDFileASCII<pcl::PointXYZ>(name, object_cloud);
            }
            else if (file_suffix == "ply")
            {
                pcl::io::savePLYFileASCII<pcl::PointXYZ>(name, object_cloud);
            }

            ui->statusBar->showMessage(QString::fromUtf8("保存文件完成!"));
        }
        else
        {
            QMessageBox::critical(nullptr, error_title, QString::fromUtf8("没有点云数据文件"));
            return;
        }
    }
}
void MainWindow::GetAxisFromProjection()
{
    if (object_cloud.points.size() == 0)
    {
        QMessageBox::critical(this, QString::fromUtf8("投影提取轴线"), QString::fromUtf8("请先加载文件！"));
        return;
    }
    ui->statusBar->showMessage(QString::fromUtf8("开始提取轴线, 请稍候..."));
    tunnel_direction.clear();

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    object_normals.resize(object_cloud.points.size());

    if ((maxPt.y - minPt.y) < (maxPt.x - minPt.x))
    {
        MARK = 1;   //走向沿X轴
    }
    else
    {
        MARK = 0;   //走向沿Y轴
    }

    vector<float> XOYdirection, MARKdirection;   //投影后二维隧道走向

    GetXOYDirection(object_cloud, XOYdirection, object_normals, *tree);
    GetMARKDirection(object_cloud, MARKdirection, object_normals, *tree, MARK);

    if (MARK == 0)
    {
        tunnel_direction.push_back(XOYdirection[0]);                                         //取XOY平面的X
        tunnel_direction.push_back(XOYdirection[1]);                                         //取XOY平面的Y
        tunnel_direction.push_back(MARKdirection[2] * XOYdirection[1] / MARKdirection[1]);   //取以Y坐标比例转换后的Z
    }
    else
    {
        tunnel_direction.push_back(XOYdirection[0]);                                         //取XOY平面的X
        tunnel_direction.push_back(XOYdirection[1]);                                         //取XOY平面的Y
        tunnel_direction.push_back(MARKdirection[2] * XOYdirection[0] / MARKdirection[0]);   //取以X坐标比例转换后的Z
    }
    //QString direction0=QString::number(,10);
    qDebug() << tunnel_direction[0] << "," << tunnel_direction[1] << "," << tunnel_direction[2] << endl;
    ui->statusBar->showMessage(QString::fromUtf8("中轴线生成完成!"));
}
void MainWindow::GetAxisFromNormals()
{
    if (object_cloud.points.size() == 0)
    {
        QMessageBox::critical(this, QString::fromUtf8("点云法向量提取轴线"), QString::fromUtf8("请先加载点云文件"));
        return;
    }
    ui->statusBar->showMessage(QString::fromUtf8("开始提取轴线，请稍候..."));
    tunnel_direction.clear();
    if (object_normals.points.size() != object_cloud.points.size())
    {
        //计算各点法向
        //pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        *tree = object_tree;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;   //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
        normEst.setInputCloud(object_cloud.makeShared());
        normEst.setSearchMethod(tree);
        //normEst.setRadiusSearch(0.01);
        normEst.setKSearch(10);   //用于法向估计的近邻点数
        normEst.compute(object_normals);

        //        //计算平均点密度
        //        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
        //        kdtree.setInputCloud(object_cloud.makeShared());
        //        int k =2;
        //        //float everagedistance =0;
        //        for (int i =0; i < object_cloud.makeShared()->size()/2;i++)
        //            {
        //            vector<int> nnh ;
        //            vector<float> squaredistance;
        //            //  pcl::PointXYZ p;
        //            //   p = projection_cloud->points[i];
        //            kdtree.nearestKSearch(object_cloud.makeShared()->points[i],k,nnh,squaredistance);
        //            everagedistance += sqrt(squaredistance[1]);
        //            //   cout<<everagedistance<<endl;
        //            }
        //        everagedistance = everagedistance/(object_cloud.makeShared()->size()/2);
        //        //cout<<"everage distance is : "<<everagedistance<<endl;
    }
    GetTunnelDirectionFromNormals(tunnel_direction, object_normals);

    //计算轴向与X\Y轴夹角的余弦
    float normal    = sqrt(tunnel_direction[0] * tunnel_direction[0] +
                        tunnel_direction[1] * tunnel_direction[1] +
                        tunnel_direction[2] * tunnel_direction[2]);
    float COSthet_x = tunnel_direction[0] / normal;
    float COSthet_y = tunnel_direction[1] / normal;

    if (1 - abs(COSthet_x) < 1 - abs(COSthet_y))
    {
        MARK = 1;   //走向沿X轴
    }
    else
    {
        MARK = 0;   //走向沿Y轴
    }

    qDebug() << tunnel_direction[0] << "," << tunnel_direction[1] << "," << tunnel_direction[2] << endl;
    ui->statusBar->showMessage("中轴线生成完成!");
}
void MainWindow::ExtractSection()
{
    ExtractionDialog a;
    a.exec();
    if(a.method==0)
    {
        GetSection_action();
    }
    else if(a.method==1)
    {
        GetSections_action();
    }

}
void MainWindow::GetAxis()
{

    AxisDialog a;
    a.exec();
    if(a.method==0)
    {
        GetAxisFromProjection();
    }
    else if(a.method==1)
    {
        GetAxisFromNormals();
    }

}


void MainWindow::GetSection_action()
{

    if (object_cloud.points.size() == 0)
    {
        QMessageBox::information(NULL, "Error", "请先加载横断面文件!");
        return;
    }
    if (tunnel_direction.size() == 0)
    {
        QString                     dlgTitle   = QStringLiteral("没有中轴线信息");
        QString                     strInfo    = QStringLiteral("没有中轴线信息,是否导入自定义中轴线？");
        /*QMessageBox::StandardButton defaultBtn = QMessageBox::NoButton;
        QMessageBox::StandardButton result;
        result = QMessageBox::question(this, dlgTitle, strInfo, QMessageBox::Yes | QMessageBox::No, defaultBtn);
*/
        QMessageBox mess(QMessageBox::Warning, dlgTitle, strInfo, NULL);
        QPushButton *ok_Bt = mess.addButton(tr("确定"), QMessageBox::AcceptRole);
        QPushButton *cancel_Bt = mess.addButton(tr("取消"),QMessageBox::RejectRole);
        mess.exec();
        /*
        if (result == QMessageBox::Yes)
        {

        }
        else if()
        {

        }*/
        if ((QPushButton*)mess.clickedButton() == ok_Bt)
        {
            QString LinePath = QFileDialog::getOpenFileName(
                this, "Load Axis Path", "D:\\qt_data\\tunnel", "PointCloud files (*.line);");

            if (!LinePath.isEmpty())
            {
                vector<float> axis;

                QTextCodec *code      = QTextCodec::codecForName("GB2312");   //解决中文路径问题
                std::string name_line = code->fromUnicode(LinePath).data();
                QString     ss        = QString::fromStdString(name_line);
                ReadLine(QString::fromStdString(name_line), axis);
                tunnel_direction = axis;
                float normal     = sqrt(tunnel_direction[0] * tunnel_direction[0] +
                                    tunnel_direction[1] * tunnel_direction[1] +
                                    tunnel_direction[2] * tunnel_direction[2]);
                float COSthet_x  = tunnel_direction[0] / normal;
                float COSthet_y  = tunnel_direction[1] / normal;

                if (1 - abs(COSthet_x) < 1 - abs(COSthet_y))
                {
                    MARK = 1;   //走向沿X轴
                }
                else
                {
                    MARK = 0;   //走向沿Y轴
                }
                //ui->statusBar->showMessage("开始提取横断面,请稍等...");

                //以输入中轴线进行提取断面
                Dialog_Section a;
                a.exec();
                QTextCodec *code0 = QTextCodec::codecForName("GB2312");   //解决中文路径问题
                std::string name0 = code0->fromUnicode(a.SaveSectionPath).data();
                if (a.d == -1)
                {
                    return;
                }
                ui->statusBar->showMessage("开始提取横断面,请稍等...");
                pcl::PointXYZ point;
                if (MARK == 0)
                {
                    point.y = a.Mileage;
                    point.x = (maxPt.x + minPt.x) / 2;
                    point.z = (maxPt.z + minPt.z) / 2;
                }
                else if (MARK == 1)
                {
                    point.x = a.Mileage;
                    point.y = (maxPt.y + minPt.y) / 2;
                    point.z = (maxPt.z + minPt.z) / 2;
                }
                pcl::PointCloud<pcl::PointXYZ> section;
                GetSection(object_cloud, tunnel_direction, point, a.d, section);
                if (section.points.size() == 0)
                {
                    QMessageBox::information(NULL, "Error", "没有横断面点被提取,请重新设置参数!");
                    return;
                }
                section.width  = section.points.size();
                section.height = 1;
                pcl::io::savePCDFileASCII<pcl::PointXYZ>(name0, section);
                //显示断面
                copyPointCloud(section);
                display(section);
                ui->statusBar->showMessage("断面提取完成!");
            }
            else
            {
                //QMessageBox::information(NULL, "Error", "中轴线路径错误,请重新设置路径!");
                return;
            }
        }
        //ui->statusBar->showMessage("YES");
        else if ((QPushButton*)mess.clickedButton() == cancel_Bt)
        {
            //ui->statusBar->showMessage("NO");
            return;
        }
    }
    else
    {
        Dialog_Section a;
        a.exec();

        QTextCodec *code  = QTextCodec::codecForName("GB2312");   //解决中文路径问题
        std::string name0 = code->fromUnicode(a.SaveSectionPath).data();

        if (a.d == -1)
        {
            return;
        }
        ui->statusBar->showMessage("开始提取横断面,请稍等...");
        pcl::PointXYZ point;
        if (MARK == 0)
        {
            point.y = a.Mileage;
            point.x = (maxPt.x + minPt.x) / 2;
            point.z = (maxPt.z + minPt.z) / 2;
        }
        else if (MARK == 1)
        {
            point.x = a.Mileage;
            point.y = (maxPt.y + minPt.y) / 2;
            point.z = (maxPt.z + minPt.z) / 2;
        }
        pcl::PointCloud<pcl::PointXYZ> section;
        /*
        tunnel_direction[0]=-1*tunnel_direction[0];
        tunnel_direction[1]=-1*tunnel_direction[1];
        tunnel_direction[2]=-1*tunnel_direction[2];*/
        qDebug() <<MARK;
        qDebug() << tunnel_direction[0] << "," << tunnel_direction[1] << "," << tunnel_direction[2] << endl;
        GetSection(object_cloud, tunnel_direction, point, a.d, section);
        if (section.points.size() == 0)
        {
            QMessageBox::information(NULL, "Error", "没有横断面点被提取,请重新设置参数!");
            return;
        }
        section.width  = section.points.size();
        section.height = 1;
        pcl::io::savePCDFileASCII<pcl::PointXYZ>(name0, section);
        //显示断面
        copyPointCloud(section);
        display(section);
        ui->statusBar->showMessage("断面提取完成!");
    }
}
void MainWindow::GetSections_action()
{
    if (object_cloud.points.size() == 0)
    {
        QMessageBox::information(NULL, "Error", "请先加载横断面文件!");
        return;
    }
    if (tunnel_direction.size() == 0)
    {
        QString                     dlgTitle   = QStringLiteral("None axis");
        QString                     strInfo    = QStringLiteral("没有中轴线信息,是否导入自定义中轴线？");
        /*QMessageBox::StandardButton defaultBtn = QMessageBox::NoButton;
        QMessageBox::StandardButton result;
        result = QMessageBox::question(this, dlgTitle, strInfo, QMessageBox::Yes | QMessageBox::No, defaultBtn);
        */
        QMessageBox mess(QMessageBox::Warning, dlgTitle, strInfo, NULL);
        QPushButton *ok_Bt = mess.addButton(tr("确定"), QMessageBox::AcceptRole);
        QPushButton *cancel_Bt = mess.addButton(tr("取消"),QMessageBox::RejectRole);
        mess.exec();
        if ((QPushButton*)mess.clickedButton() == ok_Bt)
        {
            QString LinePath = QFileDialog::getOpenFileName(
                this, "Load Axis Path", "D:\\qt_data\\tunnel", "PointCloud files (*.line);");
            if (!LinePath.isEmpty())
            {
                vector<float> axis;

                QTextCodec *code      = QTextCodec::codecForName("GB2312");   //解决中文路径问题
                std::string name_line = code->fromUnicode(LinePath).data();

                //QString qstr2 = ;
                ReadLine(QString::fromStdString(name_line), axis);
                tunnel_direction = axis;
                float normal     = sqrt(tunnel_direction[0] * tunnel_direction[0] +
                                    tunnel_direction[1] * tunnel_direction[1] +
                                    tunnel_direction[2] * tunnel_direction[2]);
                float COSthet_x  = tunnel_direction[0] / normal;
                float COSthet_y  = tunnel_direction[1] / normal;

                if (1 - abs(COSthet_x) < 1 - abs(COSthet_y))
                {
                    MARK = 1;   //走向沿X轴
                }
                else
                {
                    MARK = 0;   //走向沿Y轴
                }

                Dialog_Sections a;
                a.exec();
                if (a.d == -1)
                {
                    return;
                }
                ui->statusBar->showMessage("开始提取横断面,请稍等...");

                float mile_min, mile_max, MileageMarkFromDirection;
                if (MARK == 0)
                {
                    //取y最大最小值
                    mile_min = minPt.y;
                    mile_max = maxPt.y;
                    //计算沿着走向的断面间隔
                    MileageMarkFromDirection = tunnel_direction[1] * a.MileageInterval / (sqrt(tunnel_direction[0] * tunnel_direction[0] + tunnel_direction[1] * tunnel_direction[1]));
                }
                else if (MARK == 1)
                {
                    //取x最大最小值
                    mile_min = minPt.x;
                    mile_max = maxPt.x;
                    //计算沿着走向的断面间隔
                    MileageMarkFromDirection = tunnel_direction[0] * a.MileageInterval / (sqrt(tunnel_direction[0] * tunnel_direction[0] + tunnel_direction[1] * tunnel_direction[1]));
                }
                pcl::PointCloud<pcl::PointXYZ> sections;
                for (float ii = mile_min + 2 * a.d; ii < mile_max; ii = ii + abs(MileageMarkFromDirection))
                {
                    pcl::PointXYZ point;

                    if (MARK == 0)
                    {
                        point.y = ii;
                        point.x = (maxPt.x + minPt.x) / 2;
                        point.z = (maxPt.z + minPt.z) / 2;
                    }
                    else if (MARK == 1)
                    {
                        point.x = ii;
                        point.y = (maxPt.y + minPt.y) / 2;
                        point.z = (maxPt.z + minPt.z) / 2;
                    }

                    pcl::PointCloud<pcl::PointXYZ> section;
                    GetSection(object_cloud, tunnel_direction, point, a.d, section);
                    if (section.points.size() == 0)
                    {
                        ii = ii - abs(MileageMarkFromDirection) + a.d;
                        continue;
                    }
                    if (ii == mile_min + 2 * a.d)
                    {
                        sections = section;
                    }
                    else
                    {
                        sections = sections + section;
                    }

                    section.width  = section.points.size();
                    section.height = 1;

                    QString name      = QString("\\Section%1.pcd").arg(ii);
                    QString finalPath = a.SaveSectionPath + name;

                    QTextCodec *code  = QTextCodec::codecForName("GB2312");   //解决中文路径问题
                    std::string name0 = code->fromUnicode(finalPath).data();

                    pcl::io::savePCDFileASCII<pcl::PointXYZ>(name0, section);
                    ui->statusBar->showMessage(finalPath + " complete!");
                }

                copyPointCloud(sections);
                display(object_cloud);
                ui->statusBar->showMessage("断面提取完成!");
            }
            else
            {
                //QMessageBox::information(NULL, "Error", "中轴线路径错误,请重新设置路径!");
                return;
            }
        }
        else if ((QPushButton*)mess.clickedButton() == cancel_Bt)
        {
            //ui->statusBar->showMessage("NO");
            return;
        }
    }
    else
    {
        Dialog_Sections a;
        a.exec();
        if (a.d == -1)
        {
            return;
        }
        ui->statusBar->showMessage("开始提取横断面,请稍等...");

        float mile_min, mile_max, MileageMarkFromDirection;
        if (MARK == 0)
        {
            //取y最大最小值
            mile_min = minPt.y;
            mile_max = maxPt.y;
            //计算沿着走向的断面间隔
            MileageMarkFromDirection = tunnel_direction[1] * a.MileageInterval / (sqrt(tunnel_direction[0] * tunnel_direction[0] + tunnel_direction[1] * tunnel_direction[1]));
        }
        else if (MARK == 1)
        {
            //取x最大最小值
            mile_min = minPt.x;
            mile_max = maxPt.x;
            //计算沿着走向的断面间隔
            MileageMarkFromDirection = tunnel_direction[0] * a.MileageInterval / (sqrt(tunnel_direction[0] * tunnel_direction[0] + tunnel_direction[1] * tunnel_direction[1]));
        }
        pcl::PointCloud<pcl::PointXYZ> sections;
        for (float ii = mile_min + 2 * a.d; ii < mile_max; ii = ii + abs(MileageMarkFromDirection))
        {
            pcl::PointXYZ point;
            if (MARK == 0)
            {
                point.y = ii;
                point.x = (maxPt.x + minPt.x) / 2;
                point.z = (maxPt.z + minPt.z) / 2;
            }
            else if (MARK == 1)
            {
                point.x = ii;
                point.y = (maxPt.y + minPt.y) / 2;
                point.z = (maxPt.z + minPt.z) / 2;
            }

            pcl::PointCloud<pcl::PointXYZ> section;
            GetSection(object_cloud, tunnel_direction, point, a.d, section);
            if (section.points.size() == 0)
            {
                ii = ii - abs(MileageMarkFromDirection) + a.d;
                continue;
            }
            if (ii == mile_min + 2 * a.d)
            {
                sections = section;
            }
            else
            {
                sections = sections + section;
            }

            section.width  = section.points.size();
            section.height = 1;

            QString name      = QString("\\Section%1.pcd").arg(ii);
            QString finalPath = a.SaveSectionPath + name;

            QTextCodec *code  = QTextCodec::codecForName("GB2312");   //解决中文路径问题
            std::string name0 = code->fromUnicode(finalPath).data();

            pcl::io::savePCDFileASCII<pcl::PointXYZ>(name0, section);
            ui->statusBar->showMessage(finalPath + " complete!");
        }

        copyPointCloud(sections);
        display(object_cloud);
        ui->statusBar->showMessage("断面提取完成!");
    }
}

void MainWindow::GetAttributePoints()
{
    if (object_cloud.points.size() == 0)
    {
        QMessageBox::information(NULL, "Error", "请先加载横断面文件!");
        return;
    }
    Dialog_Attribute a;
    a.exec();
    if (a.parameter.size() == 0)
    {
        return;
    }
    ui->statusBar->showMessage("开始计算,请稍等!");

    if (maxPt.y - minPt.y > maxPt.x - minPt.x)   //断面走向方向比较薄
    {
        MARK = 1;   //走向沿X轴
    }
    else
    {
        MARK = 0;   //走向沿Y轴
    }

    float Mark0;
    if (MARK == 0)
    {
        Mark0 = (maxPt.x + minPt.x) / 2;
    }
    else if (MARK == 1)
    {
        Mark0 = (maxPt.y + minPt.y) / 2;
    }
    //pcl::PointCloud<pcl::PointXYZ> TargetCloud;
    float                          highest  = minPt.z;
    int                            TopIndex = -1;

    vector<int>   LeftIndex, RightIndex;
    vector<float> LeftD, RightD;
    int           size = a.parameter.size();
    LeftIndex.resize(size);
    RightIndex.resize(size);
    LeftD.resize(size);
    RightD.resize(size);
    for (int i = 0; i < size; i++)
    {
        LeftD[i]  = 99999999;
        RightD[i] = 99999999;
    }
    for (int i = 0; i < object_cloud.points.size(); i++)
    {
        if (object_cloud.points[i].z > highest)
        {
            TopIndex = i;
            highest  = object_cloud.points[i].z;
        }

        for (int j = 0; j < size; j++)
        {
            float d = abs(object_cloud.points[i].z - a.parameter[j]);
            if (MARK == 0)   //走向沿Y轴
            {
                if (object_cloud.points[i].x > Mark0)   //右侧
                {
                    if (d < RightD[j])
                    {
                        RightIndex[j] = i;
                        RightD[j]     = d;
                    }
                }
                else   //左侧
                {
                    if (d < LeftD[j])
                    {
                        LeftIndex[j] = i;
                        LeftD[j]     = d;
                    }
                }
            }
            else   //走向沿X轴
            {
                if (object_cloud.points[i].y > Mark0)   //右侧
                {
                    if (d < RightD[j])
                    {
                        RightIndex[j] = i;
                        RightD[j]     = d;
                    }
                }
                else   //左侧
                {
                    if (d < LeftD[j])
                    {
                        LeftIndex[j] = i;
                        LeftD[j]     = d;
                    }
                }
            }
        }
    }

    TargetCloud.clear();
    if (a.TopPoint)
    {
        TargetCloud.points.push_back(object_cloud.points[TopIndex]);
    }
    for (int i = 0; i < size; i++)
    {
        TargetCloud.points.push_back(object_cloud.points[LeftIndex[i]]);
        TargetCloud.points.push_back(object_cloud.points[RightIndex[i]]);
    }

    //copyPointCloud(TargetCloud);
    display(TargetCloud);
    ui->statusBar->showMessage("特征点提取完成!");
}

void MainWindow::copyPointCloud(pcl::PointCloud<pcl::PointXYZ> input_cloud)
{
    object_cloud = input_cloud;
    object_normals.clear();
    object_tree.setInputCloud(object_cloud.makeShared());
    MARK            = -1;       //1表示走向沿X轴，0表示走向沿Y轴
    everagedistance = -1;       //平均点密度
    tunnel_direction.clear();   //隧道精确走向x,y,z
    pcl::getMinMax3D(object_cloud, minPt, maxPt);
}
void MainWindow::draw()
{
#ifdef GL_RESCALE_NORMAL   // OpenGL 1.2 Only...
    glEnable(GL_RESCALE_NORMAL);
#endif

    if (IsCloud == false)
    {
        glClearColor( (GLubyte)BackColor.red(), (GLubyte)BackColor.green(), (GLubyte)BackColor.blue() ,0);

        // Enable GL textures
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glEnable(GL_TEXTURE_2D);

        // Nice texture coordinate interpolation
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

        // Texture parameters
        float u_max = 1.0;
        float v_max = 1.0;
        float ratio = 1.0;










        QString name = ":/new/prefix1/tunnel_back.jpg";


        // In case of Cancel
        if (name.isEmpty())
          return;

        QImage img(name);

        if (img.isNull()) {
          qWarning("Unable to load file, unsupported file format");
          return;
        }

      #if QT_VERSION < 0x040000
        qWarning("Loading %s, %dx%d pixels", name.latin1(), img.width(),
                 img.height());
      #else
        qWarning("Loading %s, %dx%d pixels", name.toLatin1().constData(), img.width(),
                 img.height());
      #endif

        // 1E-3 needed. Just try with width=128 and see !
        int newWidth = 1 << (int)(1 + log(img.width() - 1 + 1E-3) / log(2.0));
        int newHeight = 1 << (int)(1 + log(img.height() - 1 + 1E-3) / log(2.0));

        u_max = img.width() / (float)newWidth;
        v_max = img.height() / (float)newHeight;

        if ((img.width() != newWidth) || (img.height() != newHeight)) {
          qWarning("Image size set to %dx%d pixels", newWidth, newHeight);
          img = img.copy(0, 0, newWidth, newHeight);
        }

        ratio = newWidth / float(newHeight);

        QImage glImg = QGLWidget::convertToGLFormat(img); // flipped 32bit RGBA

        // Bind the img texture...
        glTexImage2D(GL_TEXTURE_2D, 0, 4, glImg.width(), glImg.height(), 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, glImg.bits());







        // Display the quad
        glNormal3f(0.0, 0.0, 1.0);
        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 1.0 - v_max);
        glVertex2f(-u_max * ratio, -v_max);
        glTexCoord2f(0.0, 1.0);
        glVertex2f(-u_max * ratio, v_max);
        glTexCoord2f(u_max, 1.0);
        glVertex2f(u_max * ratio, v_max);
        glTexCoord2f(u_max, 1.0 - v_max);
        glVertex2f(u_max * ratio, -v_max);
        glEnd();


        /*
         * // Draws a spiral
        //glClear(GL_COLOR_BUFFER_BIT);
        const float nbSteps = 200.0;
        glBegin(GL_QUAD_STRIP);
        for (float i = 0; i < nbSteps; ++i)
        {
            float       ratio = i / nbSteps;
            float       angle = 21.0 * ratio;
            float       c     = cos(angle);
            float       s     = sin(angle);
            float       r1    = 1.0 - 0.8 * ratio;
            float       r2    = 0.8 - 0.8 * ratio;
            float       alt   = ratio - 0.5;
            const float nor   = .5;
            const float up    = sqrt(1.0 - nor * nor);
            glColor3f(1.0 - ratio, 0.2f, ratio);
            glNormal3f(nor * c, up, nor * s);
            glVertex3f(r1 * c, alt, r1 * s);
            glVertex3f(r2 * c, alt + 0.05, r2 * s);
        }
        glEnd();
        glFinish();
*/
    }
    else
    {

        glClearColor( (float)BackColor.red()/255.0, (float)BackColor.green()/255.0, (float)BackColor.blue()/255.0  ,0);


        // Make sure the manipulatedFrame is not easily clipped by the zNear and zFar
        // planes

        this->ui->viewer->setSceneRadius(30);


        vector<float> centerPoint;
        centerPoint.push_back((minPt.x + maxPt.x) / 2);
        centerPoint.push_back((minPt.y + maxPt.y) / 2);
        centerPoint.push_back((minPt.z + maxPt.z) / 2);




         qglviewer::Vec CamPos;

        CamPos.x = centerPoint[0];
        CamPos.y = centerPoint[1];
        CamPos.z = centerPoint[2];


        glDisable(GL_LIGHT0);
         glEnable(GL_LIGHT1);

          // Light default parameters
          const GLfloat light_ambient[4] = {1.0, 1.0, 1.0, 1.0};
          const GLfloat light_specular[4] = {1.0, 1.0, 1.0, 1.0};
          const GLfloat light_diffuse[4] = {1.0, 1.0, 1.0, 1.0};

          glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 3.0);
          glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 10.0);
          glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 0.1f);
          glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.3f);
          glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.3f);
          glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
          glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
          glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);


          const Vec cameraPos = this->ui->viewer->camera()->position();
          const GLfloat pos[4] = {(float)cameraPos[0], (float)cameraPos[1],
                                    (float)cameraPos[2], 1.0f};
            glLightfv(GL_LIGHT1, GL_POSITION, pos);

            // Orientate light along view direction
            glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, this->ui->viewer->camera()->viewDirection());

















         glPointSize(PointSize);
        glBegin(GL_POINTS);
        float nbSteps = display_cloud.points.size();
        for (float i = 0; i < nbSteps; ++i)
        {
            qglviewer::Vec Pt;
            Pt.x = display_cloud.points[i].x - centerPoint[0];
            Pt.y = display_cloud.points[i].y - centerPoint[1];
            Pt.z = display_cloud.points[i].z - centerPoint[2];
            //glNormal3f(1,0,0);


            glColor3ub((GLubyte)PointColor.red(), (GLubyte)PointColor.green(), (GLubyte)PointColor.blue() );
            //QColor(0,255,0);
           // glColor3ub(0.3f, 0.3f, 0.3f);



            //glVertex3f(display_cloud.points[i].x - centerPoint[0], display_cloud.points[i].y - centerPoint[1], display_cloud.points[i].z - centerPoint[2]);
            glVertex3fv(Pt);
        }
        glEnd();
    }

    //this->DrawCube();

    glFinish();
    this->ui->viewer->update();
}
void MainWindow::setting()
{
    Palette a;
    a.exec();
    if(a.update==1)
    {
        BackColor=a.BackColor;
        PointColor=a.PointColor;
        PointSize=a.PointSize;
        a.update=0;
    }

}

void MainWindow::DrawCube()
{
    glBegin(GL_QUADS);
    glColor3ub(255u, 255u, 0u);   //颜色设置为黄色
    //0----------------------------
    glNormal3f(0.0F, 0.0F, 1.0F);

    glVertex3f(maxPt.x, maxPt.y, maxPt.z);
    glVertex3f(minPt.x, maxPt.y, maxPt.z);
    glVertex3f(minPt.x, minPt.y, maxPt.z);
    glVertex3f(maxPt.x, minPt.y, maxPt.z);
    //1----------------------------
    //this is to show the face is remnant
    glNormal3f(0.0F, 0.0F, -1.0F);

    glVertex3f(minPt.x, minPt.y, minPt.z);
    glVertex3f(minPt.x, maxPt.y, minPt.z);
    glVertex3f(maxPt.x, maxPt.y, minPt.z);
    glVertex3f(maxPt.x, minPt.y, minPt.z);
    //2----------------------------
    glNormal3f(0.0F, 1.0F, 0.0F);

    glVertex3f(maxPt.x, maxPt.y, maxPt.z);
    glVertex3f(maxPt.x, maxPt.y, minPt.z);
    glVertex3f(minPt.x, maxPt.y, minPt.z);
    glVertex3f(minPt.x, maxPt.y, maxPt.z);
    //3----------------------------
    glNormal3f(0.0F, -1.0F, 0.0F);

    glVertex3f(minPt.x, minPt.y, minPt.z);
    glVertex3f(maxPt.x, minPt.y, minPt.z);
    glVertex3f(maxPt.x, minPt.y, maxPt.z);
    glVertex3f(minPt.x, minPt.y, maxPt.z);
    //4----------------------------
    glNormal3f(1.0F, 0.0F, 0.0F);

    glVertex3f(maxPt.x, maxPt.y, maxPt.z);
    glVertex3f(maxPt.x, minPt.y, maxPt.z);
    glVertex3f(maxPt.x, minPt.y, minPt.z);
    glVertex3f(maxPt.x, maxPt.y, minPt.z);
    //5----------------------------
    glNormal3f(-1.0F, 0.0F, 0.0F);

    glVertex3f(minPt.x, minPt.y, minPt.z);
    glVertex3f(minPt.x, minPt.y, maxPt.z);
    glVertex3f(minPt.x, maxPt.y, maxPt.z);
    glVertex3f(minPt.x, maxPt.y, minPt.z);
    glEnd();
}

void MainWindow::selectTunnelType()
{
    TunnelTypeDialog dialog(this);
    dialog.exec();
}

void MainWindow::sectionFitting()
{
    SectionFittingDialog dialog(this);
    SectionFitting::Type type = SectionFitting::None;
    if (dialog.exec() == QDialog::Accepted)
    {
        type = dialog.selectType();
        if (type == SectionFitting::None)
        {
            QMessageBox::critical(this, QString::fromUtf8("断面拟合"), QString::fromUtf8("请选择断面类型"));
            return;
        }
    }
    else
        return;

    if (object_cloud.points.size() == 0)
    {
        QMessageBox::critical(this, QString::fromUtf8("断面拟合"), QString::fromUtf8("请先加载断面文件"));
        return;
    }

    ui->statusBar->showMessage(QString::fromUtf8("开始拟合"));
    TunnelPointCloud::Ptr Bspline_vector(new TunnelPointCloud);
    int                   numberofnewpoint = 4;

    SectionFitting fitting;
    fitting.async_fitting(type, object_cloud, *Bspline_vector, numberofnewpoint);
    connect(&fitting, &SectionFitting::processNum, [this, type](int num) {
        ui->statusBar->showMessage(SectionFitting::TypeToQString(type) + QString::fromUtf8("进度： ") + QString::number(num));
    });

    QMessageBox::information(this, QString::fromUtf8("断面拟合"), QString::fromUtf8("正在处理请稍候"));

    copyPointCloud(*Bspline_vector);
    display(object_cloud);
    ui->statusBar->showMessage(QString::fromUtf8("断面拟合完成!"));
}
void MainWindow::display(pcl::PointCloud<pcl::PointXYZ> display)
{

    if (display.points.size() > 0)
    {
        display_cloud = display;
        IsCloud       = true;
    }

    //ui->textEdit->setPlainText("Display complete!");
}
bool ExcelHelper::ExportDataToExcel(const QString &fileName,pcl::PointCloud<pcl::PointXYZ> cloud,int direction ,double h)
{
  if(!newExcel(fileName))
        return false;


    setCellValue(1,1,"隧道断面形变分析报告");
    mergeCells("A1:G1");
    borders("A1:G1");



    setCellValue(2,1,"处理人:");
    borders(2,1);
    //setCellValue(2,2,"people");
    borders(2,2);

    setCellValue(2,3,"日期:");
    borders(2,3);



    SYSTEMTIME sys;
    GetLocalTime( &sys );

    QString date=QString::number(sys.wYear)+"/"+QString::number(sys.wMonth)+"/"+QString::number(sys.wDay)+" "+QString::number(sys.wHour)+":"+QString::number(sys.wMinute);
    cout<<date.toStdString();



    setCellValue(2,4,date);
    mergeCells("D2:G2");
    borders("D2:G2");

    setCellValue(3,1,"隧道高度");
    mergeCells("A3:B3");
    borders("A3:B3");
    setCellValue(3,3,QString::number(h));
    mergeCells("C3:G3");
    borders("C3:G3");

    setCellValue(4,1,"特征点坐标");
    mergeCells("A4:D4");
    borders("A4:D4");

    setCellValue(4,5,"隧道横距");
    mergeCells("E4:G5");
    borders("E4:G5");

    setCellValue(5,1,"点号");
    borders("A5:A5");
    setCellValue(5,2,"X");
    borders("B5:B5");
    setCellValue(5,3,"Y");
    borders("C5:C5");
    setCellValue(5,4,"Z");
    borders("D5:D5");

    if(cloud.points.size()%2==0)
    {
        for(int i=0;i<(cloud.points.size())/2;i++)//4组特征点
        {

            for(int j=0;j<2;j++)//左右两个点
            {
                setCellValue(6+2*i+j,1,QString::number(2*i+j));
                borders(6+2*i+j,1);
                setCellValue(6+2*i+j,2,QString::number(cloud.points[2*i+j].x));
                borders(6+2*i+j,2);
                setCellValue(6+2*i+j,3,QString::number(cloud.points[2*i+j].y));
                borders(6+2*i+j,3);
                setCellValue(6+2*i+j,4,QString::number(cloud.points[2*i+j].z));
                borders(6+2*i+j,4);
            }

            if(direction==0)
            {
                double width=abs(cloud.points[2*i].x-cloud.points[2*i+1].x);
                setCellValue(6+2*i,5,QString::number(width));
            }
            else if(direction==1)
            {
                double width=abs(cloud.points[2*i].y-cloud.points[2*i+1].y);
                setCellValue(6+2*i,5,QString::number(width));
            }

            mergeCells(6+2*i,5,6+2*i+1,7);
            borders(6+2*i,5,6+2*i+1,7);
        }

    }
    else
    {
        setCellValue(6,1,QString::number(0));
        borders(6,1);
        setCellValue(6,2,QString::number(cloud.points[0].x));
        borders(6,2);
        setCellValue(6,3,QString::number(cloud.points[0].y));
        borders(6,3);
        setCellValue(6,4,QString::number(cloud.points[0].z));
        borders(6,4);
        setCellValue(6,5,QString::number(0));
        borders(6,5,6,7);
        mergeCells(6,5,6,7);
        for(int i=0;i<(cloud.points.size()-1)/2;i++)//4组特征点
        {

            for(int j=0;j<2;j++)//左右两个点
            {
                setCellValue(7+2*i+j,1,QString::number(2*i+j+1));
                borders(7+2*i+j,1);
                setCellValue(7+2*i+j,2,QString::number(cloud.points[2*i+j+1].x));
                borders(7+2*i+j,2);
                setCellValue(7+2*i+j,3,QString::number(cloud.points[2*i+j+1].y));
                borders(7+2*i+j,3);
                setCellValue(7+2*i+j,4,QString::number(cloud.points[2*i+j+1].z));
                borders(7+2*i+j,4);
            }

            if(direction==0)
            {
                double width=abs(cloud.points[2*i+1].x-cloud.points[2*i+1+1].x);
                setCellValue(7+2*i,5,QString::number(width));
            }
            else if(direction==1)
            {
                double width=abs(cloud.points[2*i+1].y-cloud.points[2*i+1+1].y);
                setCellValue(7+2*i,5,QString::number(width));
            }

            mergeCells(7+2*i,5,7+2*i+1,7);
            borders(7+2*i,5,7+2*i+1,7);
        }

    }





    saveExcel(fileName);
    freeExcel();
    return true;
}
void MainWindow::deformation()
{
    if (TargetCloud.points.size() == 0)
    {
        QMessageBox::critical(this, QString::fromUtf8("形变分析"), QString::fromUtf8("请先提取特征点"));
        return;
    }
    QString savePath    = QFileDialog::getSaveFileName(this, tr("SavePath"), "F:\\毕业论文\\隧道变形分析报表.xlsx", tr("*.xlsx"));
    ui->statusBar->showMessage(QString::fromUtf8("开始计算"));
    ExcelHelper excel;
    excel.ExportDataToExcel(savePath,TargetCloud,MARK,maxPt.z-minPt.z);

/*
    QFile file(savePath);
     if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
     {
         cout<<"Fail to open the file"<<endl;

     }

     QTextStream write_stream(&file);
     write_stream.setRealNumberNotation(QTextStream::FixedNotation);//设置输出浮点数的表示方法
     write_stream.setFieldAlignment(QTextStream::AlignRight);//设置输出对齐方式
     write_stream.setFieldWidth(10);//设置字段宽度
     write_stream<<"X"<<","<<"Y"<<","<<"Z"<<","<<"length"<<endl;
     pcl::PointXYZ min,max;
     pcl::getMinMax3D(object_cloud, min, max);
     if(MARK==0)
     {
         double mean_x=0.5*(min.x+max.x);
         for(int i=0;i<TargetCloud.points.size();i++)
         {
             write_stream<<TargetCloud.points[i].x<<","<<TargetCloud.points[i].y<<","<<TargetCloud.points[i].z<<","
                        <<TargetCloud.points[i].x-mean_x<<"\n";
         }

         file.close();
     }
     else if(MARK==1)
     {
         double mean_y=0.5*(min.y+max.y);
         for(int i=0;i<TargetCloud.points.size();i++)
         {
             write_stream<<TargetCloud.points[i].x<<","<<TargetCloud.points[i].y<<","<<TargetCloud.points[i].z<<","
                        <<TargetCloud.points[i].y-mean_y<<"\n";
         }
         file.close();
     }
     else
     {
         file.close();
         QMessageBox::critical(this, QString::fromUtf8("形变分析"), QString::fromUtf8("无断面方向"));
         return;
     }*/
     ui->statusBar->showMessage(QString::fromUtf8("报表完成!"));
}

void GetXOYDirection(pcl::PointCloud<pcl::PointXYZ> object_cloud, vector<float> &direction, pcl::PointCloud<pcl::Normal> object_normals, pcl::search::KdTree<pcl::PointXYZ> object_tree)
{
    vector<vector<float>> direction_vector;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    *tree = object_tree;
    //进行平面投影
    pcl::PointCloud<pcl::PointXYZ>::Ptr projection_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *projection_cloud = object_cloud;
    for (int i = 0; i < projection_cloud->points.size(); i++)
    {
        projection_cloud->points[i].z = 0;
    }
    //计算平均点密度
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;   //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
    kdtree.setInputCloud(projection_cloud);
    int   k               = 2;
    float everagedistance = 0;
    for (int i = 0; i < projection_cloud->size() / 2; i++)
    {
        vector<int>   nnh;
        vector<float> squaredistance;
        //  pcl::PointXYZ p;
        //   p = projection_cloud->points[i];
        kdtree.nearestKSearch(projection_cloud->points[i], k, nnh, squaredistance);
        everagedistance += sqrt(squaredistance[1]);
        //   cout<<everagedistance<<endl;
    }
    everagedistance = everagedistance / (projection_cloud->size() / 2);
    // cout << "everage distance is : " << everagedistance << endl;
    //计算投影后法向量
    pcl::PointCloud<pcl::Normal>::Ptr projection_normals(new pcl::PointCloud<pcl::Normal>);
    *projection_normals = object_normals;
    for (int i = 0; i < projection_normals->points.size(); i++)
    {
        pcl::Normal tem_normals;
        tem_normals.normal_x          = 0;
        tem_normals.normal_y          = 0;
        tem_normals.normal_z          = 1;
        projection_normals->points[i] = tem_normals;
        //projection_normals->points[i].
    }
    //计算边界
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    pcl::PointCloud<pcl::Boundary>                                     boundaries;

    est.setInputCloud(projection_cloud);
    est.setInputNormals(projection_normals);

    est.setSearchMethod(tree);
    //est.setKSearch(50);  //一般这里的数值越高，最终边界识别的精度越好
    est.setRadiusSearch(everagedistance);   //搜索半径
    double pi = 3.14159265358979323846;
    //est.setAngleThreshold(pi);
    std::cout << "start to compute boundaries..." << std::endl;
    est.compute(boundaries);
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
    int                                 countBoundaries = 0;
    for (int i = 0; i < projection_cloud->size(); i++)
    {
        uint8_t x = (boundaries.points[i].boundary_point);
        int     a = static_cast<int>(x);   //该函数的功能是强制类型转换
        if (a == 1)
        {
            (*boundPoints).push_back(projection_cloud->points[i]);
            countBoundaries++;
        }
    }
    //std::cout << "boudary size is：" << countBoundaries << std::endl;
    //pcl::io::savePCDFileASCII("D:\\qt_data\\tunnel\\processed\\boudary.pcd", *boundPoints);
    //检测直线
    double Modelerror = 3 * everagedistance;
    for (int i = 0; i < 2; i++)
    {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst2;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr           tree2(new pcl::search::KdTree<pcl::PointXYZ>());
        pcl::PointCloud<pcl::Normal>::Ptr                 normals2(new pcl::PointCloud<pcl::Normal>);
        normEst2.setInputCloud(boundPoints);
        normEst2.setSearchMethod(tree2);
        // normEst.setRadiusSearch(2);  //法向估计的半径
        normEst2.setKSearch(9);   //用于法向估计的近邻点数
        normEst2.compute(*normals2);
        //cout << "normal size is " << normals2->size() << endl;
        //创建一个模型参数对象，用于记录结果
        pcl::ModelCoefficients::Ptr                                 coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr                                      inliers(new pcl::PointIndices);   //inliers表示误差内点云的序号
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;                              //以法向分割
        seg.setInputNormals(normals2);
        seg.setOptimizeCoefficients(true);   // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
        seg.setMaxIterations(300);
        seg.setModelType(pcl::SACMODEL_LINE);   // Mandatory-设置目标几何形状
        seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
        seg.setDistanceThreshold(Modelerror);   //设置误差阈值
        seg.setInputCloud(boundPoints);         //输入点云
        seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
        stringstream ss;
        ss << i;
        string        string_i = ss.str();
        vector<float> line_direction;
        line_direction.push_back(coefficients->values[3]);
        line_direction.push_back(coefficients->values[4]);
        line_direction.push_back(coefficients->values[5]);
        direction_vector.push_back(line_direction);
        //cout <<"Line"+string_i+" coefficients:"<<coefficients->values[3]<<","<<coefficients->values[4]<<","<<coefficients->values[5]<<endl;//输出边界线方向
        //获取inliers索引的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);   //存储平面点云
        pcl::ExtractIndices<pcl::PointXYZ>  extract;                                       //创建点云提取对象
        extract.setInputCloud(boundPoints);                                                //设置输入点云
        extract.setIndices(inliers);                                                       //设置分割后的内点为需要提取的点集
        extract.setNegative(false);                                                        //false提取cloud中inliers内点, true提取cloud中非inliers外点
        extract.filter(*c_plane);                                                          //提取输出存储到c_plane

        //pcl::io::savePCDFileASCII("D:\\qt_data\\tunnel\\processed\\line_extract" + string_i + ".pcd", *c_plane);
        extract.setNegative(true);   //false提取cloud中inliers内点, true提取cloud中非inliers外点
        extract.filter(*c_plane);    //提取输出存储到c_plane
        *boundPoints = *c_plane;
        if (c_plane->points.size() == 0)
        {

            break;
        }
    }

    direction.push_back((direction_vector[0][0] + direction_vector[1][0]) / 2);
    direction.push_back((direction_vector[0][1] + direction_vector[1][1]) / 2);
    direction.push_back((direction_vector[0][2] + direction_vector[1][2]) / 2);
}
void GetMARKDirection(pcl::PointCloud<pcl::PointXYZ> object_cloud, vector<float> &direction, pcl::PointCloud<pcl::Normal> object_normals, pcl::search::KdTree<pcl::PointXYZ> object_tree, int MARK)
{
    //MARK=0表示隧道沿着Y轴，投影到YOZ面进行处理
    //MARK=1表示隧道沿着X轴，投影到XOZ面进行处理

    vector<vector<float>> direction_vector;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    *tree = object_tree;
    //进行平面投影
    pcl::PointCloud<pcl::PointXYZ>::Ptr projection_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *projection_cloud = object_cloud;
    for (int i = 0; i < projection_cloud->points.size(); i++)
    {
        if (MARK == 0)
        {
            projection_cloud->points[i].x = 0;
        }
        else if (MARK == 1)
        {
            projection_cloud->points[i].y = 0;
        }
    }
    //计算平均点密度
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;   //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
    kdtree.setInputCloud(projection_cloud);
    int   k               = 2;
    float everagedistance = 0;
    for (int i = 0; i < projection_cloud->size() / 2; i++)
    {
        vector<int>   nnh;
        vector<float> squaredistance;
        //  pcl::PointXYZ p;
        //   p = projection_cloud->points[i];
        kdtree.nearestKSearch(projection_cloud->points[i], k, nnh, squaredistance);
        everagedistance += sqrt(squaredistance[1]);
        //   cout<<everagedistance<<endl;
    }
    everagedistance = everagedistance / (projection_cloud->size() / 2);
    // cout << "everage distance is : " << everagedistance << endl;
    //计算投影后法向量
    pcl::PointCloud<pcl::Normal>::Ptr projection_normals(new pcl::PointCloud<pcl::Normal>);
    *projection_normals = object_normals;
    for (int i = 0; i < projection_normals->points.size(); i++)
    {
        pcl::Normal tem_normals;
        if (MARK == 0)
        {
            tem_normals.normal_x = 1;
            tem_normals.normal_y = 0;
            tem_normals.normal_z = 0;
        }
        else if (MARK == 1)
        {
            tem_normals.normal_x = 0;
            tem_normals.normal_y = 1;
            tem_normals.normal_z = 0;
        }

        projection_normals->points[i] = tem_normals;
        //projection_normals->points[i].
    }
    //计算边界
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    pcl::PointCloud<pcl::Boundary>                                     boundaries;

    est.setInputCloud(projection_cloud);
    est.setInputNormals(projection_normals);

    est.setSearchMethod(tree);
    //est.setKSearch(50);  //一般这里的数值越高，最终边界识别的精度越好
    est.setRadiusSearch(everagedistance);   //搜索半径
    double pi = 3.14159265358979323846;
    //est.setAngleThreshold(pi);
    std::cout << "start to compute boundaries..." << std::endl;
    est.compute(boundaries);
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
    int                                 countBoundaries = 0;
    for (int i = 0; i < projection_cloud->size(); i++)
    {
        uint8_t x = (boundaries.points[i].boundary_point);
        int     a = static_cast<int>(x);   //该函数的功能是强制类型转换
        if (a == 1)
        {
            (*boundPoints).push_back(projection_cloud->points[i]);
            countBoundaries++;
        }
    }
    //std::cout << "boudary size is：" << countBoundaries << std::endl;
    //pcl::io::savePCDFileASCII("D:\\qt_data\\tunnel\\processed\\boudary.pcd", *boundPoints);
    //检测直线
    double Modelerror = 3 * everagedistance;
    for (int i = 0; i < 2; i++)
    {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst2;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr           tree2(new pcl::search::KdTree<pcl::PointXYZ>());
        pcl::PointCloud<pcl::Normal>::Ptr                 normals2(new pcl::PointCloud<pcl::Normal>);
        normEst2.setInputCloud(boundPoints);
        normEst2.setSearchMethod(tree2);
        // normEst.setRadiusSearch(2);  //法向估计的半径
        normEst2.setKSearch(9);   //用于法向估计的近邻点数
        normEst2.compute(*normals2);
        //cout << "normal size is " << normals2->size() << endl;
        //创建一个模型参数对象，用于记录结果
        pcl::ModelCoefficients::Ptr                                 coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr                                      inliers(new pcl::PointIndices);   //inliers表示误差内点云的序号
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;                              //以法向分割
        seg.setInputNormals(normals2);
        seg.setOptimizeCoefficients(true);   // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
        seg.setMaxIterations(300);
        seg.setModelType(pcl::SACMODEL_LINE);   // Mandatory-设置目标几何形状
        seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
        seg.setDistanceThreshold(Modelerror);   //设置误差阈值
        seg.setInputCloud(boundPoints);         //输入点云
        seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
        stringstream ss;
        ss << i;
        string        string_i = ss.str();
        vector<float> line_direction;
        line_direction.push_back(coefficients->values[3]);
        line_direction.push_back(coefficients->values[4]);
        line_direction.push_back(coefficients->values[5]);
        direction_vector.push_back(line_direction);
        //cout <<"Line"+string_i+" coefficients:"<<coefficients->values[3]<<","<<coefficients->values[4]<<","<<coefficients->values[5]<<endl;//输出边界线方向
        //获取inliers索引的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);   //存储平面点云
        pcl::ExtractIndices<pcl::PointXYZ>  extract;                                       //创建点云提取对象
        extract.setInputCloud(boundPoints);                                                //设置输入点云
        extract.setIndices(inliers);                                                       //设置分割后的内点为需要提取的点集
        extract.setNegative(false);                                                        //false提取cloud中inliers内点, true提取cloud中非inliers外点
        extract.filter(*c_plane);                                                          //提取输出存储到c_plane

        //pcl::io::savePCDFileASCII("D:\\qt_data\\tunnel\\processed\\line_extract" + string_i + ".pcd", *c_plane);
        extract.setNegative(true);   //false提取cloud中inliers内点, true提取cloud中非inliers外点
        extract.filter(*c_plane);    //提取输出存储到c_plane
        *boundPoints = *c_plane;
        if (c_plane->points.size() == 0)
        {

            break;
        }
    }

    direction.push_back((direction_vector[0][0] + direction_vector[1][0]) / 2);
    direction.push_back((direction_vector[0][1] + direction_vector[1][1]) / 2);
    direction.push_back((direction_vector[0][2] + direction_vector[1][2]) / 2);
}
void GetTunnelDirectionFromNormals(vector<float> &approximate_direction, pcl::PointCloud<pcl::Normal> object_normals)
{
    //计算隧道点云大致走向，与直线走向相近
    //vector<float> approximate_direction;//隧道大致方向x,y,z

    pcl::PointCloud<pcl::PointXYZ>::Ptr tunnel_normals(new pcl::PointCloud<pcl::PointXYZ>);   //高斯球点云
    pcl::PointCloud<pcl::Normal>::Ptr   circle_normals(new pcl::PointCloud<pcl::Normal>);     //高斯球上点云法向量
    *circle_normals        = object_normals;
    tunnel_normals->width  = object_normals.points.size();
    tunnel_normals->height = 1;
    tunnel_normals->points.resize(tunnel_normals->height * tunnel_normals->width);
    for (int i = 0; i < object_normals.points.size(); i++)
    {
        float normal = sqrt(object_normals.points[i].normal_x * object_normals.points[i].normal_x +
                            object_normals.points[i].normal_y * object_normals.points[i].normal_y +
                            object_normals.points[i].normal_z * object_normals.points[i].normal_z);

        circle_normals->points[i].normal_x = object_normals.points[i].normal_x / normal;
        circle_normals->points[i].normal_y = object_normals.points[i].normal_y / normal;
        circle_normals->points[i].normal_z = object_normals.points[i].normal_z / normal;

        tunnel_normals->points[i].x = object_normals.points[i].normal_x / normal;
        tunnel_normals->points[i].y = object_normals.points[i].normal_y / normal;
        tunnel_normals->points[i].z = object_normals.points[i].normal_z / normal;
    }
    //pcl::io::savePCDFileASCII("D:\\qt_data\\tunnel\\processed\\tunnel_normals.pcd",*tunnel_normals);

    //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr                                 coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr                                      inliers(new pcl::PointIndices);   //inliers表示误差内点云的序号
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segNormals;                       //以法向分割
    segNormals.setInputNormals(circle_normals);
    segNormals.setOptimizeCoefficients(true);          // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    segNormals.setModelType(pcl::SACMODEL_CYLINDER);   // Mandatory-设置目标几何形状
    segNormals.setMaxIterations(30);
    segNormals.setRadiusLimits(0.999, 1.001);      //
    segNormals.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    segNormals.setDistanceThreshold(0.0001);       //设置误差阈值
    segNormals.setInputCloud(tunnel_normals);      //输入点云
    segNormals.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    approximate_direction.push_back(coefficients->values[3]);
    approximate_direction.push_back(coefficients->values[4]);
    approximate_direction.push_back(coefficients->values[5]);
    /*显示用于获得隧道大致走向的点云
    //获取inliers索引的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);  //存储平面点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;  //创建点云提取对象
    extract.setInputCloud(tunnel_normals);    //设置输入点云
    extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
    extract.setNegative(false);      //false提取cloud中inliers内点, true提取cloud中非inliers外点
    extract.filter(*c_plane);        //提取输出存储到c_plane
    pcl::io::savePCDFileASCII(SavePath+"direction_plane.pcd",*c_plane);
    */
}
void GetSection(pcl::PointCloud<pcl::PointXYZ> object_cloud, vector<float> direction, pcl::PointXYZ Point, float d, pcl::PointCloud<pcl::PointXYZ> &section)
{
    float m      = -direction[0] * Point.x - direction[1] * Point.y - direction[2] * Point.z;
    float normal = sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    for (int i = 0; i < object_cloud.points.size(); i++)
    {
        if (abs((direction[0] * object_cloud.points[i].x + direction[1] * object_cloud.points[i].y + direction[2] * object_cloud.points[i].z + m) / normal) > (d / 2))
        {
            continue;
        }
        else
        {
            section.points.push_back(object_cloud.points[i]);
        }
    }
}
void ReadLine(QString datapath, vector<float> &axis)
{
    /*
    QTextCodec *code = QTextCodec::codecForName("GB2312");//解决中文路径问题
     std::string name_read = code->fromUnicode(datapath).data();
    datapath=QString::fromStdString(name_read);

*/
    QFile file(datapath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::information(NULL, "Error", "Fail to open the file!");
    }
    QTextStream read_stream(&file);
    /*

   //std::locale loc1=std::locale::global(std::locale(".936"));
    ifstream file(datapath.toStdString());
    if(!file.is_open())
    {
        QMessageBox::information(NULL, "Error", "Fail to open the file!");
    }
    else
    {
        file.open(datapath.toStdString(), ios::binary);
    }

/*
        string line;
        while(getline(read_file, line))
        {
            cout<<"line:"<<line.c_str()<<endl;
        }



         while((str=read_stream.readLine())!="")
        */

    QString str;

    while ((str = read_stream.readLine()) != "")
    {

        vector<string> v;
        SplitString(str.toStdString(), v, "\t");   //可按多个字符来分隔;
        QString parts = QString::fromStdString(v[0]);
        axis.push_back(parts.toFloat());

        parts = QString::fromStdString(v[1]);
        axis.push_back(parts.toFloat());
        parts = QString::fromStdString(v[2]);
        axis.push_back(parts.toFloat());
    }

    file.close();
    //cout<<"reading complete!"<<endl;
}
void SplitString(const string &s, vector<string> &v, const string &c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
}

void bezier(const TunnelPointCloud &section, TunnelPointCloud &newvector, int numberofnewpoint)
{

    newvector = section;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(section, minPt, maxPt);
    int MARK;
    if (maxPt.y - minPt.y > maxPt.x - minPt.x)   //断面走向方向比较薄
    {
        MARK = 1;   //走向沿X轴
    }
    else
    {
        MARK = 0;   //走向沿Y轴
    }

    double n0, n1, n2, n3;
    double dt = 1.0 / numberofnewpoint;
    dt        = dt - 0.000000000000000000000000000000001;
    double z0 = (maxPt.z + minPt.z) / 2;
    double MARK0;
    if (MARK == 0)
    {
        MARK0 = (maxPt.x + minPt.x) / 2;
    }
    else
    {
        MARK0 = (maxPt.y + minPt.y) / 2;
    }
    pcl::PointCloud<pcl::PointXYZI> SectionWithAzimuth;
    pcl::copyPointCloud(section, SectionWithAzimuth);

    //SectionWithAzimuth.points[0].
    //计算方位角
    int process_time = SectionWithAzimuth.points.size();
    int add_count    = process_time / 30;
    if (MARK == 0)
    {
        for (int i = 0; i < SectionWithAzimuth.points.size(); i++)
        {
            if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].x > MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].x < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = 360 + (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z < z0 && SectionWithAzimuth.points[i].x < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535 + 180;
            }
            else
            {
                SectionWithAzimuth.points[i].intensity = 180 + (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }

        }
    }
    else
    {
        for (int i = 0; i < SectionWithAzimuth.points.size(); i++)
        {
            if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].y > MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].y < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = 360 + (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z < z0 && SectionWithAzimuth.points[i].y < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535 + 180;
            }
            else
            {
                SectionWithAzimuth.points[i].intensity = 180 + (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }

        }
    }

    for (int i = 0; i < SectionWithAzimuth.points.size() - 1; i++)   //冒泡法排序
    {
        for (int j = 0; j < SectionWithAzimuth.points.size() - 1 - i; j++)   // j开始等于0，
        {
            if (SectionWithAzimuth.points[j].intensity < SectionWithAzimuth.points[j + 1].intensity)
            {

                pcl::PointXYZI point0            = SectionWithAzimuth.points[j];
                SectionWithAzimuth.points[j]     = SectionWithAzimuth.points[j + 1];
                SectionWithAzimuth.points[j + 1] = point0;
            }
        }

    }

    for (int i = 0; i < SectionWithAzimuth.points.size() - 3; i++)
    {
        for (double t = dt; t < 1; t = t + dt)
        {
            n0 = (1 - t) * (1 - t) * (1 - t);
            n1 = (3 * t) * (1 - t) * (1 - t);
            n2 = (3 * t * t) * (1 - t);
            n3 = t * t * t;
            pcl::PointXYZ point0;
            point0.x = SectionWithAzimuth.points[i].x * n0 + SectionWithAzimuth.points[i + 1].x * n1 + SectionWithAzimuth.points[i + 2].x * n2 + SectionWithAzimuth.points[i + 3].x * n3;
            point0.y = SectionWithAzimuth.points[i].y * n0 + SectionWithAzimuth.points[i + 1].y * n1 + SectionWithAzimuth.points[i + 2].y * n2 + SectionWithAzimuth.points[i + 3].y * n3;
            point0.z = SectionWithAzimuth.points[i].z * n0 + SectionWithAzimuth.points[i + 1].z * n1 + SectionWithAzimuth.points[i + 2].z * n2 + SectionWithAzimuth.points[i + 3].z * n3;
            newvector.points.push_back(point0);
        }

    }

}

void bspline(const TunnelPointCloud &section, TunnelPointCloud &newvector, int numberofnewpoint)
{

    newvector = section;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(section, minPt, maxPt);
    int MARK;
    if (maxPt.y - minPt.y > maxPt.x - minPt.x)   //断面走向方向比较薄
    {
        MARK = 1;   //走向沿X轴
    }
    else
    {
        MARK = 0;   //走向沿Y轴
    }

    double n0, n1, n2, n3;
    double dt = 1.0 / numberofnewpoint;
    dt        = dt - 0.000000000000000000000000000000001;
    double z0 = (maxPt.z + minPt.z) / 2;
    double MARK0;
    if (MARK == 0)
    {
        MARK0 = (maxPt.x + minPt.x) / 2;
    }
    else
    {
        MARK0 = (maxPt.y + minPt.y) / 2;
    }
    pcl::PointCloud<pcl::PointXYZI> SectionWithAzimuth;
    pcl::copyPointCloud(section, SectionWithAzimuth);

    //SectionWithAzimuth.points[0].
    //计算方位角
    int process_time = SectionWithAzimuth.points.size();
    int add_count    = process_time / 30;
    if (MARK == 0)
    {
        for (int i = 0; i < SectionWithAzimuth.points.size(); i++)
        {
            if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].x > MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].x < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = 360 + (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z < z0 && SectionWithAzimuth.points[i].x < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535 + 180;
            }
            else
            {
                SectionWithAzimuth.points[i].intensity = 180 + (atan((SectionWithAzimuth.points[i].x - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }

        }
    }
    else
    {
        for (int i = 0; i < SectionWithAzimuth.points.size(); i++)
        {
            if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].y > MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z > z0 && SectionWithAzimuth.points[i].y < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = 360 + (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }
            else if (SectionWithAzimuth.points[i].z < z0 && SectionWithAzimuth.points[i].y < MARK0)
            {
                SectionWithAzimuth.points[i].intensity = (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535 + 180;
            }
            else
            {
                SectionWithAzimuth.points[i].intensity = 180 + (atan((SectionWithAzimuth.points[i].y - MARK0) / (SectionWithAzimuth.points[i].z - z0))) * 180 / 3.1415926535;
            }

        }
    }

    for (int i = 0; i < SectionWithAzimuth.points.size() - 1; i++)   //冒泡法排序
    {
        for (int j = 0; j < SectionWithAzimuth.points.size() - 1 - i; j++)   // j开始等于0，
        {
            if (SectionWithAzimuth.points[j].intensity < SectionWithAzimuth.points[j + 1].intensity)
            {

                pcl::PointXYZI point0            = SectionWithAzimuth.points[j];
                SectionWithAzimuth.points[j]     = SectionWithAzimuth.points[j + 1];
                SectionWithAzimuth.points[j + 1] = point0;
            }
        }

    }

    for (int i = 0; i < SectionWithAzimuth.points.size() - 3; i++)
    {
        for (double t = dt; t < 1; t = t + dt)
        {
            n0 = (-t * t * t + 3 * t * t - 3 * t + 1) / 6.0;
            n1 = (3 * t * t * t - 6 * t * t + 4) / 6.0;
            n2 = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6.0;
            n3 = t * t * t / 6.0;
            pcl::PointXYZ point0;
            point0.x = SectionWithAzimuth.points[i].x * n0 + SectionWithAzimuth.points[i + 1].x * n1 + SectionWithAzimuth.points[i + 2].x * n2 + SectionWithAzimuth.points[i + 3].x * n3;
            point0.y = SectionWithAzimuth.points[i].y * n0 + SectionWithAzimuth.points[i + 1].y * n1 + SectionWithAzimuth.points[i + 2].y * n2 + SectionWithAzimuth.points[i + 3].y * n3;
            point0.z = SectionWithAzimuth.points[i].z * n0 + SectionWithAzimuth.points[i + 1].z * n1 + SectionWithAzimuth.points[i + 2].z * n2 + SectionWithAzimuth.points[i + 3].z * n3;
            newvector.points.push_back(point0);
        }

    }

}

void MainWindow::on_actionspline_triggered()
{

    if (object_cloud.points.size() == 0)
    {
        QMessageBox::critical(this, QString::fromUtf8("断面拟合"), QString::fromUtf8("请先加载断面文件"));
        return;
    }

    ui->statusBar->showMessage(QString::fromUtf8("开始拟合"));




    int                   numberofnewpoint = 4;

    TunnelPointCloud::Ptr Bspline_vector(new TunnelPointCloud);

    bspline(object_cloud, *Bspline_vector, numberofnewpoint);

    QMessageBox::information(this, QString::fromUtf8("断面拟合"), QString::fromUtf8("正在处理请稍候"));

    copyPointCloud(*Bspline_vector);
    display(object_cloud);
    ui->statusBar->showMessage(QString::fromUtf8("断面拟合完成!"));
}

void MainWindow::on_actionbezie_triggered()
{
    if (object_cloud.points.size() == 0)
    {
        QMessageBox::critical(this, QString::fromUtf8("断面拟合"), QString::fromUtf8("请先加载断面文件"));
        return;
    }

    ui->statusBar->showMessage(QString::fromUtf8("开始拟合"));




    int                   numberofnewpoint = 4;

    TunnelPointCloud::Ptr Bspline_vector(new TunnelPointCloud);

    bezier(object_cloud, *Bspline_vector, numberofnewpoint);

    QMessageBox::information(this, QString::fromUtf8("断面拟合"), QString::fromUtf8("正在处理请稍候"));

    copyPointCloud(*Bspline_vector);
    display(object_cloud);
    ui->statusBar->showMessage(QString::fromUtf8("断面拟合完成!"));
}
