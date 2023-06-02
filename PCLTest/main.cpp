#include <iostream>
#include <thread>

#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QFileInfo>
#include <QFileDialog>
#include <QApplication>

using namespace std;
using namespace Eigen;

pcl::PointCloud<pcl::PointXYZ>::Ptr saveFeatureFile2PCD(const QString &file_name);
Eigen::VectorXd calculatePlaneFeatureParam(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

struct Analysis
{
    QString file_name;
    int used_point_num = 0;
    int all_point_num = 0;
    Eigen::VectorXf ransac_param;
    Eigen::Vector4d own_param;
};
int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    QStringList txt_files = QFileDialog::getOpenFileNames(nullptr, "open cal txt", "E:/Calibration", "Text files (*.txt)");

    int count = 1;
    const int size = txt_files.size();
    QVector<Analysis> analysis_list;
    for(const auto& file_name: txt_files)
    {
        auto cloud = saveFeatureFile2PCD(file_name);
        if(cloud)
        {
            //--------------------------RANSAC拟合平面--------------------------
            pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
            ransac.setDistanceThreshold(0.01);	//设置距离阈值，与平面距离小于0.1的点作为内点
            ransac.computeModel();				//执行模型估计
            //-------------------------根据索引提取内点--------------------------
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
            vector<int> inliers;				//存储内点索引的容器
            ransac.getInliers(inliers);			//提取内点索引
            pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_plane);
            //----------------------------输出模型参数---------------------------
            Eigen::VectorXf coefficient;
            ransac.getModelCoefficients(coefficient);

            auto param = calculatePlaneFeatureParam(cloud_plane);
            QString base_file_name = QFileInfo(file_name).fileName();
            fmt::print("{}/{}: {} {}/{}={:.3f}%\n",count, size, base_file_name.toStdString(), inliers.size() , cloud->size(), (inliers.size() /(double) cloud->size()) * 100.0);
            fmt::print("{: < 10.5f}, {: < 10.5f}, {: < 10.5f}, {: < 10.5f}\n", coefficient[0], coefficient[1], coefficient[2], coefficient[3]);
            fmt::print("{: < 10.5f}, {: < 10.5f}, {: < 10.5f}, {: < 10.5f}\n", param[0], param[1], param[2],-param[3]);
            analysis_list.push_back({base_file_name, static_cast<int>(inliers.size()), static_cast<int>(cloud->size()), coefficient, param});
        }
        ++count;
    }
    std::vector<int> counts(6, 0);
    for(const auto& analysis: analysis_list)
    {
        double used_rate = (analysis.used_point_num / (double) analysis.all_point_num) * 100.0;
        if(used_rate < 50.0)
            ++counts[0];
        else if(50.0 <= used_rate && used_rate < 65.0)
            ++counts[1];
        else if(65.0 <= used_rate && used_rate< 75.0)
            ++counts[2];
        else if(75.0 <= used_rate && used_rate< 85.0)
            ++counts[3];
        else if(85.0 <= used_rate && used_rate< 95.0)
            ++counts[4];
        else
            ++counts[5];
    }
    fmt::print("<50%:{:.3f} >=50%:{:.3f}, >=65%:{:.3f}, >=75%:{:.3f}, >=85%:{:.3f}, >=95%:{:.3f}\n",
               counts[0]/(double)count, counts[1]/(double)count, counts[2]/(double)count, counts[3]/(double)count, counts[4]/(double)count, counts[5]/(double)count);

    //-----------------------------结果可视化----------------------------
    /*
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("拟合结果"));

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    viewer->addPointCloud<pcl::PointXYZ>(cloud_plane, "plane");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "plane");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
     */
    return 0;
}
Eigen::VectorXd calculatePlaneFeatureParam(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // Ref: 一种稳健的点云数据平面拟合方法，官云兰
    /**>>>>>>>>>>>>>>>>>>>>>>>>>>**************计算出平面参数初值***************************/

    double   p_xi = 0, p_yi = 0, p_zi = 0;
    double   sum_xi = 0, sum_yi = 0, sum_zi = 0;
    Matrix3d A = Matrix3d::Zero();

    int size = cloud->size();
    for (const auto & p : *cloud)
    {
        p_xi += p.x;
        p_yi += p.y;
        p_zi += p.z;
    }
    p_xi /= size;
    p_yi /= size;
    p_zi /= size;

    for (const auto & p : *cloud)
    {
        sum_xi = p.x - p_xi;
        sum_yi = p.y - p_yi;
        sum_zi = p.z - p_zi;

        A(0, 0) += sum_xi * sum_xi;
        A(0, 1) += sum_xi * sum_yi;
        A(0, 2) += sum_xi * sum_zi;

        A(1, 0) = A(0, 1);
        A(1, 1) += sum_yi * sum_yi;
        A(1, 2) += sum_yi * sum_zi;

        A(2, 0) = A(0, 2);
        A(2, 1) = A(1, 2);
        A(2, 2) += sum_zi * sum_zi;
    }
    EigenSolver<Matrix3d> es(A);

    Matrix3d D         = es.pseudoEigenvalueMatrix();
    Matrix3d V         = es.pseudoEigenvectors();
    double   mindi     = D(0, 0);
    int      min_index = 0;
    for (int n = 0; n < 3; n++)
    {
        if (D(n, n) <= mindi)
        {
            mindi     = D(n, n);
            min_index = n;
        }
    }

    Eigen::Vector4d param;
    param(0) = V(0, min_index);
    param(1) = V(1, min_index);
    param(2) = V(2, min_index);
    param(3) = param(0) * p_xi + param(1) * p_yi + param(2) * p_zi;

    return param;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr saveFeatureFile2PCD(const QString &file_name)
{
    if (file_name.isEmpty())
        return nullptr;

    QFile measure_file(file_name);

    if (!measure_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_ERROR("无法打开文件 {}", file_name.toStdString());
        return {};
    }

    QTextStream mReadStream(&measure_file);

    QString     mReadLine;
    QStringList mItemList;
    int         count = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ xyz_e;
    static Vector3i offset = {0,0,0};
    while (!mReadStream.atEnd())
    {
        mReadLine = mReadStream.readLine();
        mItemList = mReadLine.split(',', Qt::KeepEmptyParts);
        if(count == 0)
        {
            offset.x() = mItemList[0].toDouble();
            offset.y() = mItemList[1].toDouble();
            offset.z() = mItemList[2].toDouble();
        }

        if (mItemList.size() < 3)
        {
            SPDLOG_WARN("第 {} 行数据项不足 3", count);
            continue;
        }
        xyz_e.x         = mItemList[0].toDouble() - offset.x();
        xyz_e.y         = mItemList[1].toDouble() - offset.y();
        xyz_e.z         = mItemList[2].toDouble() - offset.z();

        cloud->push_back(xyz_e);
        count++;
    }

    return cloud;
}
