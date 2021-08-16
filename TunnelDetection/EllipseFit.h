#ifndef ELLIPSEFIT_H
#define ELLIPSEFIT_H
#include <LASFile.h>
#include <LASlib/lasreader.hpp>
#include <LASlib/laswriter.hpp>
#include <QString>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#define n_polynomial 2   //需要拟合的多项式次数
#include <QVector>
#include <vector>

struct Ellipse_struct
{
    double A, B, C, D, E, F;
};
struct ellipseCenter
{
    double x0 = 0.0;
    double y0 = 0.0;
};

struct lineparam
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
};
struct axisdeormation
{
    double x_dmf;
    double y_dmf;
};
class EllipseFit
{
public:
    EllipseFit();
    static void                RANSAC();
    static Ellipse_struct      RansacFitEllipse(const pcl::PointCloud<pcl::PointXYZI> &select_points);
    static void                GetEllipseParam(Ellipse_struct best_ellipse,
                                               double &       x0,
                                               double &       y0,
                                               double &       alpha,
                                               double &       majorAxis,
                                               double &       minorAxis);
    static void                filter(const pcl::PointCloud<pcl::PointXYZI> &data_pts,
                                      const Ellipse_struct &                 best_ellipse,
                                      double &                               mean,
                                      double &                               sd,
                                      pcl::PointCloud<pcl::PointXYZI> &      noise_pts,
                                      pcl::PointCloud<pcl::PointXYZI> &      use_pts);
    static double              GetOvality(double &majorAxis, double &minorAxis);
    static void                ellipseAllLine(const QString &pcd_file, QVector<QVector<ellipseCenter>> &allline_center_points, const QString &output_root);
    static void                ellipseLine(const QString &pcd_file, const QString &output_root, const int lineth);
    static void                DiameterOfConvergence(const pcl::PointCloud<pcl::PointXYZI> &use_point, double x0, double y0, double &dia_of_convergence);
    static std::vector<double> gaussian_elimination(std::vector<std::vector<double>> a, std::vector<double> b);            //高斯消去法求解线性方程组AX=B
    static std::vector<double> least_square_method(std::vector<double> x, std::vector<double> y, std::vector<double> a);   //声明最小二乘法函数,x,y为原始数据，a为拟合多项式的系数列向量
    static double              phi(double x, int n_cishu);
    // static void ellipseLine(const pcl::PointCloud<pcl::PointXYZI>& line, );

    static bool linefit(QVector<QVector<ellipseCenter>> &allpoints, QVector<lineparam> &axisparam);
    static void Axis_deformation(const QString &pcd_file, QVector<QVector<ellipseCenter>> &allpoints, QVector<lineparam> &allaxisparam, const QString &output_root);

    static double xozDistance(const pcl::PointXYZI &point);
};

#endif   // ELLIPSEFIT_H
