#include "EllipseFit.h"
#include <Eigen/Core>
#include <Eigen/Dense>

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <vector>

#include <spdlog/custom_log.h>
#include <spdlog/spdlog.h>

#include <math.h>

using namespace std;
EllipseFit::EllipseFit()
{
}

Ellipse_struct EllipseFit::RansacFitEllipse(const pcl::PointCloud<pcl::PointXYZI> &select_points)
{
    Ellipse_struct ellipses_null = {0};
    if (select_points.size() < 5)
    {
        SPDLOG_CRITICAL("No enough points to fit ellipse.");
        return ellipses_null;
    }

    //   椭圆的一般方程：Ax2 + Bxy + Cy2 + Dx + Ey + F = 0
    Eigen::MatrixXf leftm((select_points.size()), 5);
    Eigen::MatrixXf rightm((select_points.size()), 1);
    SPDLOG_TRACE("start compute Ax=b!");
    for (int i = 0; i < select_points.size(); i++)
    {
        leftm(i, 0) = select_points[i].x * select_points[i].z;
        leftm(i, 1) = select_points[i].z * select_points[i].z;
        leftm(i, 2) = select_points[i].x;
        leftm(i, 3) = select_points[i].z;
        leftm(i, 4) = 1;

        rightm(i, 0) = -(select_points[i].x * select_points[i].x);
    }

    Eigen::MatrixXf cof_mat(5, 1);

    cof_mat = ((leftm.transpose() * leftm).inverse()) * (leftm.transpose() * rightm);
    double A, B, C, D, E, F;
    A                            = 1;
    B                            = cof_mat(0, 0);
    C                            = cof_mat(1, 0);
    D                            = cof_mat(2, 0);
    E                            = cof_mat(3, 0);
    F                            = cof_mat(4, 0);
    Ellipse_struct ellipse_model = {A, B, C, D, E, F};

    SPDLOG_TRACE("A: {}", A);
    SPDLOG_TRACE("B: {}", B);
    SPDLOG_TRACE("C: {}", C);
    SPDLOG_TRACE("D: {}", D);
    SPDLOG_TRACE("E: {}", E);
    SPDLOG_TRACE("F: {}", F);

    return ellipse_model;
}

void EllipseFit::GetEllipseParam(Ellipse_struct best_ellipse,
                                 double &       x0,
                                 double &       y0,
                                 double &       alpha,
                                 double &       majorAxis,
                                 double &       minorAxis)
{
    double A, B, C, D, E, F;
    // F归一
    A = best_ellipse.A / best_ellipse.F;
    B = best_ellipse.B / best_ellipse.F;
    C = best_ellipse.C / best_ellipse.F;
    D = best_ellipse.D / best_ellipse.F;
    E = best_ellipse.E / best_ellipse.F;
    F = best_ellipse.F / best_ellipse.F;

    // 中心坐标
    x0 = ((B * E) - (2 * C * D)) / ((4 * A * C) - (B * B));
    y0 = ((B * D) - (2 * A * E)) / ((4 * A * C) - (B * B));

    // 长短轴
    majorAxis = (std::max)(sqrt(2 * (A * x0 * x0 + C * y0 * y0 + B * x0 * y0 - 1) /
                                (A + C - sqrt((A - C) * (A - C) + B * B))),
                           sqrt(2 * (A * x0 * x0 + C * y0 * y0 + B * x0 * y0 - 1) /
                                (A + C + sqrt((A - C) * (A - C) + B * B))));
    minorAxis = (std::min)(sqrt(2 * (A * x0 * x0 + C * y0 * y0 + B * x0 * y0 - 1) /
                                (A + C - sqrt((A - C) * (A - C) + B * B))),
                           sqrt(2 * (A * x0 * x0 + C * y0 * y0 + B * x0 * y0 - 1) /
                                (A + C + sqrt((A - C) * (A - C) + B * B))));
    // 角度
    alpha = (atan(B / (A - C))) / 2;
}

void EllipseFit::filter(const pcl::PointCloud<pcl::PointXYZI> &data_pts,
                        const Ellipse_struct &                 best_ellipse,
                        double &                               mean,
                        double &                               sd,
                        pcl::PointCloud<pcl::PointXYZI> &      noise_pts,
                        pcl::PointCloud<pcl::PointXYZI> &      use_pts)
{
    float *dis            = new float[data_pts.size()];
    float  di_sum         = 0;
    float  sum_of_squares = 0;
    mean                  = 0;
    sd                    = 0;

    for (int i = 0; i < data_pts.size(); ++i)
    {
        float x  = data_pts[i].x;
        float x2 = data_pts[i].x * data_pts[i].x;
        float y  = data_pts[i].z;
        float y2 = data_pts[i].z * data_pts[i].z;
        float xy = data_pts[i].x * data_pts[i].z;

        dis[i] = fabs(best_ellipse.A * x2 + best_ellipse.B * xy + best_ellipse.C * y2 + best_ellipse.D * x + best_ellipse.E * y + best_ellipse.F);
        di_sum += dis[i];
    }
    mean = di_sum / data_pts.size();

    for (int i = 0; i < data_pts.size(); ++i)
    {
        sum_of_squares += pow(dis[i] - mean, 2);
    }
    sd = sqrt(sum_of_squares / data_pts.size());
    //    qDebug() << "mean:" << mean << "sd:" << sd;
    SPDLOG_TRACE("mean: {}", mean);
    SPDLOG_TRACE("sd: {}", sd);
    noise_pts.clear();
    use_pts.clear();

    noise_pts.reserve(data_pts.size());
    use_pts.reserve(data_pts.size());

    for (int i = 0; i < data_pts.size(); ++i)
    {
        if (fabs(dis[i] - mean) > fabs(2 * sd))
        {
            noise_pts.push_back(data_pts[i]);
        }
        else
        {
            use_pts.push_back(data_pts[i]);
        }
        //        SPDLOG_TRACE("di_{}:{}", i, dis[i]);
    }
    delete[] dis;
    dis = NULL;
}

double EllipseFit::GetOvality(double &majorAxis, double &minorAxis)
{
    double ovality;
    ovality = (majorAxis - minorAxis) / 2.9 * 1000;

    return ovality;
}

void EllipseFit::ellipseAllLine(const QString &pcd_file, QVector<QVector<ellipseCenter>> &allline_center_points, const QString &output_root)
{
    pcl::PointCloud<pcl::PointXYZI> data_set;      //全部点集
    pcl::PointCloud<pcl::PointXYZI> data_pts;      //每条线的点集
                                                   //    pcl::PointCloud<pcl::PointXYZI> data_point;
    pcl::PointCloud<pcl::PointXYZI> noise_pts;     //噪声点集
    pcl::PointCloud<pcl::PointXYZI> use_pts;       //保留下的点集
    Ellipse_struct                  ellipse_fit;   //椭圆系数
    double                          ovality;
    double                          mean = 0;
    double                          sd   = 0;
    double                          x0;
    double                          y0;
    double                          alpha;
    double                          majorAxis;
    double                          minorAxis;
    double                          dia_of_convergence = 0.0;
    //    double                          a;
    //    double                          b;
    //    double                          c;
    //    double                          x_dmf      = 0.0;
    //    double                          y_dmf      = 0.0;
    QDir    dir        = "D:";
    QString param_path = output_root + QString("/param.txt");

    dir.mkpath(output_root);

    QFile write_file(param_path);
    if (!write_file.open(QIODevice::WriteOnly))
    {
        SPDLOG_ERROR("error to write param.txt");
        return;
    }

    QTextStream write_stream(&write_file);
    // 写入param的头部
    write_stream.setPadChar(' ');
    write_stream.setRealNumberPrecision(6);
    write_stream.setFieldAlignment(QTextStream::AlignRight);
    write_stream.setFieldWidth(12);
    write_stream << "index"
                 << "times"
                 << "x0"
                 << "y0"
                 << "alpha"
                 << "majorAxis"
                 << "minorAxis"
                 << "ovality"
                 << "dia_of_convergence"
                 << "mean"
                 << "sd"
                 << "use_pts"
                 << "noise_pts"
                 << "A"
                 << "B"
                 << "C"
                 << "D"
                 << "E"
                 << "F";
    write_stream.setFieldWidth(0);
    write_stream << endl;

    SPDLOG_INFO("start to read {}", pcd_file);
    pcl::PCDReader reader;
    reader.read(pcd_file.toStdString(), data_set);   //读进全部点
    SPDLOG_INFO("read {} END ", pcd_file);

    data_pts.reserve(data_set.width);
    SPDLOG_INFO("start to filter");

    QVector<ellipseCenter> all_center_points;
    allline_center_points.resize(data_set.height);
    ellipseCenter center;
    for (int i = 0; i < data_set.height; i++)
    {
        if (i % 1 == 0)
            SPDLOG_INFO("This is line: {}", i);

        for (int j = 0; j < data_set.width; j++)
        {
            int idx = i * data_set.width + j;

            double dis = xozDistance(data_set.points[idx]);
            if (data_set.points[idx].intensity >= 220 || data_set.points[idx].intensity <= 120 || (fabs(data_set.points[idx].x) < 0.8 && data_set.points[idx].z <= 0) || dis < 0.1 || dis > 10)
            {
                continue;
            }
            else
            {
                data_pts.push_back(data_set.points[idx]);
            }
        }
        pcl::io::savePCDFileBinary("D:/YM/data/rowpcd/jn2w-0-indentity.pcd", data_pts);
        SPDLOG_TRACE("After indentity filter,data_pts.size:   {}", (data_pts.size()));
        SPDLOG_TRACE("indentity filer complete,start eliipse filter");

        for (int m = 0; m < 7; ++m)
        {
            ellipse_fit = RansacFitEllipse(data_pts);
            filter(data_pts, ellipse_fit, mean, sd, noise_pts, use_pts);

            data_pts = use_pts;
            GetEllipseParam(ellipse_fit, x0, y0, alpha, majorAxis, minorAxis);
            DiameterOfConvergence(use_pts, x0, y0, dia_of_convergence);

            center.x0 = x0;
            center.y0 = y0;
            all_center_points.push_back(center);

            ovality = GetOvality(majorAxis, minorAxis);

            QString usepoint_path   = output_root + QString("/jn2w-use-%1").arg(m);
            QString noisepoint_path = output_root + QString("/jn2w-noise-%1").arg(m);
            dir.cd(output_root);
            if (!dir.exists(usepoint_path))
            {
                dir.mkpath(usepoint_path);
            }
            if (!dir.exists(noisepoint_path))
            {
                dir.mkpath(noisepoint_path);
            }
            if (!dir.exists(param_path))
            {
                dir.mkpath(param_path);
            }
            pcl::io::savePCDFileBinary(usepoint_path.toStdString() + "/use-" + std::to_string(i) + ".pcd", use_pts);
            SPDLOG_TRACE("save use-pts end");
            if (noise_pts.size() == 0)
            {
                pcl::PointXYZI point;
                point.x         = 0;
                point.y         = 0;
                point.z         = 0;
                point.intensity = 160;

                noise_pts.push_back(point);
            }

            pcl::io::savePCDFileBinary(noisepoint_path.toStdString() + "/noise-" +
                                           std::to_string(i) + ".pcd",
                                       noise_pts);
            SPDLOG_TRACE("save noise-pts end");

            write_stream.setFieldWidth(12);
            write_stream << i
                         << m
                         << x0
                         << y0
                         << alpha
                         << majorAxis
                         << minorAxis
                         << ovality
                         << dia_of_convergence
                         << mean
                         << sd
                         << use_pts.size()
                         << noise_pts.size()
                         << ellipse_fit.A
                         << ellipse_fit.B
                         << ellipse_fit.C
                         << ellipse_fit.D
                         << ellipse_fit.E
                         << ellipse_fit.F;
            write_stream.setFieldWidth(0);
            write_stream << endl;

            if (noise_pts.size() < 5)
            {
                SPDLOG_WARN("num of {} line, times {}, break", i, m);
                break;
            }
        }
        data_pts.clear();
        allline_center_points.push_back(all_center_points);
        allline_center_points.clear();
    }
}

void EllipseFit::ellipseLine(const QString &pcd_file, const QString &output_root, const int lineth)
{

    pcl::PointCloud<pcl::PointXYZI> data_set;
    pcl::PointCloud<pcl::PointXYZI> data_pts;      //每条线的点集
                                                   //    pcl::PointCloud<pcl::PointXYZI> data_point;
    pcl::PointCloud<pcl::PointXYZI> noise_pts;     //噪声点集
    pcl::PointCloud<pcl::PointXYZI> use_pts;       //保留下的点集
    Ellipse_struct                  ellipse_fit;   //椭圆系数
    double                          mean = 0;
    double                          sd   = 0;
    double                          x0;
    double                          y0;
    double                          alpha;
    double                          majorAxis;
    double                          minorAxis;
    double                          ovality;
    double                          dia_of_convergence = 0.0;
    //    double                          a;
    //    double                          b;
    //    double                          c;
    double x_dmf = 0.0;
    double y_dmf = 0.0;

    QDir    dir        = "D:";
    QString param_path = output_root + QString("/param.txt");

    dir.mkpath(output_root);

    QFile write_file(param_path);
    if (!write_file.open(QIODevice::WriteOnly))
    {
        SPDLOG_ERROR("error to write param.txt");
        return;
    }

    QTextStream write_stream(&write_file);
    // 写入param的头部
    write_stream.setPadChar(' ');
    write_stream.setRealNumberPrecision(6);
    write_stream.setFieldAlignment(QTextStream::AlignRight);
    write_stream.setFieldWidth(12);
    write_stream << "index"
                 << "times"
                 << "x0"
                 << "y0"
                 << "alpha"
                 << "majorAxis"
                 << "minorAxis"
                 << "ovality"
                 << "dia_of_convergence"
                 << "x_dmf"
                 << "y_dmf"
                 << "mean"
                 << "sd"
                 << "use_pts"
                 << "noise_pts"
                 << "A"
                 << "B"
                 << "C"
                 << "D"
                 << "E"
                 << "F";
    write_stream.setFieldWidth(0);
    write_stream << endl;

    SPDLOG_INFO("start to read {}", pcd_file);
    pcl::PCDReader reader;
    reader.read(pcd_file.toStdString(), data_set);   //读进全部点
    SPDLOG_INFO("read {} END ", pcd_file);

    data_pts.reserve(data_set.width);
    SPDLOG_INFO("start to filter");

    for (int i = 0; i < data_set.width; ++i)
    {
        double dis = xozDistance(data_set.points[i]);
        if (data_set.points[i].intensity >= 220 || data_set.points[i].intensity <= 120 || dis < 0.1 || dis > 10)
        {
            continue;
        }
        else
        {
            data_pts.push_back(data_set.points[i]);
        }
    }
    pcl::io::savePCDFileBinary("data_pts.pcd", data_pts);
    SPDLOG_TRACE("After indentity filter,data_pts.size:   {}", (data_pts.size()));
    SPDLOG_TRACE("indentity filer complete,start eliipse filter");

    for (int m = 0; m < 7; ++m)
    {
        ellipse_fit = RansacFitEllipse(data_pts);
        filter(data_pts, ellipse_fit, mean, sd, noise_pts, use_pts);
        data_pts = use_pts;
        GetEllipseParam(ellipse_fit, x0, y0, alpha, majorAxis, minorAxis);
        DiameterOfConvergence(use_pts, x0, y0, dia_of_convergence);
        ovality = GetOvality(majorAxis, minorAxis);

        QString usepoint_path   = output_root + QString("/4-5000-198-f3360-%1-use-%2").arg(lineth).arg(m);
        QString noisepoint_path = output_root + QString("/4-5000-198-f3360-%1-noise-%2").arg(lineth).arg(m);
        dir.cd(output_root);
        if (!dir.exists(usepoint_path))
        {
            dir.mkpath(usepoint_path);
        }
        if (!dir.exists(noisepoint_path))
        {
            dir.mkpath(noisepoint_path);
        }
        if (!dir.exists(param_path))
        {
            dir.mkpath(param_path);
        }
        pcl::io::savePCDFileBinary(usepoint_path.toStdString() + "/use" + ".pcd", use_pts);
        SPDLOG_TRACE("save use-pts end");
        if (noise_pts.size() == 0)
        {
            pcl::PointXYZI point;
            point.x         = 0;
            point.y         = 0;
            point.z         = 0;
            point.intensity = 160;

            noise_pts.push_back(point);
        }

        pcl::io::savePCDFileBinary(noisepoint_path.toStdString() +
                                       "/noise" + ".pcd",
                                   noise_pts);
        SPDLOG_TRACE("save noise-pts end");

        write_stream.setFieldWidth(12);
        write_stream << lineth
                     << m
                     << x0
                     << y0
                     << alpha
                     << majorAxis
                     << minorAxis
                     << ovality
                     << dia_of_convergence
                     << x_dmf
                     << y_dmf
                     << mean
                     << sd
                     << use_pts.size()
                     << noise_pts.size()
                     << ellipse_fit.A
                     << ellipse_fit.B
                     << ellipse_fit.C
                     << ellipse_fit.D
                     << ellipse_fit.E
                     << ellipse_fit.F;
        write_stream.setFieldWidth(0);
        write_stream << endl;

        if (noise_pts.size() < 5)
        {
            SPDLOG_WARN("num of {} line, times {}, break", lineth, m);
            break;
        }
    }
}

void EllipseFit::DiameterOfConvergence(const pcl::PointCloud<pcl::PointXYZI> &use_point, double x0, double y0, double &dia_of_convergence)
{
    pcl::PointCloud<pcl::PointXYZI> zero_point;
    pcl::PointCloud<pcl::PointXYZI> opposite_zero_point;
    zero_point.reserve(use_point.size());
    opposite_zero_point.reserve(use_point.size());
    std::vector<double> x_0;
    std::vector<double> x_1;
    std::vector<double> y_0;
    std::vector<double> y_1;
    std::vector<double> a(n_polynomial + 1);
    std::vector<double> b(n_polynomial + 1);
    double              left_x  = 0.0;
    double              right_x = 0.0;
    dia_of_convergence          = 0.0;

    for (size_t i = 0; i < use_point.size(); ++i)
    {
        if (use_point[i].x > 0 && fabs(use_point[i].z - y0) < 0.55)
        {

            //            SPDLOG_INFO("use_point[ {} ].x: {}  ; use_point[ {} ].y: {}", i, use_point[i].x, i, use_point[i].z);

            zero_point.push_back(use_point[i]);
        }

        if (use_point[i].x < 0 && fabs(use_point[i].z - y0) < 0.55)

        {
            //            SPDLOG_INFO("use_point[ {} ].x: {}  ; use_point[ {} ].y: {}", i, use_point[i].x, i, use_point[i].z);
            opposite_zero_point.push_back(use_point[i]);
        }
    }
    x_0.resize(zero_point.size());
    x_1.resize(opposite_zero_point.size());
    y_0.resize(zero_point.size());
    y_1.resize(opposite_zero_point.size());

    for (size_t i = 0; i < zero_point.size(); i++)
    {
        x_0.push_back(zero_point[i].x);
        y_0.push_back(zero_point[i].z);
    }
    for (size_t j = 0; j < opposite_zero_point.size(); j++)
    {
        x_1.push_back(opposite_zero_point[j].x);
        y_1.push_back(opposite_zero_point[j].z);
    }
    pcl::io::savePCDFileBinary("D:/YM/data/rowpcd/zero_point.pcd", zero_point);
    pcl::io::savePCDFileBinary("D:/YM/data/rowpcd/oppsite_zero_point.pcd", opposite_zero_point);

    a = least_square_method(x_0, y_0, a);
    SPDLOG_TRACE("a[0]: {} ;a[1]: {} ;a[2]: {} ", a[0], a[1], a[2]);

    right_x = (-a[1] - std::sqrt(std::pow(a[1], 2) - 4 * a[2] * (a[0] - y0))) / (2 * a[2]);

    SPDLOG_TRACE("x0: {}", x0);
    SPDLOG_TRACE("right_x: {}", right_x);

    b = least_square_method(x_1, y_1, b);
    SPDLOG_TRACE("b[0]: {} ;b[1]: {} ;b[2]: {} ", b[0], b[1], b[2]);
    left_x = (-b[1] - std::sqrt(std::pow(b[1], 2) - 4 * b[2] * (b[0] - y0))) / (2 * b[2]);

    SPDLOG_TRACE("x0: {}", x0);
    SPDLOG_TRACE("left_x: {}", left_x);

    dia_of_convergence = right_x - left_x;
    zero_point.clear();
    opposite_zero_point.clear();
}

std::vector<double> EllipseFit::gaussian_elimination(std::vector<std::vector<double>> a, std::vector<double> b)
{
    int                 n = size(b);
    std::vector<double> x;   //定义方程组解
    x.resize(n);
    std::vector<double> mi_k;   //定义消去过程中的中间变量
    mi_k.resize(n);
    double sum;
    for (int i = 0; i < n; i++)
    {
        //判断能否用高斯消去法
        if (a[i][i] == 0)
        {
            std::cout << "can't use Gaussian meathod" << endl;
        }
    }
    //n-1步消元
    for (int k = 0; k < n - 1; k++)
    {
        //求出第i次初等行变换系数
        for (int j = k + 1; j < n; j++)
        {
            mi_k[j] = a[j][k] / a[k][k];
        }
        for (int i = k + 1; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                a[i][j] = a[i][j] - mi_k[i] * a[k][j];
            }
            b[i] = b[i] - mi_k[i] * b[k];
        }
    }   //回代过程
    x[n - 1] = b[n - 1] / a[n - 1][n - 1];
    for (int i = n - 2; i >= 0; i--)
    {
        sum = 0;
        for (int j = i + 1; j < n; j++)
        {
            sum = sum + a[i][j] * x[j];
        }
        x[i] = (b[i] - sum) / a[i][i];
    }
    return x;
}

std::vector<double> EllipseFit::least_square_method(std::vector<double> x, std::vector<double> y, std::vector<double> a)
{
    size_t                           n_data = size(x), n_num = size(a);
    std::vector<std::vector<double>> phi_phi;
    std::vector<double>              phi_f;
    phi_f.resize(n_num);
    phi_phi.resize(n_num, std::vector<double>(n_num));
    for (int i = 0; i < n_num; i++)
    {
        for (int j = 0; j < n_num; j++)
        {
            for (int k = 0; k < n_data; k++)
            {
                phi_phi[i][j] = phi_phi[i][j] + phi(x[k], i) * phi(x[k], j);
            }
        }
        for (int k = 0; k < n_data; k++)
        {
            phi_f[i] = phi_f[i] + phi(x[k], i) * y[k];
        }
    }
    a              = gaussian_elimination(phi_phi, phi_f);   //多项式系数求解采用高斯消去法
    double delta_2 = 0;
    for (int i = 0; i < n_data; i++)
    {
        delta_2 = delta_2 + y[i] * y[i];
    }
    for (int i = 0; i < n_num; i++)
    {
        delta_2 = delta_2 - a[i] * phi_f[i];
    }
    //系数矩阵与平方误差输出
    for (int i = 0; i < n_num; i++)
    {
        for (int j = 0; j < n_num; j++)
        {
            SPDLOG_TRACE("phi_phi[{}][{}]: {}", i, j, phi_phi[i][j]);
        }
        SPDLOG_TRACE("phi_f[{}]: {}", i, phi_f[i]);
    }
    SPDLOG_TRACE("delta_2: {}", delta_2);
    return a;
}

double EllipseFit::phi(double x, int n_cishu)
{
    double y;
    y = pow(x, n_cishu);
    return y;
}

bool EllipseFit::linefit(QVector<QVector<ellipseCenter>> &allpoints, QVector<lineparam> &axisparam)
{
    const int              times_count = allpoints[0].size();
    const int              line_count  = allpoints.size();
    QVector<ellipseCenter> times_points;
    //    QVector<lineparam>     axisparam;
    axisparam.reserve(times_count);
    times_points.reserve(allpoints.size());
    SPDLOG_INFO("start to compute lineparam");
    for (int j = 0; j < times_count; ++j)
    {
        for (int i = 0; i < line_count; ++i)
        {
            times_points.push_back(allpoints[i][j]);
        }

        int size = times_points.size();

        if (size < 2)
        {
            axisparam[j].a = 0;
            axisparam[j].b = 0;
            axisparam[j].c = 0;
            return false;
        }
        double x_mean = 0;
        double y_mean = 0;

        for (int i = 0; i < size; i++)
        {
            x_mean += times_points[i].x0;
            y_mean += times_points[i].y0;
        }
        x_mean /= size;
        y_mean /= size;
        double Dxx = 0, Dxy = 0, Dyy = 0;

        for (int i = 0; i < size; i++)
        {
            Dxx += (times_points[i].x0 - x_mean) * (times_points[i].x0 - x_mean);
            Dxy += (times_points[i].x0 - x_mean) * (times_points[i].y0 - y_mean);
            Dyy += (times_points[i].y0 - y_mean) * (times_points[i].y0 - y_mean);
        }

        double lambda = ((Dxx + Dyy) - sqrt((Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy)) / 2.0;
        double den    = sqrt(Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx));

        if (fabs(den) < 1e-5)
        {
            if (fabs(Dxx / Dyy - 1) < 1e-5)   //这时没有一个特殊的直线方向，无法拟合
            {
                return false;
            }
            else
            {
                axisparam[j].a = 1;
                axisparam[j].b = 0;
                axisparam[j].c = -x_mean;
            }
        }
        else
        {
            axisparam[j].a = Dxy / den;
            axisparam[j].b = (lambda - Dxx) / den;
            axisparam[j].c = -axisparam[j].a * x_mean - axisparam[j].b * y_mean;
        }
        return true;
    }
    return true;
}

void EllipseFit::Axis_deformation(const QString &pcd_file, QVector<QVector<ellipseCenter>> &allpoints, QVector<lineparam> &allaxisparam, const QString &output_root)
{

    pcl::PointCloud<pcl::PointXYZI> data_set;   //全部点集
    pcl::PointCloud<pcl::PointXYZI> data_pts;   //每条线的点集
    axisdeormation                  defomation;
    double                          y_sum = 0.0;
    double                          x     = 0.0;
    double                          y     = 0.0;

    QDir    dir                  = "D:";
    QString axisdeformation_path = output_root + QString("/axisdeformation.txt");
    dir.mkpath(output_root);

    QFile write_file(axisdeformation_path);
    if (!write_file.open(QIODevice::WriteOnly))
    {
        SPDLOG_ERROR("error to write axisdeformation.txt");
        return;
    }

    QTextStream write_stream(&write_file);
    // 写入param的头部
    write_stream.setPadChar(' ');
    write_stream.setRealNumberPrecision(6);
    write_stream.setFieldAlignment(QTextStream::AlignRight);
    write_stream.setFieldWidth(12);
    write_stream << "index"
                 << "time"
                 << "x_dmf"
                 << "y_dmf";

    write_stream.setFieldWidth(0);
    write_stream << endl;

    SPDLOG_INFO("start to read {}", pcd_file);
    pcl::PCDReader reader;
    reader.read(pcd_file.toStdString(), data_set);   //读进全部点
    SPDLOG_INFO("read {} END ", pcd_file);
    const int times_count = allpoints[0].size();
    const int line_count  = allpoints.size();

    QVector<axisdeormation>          axis_deformation;
    QVector<QVector<axisdeormation>> allaxis_deformation;

    axis_deformation.reserve(allpoints.size());
    allaxis_deformation.reserve(allaxisparam.size());

    SPDLOG_INFO("start to compute lineparam");
    for (int j = 0; j < times_count; ++j)
    {
        for (int i = 0; i < line_count; ++i)
        {

            for (int m = 0; m < data_set.height; m++)
            {
                if (m % 1000 == 0)
                    SPDLOG_INFO("This is line:  {}", m);

                for (int j = 0; j < data_set.width; j++)
                {
                    int idx = m * data_set.width + j;

                    double dis = xozDistance(data_set.points[idx]);
                    if (data_set.points[idx].intensity >= 220 || data_set.points[idx].intensity <= 120 || (fabs(data_set.points[idx].x) < 0.8 && data_set.points[idx].z <= 0) || dis < 0.1 || dis > 10)
                    {
                        continue;
                    }
                    else
                    {
                        data_pts.push_back(data_set.points[idx]);
                    }
                }
                for (size_t n = 0; n < data_pts.size(); n++)
                {
                    y_sum += data_pts[n].y;
                }
                x                = y_sum / data_pts.size();
                y                = ((-allaxisparam[i].c) - (allaxisparam[i].a * x)) / allaxisparam[i].b;
                defomation.x_dmf = x - allpoints[i][j].x0;
                defomation.y_dmf = y - allpoints[i][j].y0;
                axis_deformation.push_back(defomation);

                SPDLOG_TRACE("x0: {}    x: {}", allpoints[i][j].x0, x);
                SPDLOG_TRACE("y0: {}    y: {}", allpoints[i][j].y0, y);
                SPDLOG_TRACE("x_dmf: {}    y_dmf: {}", defomation.x_dmf, defomation.y_dmf);
                write_stream.setFieldWidth(12);
                write_stream << i
                             << j
                             << defomation.x_dmf
                             << defomation.y_dmf;
                write_stream.setFieldWidth(0);
                write_stream << endl;
            }

            allaxis_deformation.push_back(axis_deformation);
            data_pts.clear();
        }
    }
}
double EllipseFit::xozDistance(const pcl::PointXYZI &point)
{
    return std::sqrt(point.x * point.x + point.z * point.z);
}
