#include "stdafx.h"

#include "TunnelCrossSection.h"
#include <QDir>
#include <QTextStream>
#include <QTime>
#include <QVector>
#include <limits>
#include <math.h>

#include <random>
TunnelCrossSection::TunnelCrossSection()
{
}

bool TunnelCrossSection::ransac(int iterations, int n)
{
    std::random_device                     rd;          //Will be used to obtain a seed for the random number engine
    std::mt19937                           gen(rd());   //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<int32_t> dis(0, mLineData.size() - 1);

    double t_n = 0;
    double P   = 0.9;

    // TODO: iterations 可以设置为RANSAC类的参数
    //    int         iterations           = 10000;
    int64_t n_best_inliers_count = 0;
    int64_t max_iterations       = INT64_MAX;

    QVector<int> randflags(n);
    //TODO: 6 可以设置为RANSAC类的参数

    EllipseCoefficients best_ellipse_coefficients;
    QVector<int>        inliers, unInliers;
    double              mean, std;
    for (int64_t i = 0; i < iterations; i++)
    {
        for (int j = 0; j < n; ++j)
        {
            randflags[j] = dis(gen);
        }
        if (!computeModelCoefficients(randflags, &best_ellipse_coefficients))
        {
            SPDLOG_TRACE("未计算出椭圆参数");
            continue;
        }
        countWithinDistance(best_ellipse_coefficients, inliers, unInliers, mean, std);

        if (inliers.size() > n_best_inliers_count)
        {
            //            P = pow(1 - t_n, max_iterations);
            n_best_inliers_count     = inliers.size();
            mBestEllipseCoefficients = best_ellipse_coefficients;
            mInlierflags             = inliers;
            mUnlierflags             = unInliers;
            mMean                    = mean;
            mStd                     = std;
            t_n                      = pow(inliers.size() / (double) mLineData.size(), n);
            max_iterations           = log(1 - P) / log(1 - t_n);
            if (max_iterations < 0)
                max_iterations = INT64_MAX;   // 由于随机点选择非常糟糕导致inliers.size() 非常小， t_n 很接近0， 导致计算出负数
            SPDLOG_INFO("iter:{}, inliners: {}, mean: {}, std: {}, max_iter: {}",
                        i,
                        n_best_inliers_count,
                        mean,
                        std,
                        max_iterations);
            if (i > max_iterations)
            {
                break;
            }
        }
    }
    return true;
}

bool TunnelCrossSection::computeModelCoefficients(const QVector<int> &randflags, EllipseCoefficients *ellipse_coefficients)
{
    assert(ellipse_coefficients);
    if (randflags.size() < 6)
    {
        SPDLOG_CRITICAL("No enough points to fit ellipse.");
        return false;
    }

    // 椭圆的一般方程：Ax2 + Bxy + Cy2 + Dx + Ey + F = 0
    Eigen::MatrixXf leftm((randflags.size()), 5);
    Eigen::MatrixXf rightm((randflags.size()), 1);
    for (int i = 0; i < randflags.size(); i++)
    {
        PointXYZIT point = mLineData[randflags[i]];
        leftm(i, 0)      = point.x * point.z;
        leftm(i, 1)      = point.z * point.z;
        leftm(i, 2)      = point.x;
        leftm(i, 3)      = point.z;
        leftm(i, 4)      = 1;

        rightm(i, 0) = -(point.x * point.x);
    }

    Eigen::MatrixXf cof_mat(5, 1);

    cof_mat                 = ((leftm.transpose() * leftm).inverse()) * (leftm.transpose() * rightm);
    ellipse_coefficients->A = 1;
    ellipse_coefficients->B = cof_mat(0, 0);
    ellipse_coefficients->C = cof_mat(1, 0);
    ellipse_coefficients->D = cof_mat(2, 0);
    ellipse_coefficients->E = cof_mat(3, 0);
    ellipse_coefficients->F = cof_mat(4, 0);

    return true;
}

bool TunnelCrossSection::countWithinDistance(const EllipseCoefficients &coefficient,
                                             QVector<int> &             inliers,
                                             QVector<int> &             unInliers,
                                             double &                   mean,
                                             double &                   std)
{

    const int32_t   data_size = mLineData.size();
    QVector<double> dis(data_size);

    float di_sum         = 0.0;
    float sum_of_squares = 0.0;
    unInliers.clear();
    inliers.clear();

    for (int i = 0; i < data_size; ++i)
    {
        float x  = mLineData[i].x;
        float x2 = mLineData[i].x * mLineData[i].x;
        float y  = mLineData[i].z;
        float y2 = mLineData[i].z * mLineData[i].z;
        float xy = mLineData[i].x * mLineData[i].z;

        // clang-format off
        dis[i] = coefficient.A * x2
               + coefficient.B * xy
               + coefficient.C * y2
               + coefficient.D * x
               + coefficient.E * y
               + coefficient.F;
        // clang-format on
    }

    unInliers.reserve(data_size);
    inliers.reserve(data_size);

    for (int i = 0; i < data_size; ++i)
    {
        if (fabs(dis[i]) > 0.1)
        {
            unInliers.push_back(i);
        }
        else
        {
            inliers.push_back(i);
        }
    }

    for (int i = 0; i < inliers.size(); ++i)
    {
        int index = inliers[i];
        di_sum += dis[index];
    }
    mean = di_sum / inliers.size();

    for (int i = 0; i < inliers.size(); ++i)
    {
        int index = inliers[i];
        sum_of_squares += pow(dis[index] - mean, 2);
    }
    std = sqrt(sum_of_squares / inliers.size());

    return true;
}

void TunnelCrossSection::calcuateEllipseStandParam()
{
    double A, B, C, D, E, F;
    // F归一
    A = mBestEllipseCoefficients.A / mBestEllipseCoefficients.F;
    B = mBestEllipseCoefficients.B / mBestEllipseCoefficients.F;
    C = mBestEllipseCoefficients.C / mBestEllipseCoefficients.F;
    D = mBestEllipseCoefficients.D / mBestEllipseCoefficients.F;
    E = mBestEllipseCoefficients.E / mBestEllipseCoefficients.F;
    F = mBestEllipseCoefficients.F / mBestEllipseCoefficients.F;

    // 中心坐标
    double x0             = 0.0;
    double y0             = 0.0;
    x0                    = ((B * E) - (2 * C * D)) / ((4 * A * C) - (B * B));
    y0                    = ((B * D) - (2 * A * E)) / ((4 * A * C) - (B * B));
    mBestStandardParam.x0 = x0;
    mBestStandardParam.y0 = y0;

    // 长短轴
    mBestStandardParam.majorAxis = (std::max)(sqrt(2 * (A * x0 * x0 + C * y0 * y0 + B * x0 * y0 - 1) /
                                                   (A + C - sqrt((A - C) * (A - C) + B * B))),
                                              sqrt(2 * (A * x0 * x0 + C * y0 * y0 + B * x0 * y0 - 1) /
                                                   (A + C + sqrt((A - C) * (A - C) + B * B))));
    mBestStandardParam.minorAxis = (std::min)(sqrt(2 * (A * x0 * x0 + C * y0 * y0 + B * x0 * y0 - 1) /
                                                   (A + C - sqrt((A - C) * (A - C) + B * B))),
                                              sqrt(2 * (A * x0 * x0 + C * y0 * y0 + B * x0 * y0 - 1) /
                                                   (A + C + sqrt((A - C) * (A - C) + B * B))));
    // 角度
    mBestStandardParam.alpha = (atan(B / (A - C))) / 2;
}

void TunnelCrossSection::calcuateConvergenceDiameter()
{
}

void TunnelCrossSection::calcuateEllipseDeformation(double radial)
{
    mAxisDelta = mBestStandardParam.majorAxis - mBestStandardParam.minorAxis;
    mOvality   = mAxisDelta / radial * 1000;
}
