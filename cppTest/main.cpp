#include "cppTest.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    TestThread   t;
    t.start();
    QObject::connect(&t, &TestThread::finished, [&]() {
        std::cout << "test thread finished: " << t.isFinished();
    });

    return a.exec();
}

#if 0
#    include <gtsam/geometry/PreintegratedImuMeasurements.h>
#    include <gtsam/navigation/ImuFactor.h>

// 定义一个IMU预积分因子类
class CustomImuFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Velocity3>
{
public:
    CustomImuFactor(const gtsam::PreintegratedImuMeasurements &preintegratedMeasurements,
                    const gtsam::Key &                         poseKey,
                    const gtsam::Key &                         velocityKey,
                    const gtsam::Vector3 &                     measuredAccel,
                    const gtsam::Vector3 &                     measuredOmega,
                    const gtsam::SharedNoiseModel &            model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Velocity3>(model, poseKey, velocityKey)
    {
        // 根据IMU测量数据计算预积分因子的残差
        // 这里只是一个简化的实现，实际应根据IMU预积分方法进行更详细的计算
        gtsam::Vector6 imuDelta      = preintegratedMeasurements.predict(gtsam::Vector3::Zero(), gtsam::Vector3::Zero(), 0);
        gtsam::Vector3 deltaVel      = imuDelta.head<3>();
        gtsam::Vector3 deltaPos      = imuDelta.tail<3>();
        gtsam::Vector3 residual      = measuredAccel - deltaVel;
        residual_.template head<3>() = residual;
        residual.template tail<3>()  = measuredOmega - deltaPos;
    }

    // 重载计算残差的函数
    gtsam::Vector evaluateError(const gtsam::Pose3 &pose, const gtsam::Velocity3 &velocity, boost::optional<gtsam::Matrix &> H1 = boost::none, boost::optional<gtsam::Matrix &> H2 = boost::none) const override
    {
        if (H1)
            *H1 = gtsam::Matrix::Zero(6, 6);
        if (H2)
            *H2 = gtsam::Matrix::Zero(6, 3);
        return residual_;
    }

private:
    gtsam::Vector6 residual_;   // 残差
};

// 在主函数中使用IMU预积分因子
int main()
{
    // 创建IMU预积分对象
    gtsam::PreintegratedImuMeasurements preintegratedMeasurements(/* imuParams */);

    // 构建因子图
    gtsam::NonlinearFactorGraph graph;
    gtsam::Key                  poseKey     = gtsam::Symbol('x', 0);
    gtsam::Key                  velocityKey = gtsam::Symbol('v', 0);
    gtsam::Vector3              measuredAccel, measuredOmega;                          // IMU测量数据
    gtsam::SharedNoiseModel     model = gtsam::noiseModel::Isotropic::Sigma(6, 1.0);   // 噪声模型
    CustomImuFactor             imuFactor(preintegratedMeasurements, poseKey, velocityKey, measuredAccel, measuredOmega, model);
    graph.add(imuFactor);

    // 构建初始值估计
    gtsam::Values initialEstimate;
    initialEstimate.insert(poseKey, gtsam::Pose3());
    initialEstimate.insert(velocityKey, gtsam::Velocity3());

    // 优化因子图
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    gtsam::Values                      result = optimizer.optimize();

    return 0;
}
#    include <QApplication>
#    include <QDebug>
#    include <iomanip>
#    include <iostream>
#    include <time.h>
#    include <windows.h>

#    include <QAxBase>
#    include <QDateTime>
#    include <QFile>
#    include <QString>
#    include <QStringRef>
#    include <QTextStream>
#    include <QVector>

//#include <execution>
#    include <spdlog/qt_spdlog.h>

#    define _USE_MATH_DEFINES
#    include <math.h>

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

using namespace std;

struct Pose
{
    int64_t usec_in_day = -1;
    double  latitude    = 0.0;
    double  longitude   = 0.0;
    double  height      = 0.0;
    double  x           = 0.0;
    double  y           = 0.0;
    double  z           = 0.0;
    double  roll        = 0.0;
    double  pitch       = 0.0;
    double  yaw         = 0.0;
    double  heave       = 0.0;

    static Pose interpolation(const Pose &b, const Pose e, uint64_t time);
};

Pose operator-(const Pose &left, const Pose &right)
{
    Pose sub;
    sub.usec_in_day = left.usec_in_day - right.usec_in_day;
    sub.latitude    = left.latitude - right.latitude;
    sub.longitude   = left.longitude - right.longitude;
    sub.height      = left.height - right.height;
    sub.x           = left.x - right.x;
    sub.y           = left.y - right.y;
    sub.z           = left.z - right.z;
    sub.roll        = left.roll - right.roll;
    sub.pitch       = left.pitch - right.pitch;
    sub.yaw         = left.yaw - right.yaw;
    sub.heave       = left.heave - right.heave;
    return sub;
}

Pose operator+(const Pose &left, const Pose &right)
{
    Pose tmp;
    tmp.usec_in_day = left.usec_in_day + right.usec_in_day;
    tmp.latitude    = left.latitude + right.latitude;
    tmp.longitude   = left.longitude + right.longitude;
    tmp.height      = left.height + right.height;
    tmp.x           = left.x + right.x;
    tmp.y           = left.y + right.y;
    tmp.z           = left.z + right.z;
    tmp.roll        = left.roll + right.roll;
    tmp.pitch       = left.pitch + right.pitch;
    tmp.yaw         = left.yaw + right.yaw;
    tmp.heave       = left.heave + right.heave;
    return tmp;
}

Pose operator*(const Pose &left, double scale)
{
    Pose tmp;
    tmp.usec_in_day = left.usec_in_day * scale;
    tmp.latitude    = left.latitude * scale;
    tmp.longitude   = left.longitude * scale;
    tmp.height      = left.height * scale;
    tmp.x           = left.x * scale;
    tmp.y           = left.y * scale;
    tmp.z           = left.z * scale;
    tmp.roll        = left.roll * scale;
    tmp.pitch       = left.pitch * scale;
    tmp.yaw         = left.yaw * scale;
    tmp.heave       = left.heave * scale;
    return tmp;
}

Pose Pose::interpolation(const Pose &b, const Pose e, uint64_t time)
{
    Pose inter_pose;
    if (b.usec_in_day <= time && time < e.usec_in_day)
    {
        double scale = (double) (time - b.usec_in_day) / (double) (e.usec_in_day - b.usec_in_day);
        inter_pose   = b + (e - b) * scale;

        inter_pose.usec_in_day = time;
    }
    return inter_pose;
}

struct PoseStd
{
    float latitude_std  = 0.0f;
    float longitude_std = 0.0f;
    float height_std    = 0.0f;
    float x_std         = 0.0f;
    float y_std         = 0.0f;
    float z_std         = 0.0f;
    float roll_std      = 0.0f;
    float pitch_std     = 0.0f;
    float yaw_std       = 0.0f;
    float heave_std     = 0.0f;
};

using RangeIndex = std::pair<size_t, size_t>;   // [first, second)

struct IEPose
{
    QDate   date;
    Pose    pos;
    PoseStd std;
    int     quality = -1;

    static IEPose interpolation(const IEPose &b, const IEPose e, uint64_t time)
    {
        IEPose inter;
        inter.pos = Pose::interpolation(b.pos, e.pos, time);
        if (inter.pos.usec_in_day != -1)
        {
            inter.date    = b.date;
            inter.quality = b.quality;
        }
        return inter;
    }

    static RangeIndex findMinRange(const std::vector<IEPose> &poses, uint64_t time)
    {
        RangeIndex range;

        auto upper = std::upper_bound(poses.begin(), poses.end(), time, [](uint64_t usec, const IEPose &pose) {
            return usec < pose.pos.usec_in_day;
        });

        size_t index = std::distance(poses.begin(), upper);

        if (upper == poses.begin() || upper == poses.end())
        {
            range.second = index;
            range.first  = index;
        }
        else
        {
            range.second = index;
            range.first  = index - 1;
        }

        return range;
    }
};
std::vector<IEPose> readIEPoseFile(const QString &filename)
{
    QFile pose_file(filename);
    if (!pose_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_ERROR("打开轨迹文件失败！！！");
        return {};
    }

    QTextStream read_stream(&pose_file);

    QString             read_line;
    QStringList         items;
    std::vector<IEPose> poses;
    IEPose              pose;
    int                 old_msec;
    size_t              count = 0;
    while (!read_stream.atEnd())
    {
        read_line = read_stream.readLine();
        count++;
        if (count % 100000 == 0)
        {
            SPDLOG_DEBUG("read pos num: {}", count);
        }
        items = read_line.split(" ", QString::SkipEmptyParts);
        if (items.size() < 23)
            continue;

        pose.date = QDate::fromString(items[0], "MM/dd/yyyy");
        if (!pose.date.isValid())
            continue;

        double sec  = items[3].toDouble();
        int    msec = std::round((sec - (int) sec) * 1000);
        QTime  time(items[1].toInt(), items[2].toInt(), (int) sec, msec);

        old_msec             = pose.pos.usec_in_day;
        pose.pos.usec_in_day = static_cast<int64_t>(time.msecsSinceStartOfDay()) * 1000;
        if (old_msec > pose.pos.usec_in_day)
        {
            SPDLOG_ERROR("轨迹文件中的数据未按时间排序！！！");
        }
        pose.pos.latitude  = (items[4].toInt() + items[5].toInt() / 60.0 + items[6].toDouble() / 3600.0) * DEG2RAD;
        pose.pos.longitude = (items[7].toInt() + items[8].toInt() / 60.0 + items[9].toDouble() / 3600.0) * DEG2RAD;
        pose.pos.height    = items[10].toDouble();
        pose.pos.x         = items[11].toDouble();
        pose.std.x_std     = items[12].toFloat();
        pose.pos.y         = items[13].toDouble();
        pose.std.y_std     = items[14].toFloat();
        pose.pos.z         = items[15].toDouble();
        pose.std.z_std     = items[16].toFloat();
        pose.pos.roll      = items[17].toDouble() * DEG2RAD;
        pose.std.roll_std  = items[18].toFloat() * DEG2RAD;
        pose.pos.pitch     = items[19].toDouble() * DEG2RAD;
        pose.std.pitch_std = items[20].toFloat() * DEG2RAD;
        pose.pos.yaw       = items[21].toDouble() * DEG2RAD;
        pose.std.yaw_std   = items[22].toFloat() * DEG2RAD;
        pose.quality       = items.value(23, "0").toInt();
        pose.pos.heave     = items.value(24, "0.0").toDouble();
        poses.push_back(pose);
    }
    return poses;
}

IEPose string2IEPose(const QString &str)
{
    IEPose pose;

    QStringList items = str.split(" ", QString::SkipEmptyParts);
    if (items.size() < 23)
        return pose;

    pose.date = QDate::fromString(items[0], "MM/dd/yyyy");
    if (!pose.date.isValid())
        return pose;

    double sec  = items[3].toDouble();
    int    msec = std::round((sec - (int) sec) * 1000);
    QTime  time(items[1].toInt(), items[2].toInt(), (int) sec, msec);

    pose.pos.usec_in_day = static_cast<int64_t>(time.msecsSinceStartOfDay()) * 1000;
    pose.pos.latitude    = (items[4].toInt() + items[5].toInt() / 60.0 + items[6].toDouble() / 3600.0) * DEG2RAD;
    pose.pos.longitude   = (items[7].toInt() + items[8].toInt() / 60.0 + items[9].toDouble() / 3600.0) * DEG2RAD;
    pose.pos.height      = items[10].toDouble();
    pose.pos.x           = items[11].toDouble();
    pose.std.x_std       = items[12].toFloat();
    pose.pos.y           = items[13].toDouble();
    pose.std.y_std       = items[14].toFloat();
    pose.pos.z           = items[15].toDouble();
    pose.std.z_std       = items[16].toFloat();
    pose.pos.roll        = items[17].toDouble() * DEG2RAD;
    pose.std.roll_std    = items[18].toFloat() * DEG2RAD;
    pose.pos.pitch       = items[19].toDouble() * DEG2RAD;
    pose.std.pitch_std   = items[20].toFloat() * DEG2RAD;
    pose.pos.yaw         = items[21].toDouble() * DEG2RAD;
    pose.std.yaw_std     = items[22].toFloat() * DEG2RAD;
    pose.quality         = items.value(23, "0").toInt();
    pose.pos.heave       = items.value(24, "0.0").toDouble();
    return pose;
}

#    include <QFileDialog>

#    pragma pack(push, 1)
//! bin文件字段
struct InterpolateAllInformation
{
    int32_t lineID;     ///<扫描仪信息-扫描仪线号。
    double  lineTime;   ///<扫描仪信息-线第一个点时间作为lineTime时间。
    double  gpsTime;    ///<扫描仪信息-插值后扫描仪时间。

    double dist;     ///<扫描仪信息-扫描仪极径。
    double hAngle;   ///<扫描仪信息-扫描仪水平距离角。
    double vAngle;   ///<扫描仪信息-扫描仪竖直距离角。

    uint16_t intensity;   ///<扫描仪信息-扫描仪强度。

    double sigma_distance;   ///<扫描仪信息-扫描仪极径误差。
    double sigma_hAngle;     ///<扫描仪信息-扫描仪极径误差。
    double sigma_vAngle;     ///<扫描仪信息-扫描仪极径误差。

    double roll;      ///<惯导信息-同时刻POS横滚角。
    double pitch;     ///<惯导信息-同时刻POS俯仰角。
    double heading;   ///<惯导信息-同时刻POS方向角。

    double sigma_roll;      ///<惯导信息-同时刻POS横滚角误差。
    double sigma_pitch;     ///<惯导信息-同时刻POS俯仰角误差。
    double sigma_heading;   ///<惯导信息-同时刻POS方向角误差。

    double latitude;     ///<POS信息-同时刻POS纬度-B。
    double longtitude;   ///<POS信息-同时刻POS经度-L。
    double height;       ///<POS信息-同时刻POS高度-H。

    double x_oe;   ///<WGS84  X
    double y_oe;   ///<WGS84  Y
    double z_oe;   ///<WGS84  Z

    double sigma_x_oe;
    double sigma_y_oe;
    double sigma_z_oe;

    double laser_x_e;
    double laser_y_e;
    double laser_z_e;

    uint16_t r;   ///<点云颜色R
    uint16_t g;   ///<点云颜色G
    uint16_t b;   ///<点云颜色B
};
#    pragma pack(pop)

#    include <Dialog.h>
#    include <random>

#    include <QInputDialog>
typedef void (*ProgressFun)(void *context, int progress);
typedef void(__stdcall *ProgressCallback)(float progress, const char *msg, const char *point_cloud_path);
void create(int a, ProgressCallback cb = nullptr)
{
    for (int i = 0; i < a; ++i)
    {
        if (cb)
            cb(i, "create progress", "");
    }
}

#    include <functional>

using ProgressCallbackEx = std::function<void(float progress, const char *msg, const char *point_cloud_path)>;

void createEx(int a, ProgressCallbackEx cb = nullptr)
{
    for (int i = 0; i < a; ++i)
    {
        if (cb)
            cb(i, "createEx", "");
    }
}

void generateDEMEx(ProgressFun pf = nullptr)
{
    createEx(10, [=](float progress, const char *msg, const char *point_cloud_path) {
        if (pf)
            pf((void *) nullptr, progress);
    });
}

void generateDEM(void *context, ProgressFun pf = nullptr)
{
    static ProgressFun s_pf = nullptr;
    s_pf                    = pf;
    static void *s_context  = nullptr;
    s_context               = context;
    create(10, [](float progress, const char *msg, const char *point_cloud_path) {
        if (s_pf)
            s_pf(s_context, progress);
    });
}

struct Context
{
    double a;
    double b;
};

#    include <Eigen/Dense>
#    include <iostream>

Eigen::MatrixXd removeRow(Eigen::MatrixXd originalMatrix, int rowIndexToRemove)
{
    Eigen::MatrixXd newMatrix(originalMatrix.rows() - 1, originalMatrix.cols());
    newMatrix << originalMatrix.topRows(rowIndexToRemove),
        originalMatrix.bottomRows(originalMatrix.rows() - rowIndexToRemove - 1);
    return newMatrix;
}

int main()
{
    vector<double> s1;
    vector<double> s2;
    for (int i = 0; i < t.size(); i++)
    {
        s1.push_back(A * cos(2 * M_PI * flow_2fsk1 * t[i]));
        s2.push_back(A * cos(2 * M_PI * flow_2fsk2 * t[i]));
    }
    // 假设我们有一个3x4的矩阵
    Eigen::MatrixXd originalMatrix(4, 4);
    originalMatrix << 1, 2, 3, 4,
        5, 6, 7, 8,
        8, 6, 7, 8,
        9, 10, 11, 12;

    vector<int> indexs {1, 3};

    Eigen::MatrixXd newMatrix = originalMatrix;
    for (int i = indexs.size() - 1; i >= 0; --i)
    {
        // 要去除的列的索引
        int columnIndexToRemove = indexs[i];   // 假设要去除第2列（索引从0开始）

        // 创建新的矩阵，去除指定的列

        newMatrix = removeRow(newMatrix, columnIndexToRemove).eval();
    }

    // 输出结果
    std::cout << "Original Matrix:" << std::endl
              << originalMatrix << std::endl;
    std::cout << "New Matrix (after removing column):" << std::endl
              << newMatrix << std::endl;

    return 0;
}

//    QApplication a(argc, argv);

//    QString file_name = QFileDialog::getOpenFileName(nullptr, "pcap", "E:");
//    QFile   sdc_file(file_name);

//    sdc_file.open(QIODevice::ReadOnly);
//    int32_t size = 0;
//    sdc_file.read((char *) &size, sizeof(int32_t));
//    int16_t major, minor;
//    sdc_file.read((char *) &major, sizeof(int16_t));
//    sdc_file.read((char *) &minor, sizeof(int16_t));
//    char *head_info = new char[size - 8];
//    sdc_file.read(head_info, size - 8);
//    QString head_str = QString::fromUtf8(head_info, size - 8);
//    qDebug() << head_str;
//std::cout << head_info << std::endl;

/*
    Context co = {1.1, 2.2};

    generateDEM((void *) &co, [](void *context, int progress) {
        Context *c1 = reinterpret_cast<Context *>(context);
        printf("%f + %f : %d\n", c1->a, c1->b, progress);
    });

    generateDEMEx([](void *context, int progress) {
        printf("Ex : %d\n", progress);
    });
    QStringList files = QFileDialog::getOpenFileNames(nullptr, "pcap", "X://Calibration");

    QString num = QInputDialog::getText(nullptr, "set num", "num");
    for (auto filename : files)
    {
        QFileInfo   file_info(filename);
        QString     name  = file_info.completeBaseName();
        QStringList items = name.split("_");
        if (items.size() < 3)
        {
            items.push_back(num);
        }
        QDateTime datetime = QDateTime::fromString(items[0], "yyyy-MM-dd-HH-mm-ss");
        //QString   new_name = QString("%1_vlp%2%3").arg(datetime.toString("yyMMdd-HHmmss")).arg(items[2]).arg(file_info.suffix());
        QFile       file(filename);
        QStringList new_items = {items[3], datetime.toString("yyMMdd-HHmmss"), num};
        QString     new_name  = new_items.join("_") + ".txt";
        //QFile       file(filename);
        qDebug() << name << "-->" << new_name << ":" << file.rename(file_info.absolutePath() + "/" + new_name);
    }

    /*
    QFile bin_file(filename);

    if (!bin_file.open(QIODevice::ReadOnly))
    {
        SPDLOG_ERROR("打开 bin 文件失败！！！");
        return -1;
    }

    QVector<InterpolateAllInformation> bins;
    while (!bin_file.atEnd())
    {
        InterpolateAllInformation bin_item;
        bin_file.read((char *) &bin_item, sizeof(InterpolateAllInformation));
        bins.push_back(bin_item);
    }
    */

#endif
