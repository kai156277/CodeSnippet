#include "7kdef.h"
//#include "bathyformat.h"

#include <QApplication>
#include <QDataStream>
#include <QDateTime>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <spdlog/qt_spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

constexpr float rad_to_deg = 57.29578;

template <>
struct fmt::formatter<R7027RD>
{
    template <typename ParseContext>
    constexpr auto parse(ParseContext &ctx)
    {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const R7027RD &rd, FormatContext &ctx)
    {
        return format_to(ctx.out(),
                         "{:10d} {:10f} {:10f} {:5b} {:5b} {:10f} ",   //{:20f} {:20f} {:20f}",
                         rd.mBeamDescriptor,
                         rd.mDetectionPoint,
                         rd.mRxAngle * rad_to_deg,
                         rd.mFlags,
                         rd.mQuality,
                         rd.mUncertainty);
        //                         rd.mIntensity,
        //                         rd.mMinLimit,
        //                         rd.mMaxLimit);
    }
};

template <>
struct fmt::formatter<R7027RTH>
{
    template <typename ParseContext>
    constexpr auto parse(ParseContext &ctx)
    {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const R7027RTH &rd, FormatContext &ctx)
    {
        return format_to(ctx.out(),
                         "sonar id:{} ping num:{} multiping seq:{} num:{} ds:{} da:{} flags:{} sampling rate:{} tx angle:{} applied roll:{}",
                         rd.mSerialId,
                         rd.mPingNumber,
                         rd.mMultipingSequence,
                         rd.mDetectionNums,
                         rd.mDataSize,
                         rd.mDetectionAlgorithm,
                         rd.mFlags,
                         rd.mSamplingRate,
                         rd.mTxAngle,
                         rd.mAppliedRoll);
    }
};

void log_init();
void saveTxtInfo(const QString &s7k_file_str, const QVector<R7027> &r7027s, const QVector<R7000> &r7000s);

int main(int argc, char *argv[])
{
    system("chcp 65001");
    QApplication a(argc, argv);
    log_init();

    /*
    QString xsmb = QFileDialog::getOpenFileName(nullptr, "xsmb", "D:\\Data\\0806-7125\\PDS2000-s7k", "xsmb (*.xsmb)");
    //    QString xsmb = "D:\\Data\\0806-7125\\PDS2000-s7k\\DATA2021080203_001.xsmb";
    if (!xsmb.isEmpty())
    {
        QFile xsmb_file(xsmb);
        xsmb_file.open(QIODevice::ReadOnly);
        BATHYFORMAT bf;
        bf.fileReadMbFile(xsmb_file);
        qDebug() << "END";
    }
    return 0;
*/
    QStringList s7k_file_strs = QFileDialog::getOpenFileNames(nullptr, "open s7k file", "", "s7k (*.s7k)");
    qDebug() << "num of file: " << s7k_file_strs.size();
    for (int i = 0; i < s7k_file_strs.size(); ++i)
    {
        QString   s7k_file_str = s7k_file_strs[i];
        QFileInfo s7k_file_info(s7k_file_str);

        if (s7k_file_str.isEmpty())
        {
            qDebug() << "s7k file is empty:" << s7k_file_str;
            return -1;
        }

        QFile s7k_file(s7k_file_str);
        if (!s7k_file.open(QIODevice::ReadOnly))
        {
            qDebug() << "can`t read s7k file";
            return -1;
        }

        qDebug() << QString("[%1] file: %2").arg(i + 1).arg(s7k_file_str);
        QDataStream read_stream(&s7k_file);
        read_stream.setByteOrder(QDataStream::LittleEndian);

        DataRecordFrame         drf;
        QVector<R7000>          r7000s;
        QVector<R7004>          r7004s;
        QVector<R7027>          r7027s;
        const int32_t           drf_size = sizeof(drf);
        QMap<uint32_t, int32_t> rt_id;
        while (!read_stream.atEnd())
        {
            read_stream.readRawData((char *) &drf, drf_size);
            const int32_t data_size = drf.mSize - drf_size;
            char *        raw_data  = new char[data_size];
            read_stream.readRawData((char *) raw_data, data_size);   // 包含尾部
            // TODO: CRC
            // read_stream.skipRawData(data_size);
            if (drf.mSyncPattern != 0x0000FFFF)
            {
                qDebug() << "sync pattern error!";
                continue;
            }
            if (rt_id.contains(drf.mRecordTypeIdentifier))
            {
                int32_t count = rt_id.value(drf.mRecordTypeIdentifier);
                rt_id.insert(drf.mRecordTypeIdentifier, count + 1);
            }
            else
            {
                rt_id.insert(drf.mRecordTypeIdentifier, 1);
            }
            switch (drf.mRecordTypeIdentifier)
            {
            case S7K_R7000: {
                R7000 r7000;
                memcpy_s((void *) &r7000.mRTH, sizeof(R7000RTH), (void *) raw_data, sizeof(R7000RTH));
                r7000.mDRF = drf;
                SPDLOG_TRACE("ping num : {}, sv: {}", r7000.mRTH.mPingNumber, r7000.mRTH.mSoundVelocity);
                r7000s.push_back(r7000);
                break;
            }
            case S7K_R7004: {
                R7004         r7004;
                const int32_t rth_shift = sizeof(R7004RTH);
                memcpy_s((void *) &r7004.mRTH, rth_shift, (void *) raw_data, rth_shift);
                const int32_t beams    = r7004.mRTH.mNumOfBeams;
                const int32_t rd_shift = sizeof(float) * beams;
                r7004.mBeamVAngles     = new float[beams];
                r7004.mBeamHAngles     = new float[beams];
                r7004.mBeamWidthYs     = new float[beams];
                r7004.mBeamWidthXs     = new float[beams];

                memcpy_s((void *) r7004.mBeamVAngles,
                         rd_shift,
                         (void *) (raw_data + rth_shift),
                         rd_shift);
                memcpy_s((void *) r7004.mBeamHAngles,
                         rd_shift,
                         (void *) (raw_data + rth_shift + rd_shift),
                         rd_shift);
                memcpy_s((void *) r7004.mBeamWidthYs,
                         rd_shift,
                         (void *) (raw_data + rth_shift + rd_shift * 2),
                         rd_shift);
                memcpy_s((void *) r7004.mBeamWidthXs,
                         rd_shift,
                         (void *) (raw_data + rth_shift + rd_shift * 3),
                         rd_shift);
                r7004.mDRF = drf;
                r7004s.push_back(r7004);
                break;
            }
            case S7K_R7027: {
                R7027         r7027;
                const int32_t rth_shift = sizeof(R7027RTH);
                const int32_t rd_shift  = sizeof(R7027RD);
                memcpy_s((void *) &r7027.mRTH, rth_shift, (void *) raw_data, rth_shift);
                // SPDLOG_TRACE("{}", r7027.mRTH);
                for (int j = 0; j < r7027.mRTH.mDetectionNums; ++j)
                {
                    R7027RD r7027rd;
                    memcpy_s((void *) &r7027rd, rd_shift, (void *) (raw_data + rth_shift + j * rd_shift), rd_shift);
                    if (r7027rd.mQuality == 0)
                        continue;
                    float two_way_time = r7027rd.mDetectionPoint / r7027.mRTH.mSamplingRate;
                    float range        = two_way_time * r7000s[0].mRTH.mSoundVelocity / 2;
                    //                    SPDLOG_INFO("{:10f}, {:10f} s, {:10f} m", r7027rd.mRxAngle * rad_to_deg, two_way_time, range);
                    r7027.mRDs.push_back(r7027rd);
                }
                r7027.mDRF = drf;
                r7027s.push_back(r7027);
                break;
            }
            default:
                break;
            }

            delete[] raw_data;
        }
        std::sort(r7000s.begin(),
                  r7000s.end(),
                  [](const R7000 &left, const R7000 &right) {
                      return left.mRTH.mPingNumber < right.mRTH.mPingNumber;
                  });

        std::sort(r7027s.begin(),
                  r7027s.end(),
                  [](const R7027 &left, const R7027 &right) {
                      return left.mRTH.mPingNumber < right.mRTH.mPingNumber;
                  });
        qDebug() << "record type ids:" << rt_id;
        const int size = std::min(r7000s.size(), r7027s.size());

        int r7000_index = 0;
        int r7027_index = 0;

        /*
        QFile xsmb_file(s7k_file_info.absolutePath() + "\\" + s7k_file_info.baseName() + ".xsmb");
        if (!xsmb_file.open(QIODevice::WriteOnly))
        {
            qDebug() << "can`t open xsmb_file file";
            return -1;
        }

        QByteArray  mbFileHeadarray;
        BATHYFORMAT sonic_bathy_object;
        MBFILEHEAD  mbFileSaveHead;
        mbFileSaveHead.header_size   = 80;
        mbFileSaveHead.version_major = 1;
        mbFileSaveHead.version_minor = 0;
        QByteArray _sensorNameArr("Seabat7125");
        int        _nameSize = 16;
        if (_sensorNameArr.size() < 16)
            _nameSize = _sensorNameArr.size();
        memcpy(&(mbFileSaveHead.multibeam_sensor[0]), _sensorNameArr.data(), _nameSize);
        mbFileSaveHead.day         = 20220813;     // yyyyMMdd
        mbFileSaveHead.rxMountTilt = 0;            // 换能器倾角(-30)
        mbFileSaveHead.start_time  = 103000.776;   // "hhmmss.zzz";
        mbFileSaveHead.head_num    = 1;            // 换能器数量
        mbFileSaveHead.sector_num  = 1;            // 换能器扇区数目
        mbFileSaveHead.swath_num   = 1;            // 缺失多波束换能器条带数目
        mbFileSaveHead.detaGPS_X   = 0;
        mbFileSaveHead.detaGPS_Y   = 0;
        mbFileSaveHead.detaGPS_Z   = 0;
        for (int i = 0; i < mbFileSaveHead.head_num; i++)
        {
            TRANSDUCERALIG transdata;
            transdata.head_count = 1 + i;
            transdata.angle_X    = 0;
            transdata.angle_Y    = 0;
            transdata.angle_Z    = 0;
            transdata.deta_X     = 0;
            transdata.deta_Y     = 0;
            transdata.deta_Z     = 0;
            transdata.draught    = 0;
            mbFileSaveHead.mbtrans.append(transdata);
        }
        sonic_bathy_object.TransMBHeadStructToQbyte(mbFileHeadarray, mbFileSaveHead);
        xsmb_file.write(mbFileHeadarray);

        for (; r7000_index < size || r7027_index < size;)
        {
            uint32_t r7000_ping_num = r7000s[r7000_index].mRTH.mPingNumber;
            uint32_t r7027_ping_num = r7027s[r7027_index].mRTH.mPingNumber;
            if (r7000_ping_num < r7027_ping_num)
            {
                ++r7000_index;
                continue;
            }
            else if (r7000_ping_num > r7027_ping_num)
            {
                ++r7027_index;
            }
            else
            {
                MBBATHYDATA sonic_bathy_struct;
                QByteArray  sonic_bathy_qbyte;
                PINGINFO    info = ping_merge(r7000s[r7000_index], r7027s[r7027_index]);
                sonic_bathy_object.parse7kDataToMBStruct(info, sonic_bathy_struct);
                sonic_bathy_object.TransMBStructToQbyte(sonic_bathy_qbyte, sonic_bathy_struct);
                if (xsmb_file.write(sonic_bathy_qbyte) == -1)
                {
                    qDebug() << "write xsmb item error!";
                }
                ++r7000_index;
                ++r7027_index;
            }
        }
        */
        saveTxtInfo(s7k_file_str, r7027s, r7000s);
    }
    qDebug() << "END!";

    return 0;
}

void log_init()
{
    QString log_name = "logs/" + QCoreApplication::applicationName() + QDateTime::currentDateTime().toString("_yyyyMMdd_HHmmss") + ".log";
    spdlog::flush_every(std::chrono::seconds(1));

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_name.toStdString(), true);
    file_sink->set_level(spdlog::level::trace);
    // 时间，级别，线程，源码位置，信息
    file_sink->set_pattern("[%H:%M:%S.%F] [%n] [%=8l] [%=8t] [%s:%#] %v");

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    // 时间，级别，线程，信息
    console_sink->set_pattern("[%Y/%m/%d %H:%M:%S.%f] [%n] %^[%=8l]%$ [%=8t] [%s:%#] %v");

    spdlog::sinks_init_list         sinks {file_sink, console_sink};
    std::shared_ptr<spdlog::logger> logger = std::make_shared<spdlog::logger>("s7k", sinks);
    spdlog::set_default_logger(logger);
    spdlog::set_level(spdlog::level::trace);
}

void saveTxtInfo(const QString &s7k_file_str, const QVector<R7027> &r7027s, const QVector<R7000> &r7000s)
{
    QFileInfo s7k_file_info(s7k_file_str);
    QFile     mb_file(s7k_file_info.absolutePath() + "\\" + s7k_file_info.baseName() + ".txt");
    if (!mb_file.open(QIODevice::WriteOnly))
    {
        qDebug() << "can`t open mb file";
        return;
    }
    qDebug() << "save :" << mb_file.fileName();
    QTextStream write_stream(&mb_file);
    write_stream.setFieldAlignment(QTextStream::AlignRight);
    write_stream.setRealNumberPrecision(10);
    write_stream.setRealNumberNotation(QTextStream::FixedNotation);

    //    QVector<R7000> r7000s;
    QVector<R7004> r7004s;
    //    QVector<R7027> r7027s;

    // qDebug() << "record type ids:" << rt_id;
    /*
    for (int j = 0; j < r7000s.size(); ++j)
    {
        write_stream << qSetFieldWidth(16)
                     << QString("R7000 [%1] ").arg(j)
                     << " ping number: " << r7000s[j].mRTH.mPingNumber
                     << " freq: " << r7000s[j].mRTH.mFrequency
                     << " sample rate: " << r7000s[j].mRTH.mSampleRate
                     << " sound velocity: " << r7000s[j].mRTH.mSoundVelocity
                     << qSetFieldWidth(0)
                     << endl;
    }
    */

    for (int j = 0; j < r7004s.size(); ++j)
    {
        write_stream << qSetFieldWidth(16)
                     << QString("R7004 [%1] ").arg(j)
                     << " number: " << r7004s[j].mRTH.mNumOfBeams
                     << qSetFieldWidth(0)
                     << endl;
        for (int k = 0; k < r7004s[j].mRTH.mNumOfBeams; ++k)
        {
            write_stream << qSetFieldWidth(16)
                         << k
                         << r7004s[j].mBeamVAngles[k] * rad_to_deg
                         << r7004s[j].mBeamHAngles[k] * rad_to_deg
                         << r7004s[j].mBeamWidthYs[k] * rad_to_deg
                         << r7004s[j].mBeamWidthXs[k] * rad_to_deg
                         << qSetFieldWidth(0)
                         << endl;
        }
    }

    for (int j = 0; j < r7027s.size(); ++j)
    {
        write_stream << qSetFieldWidth(16)
                     << QString("R7027 [%1] ").arg(j)
                     << " sonar id: " << r7027s[j].mRTH.mSerialId
                     << " ping number: " << r7027s[j].mRTH.mPingNumber
                     << " num of detections: " << r7027s[j].mRTH.mDetectionNums
                     << " sampling rate: " << r7027s[j].mRTH.mSamplingRate
                     << " Tx angle: " << r7027s[j].mRTH.mTxAngle
                     << qSetFieldWidth(0)
                     << endl;
        for (int k = 0; k < r7027s[j].mRDs.size(); ++k)
        {
            float two_way_time = r7027s[j].mRDs[k].mDetectionPoint / r7027s[j].mRTH.mSamplingRate;
            float range        = two_way_time * r7000s[0].mRTH.mSoundVelocity / 2;   // 查找声速
            write_stream << qSetFieldWidth(16)
                         << k
                         << r7027s[j].mRDs[k].mBeamDescriptor
                         << r7027s[j].mRDs[k].mDetectionPoint
                         << r7027s[j].mRDs[k].mRxAngle * rad_to_deg
                         << r7027s[j].mRDs[k].mFlags
                         << r7027s[j].mRDs[k].mQuality
                         << r7027s[j].mRDs[k].mUncertainty
                         << two_way_time
                         << range
                         << qSetFieldWidth(0)
                         << endl;
        }
    }
}
