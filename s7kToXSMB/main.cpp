#include <QApplication>
#include <QDataStream>
#include <QDebug>
#include <QFile>
#include <QFileDialog>

#include "7kdef.h"
#include "bathyformat.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QString      xsmb = QFileDialog::getOpenFileName(nullptr, "xsmb", "E:\\Data\\VSurs-W\\20210804\\20210804_175152\\Multibeam");
    if (!xsmb.isEmpty())
    {
        QFile xsmb_file(xsmb);
        xsmb_file.open(QIODevice::ReadOnly);
        BATHYFORMAT bf;
        bf.fileReadMbFile(xsmb_file);
        qDebug() << "END";
    }
    return 0;
    QStringList s7k_file_strs = QFileDialog::getOpenFileNames(nullptr, "open s7k file", "D:\\Data\\7125\\PDS2000-s7k");
    qDebug() << "num of file: " << s7k_file_strs.size();
    for (int i = 0; i < s7k_file_strs.size(); ++i)
    {
        QString s7k_file_str = s7k_file_strs[i];

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
                r7004.mTxDelays        = new float[beams];

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
                memcpy_s((void *) r7004.mTxDelays,
                         rd_shift,
                         (void *) (raw_data + rth_shift + rd_shift * 4),
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

                r7027s.push_back(r7027);
                break;
            }
            default:
                break;
            }

            delete[] raw_data;
        }

        qDebug() << QString("[%1] file: %2").arg(i + 1).arg(s7k_file_str);
        qDebug() << "record type ids:" << rt_id;
        qDebug() << "7000: " << r7000s.size();
        qDebug() << "7004: " << r7004s.size();
        qDebug() << "7027: " << r7027s.size();
    }
    qDebug() << "END!";

    return 0;
}
