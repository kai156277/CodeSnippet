#include "analysis.h"

using namespace scanlib;

Analysis::Analysis()
{
    mDataImporter = new RieglVZ2000iDataImporter();
}

Analysis::~Analysis()
{
    if (mDataImporter)
    {
        delete mDataImporter;
        mDataImporter = 0;
    }
}

void Analysis::slotStartParse(QStringList rxpFilePathList)
{
    for (int i = 0; i < rxpFilePathList.size(); i++)
    {
        qDebug() << "filename:" << rxpFilePathList.at(i);
        std::shared_ptr<basic_rconnection> rc;
        QString                            fn = rxpFilePathList.at(i);
        rc                                    = basic_rconnection::create(fn.toStdString());
        rc->open();
        decoder_rxpmarker dec(rc);
        buffer            buf;
        int               count = 0;
        try
        {
            for (dec.get(buf); !dec.eoi(); dec.get(buf))
            {
                mDataImporter->dispatch(buf.begin(), buf.end());
                if (count % 100000 == 0)
                    qDebug() << "count: " << count;

                ++count;
            }
            rc->close();
            std::cout << "Finished!" << std::endl;
        }
        catch (std::exception &e)
        {
            qDebug() << "pointcloudData Failed!....." << e.what();
        }
        catch (...)
        {
            qDebug() << "unkonw exception ....";
        }
    }
    qDebug() << "end!";
}

void Analysis::slotDirPath(QString dirPath)
{
    QDir dir(dirPath);
    if (!dir.exists())
    {
        dir.mkpath(dirPath);
    }
    mDataImporter->setSaveDirPath(dirPath);
}

void Analysis::slotStop()
{
    delete mDataImporter;
    mDataImporter = 0;
}
