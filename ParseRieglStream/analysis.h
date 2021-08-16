#ifndef ANALYSIS_H
#define ANALYSIS_H
#include <QObject>
#include <QString>
#include <QStringList>
#include <QFile>
#include <QDateTime>
#include <QDebug>

#include <thread>
#include <memory>
#include "riegl/connection.hpp"
#include "riegl/fileconn.hpp"
#include "riegl/ctrllib.hpp"
#include "riegl/ctrlifc.h"
#include "riegl/scanlib.hpp"
#include "riegl/pointcloud.hpp"
#include "riegl/ridataspec.hpp"

#include "RieglDataImporter.h"

class Analysis : public QObject
{
    Q_OBJECT
public:
    Analysis();
    ~Analysis();
    RieglVZ2000iDataImporter*    mDataImporter = 0;

public slots:
    void slotStartParse(QStringList rxpFilePathList);
    void slotDirPath(QString dirPath);
    void slotStop();
};

#endif // ANALYSIS_H
