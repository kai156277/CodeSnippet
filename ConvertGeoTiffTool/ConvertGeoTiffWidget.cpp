#include <QtCore>
#include <QtGui>

#include "ConvertGeoTiffWidget.h"
#include "ConvertParamDialog.h"
#include "ui_ConvertGeoTiffWidget.h"

#include <proj.h>
#include <geotiff.h>
#include <geotiffio.h>
#include <geo_normalize.h>
#include <tiff.h>
#include <xtiffio.h>
#include <geovalues.h>
#include <geonames.h>
#include <geokeys.h>

#define LogInfo_Function(msg)    addLogMessage((ConvertGeoTiffWidget::Info), (msg))
#define LogWarn_Function(msg)    addLogMessage((ConvertGeoTiffWidget::Warn), (msg))
#define LogError_Function(msg)    addLogMessage((ConvertGeoTiffWidget::Error), (msg))

static const double KB = 1024.0;
static const double MB = 1048576.0;
static const double GB = 1073741824.0;

class OpenGTiff
{
public:
    OpenGTiff(const QString& file)
    {
        tif = XTIFFOpen(file.toLocal8Bit().data(), "r");
        if(tif != nullptr)
            gtif = GTIFNew(tif);
    }
    ~OpenGTiff()
    {
        if(tif && gtif)
        {
            GTIFFree(gtif);
            XTIFFClose(tif);
        }
        else if(tif && !gtif)
            XTIFFClose(tif);
    }
    bool isOpen()
    {
        return tif && gtif;
    }

    TIFF* tif = nullptr;
    GTIF* gtif = nullptr;
};

inline QString tagID(int dec)
{
    return QObject::tr("Tag ID: 0x%1(hex) / %2(dec)").arg(dec, 4, 16, QChar('0')).arg(dec);
}

inline QTreeWidgetItem* createItem(const QString& property, const QString& value, const QString& tag = "")
{
    return new QTreeWidgetItem(QStringList() << property << value << tag);
}

inline QTreeWidgetItem* createItem(const QString& property, const QString& value, int num, const QString& tag)
{
    return new QTreeWidgetItem(QStringList()
                               << property
                               << QString("%1 (%2)").arg(value).arg(num)
                               << tag);
}

ConvertGeoTiffWidget::ConvertGeoTiffWidget(short epsg, QWidget *parent) :
    QWidget(parent),
    mEPSG(epsg),
    ui(new Ui::ConvertGeoTiffWidget)
{
    ui->setupUi(this);
    ui->mImportFileTreeWidget->setColumnCount(3);
    ui->mImportFileTreeWidget->setHeaderLabels(QStringList() << tr("property") << tr("value") << tr("tag"));
    ui->mImportFileTreeWidget->setAlternatingRowColors(true);

    ui->mExportFileTreeWidget->setColumnCount(3);
    ui->mExportFileTreeWidget->setHeaderLabels(QStringList() << tr("property") << tr("value") << tr("tag"));
    ui->mExportFileTreeWidget->setAlternatingRowColors(true);

    connect(ui->mImportPushButton, SIGNAL(clicked(bool)),
            this, SLOT(slotSelectImportFile()));
    connect(ui->mExportPushButton, SIGNAL(clicked(bool)),
            this, SLOT(slotSelectExportFile()));
    connect(ui->mConvertPushButton, SIGNAL(clicked(bool)),
            this, SLOT(slotConvertTiff()));
    connect(ui->mResetPushButton, SIGNAL(clicked(bool)),
            this, SLOT(slotReset()));
    ui->mLogTableWidget->verticalHeader()->setVisible(false);
    ui->mLogTableWidget->horizontalHeader()->setClickable(false);
    ui->mLogTableWidget->setSelectionMode(QAbstractItemView::NoSelection);
    QFont font;
    font.setBold(true);
    ui->mLogTableWidget->horizontalHeader()->setFont(font);
    QStringList header;
    ui->mLogTableWidget->setColumnCount(3);
    header << tr("time")
           << tr("type")
           << tr("message");
    ui->mLogTableWidget->setHorizontalHeaderLabels(header);
    ui->mLogTableWidget->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
    ui->mLogTableWidget->horizontalHeader()->setStretchLastSection(true);
    ui->mLogTableWidget->horizontalHeader()->setFrameStyle(QFrame::VLine);
    mGDALwarp = new QProcess;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    QString appPath = QCoreApplication::applicationDirPath();
    QString gdaldataPath = appPath + "/gdalwarp/gdal";
    env.insert("GDAL_DATA", gdaldataPath);
    mGDALwarp->setProcessEnvironment(env);
    connect(mGDALwarp, SIGNAL(started()), this, SLOT(slotGDALwarpStart()));
    connect(mGDALwarp, SIGNAL(finished(int)), this, SLOT(slotGDALwarpFinished()));

    mGDALProgressDialog = new QProgressDialog(tr("Convert geotiff"), tr("end convert"), 0, 0, this);
    connect(mGDALProgressDialog, SIGNAL(canceled()), this, SLOT(slotGDALwarpKill()));
    QSplitter* sp = new QSplitter(Qt::Vertical);
    sp->addWidget(ui->mConvertWidget);
    sp->addWidget(ui->mLogGroupBox);
    setLayout(new QVBoxLayout);
    layout()->addWidget(sp);
    setAcceptDrops(true);
    LogInfo_Function(tr("GeoTiff convert start."));
}

ConvertGeoTiffWidget::~ConvertGeoTiffWidget()
{
    delete ui;
}

QString ConvertGeoTiffWidget::LogLevelToString(ConvertGeoTiffWidget::LogLevel level)
{
    static QStringList levels = QStringList() << tr("Info") << tr("Warn") << tr("Error");
    return levels.value(level);
}

void ConvertGeoTiffWidget::dragEnterEvent(QDragEnterEvent *event)
{
    if(event->mimeData()->hasFormat("text/uri-list"))
    {
        QList<QUrl> urls = event->mimeData()->urls();

        if(urls.isEmpty())
            return ;

        QString fileName = urls.first().toLocalFile();
        QString suffix = QFileInfo(fileName).suffix();
        if(suffix == "tif" || suffix == "tiff" || suffix == "TIF" || suffix == "TIFF")
            event->acceptProposedAction();
    }
}

void ConvertGeoTiffWidget::dropEvent(QDropEvent *event)
{
    QList<QUrl> urls = event->mimeData()->urls();

    if(urls.isEmpty())
        return ;

    QString fileName = urls.first().toLocalFile();
    slotLoadImportTiffFile(fileName);
}

void ConvertGeoTiffWidget::slotSelectImportFile()
{
    QSettings config;
    if(!config.contains("importTiff"))
        config.setValue("importTiff", QDir::homePath());

    QString dir = config.value("importTiff").toString();
    QString importfile = QFileDialog::getOpenFileName(this, tr("Import geotiff file"),
                                                      dir, tr("Geotif(*.tif *.TIF *.tiff *.TIFF)"));
    if(!importfile.isNull())
    {
        config.setValue("importTiff", QFileInfo(importfile).absolutePath());
        slotLoadImportTiffFile(importfile);
    }
}

void ConvertGeoTiffWidget::slotSelectExportFile()
{
    QSettings config;
    if(!config.contains("exportTiff"))
        config.setValue("exportTiff", QDir::homePath());

    QString dir = config.value("exportTiff").toString();
    QString exportTiff = QFileDialog::getSaveFileName(this, tr("Import geotiff file"),
                                                      dir, tr("Geotif(*.tif *.TIF *.tiff *.TIFF)"));
    if(!exportTiff.isNull())
    {
        config.setValue("exportTiff", QFileInfo(exportTiff).absolutePath());
        clearExport();
        ui->mExportFileLineEdit->setText(exportTiff);
        LogInfo_Function(tr("Add Export GeoTiff file."));
    }
}

void ConvertGeoTiffWidget::slotConvertTiff()
{
    if(ui->mImportFileLineEdit->text().isEmpty())
    {
        LogError_Function(tr("Not set import GTiff file."));
        return ;
    }

    if(ui->mExportFileLineEdit->text().isEmpty())
    {
        LogError_Function(tr("Not set export GTiff file."));
        return ;
    }

    ConvertParamDialog dialog(mImportLng);
    if(dialog.exec() == QDialog::Accepted)
    {
        QString appPath = QCoreApplication::applicationDirPath();
        QString program = appPath + "/gdalwarp/gdalwarp.exe";
        QStringList arguments;
        QString src_file = ui->mImportFileLineEdit->text();
        QString dst_file = ui->mExportFileLineEdit->text();
        arguments << "-overwrite" // 如果目标文件存在则覆盖
    //              << "-s_srs" << "EPSG:4326" // 源文件坐标系统
                  << "-t_srs" << QString("EPSG:%1").arg(dialog.epsg) // 目标文件坐标系统
                  << "-of" << "GTiff" // 指定输出目标图像格式
                  << "-multi" // 多线程
                  << "-dstalpha" // 指定透明通道，对于无数据值的像素设为透明
                  << src_file // 源文件
                  << dst_file; // 目标文件

        mGDALwarp->start(program, arguments);
        LogInfo_Function(tr("Convert %1 => %2.").arg(src_file).arg(dst_file));
    }

}

void ConvertGeoTiffWidget::slotReset()
{
    clearImport();
    clearExport();
    LogInfo_Function(tr("Reset Convert."));
}

void ConvertGeoTiffWidget::slotGDALwarpStart()
{
    mGDALProgressDialog->show();
    LogInfo_Function(tr("Convert Start."));
}

void ConvertGeoTiffWidget::slotGDALwarpKill()
{
    mGDALwarp->kill();
    LogWarn_Function(tr("Convert Abort."));
}

void ConvertGeoTiffWidget::slotGDALwarpFinished()
{
    mGDALProgressDialog->hide();
    if(mGDALwarp->exitStatus() == QProcess::NormalExit)
    {
        LogInfo_Function(tr("Convert Finish."));
        QCoreApplication::processEvents();//处理等待事件
        QString exportfile = ui->mExportFileLineEdit->text();
        clearExport();
        ui->mExportFileLineEdit->setText(exportfile);
        slotLoadExportTiffFile(exportfile);
    }
}

void ConvertGeoTiffWidget::clearImport()
{
    QList<QTreeWidgetItem*> list;
    for(int i = 0; i < ui->mImportFileTreeWidget->topLevelItemCount(); ++i)
        list.push_back(ui->mImportFileTreeWidget->takeTopLevelItem(i));
    ui->mImportFileTreeWidget->clear();
    ui->mImportTiffView->setPixmap(QPixmap());
    qDeleteAll(list);
    ui->mImportFileLineEdit->setText("");
    ui->mImportPushButton->setEnabled(true);
}

void ConvertGeoTiffWidget::clearExport()
{
    QList<QTreeWidgetItem*> list;
    for(int i = 0; i < ui->mExportFileTreeWidget->topLevelItemCount(); ++i)
        list.push_back(ui->mExportFileTreeWidget->takeTopLevelItem(i));

    ui->mExportFileTreeWidget->clear();
    ui->mExportTiffView->setPixmap(QPixmap());
    qDeleteAll(list);
    ui->mExportFileLineEdit->setText("");
    ui->mExportPushButton->setEnabled(true);
}

void ConvertGeoTiffWidget::slotLoadImportTiffFile(const QString &importfile)
{
    LogInfo_Function(tr("Load import tiff file: %1.").arg(importfile));
    clearImport();
    ui->mImportFileLineEdit->setText(importfile);
    QPixmap tiff(importfile);
    double ratio = (double)tiff.width() / (double)tiff.height();
    tiff = tiff.scaled(ratio * 400, 400);
    ui->mImportTiffView->setPixmap(tiff);
    ui->mImportFileTreeWidget->addTopLevelItems(getTiffInfo(importfile));

    if(mGTIFDefn.Model == 2)
        mImportLng = mTiepoints.at(3);
    else if(mGTIFDefn.Model == 1)
    {
        for(int i = 0; i < mGTIFDefn.nParms; ++i)
            if(mGTIFDefn.ProjParmId[i] == ProjNatOriginLongGeoKey)
                mImportLng = mGTIFDefn.ProjParm[i];
    }
    else
        mImportLng = 120;

    QFileInfo importFileInfo(importfile);
    QString exportFile;
    int i = 0;
    bool flag = true;
    while(flag)
    {
        exportFile = importFileInfo.absolutePath() + "/"
                + importFileInfo.baseName() + QString("_%1").arg(i) + "."
                + importFileInfo.suffix();
        flag = QFile::exists(exportFile);
        ++i;
    }
    ui->mExportFileLineEdit->setText(exportFile);
}

void ConvertGeoTiffWidget::slotLoadExportTiffFile(const QString &file)
{
    LogInfo_Function(tr("Load export tiff file: %1.").arg(file));
    QCoreApplication::processEvents();//处理等待事件
    QPixmap tiff(file);
    double ratio = (double)tiff.width() / (double)tiff.height();
    tiff = tiff.scaled(ratio * 400, 400);
    ui->mExportTiffView->setPixmap(tiff);
    ui->mExportFileTreeWidget->addTopLevelItems(getTiffInfo(file));
}

QList<QTreeWidgetItem*> ConvertGeoTiffWidget::getTiffInfo(const QString& file)
{
    QList<QTreeWidgetItem*> children;
    OpenGTiff _gtiff(file);
    QFileInfo gtiffInfo(file);

    if(!_gtiff.isOpen())
    {
        LogError_Function(QObject::tr("Read geotif data failed"));
        return children;
    }

    // file name
    {
        children.push_back(createItem(tr("Filename"), gtiffInfo.absoluteFilePath()));
    }

    // file last modified time
    {
        children.push_back(createItem(tr("FileModDate"), gtiffInfo.lastModified().toString("yyyy-MM-dd hh:mm:ss")));
    }

    // data size
    {
        QTreeWidgetItem* item = nullptr;
        double size = gtiffInfo.size();
        if(0 < size && size <= KB)
            item = createItem(tr("FileSize"), QString::number(size), "bytes");
        else if(KB < size && size <= MB)
            item = createItem(tr("FileSize"), QString::number(size / KB), "KB");

        else if(MB < size && size <= GB)
            item = createItem(tr("FileSize"), QString::number(size / MB), "MB");
        else
            item = createItem(tr("FileSize"), QString::number(size / GB), "GB");

        children.push_back(item);
    }

    // image width
    {
        const TIFFField * field = TIFFFieldWithTag(_gtiff.tif, TIFFTAG_IMAGEWIDTH);
        if(field != nullptr)
        {
            uint32 value = 0;
            if(TIFFGetField(_gtiff.tif, TIFFTAG_IMAGEWIDTH, &value) == 1)
            {
                QTreeWidgetItem* item = createItem(tr("ImageWidth"), QString::number(value), TIFFFieldName(field));
                item->setToolTip(0, tr("The number of columns of pixels in the image."));
                item->setToolTip(2, tagID(TIFFTAG_IMAGEWIDTH));
                children.push_back(item);
            }
        }
    }

    // image length
    {
        const TIFFField * field = TIFFFieldWithTag(_gtiff.tif, TIFFTAG_IMAGELENGTH);
        if(field != nullptr)
        {
            uint32 value = 0;
            if(TIFFGetField(_gtiff.tif, TIFFTAG_IMAGELENGTH, &value) == 1)
            {
                QTreeWidgetItem* item = createItem(tr("ImageLength"), QString::number(value), TIFFFieldName(field));
                item->setToolTip(0, tr("The number of rows of pixels in the image."));
                item->setToolTip(2, tagID(TIFFTAG_IMAGELENGTH));
                children.push_back(item);
            }
        }

    }


    // get gtiffDefn

    GTIFDefn defn;
    if(GTIFGetDefn(_gtiff.gtif, &defn) == 1)
        children.append(getGeoTiffInfo(defn));
    else
        LogError_Function(QObject::tr("Geotif coordinate system failed"));

    mGTIFDefn = defn;

    QVector<double> tiepoints;
    // 获取Tiff像素点和地理坐标点的对应关系
    {
        double* raw_data = nullptr;
        uint32 value_count = 0;
        const TIFFField* tpfip = TIFFFieldWithTag(_gtiff.tif, 33922);
        if(tpfip != nullptr)
        {
            TIFFGetField(_gtiff.tif, 33922, &value_count, &raw_data);
            for(uint i = 0; i < value_count; i++)
                tiepoints.push_back((raw_data)[i]);

            if(tiepoints.size() >= 6)
            {
                QString latlng = QString::number(tiepoints.at(3), 'f', 10) +
                        "/" + QString::number(tiepoints.at(4), 'f', 10);
                QTreeWidgetItem* item = createItem(tr("Lng/Lat(E/N)"), latlng);
                children.push_back(item);
            }
        }
    }
    mTiepoints = tiepoints;


    return children;
}

QList<QTreeWidgetItem *> ConvertGeoTiffWidget::getGeoTiffInfo(GTIFDefn defn)
{
    static QStringList modeltype = {tr(""),
                                    tr("Projection Coordinate System"),
                                    tr("Geographic Latitude-Longitude System"),
                                    tr("GeoCentric (X,Y,Z) Coordinate System")};
    QList<QTreeWidgetItem*> children;
    QString projStr = GTIFGetProj4Defn(&defn);
    if(!projStr.isEmpty())
    {
        QTreeWidgetItem* proj4 = createItem(tr("proj4"), projStr);
        proj4->setToolTip(0, tr("PROJ is Generic coordinate transformation software."));
        proj4->setToolTip(1, tr("proj-strings."));
        children.push_back(proj4);
    }

    if(defn.Model != KvUserDefined)
    {
        QTreeWidgetItem* item = createItem(tr("Model"),
                                           modeltype.value(defn.Model),
                                           defn.Model, "GTModelTypeGeoKey");
        item->setToolTip(0, tr("Type of coordinate system used to georeference."));
        item->setToolTip(1, tr("Can have the values ModelTypeProjected(1), "
                                "ModelTypeGeographic(2) or ModelTypeGeocentric(3)"));
        item->setToolTip(2, tagID(GTModelTypeGeoKey));
        children.push_back(item);
    }

    if(defn.PCS != KvUserDefined)
    {
        char* epsgName;
        // projOp, UOMLengthCode, GeogCS same as defn.ProjCode, UOMLength, GCS

        if(GTIFGetPCSInfo(defn.PCS, &epsgName, nullptr, nullptr, nullptr) == 1)
        {
            QTreeWidgetItem *item = createItem(tr("Projection CS"),
                                               epsgName, defn.PCS,
                                               "ProjectedCSTypeGeoKey");
            item->setToolTip(0, tr("Projected Coordinate System."));
            item->setToolTip(1, tr("EPSG Projection Name and Serial Number, you can find code in epsg_pcs.inc file."));
            item->setToolTip(2, tagID(ProjectedCSTypeGeoKey));
            children.push_back(item);
        }
        else
            LogError_Function(tr("Can`t get Projection Coordinate System info. PCS code: %1").arg(defn.PCS));
    }

    if(defn.GCS != KvUserDefined)
    {
        char* epsgName;
        // datum, pm, UOMAngle same as defn.Datum, defn.PM, defn.UOMAngle
        if(GTIFGetGCSInfo(defn.GCS, &epsgName, nullptr, nullptr, nullptr) == 1)
        {
            QTreeWidgetItem* item = createItem(tr("Geographic CS"),
                                               epsgName, defn.GCS,
                                               "GeographicTypeGeoKey");
            item->setToolTip(0, tr("Geographic Coordinate System."));
            item->setToolTip(1, tr("EPSG Geographics Name and Serial Number, you can find code in epsg_gcs.inc file."));
            item->setToolTip(2, tagID(GeographicTypeGeoKey));
            children.push_back(item);
        }
        else
            LogError_Function(tr("Can`t get Geographic Coordinate System info. GCS code: %1").arg(defn.GCS));
    }

    // Units

    if(defn.UOMLength != KvUserDefined)
    {
        char* uomName;
        // metters same as defn.UOMLengthInMeters
        if(GTIFGetUOMLengthInfo(defn.UOMLength, &uomName, nullptr) == 1)
        {
            QTreeWidgetItem* item = createItem(tr("Length"),
                                               uomName, defn.UOMLength,
                                               "ProjLinearUnitsGeoKey");
            item->setToolTip(0, tr("The length(UOM) used in the projected coordinate system."));
            item->setToolTip(1, tr("Unit of Measurement, you can find code in epsg_units.inc file."));
            item->setToolTip(2, tagID(ProjLinearUnitsGeoKey));
            children.push_back(item);
        }
        else
            LogError_Function(tr("Can`t get length(UOM) info. UOMLength code: %1").arg(defn.UOMLength));
    }

    if(defn.UOMLengthInMeters != KvUserDefined)
    {
        QTreeWidgetItem* item = createItem(tr("Length in meters"),
                                           QString("1/%1m").arg(defn.UOMLengthInMeters),
                                           "ProjLinearUnitSizeGeoKey");

        item->setToolTip(0, tr("The length(UOM) used in the projected coordinate system."));
        item->setToolTip(1, tr("The length(UOM) in meters, you can find code in epsg_units.inc file."));
        item->setToolTip(2, tagID(ProjLinearUnitSizeGeoKey));
        children.push_back(item);
    }

    if(defn.UOMAngle != KvUserDefined)
    {
        char* uomName;
        // degrees same as defn.UOMAngleInDegrees
        if(GTIFGetUOMAngleInfo(defn.UOMAngle, &uomName, nullptr) == 1)
        {
            QTreeWidgetItem* item = createItem(tr("Angle"),
                                               uomName, defn.UOMAngle,
                                               "GeogAngularUnitsGeoKey");
            item->setToolTip(0, tr("The Angle(UOM) used in the geographic coordinate system."));
            item->setToolTip(1, tr("Unit of Measurement, you can find code in epsg_units.inc file."));
            item->setToolTip(2, tagID(GeogAngularUnitsGeoKey));
            children.push_back(item);
        }
        else
            LogError_Function(tr("Can`t get Angle(UOM) info. UOMAngle code: %1").arg(defn.UOMAngle));
    }

    if(defn.UOMAngleInDegrees != KvUserDefined)
    {
        QTreeWidgetItem* item = createItem(tr("Angle in degrees"),
                                           QString("1/%1deg").arg(defn.UOMAngleInDegrees),
                                           "GeogAngularUnitSizeGeoKey");
        item->setToolTip(0, tr("The Angle(UOM) used in the geographic coordinate system."));
        item->setToolTip(1, tr("The Angle(UOM) in Degrees, you can find code in epsg_units.inc file."));
        item->setToolTip(2, tagID(GeogAngularUnitSizeGeoKey));
        children.push_back(item);
    }

    //Datum
    if(defn.Datum != KvUserDefined)
    {
        char* datumName;
        // ellipsod same as ellipsoid
        if(GTIFGetDatumInfo(defn.Datum, &datumName, nullptr) == 1)
        {
            QTreeWidgetItem* item = createItem(tr("Datum"),
                                               datumName, defn.Datum,
                                               "GeogGeodeticDatumGeoKey");
            item->setToolTip(0, tr("Projection datum type."));
            item->setToolTip(1, tr("Datum describe, you can find code in epsg_datum.inc file."));
            item->setToolTip(2, tagID(GeogGeodeticDatumGeoKey));
            children.push_back(item);
        }
        else
            LogError_Function(tr("Can`t get Datum info. Datum code: %1").arg(defn.Datum));
    }

    if(defn.Ellipsoid != KvUserDefined)
    {
        char* ellipsodName;
        // semiMajor, semiMinor same as defn.semiMajor, semiMinor
        if(GTIFGetEllipsoidInfo(defn.Ellipsoid, &ellipsodName, nullptr, nullptr) == 1)
        {
            QTreeWidgetItem* item = createItem(tr("Ellipsoid"),
                                               ellipsodName, defn.Ellipsoid,
                                               "GeogEllipsoidGeoKey");
            item->setToolTip(0, tr("Datum use ellipsoid"));
            item->setToolTip(1, tr("Ellipsod describe, you can find code in epsg_ellipse.inc file."));
            item->setToolTip(2, tagID(GeogEllipsoidGeoKey));

            QTreeWidgetItem* major = createItem(tr("Semi-major axis"),
                                                QString::number(defn.SemiMajor, 'f', 16),
                                                "GeogSemiMajorAxisGeoKey");
            major->setToolTip(0, tr("Ellipsod semi-major axis"));
            major->setToolTip(1, tr("Unit of Measurement is GeoLinearUnits"));
            major->setToolTip(2, tagID(GeogSemiMajorAxisGeoKey));
            item->addChild(major);

            QTreeWidgetItem* minor = createItem(tr("Semi-minor axis"),
                                                QString::number(defn.SemiMinor, 'f', 16),
                                                "GeogSemiMinorAxisGeoKey");
            minor->setToolTip(0, tr("Ellipsod semi-minor axis"));
            minor->setToolTip(1, tr("Unit of Measurement is GeoLinearUnits"));
            minor->setToolTip(2, tagID(GeogSemiMinorAxisGeoKey));
            item->addChild(minor);

            children.push_back(item);
        }
        else
            LogError_Function(tr("Can`t get Ellipsoid info. Ellipsoid code: %1").arg(defn.Ellipsoid));
    }

    if(defn.PM != KvUserDefined)
    {
        char* name;
//        double lengthtoGreenwich same as defn.PMLongToGreenwich
        if(GTIFGetPMInfo(defn.PM, &name, nullptr) == 1)
        {
            QTreeWidgetItem* item = createItem(tr("PM"),
                                               name, defn.PM,
                                               "GeogPrimeMeridianGeoKey");
            item->setToolTip(0, tr("Prime meridian."));
            item->setToolTip(1, tr("Prime meridian, you can find code in epsg_pm.inc file."));
            item->setToolTip(2, tagID(GeogPrimeMeridianGeoKey));
            QTreeWidgetItem* greenwich = createItem(tr("Long to Greenwich"),
                                                    QString("%1(deg)").arg(defn.PMLongToGreenwich),
                                                    "");
            greenwich->setToolTip(0, tr("Indicating the decimal degrees of longitude between "
                                        "this prime meridian and Greenwich. Prime meridians "
                                        "to the west of Greenwich are negative"));
            item->addChild(greenwich);
            children.push_back(item);
        }
        else
            LogError_Function(tr("Can`t get Prime Meridian info. PM code: %1").arg(defn.PM));
    }

    if(defn.TOWGS84Count != 0)
    {
        QTreeWidgetItem* item = createItem(tr("To WGS84 param"),
                                           "", "GeogTOWGS84GeoKey");
        QStringList param = QStringList() << "dx(m)" << "dy(m)" << "dz(m)"
                                          << "rx(\")" << "ry(\")" << "rz(\")"
                                          << "ds";
        for(int i = 0; i < defn.TOWGS84Count; ++i)
        {
            QTreeWidgetItem *paramItem = createItem(param.at(i),
                                                    QString::number(defn.TOWGS84[i], 'f'),
                                                    "");
            item->addChild(paramItem);
        }
        item->setToolTip(0, tr("To wgs84 transformation."));
        item->setToolTip(2, tagID(GeogTOWGS84GeoKey));
        children.push_back(item);
    }

    if(defn.ProjCode != KvUserDefined)
    {
        char* name;
        short projMethod;
        // projMethod same as defn.Projection
        if(GTIFGetProjTRFInfo(defn.ProjCode, &name, &projMethod, nullptr) == 1)
        {
            QTreeWidgetItem* item = createItem(tr("Projection Code"),
                                                 name, defn.ProjCode,
                                                 "ProjectionGeoKey");
            item->setToolTip(0, tr("Projection Coordinate System Code."));
            item->setToolTip(1, tr("Project Name and Projection Code, you can find code in epsg_proj.inc file."));
            item->setToolTip(2, tagID(ProjectionGeoKey));
            children.push_back(item);
        }
        else
            LogError_Function(tr("This tiff is Projection CS, but can`t analysis projection name!"
                                 "projection code: %1. You can check epsg_proj.inc file.").arg(defn.ProjCode));
    }

    // Projection need gdal

    if(defn.CTProjection != KvUserDefined)
    {
        for(int i = 0; ; ++i)
        {
            if(_coordtransValue[i].ki_key == defn.CTProjection)
            {
                QTreeWidgetItem* item = createItem(tr("CTProjection"),
                                                     _coordtransValue[i].ki_name, defn.CTProjection,
                                                     "");
                item->setToolTip(0, tr("GeoTIFF identifier for underlying projection."));

                children.push_back(item);
                break;
            }
            if(_coordtransValue[i].ki_key == -1)
                break;
        }
    }

    if(defn.nParms != 0)
    {
        QTreeWidgetItem* item = new QTreeWidgetItem(QStringList() << tr("Project paramter"));
        for(int i = 0; i < defn.nParms; ++i)
        {
            if(defn.ProjParmId[i] != 0)
            {
                QString name(GTIFKeyName((geokey_t)defn.ProjParmId[i]));
                QTreeWidgetItem* parmItem = createItem("", QString::number(defn.ProjParm[i], 'f'), name);
                parmItem->setToolTip(2, tagID(defn.ProjParmId[i]));
                item->addChild(parmItem);
            }
        }
        item->setToolTip(0, tr("Projection parameter value.  The identify of this parameter "
                               "is established from the corresponding entry in ProjParmId. "
                               "The value will be measured in meters, "
                               "or decimal degrees if it is a linear or angular measure."));
        children.push_back(item);
    }

    if(defn.MapSys != KvUserDefined)
    {
        static QStringList mapSysNames = {tr("UTM_North"), tr("UTM_South"),
                                          tr("State_Plane_27"), tr("State_Plane_83")};
        QString name;
        switch (defn.MapSys) {
        case MapSys_UTM_North:
            name = mapSysNames[0];
            break;
        case MapSys_UTM_South:
            name = mapSysNames[1];
            break;
        case MapSys_State_Plane_27:
            name = mapSysNames[2];
            break;
        case MapSys_State_Plane_83:
            name = mapSysNames[3];
            break;
        default:
            break;
        }
        QTreeWidgetItem* item = createItem(tr("MapSys"), name, defn.MapSys, "");
        item->setToolTip(0, tr("Special zone map system code."));
        item->setToolTip(1, mapSysNames.join(","));
        children.push_back(item);
    }

    if(defn.Zone != 0 && defn.Zone != KvUserDefined)
    {
        QTreeWidgetItem* item = createItem(tr("Zone"), QString::number(defn.Zone), "");
        item->setToolTip(0, tr("UTM, or State Plane Zone number"));
        children.push_back(item);
    }

    return children;
}

void ConvertGeoTiffWidget::addLogMessage(ConvertGeoTiffWidget::LogLevel level, const QString &msg)
{
    int cout = ui->mLogTableWidget->rowCount();
    ui->mLogTableWidget->setRowCount(cout+1);
    ui->mLogTableWidget->setItem(cout, 0, new QTableWidgetItem(
                                     QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")));
    ui->mLogTableWidget->setItem(cout, 1, new QTableWidgetItem(LogLevelToString(level)));
    QTableWidgetItem* _item = ui->mLogTableWidget->item(cout, 1);
    if(level == LogLevel::Info)
        _item->setBackground(Qt::green);
    else if(level == LogLevel::Warn)
        _item->setBackground(Qt::yellow);
    else if(level == LogLevel::Error)
        _item->setBackground(Qt::red);
    else
        _item->setBackground(Qt::gray);
    _item->setTextAlignment(Qt::AlignCenter);
    ui->mLogTableWidget->setItem(cout, 2, new QTableWidgetItem(msg));
    ui->mLogTableWidget->scrollToBottom();
}

