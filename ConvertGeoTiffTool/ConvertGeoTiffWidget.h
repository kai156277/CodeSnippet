#pragma once // CONVERTGEOTIFFWIDGET_H

#include <tiffio.h>
#include <geo_normalize.h>
#include <QVector>
#include <QWidget>

namespace Ui {
class ConvertGeoTiffWidget;
}


class QTreeWidgetItem;
class QProcess;
class QProgressDialog;

class ConvertGeoTiffWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ConvertGeoTiffWidget(short epsg = 0, QWidget *parent = 0);
    ~ConvertGeoTiffWidget();

    enum LogLevel {
        Info = 0,
        Warn = 1,
        Error = 2
    };
    QString LogLevelToString(LogLevel level);
protected:
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;

private slots:
    void slotSelectImportFile();
    void slotSelectExportFile();
    void slotConvertTiff();
    void slotReset();
    void slotGDALwarpStart();
    void slotGDALwarpKill();
    void slotGDALwarpFinished();
    void slotLoadImportTiffFile(const QString& importfile);
    void slotLoadExportTiffFile(const QString& file);
private:
    void clearImport();
    void clearExport();
    QList<QTreeWidgetItem *> getTiffInfo(const QString &file);
    QList<QTreeWidgetItem *> getGeoTiffInfo(GTIFDefn defn);
    void addLogMessage(LogLevel level, const QString& msg);
    Ui::ConvertGeoTiffWidget *ui;
    int mEPSG;
    double mImportLng = 120;
    GTIFDefn mGTIFDefn;
    QVector<double> mTiepoints;
    QProcess* mGDALwarp;
    QProgressDialog* mGDALProgressDialog;
};
