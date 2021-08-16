#ifndef ANALYSISRXP_H
#define ANALYSISRXP_H

#include <QMainWindow>
#include <QFileDialog>
#include <analysis.h>
#include <QThread>

namespace Ui {
class ParseRieglStream;
}

class ParseRieglStream : public QMainWindow
{
    Q_OBJECT

public:
    explicit ParseRieglStream(QWidget *parent = 0);
    ~ParseRieglStream();

    QThread     mParseThread;
    QStringList mRxpFilePaths;

private slots:
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();

private:
    Ui::ParseRieglStream *ui;

signals:
    void signalDirPath(QString list);
    void signalStartParse(QStringList list);
};

#endif // ANALYSISRXP_H
