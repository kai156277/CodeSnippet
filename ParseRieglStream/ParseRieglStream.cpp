#include "ParseRieglStream.h"
#include "ui_ParseRieglStream.h"

ParseRieglStream::ParseRieglStream(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ParseRieglStream)
{
    ui->setupUi(this);

    Analysis *analysis = new Analysis;
    analysis->moveToThread(&mParseThread);
    connect( &mParseThread , &QThread::finished , analysis , &Analysis::deleteLater );
    connect( &mParseThread , &QThread::finished , &mParseThread, &QThread::deleteLater);

    connect( this , &ParseRieglStream::signalDirPath , analysis , &Analysis::slotDirPath );
    connect( this , &ParseRieglStream::signalStartParse , analysis , &Analysis::slotStartParse );
    mParseThread.start();
}

ParseRieglStream::~ParseRieglStream()
{
    mParseThread.quit();
    mParseThread.wait();

    delete ui;
}

void ParseRieglStream::on_pushButton_clicked()
{
    mRxpFilePaths = QFileDialog::getOpenFileNames(nullptr,NULL,NULL,"*.*");
    ui->lineEdit->setText(mRxpFilePaths.at(0));
}

void ParseRieglStream::on_pushButton_2_clicked()
{
    QString dirPath = QFileDialog::getExistingDirectory(this,"请选择文件保存路径..",NULL).replace("\\" , "/");

    emit signalDirPath(dirPath);
    ui->lineEdit_2->setText(dirPath);
}

void ParseRieglStream::on_pushButton_3_clicked()
{
    emit signalStartParse(mRxpFilePaths);
}
