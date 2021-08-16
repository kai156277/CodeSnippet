#ifndef EXCELHELPER_H
#define EXCELHELPER_H
#include <ActiveQt/QAxObject>
#include <QDir>
#include <QFile>
#include<QStringList>
#include<QVector>
#include<QColor>
#include "Windows.h"
#include<iostream>
using namespace std;
#pragma execution_character_set("utf-8")
class ExcelHelper
{
    public:
        ExcelHelper();
        bool newExcel(const QString &fileName);
        void appendSheet(const QString &sheetName);
        void setCellValue(int row, int column, const QString &value);
        void saveExcel(const QString &fileName);
        void freeExcel();
        void mergeCells(const QString& range);

        void mergeCells(int topLeftRow, int topLeftColumn, int bottomRightRow, int bottomRightColumn);
        void borders(const QString& range);
        void borders(int row,int column);
        void borders(int topLeftRow, int topLeftColumn, int bottomRightRow, int bottomRightColumn);
        bool ExportDataToExcel(const QString &fileName, pcl::PointCloud<pcl::PointXYZ> cloud, int direction, double h);
        QAxObject *pApplication;
        QAxObject *pWorkBooks;
        QAxObject *pWorkBook;
        QAxObject *pSheets;
        QAxObject *pSheet;
};

ExcelHelper::ExcelHelper()
{
    CoInitializeEx(NULL, COINIT_MULTITHREADED);
}

//新建Execl文件  每个创建的指针都需要判断是否为空，不为空才能用，否则调用过程中会出现崩溃现象
// 其他函数也是要相应这样
bool ExcelHelper::newExcel(const QString &fileName)
{


    pApplication = new QAxObject();
    pApplication->setControl("Excel.Application"); //连接Excel控件
    pApplication->dynamicCall("SetVisible(bool)", false); //false不显示窗体
    pApplication->setProperty("DisplayAlerts", false); //不显示任何警告信息。
    pWorkBooks = pApplication->querySubObject("Workbooks");
    QFile file(fileName);
    if(file.exists()){
        pWorkBook = pWorkBooks->querySubObject("Open(const QString &)", fileName);
    }
    else{
        pWorkBooks->dynamicCall("Add");
        pWorkBook = pApplication->querySubObject("ActiveWorkBook");
    }
    //默认有一个sheet
    pSheets = pWorkBook->querySubObject("Sheets");
    pSheet = pSheets->querySubObject("Item(int)", 1);
    return true;

}

//增加1个Worksheet，这里要注意：默认有一个sheet，需要的时候再调用这个函数
void ExcelHelper::appendSheet(const QString &sheetName)
{
    int cnt = 1;
    QAxObject *pLastSheet = pSheets->querySubObject("Item(int)", cnt);
    pSheets->querySubObject("Add(QVariant)", pLastSheet->asVariant());
    pSheet = pSheets->querySubObject("Item(int)", cnt);
    pLastSheet->dynamicCall("Move(QVariant)", pSheet->asVariant());
    pSheet->setProperty("Name", sheetName);
}

//向Excel单元格中写入数据
//PS:这里涉及到 对于数据格式的设置
void ExcelHelper::setCellValue(int row, int column, const QString &value)
{
    QAxObject *pRange = pSheet->querySubObject("Cells(int,int)", row, column);
    pRange->dynamicCall("Value", value);
    //内容居中
    pRange->setProperty("HorizontalAlignment", -4108);
    pRange->setProperty("VerticalAlignment", -4108);
       // pRange->setProperty("RowHeight", 50); //设置单元格行高
       // pRange->setProperty("ColumnWidth", 30); //设置单元格列宽
       // pRange->setProperty("HorizontalAlignment", -4108); //左对齐（xlLeft）：-4131 居中（xlCenter）：-4108 右对齐（xlRight）：-4152
       // pRange->setProperty("VerticalAlignment", -4108); //上对齐（xlTop）-4160 居中（xlCenter）：-4108 下对齐（xlBottom）：-4107
       // pRange->setProperty("WrapText", true); //内容过多，自动换行
       // pRange->dynamicCall("ClearContents()"); //清空单元格内容
       // QAxObject* interior =pRange->querySubObject("Interior");
       // interior->setProperty("Color", QColor(0, 255, 0)); //设置单元格背景色（绿色）
       // QAxObject* border = cell->querySubObject("Borders");
       // border->setProperty("Color", QColor(0, 0, 255)); //设置单元格边框色（蓝色）
       // QAxObject *font = cell->querySubObject("Font"); //获取单元格字体
       // font->setProperty("Name", QStringLiteral("华文彩云")); //设置单元格字体
       // font->setProperty("Bold", true); //设置单元格字体加粗
       // font->setProperty("Size", 20); //设置单元格字体大小
       // font->setProperty("Italic", true); //设置单元格字体斜体
       // font->setProperty("Underline", 2); //设置单元格下划线
       // font->setProperty("Color", QColor(255, 0, 0)); //设置单元格字体颜色（红色）
    if(row == 1){
    //加粗
        QAxObject *font = pRange->querySubObject("Font"); //获取单元格字体
        font->setProperty("Bold",true); //设置单元格字体加粗
    }
}

//保存Excel
void ExcelHelper::saveExcel(const QString &fileName)
{
    pWorkBook->dynamicCall("SaveAs(const QString &)",
    QDir::toNativeSeparators(fileName));
}

//释放Excel
void ExcelHelper::freeExcel()
{
    if (pApplication != NULL){
        pApplication->dynamicCall("Quit()");
        delete pApplication;
        pApplication = NULL;
    }
}

void ExcelHelper::mergeCells(const QString& cell)
{

    QAxObject *range = pSheet->querySubObject("Range(const QString&)", cell);
    range->setProperty("VerticalAlignment", -4108);//xlCenter
    range->setProperty("WrapText", true);
    range->setProperty("MergeCells", true);
}
void ExcelHelper::mergeCells(int topLeftRow, int topLeftColumn, int bottomRightRow, int bottomRightColumn)
{
    QString cell;
        cell.append(QChar(topLeftColumn - 1 + 'A'));
        cell.append(QString::number(topLeftRow));
        cell.append(":");
        cell.append(QChar(bottomRightColumn - 1 + 'A'));
        cell.append(QString::number(bottomRightRow));

        QAxObject *range = pSheet->querySubObject("Range(const QString&)", cell);
        range->setProperty("VerticalAlignment", -4108);//xlCenter
        range->setProperty("WrapText", true);
        range->setProperty("MergeCells", true);
}
void ExcelHelper::borders(int row, int column)
{
    QAxObject *range = pSheet->querySubObject("Cells(int,int)", row, column);
    QAxObject *border= range->querySubObject("Borders");
    border->setProperty("Color",QColor(0,0,0));
}

void ExcelHelper::borders(const QString& cell)
{
    QAxObject *range = pSheet->querySubObject("Range(const QString&)", cell);

    QAxObject *border= range->querySubObject("Borders");
    border->setProperty("Color",QColor(0,0,0));
}
void ExcelHelper::borders(int topLeftRow, int topLeftColumn, int bottomRightRow, int bottomRightColumn)
{
    QString cell;
        cell.append(QChar(topLeftColumn - 1 + 'A'));
        cell.append(QString::number(topLeftRow));
        cell.append(":");
        cell.append(QChar(bottomRightColumn - 1 + 'A'));
        cell.append(QString::number(bottomRightRow));




    QAxObject *range = pSheet->querySubObject("Range(const QString&)", cell);
    QAxObject *border= range->querySubObject("Borders");
    border->setProperty("Color",QColor(0,0,0));
}

// fileName为需要保存的文件名，titles为excel表头列名，dataVec为n行m列的数据


#endif // EXCELHELPER_H
