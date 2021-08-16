#include <QtCore/QCoreApplication>
#include <iostream>

#include <windows.h>

#import "C:\Windows\WinSxS\amd64_faro.ls_1d23f5635ba800ab_1.1.703.1_none_3591b17b356ae3ff\iQOpen.dll" no_namespace

#include <QDataStream>
#include <QFile>
#include <QTextStream>
#include <qstring.h>

struct FlsData
{
    float    x;
    float    y;
    float    z;
    int32_t  reflect;
    int32_t  lineNum;
    uint64_t atime;
};

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

#if 0
    FaroTrans      faro;
    ScanPluginPara para;
    faro.OnInit(L"E:\\data\\jn1w001.fls", &para);
    ScannerInformation params;
    int                nRead;
    faro.Trans(1, &nRead, &params);
#endif

    CoInitialize(NULL);
    const wchar_t *licenseCode =
        L"FARO Open Runtime License\n"
        L"Key:W2CEL7NRTCTXXKJT6KZYSPUP2\n"
        L"\n"
        L"The software is the registered property of FARO Scanner "
        L"Production GmbH, Stuttgart, Germany.\n"
        L"All rights reserved.\n"
        L"This software may only be used with written permission of "
        L"FARO Scanner Production GmbH, Stuttgart, Germany.";
    IiQLicensedInterfaceIfPtr liPtr(__uuidof(iQLibIf));
    liPtr->License           = licenseCode;
    IiQLibIfPtr    libRef    = static_cast<IiQLibIfPtr>(liPtr);
    const wchar_t *file_path = L"";
    libRef->load(L"C:\\data\\2021-04-05-faro\\4-5000-198-f3363.fls");

    QFile *      write_file   = nullptr;
    QTextStream *write_stream = nullptr;

    int numRows = libRef->getScanNumRows(0);
    int numCols = libRef->getScanNumCols(0);
    std::cout << "numRows: " << numRows << std::endl;
    std::cout << "numCols: " << numCols << std::endl;

    FlsData data;

    double *first_pos_arr      = new double[numRows * 3];
    int *   first_reflect_arr  = new int[numRows];
    double *second_pos_arr     = new double[numRows * 3];
    int *   second_reflect_arr = new int[numRows];
    int32_t half_cols          = numCols / 2;
    for (int col = 0; col < half_cols; ++col)
    {
        if (col % 1000 == 0)
        {
            std::cout << "current cols at:" << col << std::endl;
            if (write_file != nullptr)
            {
                write_file->close();
                delete write_file;
                write_file = nullptr;
            }
            if (write_stream != nullptr)
            {
                delete write_stream;
                write_stream = nullptr;
            }
            write_file = new QFile("C:\\data\\2021-04-05-faro\\4-5000-198-f3363-" + QString::number(col) + ".txt");
            if (!write_file->open(QIODevice::WriteOnly))
            {
                std::cout << "open write file faild" << std::endl;
                return 1;
            }
            write_stream = new QTextStream(write_file);
            std::cout << "open write file ok" << std::endl;
        }
        int res  = libRef->getXYZScanPoints(0, 0, col, numRows, first_pos_arr, first_reflect_arr);                 //getXYZScanPoints2
        res      = libRef->getXYZScanPoints(0, 0, col + half_cols, numRows, second_pos_arr, second_reflect_arr);   //getXYZScanPoints2
        int nBad = 0;
        for (int nRow = numRows - 1; nRow >= 0; nRow--)
        {
            int row = nRow;

            row *= 3;
            double x = first_pos_arr[row + 0];
            double y = first_pos_arr[row + 1];
            double z = first_pos_arr[row + 2];

            double dist = sqrt(x * x + y * y + z * z);

            if (dist < 0.01 || dist > 10)
            {
                nBad++;
            }
            else
            {
                (*write_stream) << x << ","
                                << (col * 0.01) << ","
                                << z << ","
                                << first_reflect_arr[nRow] << ","
                                << col << endl;
            }

            x = second_pos_arr[row + 0];
            y = second_pos_arr[row + 1];
            z = second_pos_arr[row + 2];

            dist = sqrt(x * x + y * y + z * z);

            if (dist < 0.01 || dist > 10)
            {
                nBad++;
            }
            else
            {
                (*write_stream) << x << ","
                                << (col * 0.01) << ","
                                << z << ","
                                << second_reflect_arr[nRow] << ","
                                << col << endl;
            }

        }   //for nRow
    }
    delete[] first_pos_arr;
    delete[] first_reflect_arr;
    first_pos_arr     = NULL;
    first_reflect_arr = NULL;
    delete[] second_pos_arr;
    delete[] second_reflect_arr;
    second_pos_arr     = NULL;
    second_reflect_arr = NULL;
    std::cout << "end!";
    return 0;
}
