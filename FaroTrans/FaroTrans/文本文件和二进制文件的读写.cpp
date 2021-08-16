#include <QtCore/QCoreApplication>
#include <iostream>

#include <windows.h>

#import "C:\Windows\WinSxS\amd64_faro.ls_1d23f5635ba800ab_1.1.703.1_none_3591b17b356ae3ff\iQOpen.dll" no_namespace

#include <QDataStream>
#include <QFile>
#include <QTextStream>

struct FlsData
{
    float   x;
    float   y;
    float   z;
    int32_t reflect;
    int32_t lineNum;
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
    libRef->load(L"C:\\data\\2021-04-05-faro\\4-5000-198-f3332.fls");
#if 0
    QFile write_file("C:\\data\\2021-04-05-faro\\4-5000-198-f3332.txt");
    if (!write_file.open(QIODevice::WriteOnly))
    {
        std::cout << "open write file faild" << std::endl;
        return 1;
    }
    std::cout << "open write file ok" << std::endl;
    QTextStream write_stream(&write_file);
#endif

    QFile write_file("C:\\data\\2021-04-05-faro\\4-5000-198-f3332.bin");
    if (!write_file.open(QIODevice::WriteOnly))
    {
        std::cout << "open write file faild" << std::endl;
        return 1;
    }
    std::cout << "open write file ok" << std::endl;

    QDataStream write_stream(&write_file);

    int numRows = libRef->getScanNumRows(0);
    int numCols = libRef->getScanNumCols(0);
    std::cout << "numRows: " << numRows << std::endl;
    std::cout << "numCols: " << numCols << std::endl;

    FlsData data;
    int32_t used_line = 0;
    write_stream.writeRawData((char *) &used_line, sizeof(int32_t));
    for (int col = 0; col < numCols; ++col)
    {
        if (col % 1000 == 0)
        {
            std::cout << "current cols at:" << col << std::endl;
        }
        double *m_pPositionsArr = new double[numRows * 3];
        int *   m_pReflsArr     = new int[numRows];
        int     res             = libRef->getXYZScanPoints(0, 0, col, numRows, m_pPositionsArr, m_pReflsArr);   //getXYZScanPoints2
        int     nBad            = 0;
        for (int nRow = numRows - 1; nRow >= 0; nRow--)
        {
            int row = nRow;

            short reflect = (short) m_pReflsArr[row];

            row *= 3;
            double x = m_pPositionsArr[row + 0];
            double y = m_pPositionsArr[row + 1];
            double z = m_pPositionsArr[row + 2];

            double dist = sqrt(x * x + y * y + z * z);

            if (dist < 0.01)
            {
                nBad++;
                continue;
            }
            used_line++;

            data.x       = x;
            data.y       = (col * 0.01);
            data.z       = z;
            data.reflect = reflect;
            data.lineNum = col;
            // write_stream << x << "," << (col * 0.01) << "," << z << "," << reflect << "," << col << endl;
            write_stream.writeRawData((char *) &data, sizeof(FlsData));

        }   //for nRow
        delete[] m_pPositionsArr;
        delete[] m_pReflsArr;
        m_pPositionsArr = NULL;
        m_pReflsArr     = NULL;
    }
    write_file.seek(0);
    write_file.write((char *) &used_line, sizeof(int32_t));
    write_file.close();
    std::cout << "trans end!" << std::endl;

    QFile read_file("C:\\data\\2021-04-05-faro\\4-5000-198-f3332.bin");
    if (!read_file.open(QIODevice::ReadOnly))
    {
        std::cout << "open read file faild" << std::endl;
        return 1;
    }
    std::cout << "open read file ok" << std::endl;

    QFile write_txt_file("C:\\data\\2021-04-05-faro\\4-5000-198-f3332.txt");
    if (!write_txt_file.open(QIODevice::WriteOnly))
    {
        std::cout << "open write file faild" << std::endl;
        return 1;
    }
    std::cout << "open write file ok" << std::endl;
    QTextStream write_txt_stream(&write_txt_file);

    int32_t read_line = 0;
    read_file.read((char *) &read_line, sizeof(int32_t));
    std::cout << "num of read:" << read_line << std::endl;
    FlsData read_data;
    int     read_index = 0;
    while (!read_file.atEnd())
    {
        if (read_index % 1000 == 0)
        {
            std::cout << "current cols at:" << read_index << std::endl;
        }
        read_index++;
        read_file.read((char *) &read_data, sizeof(FlsData));
        write_txt_stream << read_data.x << "," << read_data.y << "," << read_data.z << "," << read_data.reflect << "," << read_data.lineNum << endl;
    }
    std::cout << "end!";
    return 0;
}
