#include "bathyformat.h"
#include "xtfformat.h"

#include <QDateTime>
#include <QDebug>
#include <QFile>

BATHYFORMAT::BATHYFORMAT()
{
}

void BATHYFORMAT::TransMBHeadStructToQbyte(QByteArray &array, const MBFILEHEAD &data)
{
    array.append(reinterpret_cast<const char *>(&data.header_size), sizeof(data.header_size));
    array.append(reinterpret_cast<const char *>(&data.version_major), sizeof(data.version_major));
    array.append(reinterpret_cast<const char *>(&data.version_minor), sizeof(data.version_minor));
    array.append(reinterpret_cast<const char *>(data.multibeam_sensor), 16 * sizeof(char));
    array.append(reinterpret_cast<const char *>(&data.day), sizeof(data.day));
    array.append(reinterpret_cast<const char *>(&data.rxMountTilt), sizeof(data.rxMountTilt));
    array.append(reinterpret_cast<const char *>(&data.start_time), sizeof(data.start_time));
    array.append(reinterpret_cast<const char *>(&data.head_num), sizeof(data.head_num));
    array.append(reinterpret_cast<const char *>(&data.sector_num), sizeof(data.sector_num));
    array.append(reinterpret_cast<const char *>(&data.swath_num), sizeof(data.swath_num));
    array.append(reinterpret_cast<const char *>(&data.detaGPS_X), sizeof(data.detaGPS_X));
    array.append(reinterpret_cast<const char *>(&data.detaGPS_Y), sizeof(data.detaGPS_Y));
    array.append(reinterpret_cast<const char *>(&data.detaGPS_Z), sizeof(data.detaGPS_Z));
    for (int i = 0; i < data.head_num; i++)
    {
        array.append(reinterpret_cast<const char *>(&data.mbtrans.at(i).head_count), sizeof(data.mbtrans.at(i).head_count));
        array.append(reinterpret_cast<const char *>(&data.mbtrans.at(i).angle_X), sizeof(data.mbtrans.at(i).angle_X));
        array.append(reinterpret_cast<const char *>(&data.mbtrans.at(i).angle_Y), sizeof(data.mbtrans.at(i).angle_Y));
        array.append(reinterpret_cast<const char *>(&data.mbtrans.at(i).angle_Z), sizeof(data.mbtrans.at(i).angle_Z));
        array.append(reinterpret_cast<const char *>(&data.mbtrans.at(i).deta_X), sizeof(data.mbtrans.at(i).deta_X));
        array.append(reinterpret_cast<const char *>(&data.mbtrans.at(i).deta_Y), sizeof(data.mbtrans.at(i).deta_Y));
        array.append(reinterpret_cast<const char *>(&data.mbtrans.at(i).deta_Z), sizeof(data.mbtrans.at(i).deta_Z));
        array.append(reinterpret_cast<const char *>(&data.mbtrans.at(i).draught), sizeof(data.mbtrans.at(i).draught));
    }
}

void BATHYFORMAT::TransMBStructToQbyte(QByteArray &array, const MBBATHYDATA &data)
{
    array.append(reinterpret_cast<const char *>(&data.packetName), sizeof(data.packetName));
    array.append(reinterpret_cast<const char *>(&data.packetSize), sizeof(data.packetSize));
    array.append(reinterpret_cast<const char *>(&data.headNum), sizeof(data.headNum));
    int i = 0;
    for (i = 0; i < data.headNum; i++)
    {
        _fileSaveMultiHead(array, data.mltihead.at(i));
    }
}
void BATHYFORMAT::_fileSaveMultiHead(QByteArray &array, MULTIHEADER const data)
{
    array.append(reinterpret_cast<const char *>(&data.headerCount), sizeof(data.headerCount));
    array.append(reinterpret_cast<const char *>(&data.date), sizeof(data.date));
    array.append(reinterpret_cast<const char *>(&data.time), sizeof(data.time));
    array.append(reinterpret_cast<const char *>(&data.pingNum), sizeof(data.pingNum));
    array.append(reinterpret_cast<const char *>(&data.pingPeriod), sizeof(data.pingPeriod));
    array.append(reinterpret_cast<const char *>(&data.soundSpeed), sizeof(data.soundSpeed));
    array.append(reinterpret_cast<const char *>(&data.frequency), sizeof(data.frequency));
    array.append(reinterpret_cast<const char *>(&data.txPower), sizeof(data.txPower));
    array.append(reinterpret_cast<const char *>(&data.swathNum), sizeof(data.swathNum));
    for (int i = 0; i < data.swathNum; i++)
    {
        _fileSaveMultiSwath(array, data.mltswath.at(i));
    }
}
void BATHYFORMAT::_fileSaveMultiSwath(QByteArray &array, const MULTISWATH &data)
{
    array.append(reinterpret_cast<const char *>(&data.swathCount), sizeof(data.swathCount));
    array.append(reinterpret_cast<const char *>(&data.sectorNum), sizeof(data.sectorNum));
    array.append(reinterpret_cast<const char *>(&data.intensityFlag), sizeof(data.intensityFlag));

    for (int i = 0; i < data.sectorNum; i++)
    {
        _fileSaveMultiSector(array, data.mltsector.at(i));
    }
    _fileSaveBeaminfo(array, data.beams);
    if (data.beams.beamModel == vsrxtf::EQUALANGLE)
    {
        _fileSaveEqlang(array, data.eqlAng);
    }
    else
    {
        _fileSaveEqldis(array, data.eqldis, data.beams.points);
    }
    if (data.intensityFlag == vsrxtf::INTENSITYDO)
    {
        if (data.intensity.intensity.count() > 0)
        {
            _fileSaveIntensity(array, data.intensity, data.beams.points);
        }
    }
}
void BATHYFORMAT::_fileSaveMultiSector(QByteArray &array, const MULTISECTOR &data)
{
    array.append(reinterpret_cast<const char *>(&data.sectorCount), sizeof(data.sectorCount));
    array.append(reinterpret_cast<const char *>(&data.txPulseWidth), sizeof(data.txPulseWidth));
    array.append(reinterpret_cast<const char *>(&data.txbeamWidthVert), sizeof(data.txbeamWidthVert));
    array.append(reinterpret_cast<const char *>(&data.txBeamWidthHoriz), sizeof(data.txBeamWidthHoriz));
    array.append(reinterpret_cast<const char *>(&data.txSteeringVert), sizeof(data.txSteeringVert));
    array.append(reinterpret_cast<const char *>(&data.txSteeringHoriz), sizeof(data.txSteeringHoriz));
    array.append(reinterpret_cast<const char *>(&data.rxBandWidth), sizeof(data.rxBandWidth));
    array.append(reinterpret_cast<const char *>(&data.rxGain), sizeof(data.rxGain));
    array.append(reinterpret_cast<const char *>(&data.rxSpreading), sizeof(data.rxSpreading));
    array.append(reinterpret_cast<const char *>(&data.rxAbsorption), sizeof(data.rxAbsorption));
}
void BATHYFORMAT::_fileSaveBeaminfo(QByteArray &array, const BEAMSINFO &data)
{
    array.append(reinterpret_cast<const char *>(&data.points), sizeof(data.points));
    for (int i = 0; i < data.points; i++)
    {
        array.append(reinterpret_cast<const char *>(&data.range.at(i)), sizeof(data.range.at(i)));
    }
    array.append(reinterpret_cast<const char *>(&data.beamModel), sizeof(data.beamModel));
}

void BATHYFORMAT::_fileSaveEqlang(QByteArray &array, const EQLANG &data)
{
    array.append(reinterpret_cast<const char *>(&data.angleFirst), sizeof(data.angleFirst));
    array.append(reinterpret_cast<const char *>(&data.angleLast), sizeof(data.angleLast));
}

void BATHYFORMAT::_fileSaveEqldis(QByteArray &array, const EQLDIS &data, const unsigned short &points)
{
    for (int i = 0; i < points; i++)
    {
        array.append(reinterpret_cast<const char *>(&data.angles.at(i)), sizeof(data.angles.at(i)));
    }
}

void BATHYFORMAT::_fileSaveIntensity(QByteArray &array, const INTENSITY &data, const unsigned short &points)
{
    for (int i = 0; i < points; i++)
    {
        array.append(reinterpret_cast<const char *>(&data.intensity.at(i)), sizeof(data.intensity.at(i)));
    }
}
void BATHYFORMAT::byteChange(char *xbuffer, char *buffer, int init)
{
    for (int i = 0; i < init; i++)
    {
        memcpy(xbuffer + init - 1 - i, buffer + i, 1);
    }
}
void BATHYFORMAT::fileSaveMbSnipData(QByteArray &array, const MBSNIPDATA &data)
{
    array.append(reinterpret_cast<const char *>(&data.packetName), sizeof(data.packetName));
    array.append(reinterpret_cast<const char *>(&data.packetSize), sizeof(data.packetSize));
    array.append(reinterpret_cast<const char *>(&data.headNum), sizeof(data.headNum));
    for (int i = 0; i < data.headNum; i++)
    {
        _fileSaveMultiHead(array, data.mltihead.at(i));
    }
}
void BATHYFORMAT::_fileSaveMultiHead(QByteArray &array, const MULTISPHEADER &data)
{
    array.append(reinterpret_cast<const char *>(&data.headerCount), sizeof(data.headerCount));
    array.append(reinterpret_cast<const char *>(&data.date), sizeof(data.date));
    array.append(reinterpret_cast<const char *>(&data.time), sizeof(data.time));
    array.append(reinterpret_cast<const char *>(&data.pingNum), sizeof(data.pingNum));
    array.append(reinterpret_cast<const char *>(&data.pingPeriod), sizeof(data.pingPeriod));
    array.append(reinterpret_cast<const char *>(&data.soundSpeed), sizeof(data.soundSpeed));
    array.append(reinterpret_cast<const char *>(&data.frequency), sizeof(data.frequency));
    array.append(reinterpret_cast<const char *>(&data.txPower), sizeof(data.txPower));
    array.append(reinterpret_cast<const char *>(&data.swathNum), sizeof(data.swathNum));
    for (int i = 0; i < data.swathNum; i++)
    {
        _fileSaveMultiSwath(array, data.mltswath.at(i));
    }
}
void BATHYFORMAT::_fileSaveMultiSwath(QByteArray &array, const MULTISPSWATH &data)
{
    array.append(reinterpret_cast<const char *>(&data.swathCount), sizeof(data.swathCount));
    array.append(reinterpret_cast<const char *>(&data.sectorNum), sizeof(data.sectorNum));

    for (int i = 0; i < data.sectorNum; i++)
    {
        _fileSaveMultiSector(array, data.mltsector.at(i));
    }
    array.append(reinterpret_cast<const char *>(&data.snippets), sizeof(data.snippets));
    for (int i = 0; i < data.snippets; i++)
    {
        _fileSaveSnipData(array, data.snipdata.at(i));
    }
}
void BATHYFORMAT::_fileSaveSnipData(QByteArray &array, const SNIPDATA &data)
{
    array.append(reinterpret_cast<const char *>(&data.pingNum), sizeof(data.pingNum));
    array.append(reinterpret_cast<const char *>(&data.snipNum), sizeof(data.snipNum));
    array.append(reinterpret_cast<const char *>(&data.samples), sizeof(data.samples));
    array.append(reinterpret_cast<const char *>(&data.firstSamp), sizeof(data.firstSamp));
    array.append(reinterpret_cast<const char *>(&data.angle), sizeof(data.angle));
    for (int i = 0; i < data.samples; i++)
    {
        array.append(reinterpret_cast<const char *>(&data.magnitude.at(i)), sizeof(data.magnitude.at(i)));
    }
}
//read snip data from file
void BATHYFORMAT::_fileReadSnipMltHeader(char *buffer, MULTISPHEADER &headerdata, int &sizeCount)
{
    memcpy(&headerdata.headerCount, buffer + sizeCount, sizeof(headerdata.headerCount));
    sizeCount = sizeCount + sizeof(headerdata.headerCount);
    memcpy(&headerdata.date, buffer + sizeCount, sizeof(headerdata.date));
    sizeCount = sizeCount + sizeof(headerdata.date);
    memcpy(&headerdata.time, buffer + sizeCount, sizeof(headerdata.time));
    sizeCount = sizeCount + sizeof(headerdata.time);
    memcpy(&headerdata.pingNum, buffer + sizeCount, sizeof(headerdata.pingNum));
    sizeCount = sizeCount + sizeof(headerdata.pingNum);
    memcpy(&headerdata.pingPeriod, buffer + sizeCount, sizeof(headerdata.pingPeriod));
    sizeCount = sizeCount + sizeof(headerdata.pingPeriod);
    memcpy(&headerdata.soundSpeed, buffer + sizeCount, sizeof(headerdata.soundSpeed));
    sizeCount = sizeCount + sizeof(headerdata.soundSpeed);
    memcpy(&headerdata.frequency, buffer + sizeCount, sizeof(headerdata.frequency));
    sizeCount = sizeCount + sizeof(headerdata.frequency);
    memcpy(&headerdata.txPower, buffer + sizeCount, sizeof(headerdata.txPower));
    sizeCount = sizeCount + sizeof(headerdata.txPower);
    memcpy(&headerdata.swathNum, buffer + sizeCount, sizeof(headerdata.swathNum));
    sizeCount = sizeCount + sizeof(headerdata.swathNum);
    for (int i = 0; i < headerdata.swathNum; i++)
    {
        MULTISPSWATH swathdata;
        _fileReadSnipMltSwath(buffer, swathdata, sizeCount);
        headerdata.mltswath.append(swathdata);
    }
}
void BATHYFORMAT::_fileReadSnipMltSwath(char *buffer, MULTISPSWATH &swathdata, int &sizeCount)
{
    memcpy(&swathdata.swathCount, buffer + sizeCount, sizeof(swathdata.swathCount));
    sizeCount = sizeCount + sizeof(swathdata.swathCount);
    memcpy(&swathdata.sectorNum, buffer + sizeCount, sizeof(swathdata.sectorNum));
    sizeCount = sizeCount + sizeof(swathdata.sectorNum);

    for (int i = 0; i < swathdata.sectorNum; i++)
    {
        MULTISECTOR sectordata;
        _fileReadMltSectorData(buffer, sectordata, sizeCount);
        swathdata.mltsector.append(sectordata);
    }
    memcpy(&swathdata.snippets, buffer + sizeCount, sizeof(swathdata.snippets));
    sizeCount = sizeCount + sizeof(swathdata.snippets);
    for (int i = 0; i < swathdata.snippets; i++)
    {
        SNIPDATA data;
        _fileReadSNIPData(buffer, data, sizeCount);
        swathdata.snipdata.append(data);
    }
}
void BATHYFORMAT::_fileReadSNIPData(char *buffer, SNIPDATA &data, int &sizeCount)
{
    memcpy(&data.pingNum, buffer + sizeCount, sizeof(data.pingNum));
    sizeCount = sizeCount + sizeof(data.pingNum);
    memcpy(&data.snipNum, buffer + sizeCount, sizeof(data.snipNum));
    sizeCount = sizeCount + sizeof(data.snipNum);
    memcpy(&data.samples, buffer + sizeCount, sizeof(data.samples));
    sizeCount = sizeCount + sizeof(data.samples);
    memcpy(&data.firstSamp, buffer + sizeCount, sizeof(data.firstSamp));
    sizeCount = sizeCount + sizeof(data.firstSamp);
    memcpy(&data.angle, buffer + sizeCount, sizeof(data.angle));
    sizeCount = sizeCount + sizeof(data.angle);
    for (int i = 0; i < data.samples; i++)
    {
        unsigned short magnitude;
        memcpy(&magnitude, buffer + sizeCount, sizeof(magnitude));
        sizeCount = sizeCount + sizeof(magnitude);
        data.magnitude.append(magnitude);
    }
}
//read data functions from file
void BATHYFORMAT::fileReadMbFile(QFile &mbfile)
{
    int        sizeCount = 0;
    int        fileSize  = 0;
    char       at        = 0;
    char *     buffer    = &at;
    char       at2       = 0;
    char *     xbuffer   = &at2;
    QByteArray dataArray;
    dataArray = mbfile.read(2);
    uint16_t fileHeaderSize;
    buffer = dataArray.data();
    // byteChange(xbuffer, buffer, 2);
    memcpy(&fileHeaderSize, buffer, 2);
    dataArray += mbfile.read(fileHeaderSize - 2);
    buffer = dataArray.data();
    qDebug() << "readHeaderFileSize:" << fileHeaderSize;
    MBFILEHEAD fileHeadData;
    _fileReadFileHeader(buffer, fileHeadData, sizeCount);
    fileSize = fileSize + sizeCount;
    qDebug() << "header_size:" << fileHeadData.header_size;
    qDebug() << "version_major:" << fileHeadData.version_major;
    qDebug() << "version_minor:" << fileHeadData.version_minor;
    qDebug() << "multibeam_sensor:" << fileHeadData.multibeam_sensor;
    int year  = fileHeadData.day / 10000;
    int month = (fileHeadData.day - year * 10000) / 100;
    int day   = fileHeadData.day - year * 10000 - month * 100;
    qDebug() << "date:" << year << ":" << month << ":" << day;
    qDebug() << "rxMountTilt:" << fileHeadData.rxMountTilt;
    int   hour    = fileHeadData.start_time / 10000;
    int   minute  = (fileHeadData.start_time - hour * 10000) / 100;
    float seconds = fileHeadData.start_time - hour * 10000 - minute * 100;
    qDebug() << "start_time:" << hour << ":" << minute << ":" << seconds;
    qDebug() << "head_num:" << fileHeadData.head_num;
    qDebug() << "sector_num:" << fileHeadData.sector_num;
    qDebug() << "swath_num:" << fileHeadData.swath_num;
    qDebug() << "detaGPS_X:" << fileHeadData.detaGPS_X;
    qDebug() << "detaGPS_Y:" << fileHeadData.detaGPS_Y;
    qDebug() << "detaGPS_Z:" << fileHeadData.detaGPS_Z;

    for (int i = 0; i < fileHeadData.head_num; i++)
    {
        qDebug() << "mbtrans[" << 1 + i << "]head_count:" << fileHeadData.mbtrans.at(i).head_count;
        qDebug() << "mbtrans[" << 1 + i << "]angle_X:" << fileHeadData.mbtrans.at(i).angle_X;
        qDebug() << "mbtrans[" << 1 + i << "]angle_Y:" << fileHeadData.mbtrans.at(i).angle_Y;
        qDebug() << "mbtrans[" << 1 + i << "]angle_Z:" << fileHeadData.mbtrans.at(i).angle_Z;
        qDebug() << "mbtrans[" << 1 + i << "]deta_X:" << fileHeadData.mbtrans.at(i).deta_X;
        qDebug() << "mbtrans[" << 1 + i << "]deta_Y:" << fileHeadData.mbtrans.at(i).deta_Y;
        qDebug() << "mbtrans[" << 1 + i << "]deta_Z:" << fileHeadData.mbtrans.at(i).deta_Z;
        qDebug() << "mbtrans[" << 1 + i << "]draught:" << fileHeadData.mbtrans.at(i).draught;
    }
    int packetCount = 0;
    while (!mbfile.atEnd())
    {
        qDebug() << "packetCount:" << ++packetCount;
        qDebug() << "pos:" << mbfile.pos();
        QByteArray bathArray = mbfile.read(8);
        int        bathSize;
        int        sizeCount_2 = 0;
        buffer                 = bathArray.data();
        // byteChange(xbuffer, buffer, 4);
        memcpy(&bathSize, buffer + 4, 4);
        bathArray += mbfile.read(bathSize - 8);
        buffer = bathArray.data();
        qDebug() << "bathsize:" << bathSize;

        unsigned int  packetName;
        unsigned int  packetSize;
        unsigned char headNum;
        memcpy(&packetName, buffer + sizeCount_2, sizeof(packetName));
        sizeCount_2 = sizeCount_2 + sizeof(packetName);

        memcpy(&packetSize, buffer + sizeCount_2, sizeof(packetSize));
        sizeCount_2 = sizeCount_2 + sizeof(packetSize);
        memcpy(&headNum, buffer + sizeCount_2, sizeof(headNum));
        sizeCount_2 = sizeCount_2 + sizeof(headNum);
        switch (packetName)
        {
        case BATHFIELD: {
            MBBATHYDATA bathydata;
            bathydata.packetName = packetName;
            bathydata.packetSize = packetSize;
            bathydata.headNum    = headNum;
            for (int i = 0; i < bathydata.headNum; i++)
            {
                MULTIHEADER headerdata;
                _fileReadMltHeaderData(buffer, headerdata, sizeCount_2);
                bathydata.mltihead.append(headerdata);
            }
            qDebug() << "packetName:" << bathydata.packetName;
            qDebug() << "packetSize:" << bathydata.packetSize;
            qDebug() << "headNum:" << bathydata.headNum;
            for (int i = 0; i < bathydata.headNum; i++)
            {
                qDebug() << "mltihead[" << i + 1 << "]headerCount:" << bathydata.mltihead.at(i).headerCount;
                int year  = bathydata.mltihead.at(i).date / 10000;
                int month = (bathydata.mltihead.at(i).date - year * 10000) / 100;
                int day   = bathydata.mltihead.at(i).date - year * 10000 - month * 100;
                qDebug() << "ping_date:" << year << ":" << month << ":" << day;

                int   hour    = bathydata.mltihead.at(i).time / 10000;
                int   minute  = (bathydata.mltihead.at(i).time - hour * 10000) / 100;
                float seconds = bathydata.mltihead.at(i).time - hour * 10000 - minute * 100;
                qDebug() << "ping_time:" << hour << ":" << minute << ":" << seconds;

                qDebug() << "mltihead[" << i + 1 << "]pingNum:" << bathydata.mltihead.at(i).pingNum;
                qDebug() << "mltihead[" << i + 1 << "]pingPeriod:" << bathydata.mltihead.at(i).pingPeriod;
                qDebug() << "mltihead[" << i + 1 << "]soundSpeed:" << bathydata.mltihead.at(i).soundSpeed;
                qDebug() << "mltihead[" << i + 1 << "]frequency:" << bathydata.mltihead.at(i).frequency;
                qDebug() << "mltihead[" << i + 1 << "]txPower:" << bathydata.mltihead.at(i).txPower;
                qDebug() << "mltihead[" << i + 1 << "]swathNum:" << bathydata.mltihead.at(i).swathNum;
                for (int j = 0; j < bathydata.mltihead.at(i).swathNum; j++)
                {
                    qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]swathCount:" << bathydata.mltihead.at(i).mltswath.at(j).swathCount;
                    qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]sectorNum:" << bathydata.mltihead.at(i).mltswath.at(j).sectorNum;
                    qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]intensityFlag:" << bathydata.mltihead.at(i).mltswath.at(j).intensityFlag;
                    for (int m = 0; m < bathydata.mltihead.at(i).mltswath.at(j).sectorNum; m++)
                    {
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]sectorCount:" << bathydata.mltihead.at(i).mltswath.at(j).mltsector.at(m).sectorCount;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]txPulseWidth:" << bathydata.mltihead.at(i).mltswath.at(j).mltsector.at(m).txPulseWidth;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]txbeamWidthVert:" << bathydata.mltihead.at(i).mltswath.at(j).mltsector.at(m).txbeamWidthVert;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]txBeamWidthHoriz:" << bathydata.mltihead.at(i).mltswath.at(j).mltsector.at(m).txBeamWidthHoriz;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]txSteeringVert:" << bathydata.mltihead.at(i).mltswath.at(j).mltsector.at(m).txSteeringVert;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]txSteeringHoriz:" << bathydata.mltihead.at(i).mltswath.at(j).mltsector.at(m).txSteeringHoriz;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]rxBandWidth:" << bathydata.mltihead.at(i).mltswath.at(j).mltsector.at(m).rxBandWidth;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]rxGain:" << bathydata.mltihead.at(i).mltswath.at(j).mltsector.at(m).rxGain;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]rxSpreading:" << bathydata.mltihead.at(i).mltswath.at(j).mltsector.at(m).rxSpreading;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]rxAbsorption:" << bathydata.mltihead.at(i).mltswath.at(j).mltsector.at(m).rxAbsorption;
                    }
                    qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]beams.points:" << bathydata.mltihead.at(i).mltswath.at(j).beams.points;
                    for (int m = 0; m < bathydata.mltihead.at(i).mltswath.at(j).beams.points; m++)
                    {
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]beams.range[" << m + 1 << "]:" << bathydata.mltihead.at(i).mltswath.at(j).beams.range.at(m);
                    }
                    qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]beams.beamModel:" << bathydata.mltihead.at(i).mltswath.at(j).beams.beamModel;
                    switch (bathydata.mltihead.at(i).mltswath.at(j).beams.beamModel)
                    {
                    case vsrxtf::EQUALANGLE: {
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]eqlAng.angleFirst:" << bathydata.mltihead.at(i).mltswath.at(j).eqlAng.angleFirst;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]eqlAng.angleLast:" << bathydata.mltihead.at(i).mltswath.at(j).eqlAng.angleLast;
                        break;
                    }
                    case vsrxtf::EQUALDISTANCE: {
                        for (int m = 0; m < bathydata.mltihead.at(i).mltswath.at(j).beams.points; m++)
                        {
                            qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]eqldis.angles[" << m + 1 << "]:" << bathydata.mltihead.at(i).mltswath.at(j).eqldis.angles.at(m);
                        }
                        break;
                    }
                    }
                    if (bathydata.mltihead.at(i).mltswath.at(j).intensityFlag == vsrxtf::INTENSITYDO)
                    {
                        for (int m = 0; m < bathydata.mltihead.at(i).mltswath.at(j).beams.points; m++)
                        {
                            qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]intensity[" << m + 1 << "]:" << bathydata.mltihead.at(i).mltswath.at(j).intensity.intensity.at(m);
                        }
                    }
                }
            }

            break;
        }
        case SNIPFIELD: {
            MBSNIPDATA snipData;
            snipData.packetName = packetName;
            snipData.packetSize = packetSize;
            snipData.headNum    = headNum;
            for (int i = 0; i < snipData.headNum; i++)
            {
                MULTISPHEADER headerdata;
                _fileReadSnipMltHeader(buffer, headerdata, sizeCount_2);
                snipData.mltihead.append(headerdata);
            }
            qDebug() << "packetName:" << snipData.packetName;
            qDebug() << "packetSize:" << snipData.packetSize;
            qDebug() << "headNum:" << snipData.headNum;
            for (int i = 0; i < snipData.headNum; i++)
            {
                qDebug() << "mltihead[" << i + 1 << "]headerCount:" << snipData.mltihead.at(i).headerCount;
                int year  = snipData.mltihead.at(i).date / 10000;
                int month = (snipData.mltihead.at(i).date - year * 10000) / 100;
                int day   = snipData.mltihead.at(i).date - year * 10000 - month * 100;
                qDebug() << "ping_date:" << year << ":" << month << ":" << day;

                int   hour    = snipData.mltihead.at(i).time / 10000;
                int   minute  = (snipData.mltihead.at(i).time - hour * 10000) / 100;
                float seconds = snipData.mltihead.at(i).time - hour * 10000 - minute * 100;
                qDebug() << "ping_time:" << hour << ":" << minute << ":" << seconds;

                qDebug() << "mltihead[" << i + 1 << "]pingNum:" << snipData.mltihead.at(i).pingNum;
                qDebug() << "mltihead[" << i + 1 << "]pingPeriod:" << snipData.mltihead.at(i).pingPeriod;
                qDebug() << "mltihead[" << i + 1 << "]soundSpeed:" << snipData.mltihead.at(i).soundSpeed;
                qDebug() << "mltihead[" << i + 1 << "]frequency:" << snipData.mltihead.at(i).frequency;
                qDebug() << "mltihead[" << i + 1 << "]txPower:" << snipData.mltihead.at(i).txPower;
                qDebug() << "mltihead[" << i + 1 << "]swathNum:" << snipData.mltihead.at(i).swathNum;
                for (int j = 0; j < snipData.mltihead.at(i).swathNum; j++)
                {
                    qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]swathCount:" << snipData.mltihead.at(i).mltswath.at(j).swathCount;
                    qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]sectorNum:" << snipData.mltihead.at(i).mltswath.at(j).sectorNum;
                    for (int m = 0; m < snipData.mltihead.at(i).mltswath.at(j).sectorNum; m++)
                    {
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]sectorCount:" << snipData.mltihead.at(i).mltswath.at(j).mltsector.at(m).sectorCount;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]txPulseWidth:" << snipData.mltihead.at(i).mltswath.at(j).mltsector.at(m).txPulseWidth;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]txbeamWidthVert:" << snipData.mltihead.at(i).mltswath.at(j).mltsector.at(m).txbeamWidthVert;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]txBeamWidthHoriz:" << snipData.mltihead.at(i).mltswath.at(j).mltsector.at(m).txBeamWidthHoriz;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]txSteeringVert:" << snipData.mltihead.at(i).mltswath.at(j).mltsector.at(m).txSteeringVert;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]txSteeringHoriz:" << snipData.mltihead.at(i).mltswath.at(j).mltsector.at(m).txSteeringHoriz;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]rxBandWidth:" << snipData.mltihead.at(i).mltswath.at(j).mltsector.at(m).rxBandWidth;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]rxGain:" << snipData.mltihead.at(i).mltswath.at(j).mltsector.at(m).rxGain;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]rxSpreading:" << snipData.mltihead.at(i).mltswath.at(j).mltsector.at(m).rxSpreading;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]mltsector[" << m + 1 << "]rxAbsorption:" << snipData.mltihead.at(i).mltswath.at(j).mltsector.at(m).rxAbsorption;
                    }
                    qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]snippets:" << snipData.mltihead.at(i).mltswath.at(j).snippets;
                    for (int n = 0; n < snipData.mltihead.at(i).mltswath.at(j).snippets; n++)
                    {
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]snipData[" << n + 1 << "]pingNum:" << snipData.mltihead.at(i).mltswath.at(j).snipdata.at(n).pingNum;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]snipData[" << n + 1 << "]snipNum:" << snipData.mltihead.at(i).mltswath.at(j).snipdata.at(n).snipNum;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]snipData[" << n + 1 << "]samples:" << snipData.mltihead.at(i).mltswath.at(j).snipdata.at(n).samples;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]snipData[" << n + 1 << "]firstSamp:" << snipData.mltihead.at(i).mltswath.at(j).snipdata.at(n).firstSamp;
                        qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]snipData[" << n + 1 << "]angle:" << snipData.mltihead.at(i).mltswath.at(j).snipdata.at(n).angle;
                        for (int k = 0; k < snipData.mltihead.at(i).mltswath.at(j).snipdata.at(n).samples; k++)
                        {
                            qDebug() << "mltihead[" << i + 1 << "]mltswath[" << j + 1 << "]snipData[" << n + 1 << "]magnitude[" << k + 1 << "]:" << snipData.mltihead.at(i).mltswath.at(j).snipdata.at(n).magnitude.at(k);
                        }
                    }
                }
            }
            break;
        }
        }
    }
    qDebug() << "file end!";
}

void BATHYFORMAT::_fileReadFileHeader(char *buffer, MBFILEHEAD &fileHeadData, int &sizeCount)
{
    memcpy(&fileHeadData.header_size, buffer + sizeCount, sizeof(fileHeadData.header_size));
    sizeCount = sizeCount + sizeof(fileHeadData.header_size);
    memcpy(&fileHeadData.version_major, buffer + sizeCount, sizeof(fileHeadData.version_major));
    sizeCount = sizeCount + sizeof(fileHeadData.version_major);
    memcpy(&fileHeadData.version_minor, buffer + sizeCount, sizeof(fileHeadData.version_minor));
    sizeCount = sizeCount + sizeof(fileHeadData.version_minor);
    memcpy(fileHeadData.multibeam_sensor, buffer + sizeCount, 16 * sizeof(char));
    sizeCount = sizeCount + 16 * sizeof(char);
    memcpy(&fileHeadData.day, buffer + sizeCount, sizeof(fileHeadData.day));
    sizeCount = sizeCount + sizeof(fileHeadData.day);
    memcpy(&fileHeadData.rxMountTilt, buffer + sizeCount, sizeof(fileHeadData.rxMountTilt));
    sizeCount = sizeCount + sizeof(fileHeadData.rxMountTilt);
    memcpy(&fileHeadData.start_time, buffer + sizeCount, sizeof(fileHeadData.start_time));
    sizeCount = sizeCount + sizeof(fileHeadData.start_time);
    memcpy(&fileHeadData.head_num, buffer + sizeCount, sizeof(fileHeadData.head_num));
    sizeCount = sizeCount + sizeof(fileHeadData.head_num);
    memcpy(&fileHeadData.sector_num, buffer + sizeCount, sizeof(fileHeadData.sector_num));
    sizeCount = sizeCount + sizeof(fileHeadData.sector_num);
    memcpy(&fileHeadData.swath_num, buffer + sizeCount, sizeof(fileHeadData.swath_num));
    sizeCount = sizeCount + sizeof(fileHeadData.swath_num);
    memcpy(&fileHeadData.detaGPS_X, buffer + sizeCount, sizeof(fileHeadData.detaGPS_X));
    sizeCount = sizeCount + sizeof(fileHeadData.detaGPS_X);
    memcpy(&fileHeadData.detaGPS_Y, buffer + sizeCount, sizeof(fileHeadData.detaGPS_Y));
    sizeCount = sizeCount + sizeof(fileHeadData.detaGPS_Y);
    memcpy(&fileHeadData.detaGPS_Z, buffer + sizeCount, sizeof(fileHeadData.detaGPS_Z));
    sizeCount = sizeCount + sizeof(fileHeadData.detaGPS_Z);

    for (int i = 0; i < fileHeadData.head_num; i++)
    {
        TRANSDUCERALIG mbtrans;
        memcpy(&mbtrans.head_count, buffer + sizeCount, sizeof(mbtrans.head_count));
        sizeCount = sizeCount + sizeof(mbtrans.head_count);
        memcpy(&mbtrans.angle_X, buffer + sizeCount, sizeof(mbtrans.angle_X));
        sizeCount = sizeCount + sizeof(mbtrans.angle_X);
        memcpy(&mbtrans.angle_Y, buffer + sizeCount, sizeof(mbtrans.angle_Y));
        sizeCount = sizeCount + sizeof(mbtrans.angle_Y);
        memcpy(&mbtrans.angle_Z, buffer + sizeCount, sizeof(mbtrans.angle_Z));
        sizeCount = sizeCount + sizeof(mbtrans.angle_Z);
        memcpy(&mbtrans.deta_X, buffer + sizeCount, sizeof(mbtrans.deta_X));
        sizeCount = sizeCount + sizeof(mbtrans.deta_X);
        memcpy(&mbtrans.deta_Y, buffer + sizeCount, sizeof(mbtrans.deta_Y));
        sizeCount = sizeCount + sizeof(mbtrans.deta_Y);
        memcpy(&mbtrans.deta_Z, buffer + sizeCount, sizeof(mbtrans.deta_Z));
        sizeCount = sizeCount + sizeof(mbtrans.deta_Z);
        memcpy(&mbtrans.draught, buffer + sizeCount, sizeof(mbtrans.draught));
        sizeCount = sizeCount + sizeof(mbtrans.draught);

        fileHeadData.mbtrans.append(mbtrans);
    }
}
void BATHYFORMAT::_fileReadMltHeaderData(char *buffer, MULTIHEADER &headerdata, int &sizeCount)
{
    memcpy(&headerdata.headerCount, buffer + sizeCount, sizeof(headerdata.headerCount));
    sizeCount = sizeCount + sizeof(headerdata.headerCount);
    memcpy(&headerdata.date, buffer + sizeCount, sizeof(headerdata.date));
    sizeCount = sizeCount + sizeof(headerdata.date);
    memcpy(&headerdata.time, buffer + sizeCount, sizeof(headerdata.time));
    sizeCount = sizeCount + sizeof(headerdata.time);
    memcpy(&headerdata.pingNum, buffer + sizeCount, sizeof(headerdata.pingNum));
    sizeCount = sizeCount + sizeof(headerdata.pingNum);
    memcpy(&headerdata.pingPeriod, buffer + sizeCount, sizeof(headerdata.pingPeriod));
    sizeCount = sizeCount + sizeof(headerdata.pingPeriod);
    memcpy(&headerdata.soundSpeed, buffer + sizeCount, sizeof(headerdata.soundSpeed));
    sizeCount = sizeCount + sizeof(headerdata.soundSpeed);
    memcpy(&headerdata.frequency, buffer + sizeCount, sizeof(headerdata.frequency));
    sizeCount = sizeCount + sizeof(headerdata.frequency);
    memcpy(&headerdata.txPower, buffer + sizeCount, sizeof(headerdata.txPower));
    sizeCount = sizeCount + sizeof(headerdata.txPower);
    memcpy(&headerdata.swathNum, buffer + sizeCount, sizeof(headerdata.swathNum));
    sizeCount = sizeCount + sizeof(headerdata.swathNum);
    for (int i = 0; i < headerdata.swathNum; i++)
    {
        MULTISWATH swathdata;
        _fileReadMltSwathData(buffer, swathdata, sizeCount);
        headerdata.mltswath.append(swathdata);
    }
}
void BATHYFORMAT::_fileReadMltSwathData(char *buffer, MULTISWATH &swathdata, int &sizeCount)
{
    memcpy(&swathdata.swathCount, buffer + sizeCount, sizeof(swathdata.swathCount));
    sizeCount = sizeCount + sizeof(swathdata.swathCount);
    memcpy(&swathdata.sectorNum, buffer + sizeCount, sizeof(swathdata.sectorNum));
    sizeCount = sizeCount + sizeof(swathdata.sectorNum);
    memcpy(&swathdata.intensityFlag, buffer + sizeCount, sizeof(swathdata.intensityFlag));
    sizeCount = sizeCount + sizeof(swathdata.intensityFlag);

    for (int i = 0; i < swathdata.sectorNum; i++)
    {
        MULTISECTOR sectordata;
        _fileReadMltSectorData(buffer, sectordata, sizeCount);
        swathdata.mltsector.append(sectordata);
    }
    _fileReadBeamInfo(buffer, swathdata.beams, sizeCount);
    switch (swathdata.beams.beamModel)
    {
    case vsrxtf::EQUALANGLE: {
        _fileReadEQLAng(buffer, swathdata.eqlAng, sizeCount);
        break;
    }
    case vsrxtf::EQUALDISTANCE: {
        _fileReadEQLDis(buffer, swathdata.eqldis, sizeCount, swathdata.beams.points);
        break;
    }
    }
    if (swathdata.intensityFlag == vsrxtf::INTENSITYDO)
    {
        for (int i = 0; i < swathdata.beams.points; i++)
        {
            float intensity;
            memcpy(&intensity, buffer + sizeCount, sizeof(intensity));
            sizeCount = sizeCount + sizeof(intensity);
            swathdata.intensity.intensity.append(intensity);
        }
    }
}
void BATHYFORMAT::_fileReadMltSectorData(char *buffer, MULTISECTOR &sectordata, int &sizeCount)
{
    memcpy(&sectordata.sectorCount, buffer + sizeCount, sizeof(sectordata.sectorCount));
    sizeCount = sizeCount + sizeof(sectordata.sectorCount);
    memcpy(&sectordata.txPulseWidth, buffer + sizeCount, sizeof(sectordata.txPulseWidth));
    sizeCount = sizeCount + sizeof(sectordata.txPulseWidth);
    memcpy(&sectordata.txbeamWidthVert, buffer + sizeCount, sizeof(sectordata.txbeamWidthVert));
    sizeCount = sizeCount + sizeof(sectordata.txbeamWidthVert);
    memcpy(&sectordata.txBeamWidthHoriz, buffer + sizeCount, sizeof(sectordata.txBeamWidthHoriz));
    sizeCount = sizeCount + sizeof(sectordata.txBeamWidthHoriz);
    memcpy(&sectordata.txSteeringVert, buffer + sizeCount, sizeof(sectordata.txSteeringVert));
    sizeCount = sizeCount + sizeof(sectordata.txSteeringVert);
    memcpy(&sectordata.txSteeringHoriz, buffer + sizeCount, sizeof(sectordata.txSteeringHoriz));
    sizeCount = sizeCount + sizeof(sectordata.txSteeringHoriz);
    memcpy(&sectordata.rxBandWidth, buffer + sizeCount, sizeof(sectordata.rxBandWidth));
    sizeCount = sizeCount + sizeof(sectordata.rxBandWidth);
    memcpy(&sectordata.rxGain, buffer + sizeCount, sizeof(sectordata.rxGain));
    sizeCount = sizeCount + sizeof(sectordata.rxGain);
    memcpy(&sectordata.rxSpreading, buffer + sizeCount, sizeof(sectordata.rxSpreading));
    sizeCount = sizeCount + sizeof(sectordata.rxSpreading);
    memcpy(&sectordata.rxAbsorption, buffer + sizeCount, sizeof(sectordata.rxAbsorption));
    sizeCount = sizeCount + sizeof(sectordata.rxAbsorption);
}

void BATHYFORMAT::_fileReadBeamInfo(char *buffer, BEAMSINFO &beams, int &sizeCount)
{
    memcpy(&beams.points, buffer + sizeCount, sizeof(beams.points));
    sizeCount = sizeCount + sizeof(beams.points);
    for (int i = 0; i < beams.points; i++)
    {
        float range;
        memcpy(&range, buffer + sizeCount, sizeof(range));
        sizeCount = sizeCount + sizeof(range);
        beams.range.append(range);
    }
    memcpy(&beams.beamModel, buffer + sizeCount, sizeof(beams.beamModel));
    sizeCount = sizeCount + sizeof(beams.beamModel);
}
void BATHYFORMAT::_fileReadEQLAng(char *buffer, EQLANG &beamangle, int &sizeCount)
{
    memcpy(&beamangle.angleFirst, buffer + sizeCount, sizeof(beamangle.angleFirst));
    sizeCount = sizeCount + sizeof(beamangle.angleFirst);
    memcpy(&beamangle.angleLast, buffer + sizeCount, sizeof(beamangle.angleLast));
    sizeCount = sizeCount + sizeof(beamangle.angleLast);
}

void BATHYFORMAT::_fileReadEQLDis(char *buffer, EQLDIS &beamdis, int &sizeCount, unsigned short &points)
{
    for (int i = 0; i < points; i++)
    {
        float angle;
        memcpy(&angle, buffer + sizeCount, sizeof(angle));
        sizeCount = sizeCount + sizeof(angle);
        beamdis.angles.append(angle);
    }
}
void BATHYFORMAT::parseBathyDataToMBStruct(QByteArray datagram, MBBATHYDATA &Sodabathydata, PINGTIME &pingtime)
{
    char  at1[8];
    char  at2[8];
    char *buffer  = &at1[0];
    char *xbuffer = &at2[0];
    buffer        = datagram.data();
    int sizeCount = 12;
    //-------------------------------------H0_Section--------------------------------------------//
    // u16  H0_SectionName;                // 'H0'=12360
    sizeCount = sizeCount + 2;
    // u16  H0_SectionSize;                // [bytes] size of this entire section
    sizeCount = sizeCount + 2;
    // char   H0_ModelNumber[12];          // example "2024", unused chars are nulls
    sizeCount = sizeCount + 12;
    // char   H0_SerialNumber[12];         // example "100017", unused chars are nulls
    sizeCount = sizeCount + 12;
    // [seconds] ping 时间整数部分 相对于 0000 hours 1-Jan-1970, integer part//
    u32 H0_TimeSeconds;
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_TimeSeconds, xbuffer, 4);
    sizeCount           = sizeCount + 4;
    quint64 TimeSeconds = 0;
    TimeSeconds         = H0_TimeSeconds;
    // [nanoseconds] ping 时间小数部分 相对于 to 0000 hours 1-Jan-1970, fraction part//
    u32 H0_TimeNanoseconds;
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_TimeNanoseconds, xbuffer, 4);
    sizeCount = sizeCount + 4;
    qint64 milliseconds;
    milliseconds       = (H0_TimeNanoseconds / 1000000) + TimeSeconds * 1000;
    QDateTime datetime = QDateTime::fromMSecsSinceEpoch(milliseconds);
    //    inparse<<"H0_TimeSeconds:"<<H0_TimeSeconds<<'\n';
    //    inparse<<"H0_TimeNanoseconds:"<<H0_TimeNanoseconds<<'\n';
    //    inparse<<"milliseconds:"<<milliseconds<<'\n';
    datetime         = datetime.toUTC();
    QString date_Day = datetime.toString("yyyyMMdd");
    QString date_Sec = datetime.toString("hhmmss.zzz");
    int     date1    = date_Day.toInt();
    double  date2    = date_Sec.toDouble();
    //    inparse<<"date_Day:"<<date_Day<<'\n';
    //    inparse<<"date1:"<<date1<<'\n';
    //    inparse<<"date_Sec:"<<date_Sec<<'\n';
    //    inparse<<"date2:"<<date2<<'\n';
    QString hour, minute, secondf;
    hour            = datetime.toString("h");
    minute          = datetime.toString("m");
    secondf         = datetime.toString("s.zzz");
    u16   hhour     = hour.toUShort();
    u16   mminute   = minute.toUShort();
    float ssecond   = secondf.toFloat();
    pingtime.hour   = hhour;
    pingtime.minute = mminute;
    pingtime.second = ssecond;
    //    inparse<<"hhour:"<<hhour<<'\n';
    //    inparse<<"mminute:"<<mminute<<'\n';
    //    inparse<<"ssecond:"<<ssecond<<'\n';

    //  pingnums since power-up or reboot  //
    u32 H0_PingNumber;
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_PingNumber, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_PingNumber:"<<H0_PingNumber<<'\n';
    // [seconds] time between most recent two pings//
    float H0_PingPeriod;
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_PingPeriod, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_PingPeriod:"<<H0_PingPeriod<<'\n';

    float H0_SoundSpeed;   // [meters per second]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_SoundSpeed, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_SoundSpeed:"<<H0_SoundSpeed<<'\n';

    float H0_Frequency;   // [hertz] sonar center frequency
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_Frequency, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_Frequency:"<<H0_Frequency<<'\n';

    float H0_TxPower;   // [dB re 1 uPa at 1 meter]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_TxPower, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_TxPower:"<<H0_TxPower<<'\n';
    MULTIHEADER headerdatatemp;
    headerdatatemp.headerCount = 1;
    headerdatatemp.date        = date1;
    headerdatatemp.time        = date2;
    headerdatatemp.pingNum     = H0_PingNumber;
    headerdatatemp.pingPeriod  = H0_PingPeriod;
    headerdatatemp.soundSpeed  = H0_SoundSpeed;
    headerdatatemp.frequency   = H0_Frequency;
    headerdatatemp.txPower     = H0_TxPower;

    float H0_TxPulseWidth;   // [seconds]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_TxPulseWidth, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_TxPulseWidth:"<<H0_TxPulseWidth<<'\n';

    float H0_TxBeamwidthVert;   // [radians]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_TxBeamwidthVert, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_TxBeamwidthVert:"<<H0_TxBeamwidthVert<<'\n';

    float H0_TxBeamwidthHoriz;   // [radians]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_TxBeamwidthHoriz, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_TxBeamwidthHoriz:"<<H0_TxBeamwidthHoriz<<'\n';

    float H0_TxSteeringVert;   // [radians]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_TxSteeringVert, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_TxSteeringVert:"<<H0_TxSteeringVert<<'\n';

    float H0_TxSteeringHoriz;   // [radians]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_TxSteeringHoriz, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_TxSteeringHoriz:"<<H0_TxSteeringHoriz<<'\n';

    // u32  H0_TxMiscInfo;               // to be determined
    sizeCount = sizeCount + 4;

    float H0_RxBandwidth;   // [hertz]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_RxBandwidth, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_RxBandwidth:"<<H0_RxBandwidth<<'\n';
    // float  H0_RxSampleRate;             // [hertz] sample rate of data acquisition and signal processing
    sizeCount = sizeCount + 4;

    float H0_RxRange;   // [seconds two-way]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_RxRange, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_RxRange:"<<H0_RxRange<<'\n';
    float H0_RxGain;   // [dB re 1 uPa]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_RxGain, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_RxGain:"<<H0_RxGain<<'\n';
    float H0_RxSpreading;   // [dB (times log range in meters)]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_RxSpreading, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_RxSpreading:"<<H0_RxSpreading<<'\n';
    float H0_RxAbsorption;   // [dB per kilometer]
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&H0_RxAbsorption, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"H0_RxAbsorption:"<<H0_RxAbsorption<<'\n';

    // float  H0_RxMountTilt;              // [radians]
    sizeCount = sizeCount + 4;
    // u32  H0_RxMiscInfo;               // to be determined
    sizeCount = sizeCount + 4;
    //u16  H0_reserved;                 // reserved for future use
    sizeCount = sizeCount + 2;
    MULTISECTOR sectordatatemp;
    sectordatatemp.sectorCount      = 1;
    sectordatatemp.txPulseWidth     = H0_TxPulseWidth;
    sectordatatemp.txbeamWidthVert  = H0_TxBeamwidthVert;
    sectordatatemp.txBeamWidthHoriz = H0_TxBeamwidthHoriz;
    sectordatatemp.txSteeringVert   = H0_TxSteeringVert;
    sectordatatemp.txSteeringHoriz  = H0_TxSteeringHoriz;
    sectordatatemp.rxBandWidth      = H0_RxBandwidth;
    sectordatatemp.rxGain           = H0_RxGain;
    sectordatatemp.rxSpreading      = H0_RxSpreading;
    sectordatatemp.rxAbsorption     = H0_RxAbsorption;
    u16 H0_Points;   // number of bathy points
    byteChange(xbuffer, buffer + sizeCount, 2);
    memcpy(&H0_Points, xbuffer, 2);
    sizeCount = sizeCount + 2;
    //    inparse<<"H0_Points:"<<H0_Points<<'\n';
    //-----------/***      R0_Section    ***/------------//
    u16 R0_SectionName;   // 'R0'=12370
    byteChange(xbuffer, buffer + sizeCount, 2);
    memcpy(&R0_SectionName, xbuffer, 2);
    sizeCount = sizeCount + 2;
    u16 R0_SectionSize;   // [bytes] size of this entire section
    byteChange(xbuffer, buffer + sizeCount, 2);
    memcpy(&R0_SectionSize, xbuffer, 2);
    sizeCount = sizeCount + 2;

    float R0_ScalingFactor;
    byteChange(xbuffer, buffer + sizeCount, 4);
    memcpy(&R0_ScalingFactor, xbuffer, 4);
    sizeCount = sizeCount + 4;
    //    inparse<<"R0_ScalingFactor:"<<R0_ScalingFactor<<'\n';

    QVector<float> R0_Range;   // [seconds two-way] = R0_Range * R0_ScalingFactor
    for (int i = 0; i < H0_Points; i++)
    {
        byteChange(xbuffer, buffer + sizeCount, 2);
        u16 _range = 0;
        memcpy(&_range, xbuffer, 2);
        float temprange = _range * R0_ScalingFactor;
        R0_Range.append(temprange);
        sizeCount = sizeCount + 2;
        //    inparse<<"temprange:"<<temprange<<'\n';
    }
    u16 temp;
    temp = H0_Points & 1;
    if (temp != 0)
    {
        //                u16  R0_unused;    // ensure 32-bit section size
        sizeCount = sizeCount + 2;
    }
    //----------------等角模式--------------------//
    BEAMSINFO beamstemp;
    EQLANG    eqlAngtemp;
    EQLDIS    eqldistemp;

    beamstemp.points = H0_Points;
    for (int i = 0; i < H0_Points; i++)
    {
        beamstemp.range.append(R0_Range.at(i));
    }
    u16 sectionpart;
    memcpy(&sectionpart, buffer + sizeCount, 2);
    sizeCount = sizeCount + 2;
    if (sectionpart == 12353)
    { /***      A0_Section  'A0'=12353  ***/
        //qDebug()<<"A0_SectionName:"<<sectionpart;

        //                u16  A0_SectionSize;              // [bytes] size of this entire section
        //bathyObject.byteChange(xbuffer,buffer+sizeCount,2);
        //                memcpy(&A0_SectionSize,xbuffer,2);
        sizeCount = sizeCount + 2;
        //qDebug()<<"A0_SectionSize:"<<A0_SectionSize;
        float A0_AngleFirst;   // [radians] angle of first (port side) bathy point, relative to array centerline, AngleFirst < AngleLast
        byteChange(xbuffer, buffer + sizeCount, 4);
        memcpy(&A0_AngleFirst, xbuffer, 4);
        sizeCount = sizeCount + 4;
        //    inparse<<"A0_AngleFirst:"<<A0_AngleFirst<<'\n';
        float A0_AngleLast;   // [radians] angle of last (starboard side) bathy point
        byteChange(xbuffer, buffer + sizeCount, 4);
        memcpy(&A0_AngleLast, xbuffer, 4);
        sizeCount = sizeCount + 4;
        //    inparse<<"A0_AngleLast:"<<A0_AngleLast<<'\n';

        //                float  A0_MoreInfo[6];              // several to-be-determined values such as attitude info to assist the GUI display
        for (int i = 0; i < 6; i++)
        {
            //bathyObject.byteChange(xbuffer,buffer+sizeCount,4);
            //                    memcpy(&A0_MoreInfo[i],xbuffer,4);
            sizeCount = sizeCount + 4;
            //qDebug()<<"A0_MoreInfo[i]:"<<A0_MoreInfo[i];
        }
        beamstemp.beamModel   = vsrxtf::EQUALANGLE;
        eqlAngtemp.angleFirst = A0_AngleFirst;
        eqlAngtemp.angleLast  = A0_AngleLast;
    }
    else if (sectionpart == 12865)
    { /***      A2_Section   'A2'=12865 ***/
        //    inparse<<"A2_SectionName:"<<sectionpart<<'\n';

        u16 A2_SectionSize;   // [bytes] size of this entire section
        byteChange(xbuffer, buffer + sizeCount, 2);
        memcpy(&A2_SectionSize, xbuffer, 2);
        sizeCount = sizeCount + 2;
        //    inparse<<"A2_SectionSize:"<<A2_SectionSize<<'\n';
        float A2_AngleFirst;   // [radians] angle of first (port side) bathy point, relative to array centerline, AngleFirst < AngleLast
        byteChange(xbuffer, buffer + sizeCount, 4);
        memcpy(&A2_AngleFirst, xbuffer, 4);
        sizeCount = sizeCount + 4;
        //    inparse<<"A2_AngleFirst:"<<A2_AngleFirst<<'\n';
        float A2_ScalingFactor;
        byteChange(xbuffer, buffer + sizeCount, 4);
        memcpy(&A2_ScalingFactor, xbuffer, 4);
        sizeCount = sizeCount + 4;
        //    inparse<<"A2_ScalingFactor:"<<A2_ScalingFactor<<'\n';

        //                float  A2_MoreInfo[6];              // several to-be-determined values such as attitude info to assist the GUI display
        for (int i = 0; i < 6; i++)
        {
            //bathyObject.byteChange(xbuffer,buffer+sizeCount,4);
            //                    memcpy(&A2_MoreInfo[i],xbuffer,4);
            sizeCount = sizeCount + 4;
            //qDebug()<<"A2_MoreInfo[i]:"<<A2_MoreInfo[i];
        }

        //-------A2_AngleStep----- [radians] angle[n] = (32-bit sum of A2_AngleStep[0] through A2_AngleStep[n]) * A2_ScalingFactor
        int anglestepSum = 0;
        for (int i = 0; i < H0_Points; i++)
        {
            byteChange(xbuffer, buffer + sizeCount, 2);
            u16 _angleStep = 0;
            memcpy(&_angleStep, xbuffer, 2);
            sizeCount       = sizeCount + 2;
            anglestepSum    = anglestepSum + _angleStep;
            float anglereal = A2_AngleFirst + anglestepSum * A2_ScalingFactor;
            //    inparse<<"anglereal["<<i<<"]:"<<anglereal<<'\n';
            eqldistemp.angles.append(anglereal);
        }

        u16 temp1;
        temp1 = H0_Points & 1;
        if (temp1 != 0)
        {
            //                    u16  A2_unused;    // ensure 32-bit section size
            //bathyObject.byteChange(xbuffer,buffer+sizeCount,2);
            //                    memcpy(&A2_unused,xbuffer,2);
            sizeCount = sizeCount + 2;
            //qDebug()<<"A2_unused:"<<A2_unused;
        }

        beamstemp.beamModel = vsrxtf::EQUALDISTANCE;
    }

    /***      I1_Section    ***/
    memcpy(&sectionpart, buffer + sizeCount, 2);
    sizeCount                       = sizeCount + 2;
    unsigned char intensityFlagtemp = 0;
    INTENSITY     intensitytemp;
    if (sectionpart == 12617)   // 'I1'=12617
    {
        intensityFlagtemp = 1;

        //qDebug()<<"I1_SectionName:"<<sectionpart;

        //                u16  I1_SectionSize;              // [bytes] size of this entire section
        //bathyObject.byteChange(xbuffer,buffer+sizeCount,2);
        //                memcpy(&I1_SectionSize,xbuffer,2);
        sizeCount = sizeCount + 2;
        //qDebug()<<"I1_SectionSize:"<<I1_SectionSize;

        float I1_ScalingFactor;
        byteChange(xbuffer, buffer + sizeCount, 4);
        memcpy(&I1_ScalingFactor, xbuffer, 4);
        sizeCount = sizeCount + 4;
        //    inparse<<"I1_ScalingFactor:"<<I1_ScalingFactor<<'\n';

        //                I1_Intensity; // [micropascals] intensity[n] = I1_Intensity[n]) * I1_ScalingFactor
        for (int i = 0; i < H0_Points; i++)
        {
            byteChange(xbuffer, buffer + sizeCount, 2);
            u16 I1_Intensity = 0;
            memcpy(&I1_Intensity, xbuffer, 2);
            float _tensity = I1_Intensity * I1_ScalingFactor;
            intensitytemp.intensity.append(_tensity);
            sizeCount = sizeCount + 2;
            //    inparse<<"_tensity["<<i<<"]:"<<_tensity<<'\n';
        }
        u16 temp2;
        temp2 = H0_Points & 1;
        if (temp2 != 0)
        {
            //                    u16  I1_unused;    // ensure 32-bit section size
            //                    byteChange(xbuffer,buffer+sizeCount,2);
            //                    memcpy(&I1_unused,xbuffer,2);
            sizeCount = sizeCount + 2;
            //qDebug()<<"I1_unused:"<<I1_unused;
        }
    }

    /***      G0_Section    ***/
    //            u16  G0_SectionName;              // 'G0'
    //            memcpy(&G0_SectionName,buffer+sizeCount,2);
    sizeCount = sizeCount + 2;
    //qDebug()<<"G0_SectionName:"<<G0_SectionName;
    //            u16  G0_SectionSize;              // [bytes] size of this entire section
    //            byteChange(xbuffer,buffer+sizeCount,2);
    //            memcpy(&G0_SectionSize,xbuffer,2);
    sizeCount = sizeCount + 2;
    //qDebug()<<"G0_SectionSize:"<<G0_SectionSize;
    //            float  G0_DepthGateMin;             // [seconds two-way]
    //            byteChange(xbuffer,buffer+sizeCount,4);
    //            memcpy(&G0_DepthGateMin,xbuffer,4);
    sizeCount = sizeCount + 4;
    //qDebug()<<"G0_DepthGateMin:"<<G0_DepthGateMin;
    //            float  G0_DepthGateMax;             // [seconds two-way]
    //            byteChange(xbuffer,buffer+sizeCount,4);
    //            memcpy(&G0_DepthGateMax,xbuffer,4);
    sizeCount = sizeCount + 4;
    //qDebug()<<"G0_DepthGateMax:"<<G0_DepthGateMax;
    //            float  G0_DepthGateSlope;           // [radians]
    //            byteChange(xbuffer,buffer+sizeCount,4);
    //            memcpy(&G0_DepthGateSlope,xbuffer,4);
    sizeCount = sizeCount + 4;
    //qDebug()<<"G0_DepthGateSlope:"<<G0_DepthGateSlope;
    /***      G1_Section    // 'G1'***/
    memcpy(&sectionpart, buffer + sizeCount, 2);
    sizeCount = sizeCount + 2;
    if (sectionpart == 12615)
    {
        //qDebug()<<"G1_SectionName:"<<sectionpart;

        //                u16  G1_SectionSize;              // [bytes] size of this entire section
        //                byteChange(xbuffer,buffer+sizeCount,2);
        //                memcpy(&G1_SectionSize,xbuffer,2);
        sizeCount = sizeCount + 2;
        //qDebug()<<"G1_SectionSize:"<<G1_SectionSize;
        //                float  G1_ScalingFactor;
        //                byteChange(xbuffer,buffer+sizeCount,4);
        //                memcpy(&G1_ScalingFactor,xbuffer,4);
        sizeCount = sizeCount + 4;
        //qDebug()<<"G1_ScalingFactor:"<<G1_ScalingFactor;

        //                QVector <GATE>           G1_Gate;
        for (int i = 0; i < H0_Points; i++)
        {
            //                    GATE    tempGate;
            //                    memcpy(&tempGate.RangeMin,buffer+sizeCount,1);
            sizeCount = sizeCount + 1;
            //                    memcpy(&tempGate.RangeMax,buffer+sizeCount,1);
            sizeCount = sizeCount + 1;
            //                    G1_Gate.append(tempGate);
            //qDebug()<<"tempGate:"<<tempGate.RangeMin<<","<<tempGate.RangeMax;
        }

        u16 temp2;
        temp2 = H0_Points & 1;
        if (temp2 != 0)
        {
            //                    u16  G1_unused;    // ensure 32-bit section size
            //                    byteChange(xbuffer,buffer+sizeCount,2);
            //                    memcpy(&G1_unused,xbuffer,2);
            sizeCount = sizeCount + 2;
            //qDebug()<<"G1_unused:"<<G1_unused;
        }
    }

    /***      Q0_Section    ***/
    //            u16  Q0_SectionName;              // 'Q0' quality, 4-bit
    //            memcpy(&Q0_SectionName,buffer+sizeCount,2);
    sizeCount = sizeCount + 2;
    //qDebug()<<"Q0_SectionName:"<<Q0_SectionName;
    //            u16  Q0_SectionSize;              // [bytes] size of this entire section
    //            byteChange(xbuffer,buffer+sizeCount,2);
    //            memcpy(&Q0_SectionSize,xbuffer,2);
    sizeCount = sizeCount + 2;
    //qDebug()<<"Q0_SectionSize:"<<Q0_SectionSize;
    u16 groups;
    groups = (H0_Points + 7) / 8;

    //            QVector <u32>           Q0_Quality;// 8 groups of 4 flags bits (phase detect, magnitude detect, colinearity, brightness), packed left-to-right
    for (int i = 0; i < groups; i++)
    {
        //                byteChange(xbuffer,buffer+sizeCount,4);
        //                u32   _Quality=0;
        //                memcpy(&_Quality,xbuffer,4);
        //                Q0_Quality.append(_Quality);
        sizeCount = sizeCount + 4;
        //qDebug()<<"_Quality["<<i<<"]:"<<_Quality;
    }
    Sodabathydata.packetName = BATHFIELD;
    Sodabathydata.packetSize = 2139;
    Sodabathydata.headNum    = 1;
    for (int m = 0; m < Sodabathydata.headNum; m++)
    {

        MULTIHEADER headerdata;
        headerdata.headerCount = 1 + m;
        headerdata.date        = headerdatatemp.date;
        headerdata.time        = headerdatatemp.time;
        headerdata.pingNum     = headerdatatemp.pingNum;
        headerdata.pingPeriod  = headerdatatemp.pingPeriod;
        headerdata.soundSpeed  = headerdatatemp.soundSpeed;
        headerdata.frequency   = headerdatatemp.frequency;
        headerdata.txPower     = headerdatatemp.txPower;
        headerdata.swathNum    = 1;
        for (int j = 0; j < headerdata.swathNum; j++)
        {

            MULTISWATH swathdata;
            swathdata.swathCount    = 1 + j;
            swathdata.sectorNum     = 1;
            swathdata.intensityFlag = intensityFlagtemp;
            for (int i = 0; i < swathdata.sectorNum; i++)
            {

                MULTISECTOR sectordata;
                sectordata.sectorCount      = i + 1;
                sectordata.txPulseWidth     = sectordatatemp.txPulseWidth;
                sectordata.txbeamWidthVert  = sectordatatemp.txbeamWidthVert;
                sectordata.txBeamWidthHoriz = sectordatatemp.txBeamWidthHoriz;
                sectordata.txSteeringVert   = sectordatatemp.txSteeringVert;
                sectordata.txSteeringHoriz  = sectordatatemp.txSteeringHoriz;
                sectordata.rxBandWidth      = sectordatatemp.rxBandWidth;
                sectordata.rxGain           = sectordatatemp.rxGain;
                sectordata.rxSpreading      = sectordatatemp.rxSpreading;
                sectordata.rxAbsorption     = sectordatatemp.rxAbsorption;

                swathdata.mltsector.append(sectordata);
            }

            swathdata.beams.beamModel = beamstemp.beamModel;
            swathdata.beams.points    = beamstemp.points;
            for (int qq = 0; qq < beamstemp.points; qq++)
            {
                swathdata.beams.range.append(beamstemp.range.at(qq));
            }
            if (swathdata.beams.beamModel == vsrxtf::EQUALANGLE)
            {
                swathdata.eqlAng.angleFirst = eqlAngtemp.angleFirst;
                swathdata.eqlAng.angleLast  = eqlAngtemp.angleLast;
            }
            else
            {
                for (int ww = 0; ww < swathdata.beams.points; ww++)
                {
                    swathdata.eqldis.angles.append(eqldistemp.angles.at(ww));
                }
            }
            if (intensityFlagtemp != 0)
            {
                for (int ee = 0; ee < swathdata.beams.points; ee++)
                {
                    swathdata.intensity.intensity.append(intensitytemp.intensity.at(ee));
                }
            }
            headerdata.mltswath.append(swathdata);
        }
        Sodabathydata.mltihead.append(headerdata);
    }
    //--------------显示结构体中数据----------------//
    //    inbathy<<"Sodabathydata.packetName:"<<Sodabathydata.packetName<<'\n';
    //    inbathy<<"Sodabathydata.packetSize:"<<Sodabathydata.packetSize<<'\n';
    //    inbathy<<"Sodabathydata.headNum:"<<Sodabathydata.headNum<<'\n';
    //                for(int m=0;m<Sodabathydata.headNum;m++)
    //                {
    //    inbathy<<"headerCount:"<<Sodabathydata.mltihead.at(m).headerCount<<'\n';
    //    inbathy<<"date:"<<Sodabathydata.mltihead.at(m).date<<'\n';
    //    inbathy<<"time:"<<Sodabathydata.mltihead.at(m).time<<'\n';
    //    inbathy<<"pingNum:"<<Sodabathydata.mltihead.at(m).pingNum<<'\n';
    //    inbathy<<"pingPeriod:"<<Sodabathydata.mltihead.at(m).pingPeriod<<'\n';
    //    inbathy<<"soundSpeed:"<<Sodabathydata.mltihead.at(m).soundSpeed<<'\n';
    //    inbathy<<"frequency:"<<Sodabathydata.mltihead.at(m).frequency<<'\n';
    //    inbathy<<"txPowe:"<<Sodabathydata.mltihead.at(m).txPower<<'\n';
    //    inbathy<<"swathNum:"<<Sodabathydata.mltihead.at(m).swathNum<<'\n';

    //                    for(int j=0;j<Sodabathydata.mltihead.at(m).swathNum;j++)
    //                    {
    //    inbathy<<"swathCount:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).swathCount<<'\n';
    //    inbathy<<"sectorNum:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).sectorNum<<'\n';
    //    inbathy<<"intensityFlag:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).intensityFlag<<'\n';

    //                        for(int i=0;i<Sodabathydata.mltihead.at(m).mltswath.at(j).sectorNum;i++)
    //                        {
    //    inbathy<<"sectorCount:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).mltsector.at(i).sectorCount<<'\n';
    //    inbathy<<"txPulseWidth:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).mltsector.at(i).txPulseWidth<<'\n';
    //    inbathy<<"txbeamWidthVert:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).mltsector.at(i).txbeamWidthVert<<'\n';
    //    inbathy<<"txBeamWidthHoriz:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).mltsector.at(i).txBeamWidthHoriz<<'\n';
    //    inbathy<<"txSteeringVert:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).mltsector.at(i).txSteeringVert<<'\n';
    //    inbathy<<"txSteeringHoriz:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).mltsector.at(i).txSteeringHoriz<<'\n';
    //    inbathy<<"rxBandWidth:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).mltsector.at(i).rxBandWidth<<'\n';
    //    inbathy<<"rxGain:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).mltsector.at(i).rxGain<<'\n';
    //    inbathy<<"rxSpreading:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).mltsector.at(i).rxSpreading<<'\n';
    //    inbathy<<"rxAbsorption:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).mltsector.at(i).rxAbsorption<<'\n';

    //                        }
    //    inbathy<<"beamModel:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).beams.beamModel<<'\n';
    //    inbathy<<"points:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).beams.points<<'\n';

    //                        for(int qq=0;qq<Sodabathydata.mltihead.at(m).mltswath.at(j).beams.points;qq++)
    //                        {
    //    inbathy<<"range.at["<<qq<<"]:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).beams.range.at(qq)<<'\n';
    //                        }
    //                        if(Sodabathydata.mltihead.at(m).mltswath.at(j).beams.beamModel==vsrxtf::EQUALANGLE)
    //                        {
    //    inbathy<<"eqlAng.angleFirst:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).eqlAng.angleFirst<<'\n';
    //    inbathy<<"eqlAng.angleLast:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).eqlAng.angleLast<<'\n';

    //                        } else {
    //                            for(int ww =0;ww<Sodabathydata.mltihead.at(m).mltswath.at(j).beams.points;ww++)
    //                            {
    //    inbathy<<"eqldis.angles.at["<<ww<<"]:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).eqldis.angles.at(ww)<<'\n';
    //                            }
    //                        }
    //                        if(Sodabathydata.mltihead.at(m).mltswath.at(j).intensityFlag!=0)
    //                        {
    //                            for(int ee=0;ee<Sodabathydata.mltihead.at(m).mltswath.at(j).beams.points;ee++)
    //                            {
    //    inbathy<<"intensity.at["<<ee<<"]:"<<Sodabathydata.mltihead.at(m).mltswath.at(j).intensity.intensity.at(ee)<<'\n';
    //                            }
    //                        }
    //                    }
    //                }
}

void BATHYFORMAT::parse7kDataToMBStruct(PINGINFO p, MBBATHYDATA &Sodabathydata)
{
    Sodabathydata.packetName = BATHFIELD;
    Sodabathydata.packetSize = 2823;
    Sodabathydata.headNum    = 1;
    for (int m = 0; m < Sodabathydata.headNum; m++)
    {
        MULTIHEADER headerdata;
        headerdata.headerCount = 1 + m;

        QDate date(p.K7Time.mYear, 1, 1);
        QDate date1 = date.addDays(p.K7Time.mDay - 1);
        int   ms    = (p.K7Time.mSeconds - int(p.K7Time.mSeconds)) * 1000;
        QTime time(p.K7Time.mHours, p.K7Time.mMinutes, int(p.K7Time.mSeconds), ms);
        headerdata.date       = date1.toString("yyyyMMdd").toInt();       // yyyymmdd
        headerdata.time       = time.toString("HHmmss.zzz").toDouble();   // hhmmss.zzz
        headerdata.pingNum    = p.K7006PingNum;
        headerdata.pingPeriod = p.K7000SonarSettings.mPingPeriod;
        headerdata.soundSpeed = p.K7006Bathmetry.velocity;
        headerdata.frequency  = p.K7000SonarSettings.mFrequency;
        headerdata.txPower    = p.K7000SonarSettings.mPowerSelection;
        headerdata.swathNum   = 1;
        for (int j = 0; j < headerdata.swathNum; j++)
        {
            MULTISWATH swathdata;
            swathdata.swathCount    = 1 + j;
            swathdata.sectorNum     = 1;
            swathdata.intensityFlag = 1;
            for (int i = 0; i < swathdata.sectorNum; i++)
            {
                MULTISECTOR sectordata;
                sectordata.sectorCount      = i + 1;
                sectordata.txPulseWidth     = p.K7000SonarSettings.mTxPulseWidth;
                sectordata.txbeamWidthVert  = p.K7000SonarSettings.mProjectorBeamSteeringAngleHorizontal;
                sectordata.txBeamWidthHoriz = p.K7000SonarSettings.mProjectorBeamSteeringAngleVertical;
                sectordata.txSteeringVert   = p.K7000SonarSettings.mProjectorBeamSteeringAngleHorizontal;
                sectordata.txSteeringHoriz  = p.K7000SonarSettings.mProjectorBeamSteeringAngleVertical;
                sectordata.rxBandWidth      = p.K7000SonarSettings.mReceiverBandwidth;
                sectordata.rxGain           = p.K7000SonarSettings.mGainSelection;
                sectordata.rxSpreading      = p.K7000SonarSettings.mSpreading;
                sectordata.rxAbsorption     = p.K7000SonarSettings.mAbsorption;
                swathdata.mltsector.append(sectordata);
            }
            swathdata.beams.beamModel = vsrxtf::EQUALDISTANCE;
            swathdata.beams.points    = p.K7PointNum;
            for (int qq = 0; qq < p.K7PointNum; qq++)
            {
                swathdata.beams.range.append(p.K7006Bathmetry.range[qq]);
            }

            //            if(swathdata.beams.beamModel==vsrxtf::EQUALANGLE)
            //            {
            //                swathdata.eqlAng.angleFirst = eqlAngtemp.angleFirst;
            //                swathdata.eqlAng.angleLast = eqlAngtemp.angleLast;
            //            }
            //            else
            {
                for (int ww = 0; ww < swathdata.beams.points; ww++)
                {
                    swathdata.eqldis.angles.append(p.K7004Geometry.dir_ang[ww]);
                }
            }
            for (int ee = 0; ee < swathdata.beams.points; ee++)
            {
                swathdata.intensity.intensity.append(p.K7006Bathmetry.intensite.at(ee));
            }
            headerdata.mltswath.append(swathdata);
        }
        Sodabathydata.mltihead.append(headerdata);
    }
}

void BATHYFORMAT::computeVes(MBBATHYDATA &bathy, VESSEL &ves)
{
    //    float dir_angle[512];
    for (int t = 0; t < bathy.headNum; t++)
    {
        for (int s = 0; s < bathy.mltihead.at(t).swathNum; s++)
        {
            if (bathy.mltihead.at(t).mltswath.at(s).beams.beamModel == vsrxtf::EQUALANGLE)
            {
                for (int i = 0; i < bathy.mltihead.at(t).mltswath.at(s).beams.points; i++)
                {
                    float dir_angle = (bathy.mltihead.at(t).mltswath.at(s).eqlAng.angleFirst + i * (bathy.mltihead.at(t).mltswath.at(s).eqlAng.angleLast - bathy.mltihead.at(t).mltswath.at(s).eqlAng.angleFirst) / (bathy.mltihead.at(t).mltswath.at(s).beams.points - 1));
                    ves.X[i]        = bathy.mltihead.at(t).mltswath.at(s).beams.range.at(i) * bathy.mltihead.at(t).soundSpeed * sin(dir_angle) * 0.5;
                    ves.Y[i]        = 0;
                    ves.Z[i]        = bathy.mltihead.at(t).mltswath.at(s).beams.range.at(i) * bathy.mltihead.at(t).soundSpeed * cos(dir_angle) * (-0.5);
                    //outvel<<ping.ppingtime.hour<<' '<<ping.ppingtime.minute<<' '<<ping.ppingtime.second<<' '<<vel.X[i]<<' '<<vel.Y[i]<<' '<<vel.Z[i]<<'\n';
                }
            }
            else
            {
                for (int i = 0; i < bathy.mltihead.at(t).mltswath.at(s).beams.points; i++)
                {
                    float dir_angle = bathy.mltihead.at(t).mltswath.at(s).eqldis.angles.at(i);
                    ves.X[i]        = bathy.mltihead.at(t).mltswath.at(s).beams.range.at(i) * bathy.mltihead.at(t).soundSpeed * sin(dir_angle) * 0.5;
                    ves.Y[i]        = 0;
                    ves.Z[i]        = bathy.mltihead.at(t).mltswath.at(s).beams.range.at(i) * bathy.mltihead.at(t).soundSpeed * cos(dir_angle) * (-0.5);
                }
            }
        }
    }
}

void BATHYFORMAT::generateBathyViewdata(QByteArray &array, const SENDDATA &data, unsigned short &pt)
{
    array.append(reinterpret_cast<const char *>(&data.pingT.hour), sizeof(data.pingT.hour));
    array.append(reinterpret_cast<const char *>(&data.pingT.minute), sizeof(data.pingT.minute));
    array.append(reinterpret_cast<const char *>(&data.pingT.second), sizeof(data.pingT.second));
    for (int i = 0; i < pt; i++)
    {
        array.append(reinterpret_cast<const char *>(&data.X[i]), sizeof(data.X[i]));
        array.append(reinterpret_cast<const char *>(&data.Y[i]), sizeof(data.Y[i]));
        array.append(reinterpret_cast<const char *>(&data.Z[i]), sizeof(data.Z[i]));
    }
}

//--------------------解析采集软件保存数据--------------------------------//
void BATHYFORMAT::parseSaveDataFileHeader(MBFILEHEAD &headStruct, QFile &saveFile, int &prosize)
{
    QByteArray dataFileHead;
    char       at1      = 0;
    char *     buffer   = &at1;
    int        readSize = 0;

    unsigned short header_size;
    readSize     = sizeof(unsigned short);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&header_size, buffer, readSize);
    headStruct.header_size = header_size;
    prosize += readSize;

    unsigned char version_major;
    readSize     = sizeof(unsigned char);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&version_major, buffer, readSize);
    headStruct.version_major = version_major;
    prosize += readSize;

    unsigned char version_minor;
    readSize     = sizeof(unsigned char);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&version_minor, buffer, readSize);
    headStruct.version_minor = version_minor;
    prosize += readSize;

    char multibeam_sensor[16];
    dataFileHead = saveFile.read(16);
    buffer       = dataFileHead.data();
    memcpy(&multibeam_sensor, buffer, 16);
    //strcpy_s(headStruct.multibeam_sensor,multibeam_sensor);
    memcpy(&(headStruct.multibeam_sensor[0]), buffer, 16);
    prosize += 16;

    int day;
    readSize     = sizeof(int);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&day, buffer, readSize);
    headStruct.day = day;
    prosize += readSize;

    float headtilt;
    readSize     = sizeof(float);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&headtilt, buffer, readSize);
    headStruct.rxMountTilt = headtilt;
    prosize += readSize;

    double start_time;
    readSize     = sizeof(double);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&start_time, buffer, readSize);
    headStruct.start_time = start_time;
    prosize += readSize;

    unsigned char head_num;
    readSize     = sizeof(unsigned char);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&head_num, buffer, readSize);
    headStruct.head_num = head_num;
    prosize += readSize;

    unsigned char sector_num;
    readSize     = sizeof(unsigned char);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&sector_num, buffer, readSize);
    headStruct.sector_num = sector_num;
    prosize += readSize;

    unsigned char swath_num;
    readSize     = sizeof(unsigned char);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&swath_num, buffer, readSize);
    headStruct.swath_num = swath_num;
    prosize += readSize;

    float detaGPS_X;
    readSize     = sizeof(float);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&detaGPS_X, buffer, readSize);
    headStruct.detaGPS_X = detaGPS_X;
    prosize += readSize;

    float detaGPS_Y;
    readSize     = sizeof(float);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&detaGPS_Y, buffer, readSize);
    headStruct.detaGPS_Y = detaGPS_Y;
    prosize += readSize;

    float detaGPS_Z;
    readSize     = sizeof(float);
    dataFileHead = saveFile.read(readSize);
    buffer       = dataFileHead.data();
    memcpy(&detaGPS_Z, buffer, readSize);
    headStruct.detaGPS_Z = detaGPS_Z;
    prosize += readSize;

    for (int i = 0; i < head_num; i++)
    {
        TRANSDUCERALIG transdata;
        unsigned char  head_count;
        readSize     = sizeof(unsigned char);
        dataFileHead = saveFile.read(readSize);
        buffer       = dataFileHead.data();
        memcpy(&head_count, buffer, readSize);
        transdata.head_count = head_count;
        prosize += readSize;

        float angle_X;
        readSize     = sizeof(float);
        dataFileHead = saveFile.read(readSize);
        buffer       = dataFileHead.data();
        memcpy(&angle_X, buffer, readSize);
        transdata.angle_X = angle_X;
        prosize += readSize;

        float angle_Y;
        readSize     = sizeof(float);
        dataFileHead = saveFile.read(readSize);
        buffer       = dataFileHead.data();
        memcpy(&angle_Y, buffer, readSize);
        transdata.angle_Y = angle_Y;
        prosize += readSize;

        float angle_Z;
        readSize     = sizeof(float);
        dataFileHead = saveFile.read(readSize);
        buffer       = dataFileHead.data();
        memcpy(&angle_Z, buffer, readSize);
        transdata.angle_Z = angle_Z;
        prosize += readSize;

        float deta_X;
        readSize     = sizeof(float);
        dataFileHead = saveFile.read(readSize);
        buffer       = dataFileHead.data();
        memcpy(&deta_X, buffer, readSize);
        transdata.deta_X = deta_X;
        prosize += readSize;

        float deta_Y;
        readSize     = sizeof(float);
        dataFileHead = saveFile.read(readSize);
        buffer       = dataFileHead.data();
        memcpy(&deta_Y, buffer, readSize);
        transdata.deta_Y = deta_Y;
        prosize += readSize;

        float deta_Z;
        readSize     = sizeof(float);
        dataFileHead = saveFile.read(readSize);
        buffer       = dataFileHead.data();
        memcpy(&deta_Z, buffer, readSize);
        transdata.deta_Z = deta_Z;
        prosize += readSize;

        float draught;
        readSize     = sizeof(float);
        dataFileHead = saveFile.read(readSize);
        buffer       = dataFileHead.data();
        memcpy(&draught, buffer, readSize);
        transdata.draught = draught;
        prosize += readSize;

        headStruct.mbtrans.append(transdata);
    }
}
void BATHYFORMAT::parseSaveDataFilePingdata(MBBATHYDATA &Sodabathydata, QFile &saveFile, int &prosize)
{
    char       at11       = 0;
    char *     dataBuffer = &at11;
    QByteArray saveData;
    int        readSize = 0;

    unsigned int packetName;
    readSize   = sizeof(unsigned int);
    saveData   = saveFile.read(readSize);
    dataBuffer = saveData.data();
    memcpy(&packetName, dataBuffer, readSize);
    Sodabathydata.packetName = packetName;
    prosize += readSize;

    unsigned int packetSize;
    readSize   = sizeof(unsigned int);
    saveData   = saveFile.read(readSize);
    dataBuffer = saveData.data();
    memcpy(&packetSize, dataBuffer, readSize);
    Sodabathydata.packetSize = packetSize;
    prosize += readSize;

    unsigned char headNum;
    readSize   = sizeof(unsigned char);
    saveData   = saveFile.read(readSize);
    dataBuffer = saveData.data();
    memcpy(&headNum, dataBuffer, readSize);
    Sodabathydata.headNum = headNum;
    prosize += readSize;

    for (int i = 0; i < headNum; i++)
    {
        MULTIHEADER   tempHeader;
        unsigned char headerCount;
        readSize   = sizeof(unsigned char);
        saveData   = saveFile.read(readSize);
        dataBuffer = saveData.data();
        memcpy(&headerCount, dataBuffer, readSize);
        tempHeader.headerCount = headerCount;
        prosize += readSize;

        unsigned int date;
        readSize   = sizeof(unsigned int);
        saveData   = saveFile.read(readSize);
        dataBuffer = saveData.data();
        memcpy(&date, dataBuffer, readSize);
        tempHeader.date = date;
        prosize += readSize;

        double time;
        readSize   = sizeof(double);
        saveData   = saveFile.read(readSize);
        dataBuffer = saveData.data();
        memcpy(&time, dataBuffer, readSize);
        tempHeader.time = time;
        prosize += readSize;

        unsigned int pingNum;
        readSize   = sizeof(unsigned int);
        saveData   = saveFile.read(readSize);
        dataBuffer = saveData.data();
        memcpy(&pingNum, dataBuffer, readSize);
        tempHeader.pingNum = pingNum;
        prosize += readSize;

        float pingPeriod;
        readSize   = sizeof(float);
        saveData   = saveFile.read(readSize);
        dataBuffer = saveData.data();
        memcpy(&pingPeriod, dataBuffer, readSize);
        tempHeader.pingPeriod = pingPeriod;
        prosize += readSize;

        float soundSpeed;
        readSize   = sizeof(float);
        saveData   = saveFile.read(readSize);
        dataBuffer = saveData.data();
        memcpy(&soundSpeed, dataBuffer, readSize);
        tempHeader.soundSpeed = soundSpeed;
        prosize += readSize;

        float frequency;
        readSize   = sizeof(float);
        saveData   = saveFile.read(readSize);
        dataBuffer = saveData.data();
        memcpy(&frequency, dataBuffer, readSize);
        tempHeader.frequency = frequency;
        prosize += readSize;

        float txPower;
        readSize   = sizeof(float);
        saveData   = saveFile.read(readSize);
        dataBuffer = saveData.data();
        memcpy(&txPower, dataBuffer, readSize);
        tempHeader.txPower = txPower;
        prosize += readSize;

        unsigned char swathNum;
        readSize   = sizeof(unsigned char);
        saveData   = saveFile.read(readSize);
        dataBuffer = saveData.data();
        memcpy(&swathNum, dataBuffer, readSize);
        tempHeader.swathNum = swathNum;
        prosize += readSize;

        for (int j = 0; j < swathNum; j++)
        {
            MULTISWATH    tempSwath;
            unsigned char swathCount;
            readSize   = sizeof(unsigned char);
            saveData   = saveFile.read(readSize);
            dataBuffer = saveData.data();
            memcpy(&swathCount, dataBuffer, readSize);
            tempSwath.swathCount = swathCount;
            prosize += readSize;

            unsigned char sectorNum;
            readSize   = sizeof(unsigned char);
            saveData   = saveFile.read(readSize);
            dataBuffer = saveData.data();
            memcpy(&sectorNum, dataBuffer, readSize);
            tempSwath.sectorNum = sectorNum;
            prosize += readSize;

            unsigned char intensityFlag;
            readSize   = sizeof(unsigned char);
            saveData   = saveFile.read(readSize);
            dataBuffer = saveData.data();
            memcpy(&intensityFlag, dataBuffer, readSize);
            tempSwath.intensityFlag = intensityFlag;
            prosize += readSize;

            for (int k = 0; k < sectorNum; k++)
            {
                MULTISECTOR   tempSector;
                unsigned char sectorCount;
                readSize   = sizeof(unsigned char);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&sectorCount, dataBuffer, readSize);
                tempSector.sectorCount = sectorCount;
                prosize += readSize;

                float txPulseWidth;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&txPulseWidth, dataBuffer, readSize);
                tempSector.txPulseWidth = txPulseWidth;
                prosize += readSize;

                float txbeamWidthVert;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&txbeamWidthVert, dataBuffer, readSize);
                tempSector.txbeamWidthVert = txbeamWidthVert;
                prosize += readSize;

                float txBeamWidthHoriz;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&txBeamWidthHoriz, dataBuffer, readSize);
                tempSector.txBeamWidthHoriz = txBeamWidthHoriz;
                prosize += readSize;

                float txSteeringVert;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&txSteeringVert, dataBuffer, readSize);
                tempSector.txSteeringVert = txSteeringVert;
                prosize += readSize;

                float txSteeringHoriz;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&txSteeringHoriz, dataBuffer, readSize);
                tempSector.txSteeringHoriz = txSteeringHoriz;
                prosize += readSize;

                float rxBandWidth;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&rxBandWidth, dataBuffer, readSize);
                tempSector.rxBandWidth = rxBandWidth;
                prosize += readSize;

                float rxGain;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&rxGain, dataBuffer, readSize);
                tempSector.rxGain = rxGain;
                prosize += readSize;

                float rxSpreading;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&rxSpreading, dataBuffer, readSize);
                tempSector.rxSpreading = rxSpreading;
                prosize += readSize;

                float rxAbsorption;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&rxAbsorption, dataBuffer, readSize);
                tempSector.rxAbsorption = rxAbsorption;
                prosize += readSize;

                tempSwath.mltsector.append(tempSector);
            }

            unsigned short points;
            readSize   = sizeof(unsigned short);
            saveData   = saveFile.read(readSize);
            dataBuffer = saveData.data();
            memcpy(&points, dataBuffer, readSize);
            tempSwath.beams.points = points;
            prosize += readSize;

            for (int m = 0; m < points; m++)
            {
                float range;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&range, dataBuffer, readSize);
                tempSwath.beams.range.append(range);
                prosize += readSize;
            }

            unsigned short beamModel;
            readSize   = sizeof(unsigned short);
            saveData   = saveFile.read(readSize);
            dataBuffer = saveData.data();
            memcpy(&beamModel, dataBuffer, readSize);
            tempSwath.beams.beamModel = beamModel;
            prosize += readSize;

            if (beamModel == vsrxtf::EQUALANGLE)
            {
                float angleFirst;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&angleFirst, dataBuffer, readSize);
                tempSwath.eqlAng.angleFirst = angleFirst;
                prosize += readSize;

                float angleLast;
                readSize   = sizeof(float);
                saveData   = saveFile.read(readSize);
                dataBuffer = saveData.data();
                memcpy(&angleLast, dataBuffer, readSize);
                tempSwath.eqlAng.angleLast = angleLast;
                prosize += readSize;
            }
            else
            {
                for (int n = 0; n < tempSwath.beams.points; n++)
                {
                    float angles;
                    readSize   = sizeof(float);
                    saveData   = saveFile.read(readSize);
                    dataBuffer = saveData.data();
                    memcpy(&angles, dataBuffer, readSize);
                    prosize += readSize;

                    tempSwath.eqldis.angles.append(angles);
                }
            }

            if (intensityFlag == vsrxtf::INTENSITYDO)
            {
                for (int v = 0; v < tempSwath.beams.points; v++)
                {
                    float intensity;
                    readSize   = sizeof(float);
                    saveData   = saveFile.read(readSize);
                    dataBuffer = saveData.data();
                    memcpy(&intensity, dataBuffer, readSize);
                    tempSwath.intensity.intensity.append(intensity);
                    prosize += readSize;
                }
            }
            tempHeader.mltswath.append(tempSwath);
        }
        Sodabathydata.mltihead.append(tempHeader);
    }
}
void BATHYFORMAT::producePointClouds(MBBATHYDATA &Sodabathydata, MBPING &pingPoints)
{
    double tempTime     = Sodabathydata.mltihead.at(0).time;
    int    hour         = 0;
    int    minute       = 0;
    float  sec          = 0;
    hour                = tempTime / 10000;
    minute              = (tempTime - hour * 10000) / 100;
    sec                 = tempTime - hour * 10000 - minute * 100;
    pingPoints.secOfDay = hour * 3600 + minute * 60 + sec;

    pingPoints.pingNum = Sodabathydata.mltihead.at(0).pingNum;
    int points         = Sodabathydata.mltihead.at(0).mltswath.at(0).beams.points;
    pingPoints.points  = points;
    for (int i = 0; i < points; i++)
    {
        pingPoints.range[i] = Sodabathydata.mltihead.at(0).mltswath.at(0).beams.range.at(i) * Sodabathydata.mltihead.at(0).soundSpeed / 2.0;
    }

    if (Sodabathydata.mltihead.at(0).mltswath.at(0).beams.beamModel == vsrxtf::EQUALDISTANCE)
    {
        for (int i = 0; i < points; i++)
        {
            pingPoints.angle[i] = Sodabathydata.mltihead.at(0).mltswath.at(0).eqldis.angles.at(i);
        }
    }
    else
    {
        float anglefirst = Sodabathydata.mltihead.at(0).mltswath.at(0).eqlAng.angleFirst;
        float anglelast  = Sodabathydata.mltihead.at(0).mltswath.at(0).eqlAng.angleLast;

        for (int i = 0; i < points; i++)
        {
            pingPoints.angle[i] = anglefirst + i * (anglelast - anglefirst) / (points - 1);
        }
    }
    pingPoints.intensityflag = Sodabathydata.mltihead.at(0).mltswath.at(0).intensityFlag;
    if (pingPoints.intensityflag == vsrxtf::INTENSITYDO)
    {
        for (int i = 0; i < points; i++)
        {
            pingPoints.intensity[i] = Sodabathydata.mltihead.at(0).mltswath.at(0).intensity.intensity.at(i);
        }
    }
    for (int i = 0; i < points; i++)
    {
        pingPoints.X[i] = pingPoints.range[i] * sin(pingPoints.angle[i]);
        pingPoints.Y[i] = 0;
        pingPoints.Z[i] = -pingPoints.range[i] * cos(pingPoints.angle[i]);
    }
}
