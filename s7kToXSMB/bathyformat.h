#ifndef BATHYFORMAT_H
#define BATHYFORMAT_H
#define DFTHEADERS 2
#define BATHFIELD 1001
#define SNIPFIELD 1002
#define WCDFIELD 1003
#define PII 3.1415926535896
#include <QDataStream>
#include <QFile>
#include <QVector>
#include <parse7k.h>

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;
typedef signed char    s8;
typedef signed short   s16;
typedef signed int     s32;
typedef float          f32;
//#pragma pack(push)
//#pragma pack(1)
typedef struct _gate
{
    u8 RangeMin;   // [seconds two-way] = RangeMin * G1_ScalingFactor
    u8 RangeMax;   // [seconds two-way] = RangeMax * G1_ScalingFactor
} GATE;
///////////////////MB file struct start//////////////////////////////
typedef struct _transduceralig
{
    unsigned char head_count;
    float         deta_X;
    float         deta_Y;
    float         deta_Z;
    float         angle_X;
    float         angle_Y;
    float         angle_Z;
    float         draught;
} TRANSDUCERALIG;

typedef struct _mbfilehead
{
    unsigned short          header_size;
    unsigned char           version_major;
    unsigned char           version_minor;
    char                    multibeam_sensor[16];
    unsigned int            day;
    float                   rxMountTilt;
    double                  start_time;
    unsigned char           head_num;
    unsigned char           sector_num;
    unsigned char           swath_num;
    float                   detaGPS_X;
    float                   detaGPS_Y;
    float                   detaGPS_Z;
    QVector<TRANSDUCERALIG> mbtrans;
} MBFILEHEAD;
///////////////////////MB bathy data struct start//////////////////////////////////
typedef struct _beamsinfo
{
    unsigned short points;
    QVector<float> range;
    unsigned short beamModel;   //1为等角模式；2为等距模式
} BEAMSINFO;

typedef struct _eqlang
{
    float angleFirst;
    float angleLast;
} EQLANG;

typedef struct _eqldis
{
    QVector<float> angles;
} EQLDIS;
typedef struct _intensity
{
    QVector<float> intensity;
} INTENSITY;

typedef struct _multisector
{
    unsigned char sectorCount;
    float         txPulseWidth;
    float         txbeamWidthVert;
    float         txBeamWidthHoriz;
    float         txSteeringVert;
    float         txSteeringHoriz;
    float         rxBandWidth;
    float         rxGain;
    float         rxSpreading;
    float         rxAbsorption;
} MULTISECTOR;
typedef struct _multiswath
{
    unsigned char        swathCount;
    unsigned char        sectorNum;
    unsigned char        intensityFlag;
    QVector<MULTISECTOR> mltsector;
    BEAMSINFO            beams;
    EQLANG               eqlAng;
    EQLDIS               eqldis;
    INTENSITY            intensity;
} MULTISWATH;

typedef struct _multiheader
{
    unsigned char       headerCount;
    unsigned int        date;
    double              time;
    unsigned int        pingNum;
    float               pingPeriod;
    float               soundSpeed;
    float               frequency;
    float               txPower;
    unsigned char       swathNum;
    QVector<MULTISWATH> mltswath;
} MULTIHEADER;

typedef struct _mbbathydata
{
    unsigned int         packetName;
    unsigned int         packetSize;
    unsigned char        headNum;
    QVector<MULTIHEADER> mltihead;
} MBBATHYDATA;
///////////////////////////////snippet data struct start//////////////////////////////////////////
//SNIPPETS 结构体
typedef struct _Senddata
{
    double     Time;
    float      X[1024];
    float      Y[1024];
    float      Z[1024];
    int        num;
    QByteArray rawData;
} SENDSONICDATA;

typedef struct _snipdata
{
    unsigned int            pingNum;
    unsigned short          snipNum;
    unsigned short          samples;
    unsigned int            firstSamp;
    float                   angle;
    QVector<unsigned short> magnitude;   //此对象包含个数由samples数值决定
} SNIPDATA;
typedef struct _multispswath
{
    unsigned char        swathCount;
    unsigned char        sectorNum;
    QVector<MULTISECTOR> mltsector;
    unsigned short       snippets;
    QVector<SNIPDATA>    snipdata;
} MULTISPSWATH;
typedef struct _multispheader
{
    unsigned char         headerCount;
    unsigned int          date;
    double                time;
    unsigned int          pingNum;
    float                 pingPeriod;
    float                 soundSpeed;
    float                 frequency;
    float                 txPower;
    unsigned char         swathNum;
    QVector<MULTISPSWATH> mltswath;
} MULTISPHEADER;

typedef struct _mbsnipdata
{
    unsigned int           packetName;
    unsigned int           packetSize;
    unsigned char          headNum;
    QVector<MULTISPHEADER> mltihead;
} MBSNIPDATA;
/////////////////////////snippet data struct/////////////////////////////////

typedef struct _vessel
{
    float X[1024];
    float Y[1024];
    float Z[1024];
} VESSEL;

typedef struct _pingtime
{
    u16   hour;
    u16   minute;
    float second;
} PINGTIME;

typedef struct _senddata
{
    PINGTIME pingT;
    float    X[1024];
    float    Y[1024];
    float    Z[1024];
} SENDDATA;

typedef struct _pingdata
{
    float secOfDay;
    int   points;
    int   pingNum;
    float range[1024];
    float angle[1024];
    float X[1024];
    float Y[1024];
    float Z[1024];
    int   intensityflag;
    float intensity[1024];
} MBPING;

//#pragma pack(pop)

class BATHYFORMAT
{
public:
    explicit BATHYFORMAT();
    //    u16  H0_Points;
    void _fileSaveMultiHead(QByteArray &array, const MULTIHEADER data);
    void _fileSaveMultiSwath(QByteArray &array, const MULTISWATH &data);
    void _fileSaveMultiSector(QByteArray &array, const MULTISECTOR &data);
    void _fileSaveBeaminfo(QByteArray &array, const BEAMSINFO &data);
    void _fileSaveEqlang(QByteArray &array, const EQLANG &data);
    void _fileSaveEqldis(QByteArray &array, const EQLDIS &data, const unsigned short &points);
    void _fileSaveIntensity(QByteArray &array, const INTENSITY &data, const unsigned short &points);
    //save file
    void TransMBHeadStructToQbyte(QByteArray &array, const MBFILEHEAD &data);
    void TransMBStructToQbyte(QByteArray &array, const MBBATHYDATA &data);
    //save snip data
    void fileSaveMbSnipData(QByteArray &array, const MBSNIPDATA &data);
    void _fileSaveMultiHead(QByteArray &array, const MULTISPHEADER &data);
    void _fileSaveMultiSwath(QByteArray &array, const MULTISPSWATH &data);
    void _fileSaveSnipData(QByteArray &array, const SNIPDATA &data);
    void parseBathyDataToMBStruct(QByteArray datagram, MBBATHYDATA &Sodabathydata, PINGTIME &pingtime);
    void parse7kDataToMBStruct(PINGINFO p, MBBATHYDATA &Sodabathydata);
    //save snip data
    //read data from file
    void byteChange(char *xbuffer, char *buffer, int init);

    void fileReadMbFile(QFile &mbfile);
    void _fileReadFileHeader(char *buffer, MBFILEHEAD &fileHeadData, int &sizeCount);
    void _fileReadMltHeaderData(char *buffer, MULTIHEADER &headerdata, int &sizeCount);
    void _fileReadMltSwathData(char *buffer, MULTISWATH &swathdata, int &sizeCount);
    void _fileReadMltSectorData(char *buffer, MULTISECTOR &sectordata, int &sizeCount);
    void _fileReadBeamInfo(char *buffer, BEAMSINFO &beams, int &sizeCount);
    void _fileReadEQLAng(char *buffer, EQLANG &beamangle, int &sizeCount);
    void _fileReadEQLDis(char *buffer, EQLDIS &beamdis, int &sizeCount, unsigned short &points);
    //read snip data
    void _fileReadSnipMltHeader(char *buffer, MULTISPHEADER &headerdata, int &sizeCount);
    void _fileReadSnipMltSwath(char *buffer, MULTISPSWATH &swathdata, int &sizeCount);
    void _fileReadSNIPData(char *buffer, SNIPDATA &data, int &sizeCount);
    //read snip data

    void computeVes(MBBATHYDATA &bathy, VESSEL &ves);
    void generateBathyViewdata(QByteArray &array, const SENDDATA &data, unsigned short &pt);

    //------------------采集程序保存的文件解析函数--------------------------------------------//
    void parseSaveDataFileHeader(MBFILEHEAD &headStruct, QFile &saveFile, int &prosize);
    void parseSaveDataFilePingdata(MBBATHYDATA &Sodabathydata, QFile &saveFile, int &prosize);
    void producePointClouds(MBBATHYDATA &Sodabathydata, MBPING &pingPoints);
};

#endif   // BATHYFORMAT_H
