#pragma once

#include <stdint.h>
#include <string>
#include <time.h>
#include <vector>

#define PI 3.1415926535897932
#define TIME_RATIO_6 0.000001

#include <windows.h>

#import "C:\Windows\WinSxS\amd64_faro.ls_1d23f5635ba800ab_1.1.703.1_none_3591b17b356ae3ff\FARO.LS.SDK.dll" no_namespace
#import "C:\Windows\WinSxS\amd64_faro.ls_1d23f5635ba800ab_1.1.703.1_none_3591b17b356ae3ff\iQOpen.dll" no_namespace

struct FilterParaScan
{
    FilterParaScan()
        : bSingleCheck(true)
        , bEdgeCheck(true)
        , bBadLineCheck(true)
        , bRefCheck(false)
        , singleNum(2)
        , singleDist(10)
        , minRef(-20)
        , maxRef(100)
    {
    }
    bool bSingleCheck;
    bool bBadLineCheck;
    bool bEdgeCheck;
    bool bRefCheck;

    unsigned singleNum;
    double   singleDist;
    double   minRef;
    double   maxRef;
};
struct ScanPluginPara
{
    bool           bBinAll;
    FilterParaScan filterPara;
};
#pragma pack(push, 1)
struct ScannerInformation
{
    int            lineID; /**<记录扫描仪线号。*/
    double         time;   /**<记录扫描仪时间。*/
    float          dist;   /**<记录扫描仪极径。*/
    float          hAngle; /**<记录扫描仪水平极角。*/
    float          vAngle; /**<记录扫描仪竖直极角。*/
    float          x;
    float          y;
    float          z;
    unsigned short intensity; /**<记录扫描仪强度。*/
    float          reflectance;
    float          deviation;
};
#pragma pack(pop)

class FaroTrans
{
public:
    IiQLibIfPtr m_libRef;

    FaroTrans();
    ~FaroTrans();
    bool GetMinTm(const wchar_t *path, double *dMinTime);
    int  OnInit(const wchar_t *path, void *para);

    int Trans(int nCount, int *nRead, ScannerInformation *siArray);

private:
    //IScanCtrlSDKPtr licSDKIf;
    //IiQLicensedInterfaceIfPtr licLibIf;

    _bstr_t m_strFilePath;

    struct tm m_gmtm;
    time_t    m_rawtime;
    int       m_NumScans;

    uint64_t *                      m_pulsetime;
    int64_t *                       m_dlatpulsetime;   //20160803
    double *                        m_dlatspacetime;
    int                             idx;
    int                             ibegin;
    int                             cbegin;
    int                             rbegin;
    double *                        m_pPositionsArr;
    int *                           m_pReflsArr;
    bool                            m_bBinAll;
    std::vector<ScannerInformation> m_cache;
    FilterParaScan                  m_filter;
    unsigned                        m_nLen;
    //ActivationContext* actctx;
};
