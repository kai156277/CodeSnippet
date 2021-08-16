#include "FaroTrans.h"

#include <Windows.h>

#import "C:\Windows\WinSxS\amd64_faro.ls_1d23f5635ba800ab_1.1.703.1_none_3591b17b356ae3ff\FARO.LS.SDK.dll" no_namespace
#import "C:\Windows\WinSxS\amd64_faro.ls_1d23f5635ba800ab_1.1.703.1_none_3591b17b356ae3ff\iQOpen.dll" no_namespace

BSTR licenseCode =
    L"FARO Open Runtime License\n"
    L"Key:W2CEL7NRTCTXXKJT6KZYSPUP2\n"
    L"\n"
    L"The software is the registered property of FARO Scanner "
    L"Production GmbH, Stuttgart, Germany.\n"
    L"All rights reserved.\n"
    L"This software may only be used with written permission of "
    L"FARO Scanner Production GmbH, Stuttgart, Germany.";

FaroTrans::FaroTrans()
    : m_libRef(0)
{
    m_bBinAll = false;
    idx       = 0;
    m_rawtime = 0;   //mktime(&gmtm);
    ibegin    = 0;
    cbegin    = 0;
    rbegin    = 0;
    m_nLen    = 0;
    //m_filter.bSingleCheck = false;
    //m_filter.bEdgeCheck = false;
}

FaroTrans::~FaroTrans()
{
    m_libRef = nullptr;
}

bool FaroTrans::GetMinTm(const wchar_t *path, double *dMinTime)
{
    if (m_libRef == NULL)
    {
        HRESULT hret = CoInitialize(0);

        IiQLicensedInterfaceIfPtr licPtr;   // (__uuidof(iQLibIf));
        HRESULT                   hr = licPtr.CreateInstance(__uuidof(iQLibIf));
        if (FAILED(hr))
        {
            return false;
        }
        //if (licPtr == NULL)
        //{
        //    return false;
        //}
        licPtr->put_License(licenseCode);
        m_libRef = licPtr;
    }
    std::wstring strPath = path;

    size_t n = strPath.rfind(L".fls\\");

    if (n != std::wstring::npos)
    {
        strPath.resize(n + 4);
    }

    m_strFilePath = strPath.c_str();   //目录名为xxx.fls
    m_NumScans    = 0;
    if (m_libRef->load(m_strFilePath) == 0)   //加载成功
    {
        m_NumScans = m_libRef->getNumScans();
    }
    else   //加载失败
    {
        return false;
        //("加载失败，请选择一个工作区文件(.fws) 或单个扫描文件(.fls)!!");
    }

    m_pulsetime     = NULL;
    m_dlatpulsetime = NULL;
    m_dlatspacetime = NULL;

    int Rows = m_libRef->getScanNumRows(0);
    int Cols = m_libRef->getScanNumCols(0);

    if (Rows <= 0 || Cols <= 0)
    {
        return false;
    }
    int      Cols2    = Cols / 2;
    uint64_t deltaAll = 0;
    int      deltaNum = 0;
    m_pulsetime       = new uint64_t[Cols2];
    m_dlatpulsetime   = new int64_t[Cols2 + 1];
    m_dlatspacetime   = new double[Cols2 + 1];
    for (int coll = 0; coll < Cols2; coll++)
    {
        uint64_t pptime = 0;
        m_libRef->getAutomationTimeOfSyncPulse(0, coll, &pptime);
        m_pulsetime[coll] = pptime;
        if (coll > 0)
        {
            m_dlatpulsetime[coll] = m_pulsetime[coll] - m_pulsetime[coll - 1];
            m_dlatspacetime[coll] = (double) m_dlatpulsetime[coll] / ((Rows - 1) * 2.4);
            if (m_dlatpulsetime[coll] > 0 && m_dlatpulsetime[coll] < 100000)
            {
                deltaAll += m_dlatpulsetime[coll];
                deltaNum++;
            }
        }
    }
    if (m_pulsetime[Cols2 - 1] & 0xF000000000000000)
    {
        deltaAll /= deltaNum;
        double delatDbl = (double) deltaAll / ((Rows - 1) * 2.4);
        int    coli     = 1;
        while (m_dlatpulsetime[coli] < 0)
        {
            coli++;
        }
        for (int colj = coli - 1; colj >= 0; --colj)
        {
            m_pulsetime[colj]     = m_pulsetime[colj + 1] - deltaAll;
            m_dlatpulsetime[colj] = deltaAll;
            m_dlatspacetime[colj] = delatDbl;
        }
        //no use
    }
    m_dlatpulsetime[Cols2] = m_dlatpulsetime[Cols2 - 1];
    m_dlatspacetime[Cols2] = m_dlatspacetime[Cols2 - 1];

    uint64_t patime = m_pulsetime[0] + (int64_t)((Rows - 1) * 0.2 * m_dlatspacetime[1]);

    double   dtime  = patime * TIME_RATIO_6;
    uint32_t utime1 = (uint32_t)(dtime);
    dtime -= utime1;

    time_t rawt = m_rawtime + utime1;
    gmtime_s(&m_gmtm, &rawt);

    dtime += m_gmtm.tm_hour * 3600 + m_gmtm.tm_min * 60 + m_gmtm.tm_sec;
    *dMinTime = dtime;
    delete[] m_pulsetime;
    delete[] m_dlatpulsetime;
    delete[] m_dlatspacetime;
    m_pulsetime     = NULL;
    m_dlatpulsetime = NULL;
    m_dlatspacetime = NULL;
    if (m_libRef->unloadScan(0) == 0)
    {
    }

    return true;
}

int FaroTrans::OnInit(const wchar_t *path, void *para)
{
    if (m_libRef == NULL)
    {
        HRESULT                   hret = CoInitialize(0);
        IiQLicensedInterfaceIfPtr licPtr;
        // (__uuidof(iQLibIf));
        HRESULT hr = licPtr.CreateInstance(__uuidof(iQLibIf));
        if (FAILED(hr))
        {
            return -1;
        }
        //if (licPtr == NULL)
        //{
        //    return false;
        //}
        licPtr->put_License(licenseCode);
        m_libRef = licPtr;
        m_libRef.AddRef();
    }
    m_bBinAll            = ((ScanPluginPara *) para)->bBinAll;
    m_filter             = ((ScanPluginPara *) para)->filterPara;
    std::wstring strPath = path;

    size_t n = strPath.rfind(L".fls\\");

    if (n != std::wstring::npos)
    {
        strPath.resize(n + 4);
    }

    m_strFilePath = strPath.c_str();   //目录名为xxx.fls
    m_NumScans    = 0;
    if (m_libRef->load(m_strFilePath) == 0)   //加载成功
    {
        m_NumScans = m_libRef->getNumScans();
        //m_libRef->PutscanReflectionMode(2);
    }
    else   //加载失败
    {
        return 1;
        //("加载失败，请选择一个工作区文件(.fws) 或单个扫描文件(.fls)!!");
    }
    ibegin          = 0;
    cbegin          = 0;
    rbegin          = 0;
    idx             = 0;
    m_pulsetime     = NULL;
    m_dlatpulsetime = NULL;
    m_dlatspacetime = NULL;
    m_pPositionsArr = NULL;
    m_pReflsArr     = NULL;
    m_nLen          = 0;
    m_cache.clear();

    return 0;
}

int FaroTrans::Trans(int nCount, int *nRead, ScannerInformation *siArray)
{
    const float zero = 0;

    int Rows = 0, Cols = 0;

    for (int iScan = ibegin; iScan < m_NumScans; iScan++)
    {

        Rows = m_libRef->getScanNumRows(iScan);
        Cols = m_libRef->getScanNumCols(iScan);

        if (Rows <= 0 || Cols <= 0)
        {
            continue;
        }

        int      Cols2    = Cols / 2;
        uint64_t deltaAll = 0;
        int      deltaNum = 0;
        if (m_pulsetime == NULL)
        {
            cbegin          = 0;
            m_pulsetime     = new uint64_t[Cols2];
            m_dlatpulsetime = new int64_t[Cols2 + 1];
            m_dlatspacetime = new double[Cols2 + 1];
            for (int coll = 0; coll < Cols2; coll++)
            {
                uint64_t pptime = 0;
                m_libRef->getAutomationTimeOfSyncPulse(iScan, coll, &pptime);
                m_pulsetime[coll] = pptime;
                if (coll > 0)
                {
                    m_dlatpulsetime[coll] = m_pulsetime[coll] - m_pulsetime[coll - 1];
                    m_dlatspacetime[coll] = (double) m_dlatpulsetime[coll] / ((Rows - 1) * 2.4);

                    if (m_dlatpulsetime[coll] > 0 && m_dlatpulsetime[coll] < 100000)
                    {
                        deltaAll += m_dlatpulsetime[coll];
                        deltaNum++;
                    }
                }
            }

            if (m_pulsetime[Cols2 - 1] & 0xF000000000000000)
            {
                deltaAll /= deltaNum;
                double delatDbl = (double) deltaAll / ((Rows - 1) * 2.4);
                int    coli     = 1;
                while (m_dlatpulsetime[coli] < 0)
                {
                    coli++;
                }
                for (int colj = coli - 1; colj >= 0; --colj)
                {
                    m_pulsetime[colj]     = m_pulsetime[colj + 1] - deltaAll;   //10479  10447
                    m_dlatpulsetime[colj] = deltaAll;
                    m_dlatspacetime[colj] = delatDbl;
                }

                while (m_dlatpulsetime[coli] > 0)
                {
                    coli++;
                }

                while (m_dlatpulsetime[coli] < 0)
                {   //中间一段
                    m_pulsetime[coli]     = m_pulsetime[coli - 1] + deltaAll;
                    m_dlatpulsetime[coli] = deltaAll;
                    m_dlatspacetime[coli] = delatDbl;
                    coli++;
                }

                while (m_dlatpulsetime[coli] > 0)
                {
                    coli++;
                }

                for (int colj = coli; colj < Cols2; ++colj)
                {
                    m_pulsetime[colj]     = m_pulsetime[colj - 1] + deltaAll;
                    m_dlatpulsetime[colj] = deltaAll;
                    m_dlatspacetime[colj] = delatDbl;
                }
            }
            m_dlatpulsetime[Cols2] = m_dlatpulsetime[Cols2 - 1];
            m_dlatspacetime[Cols2] = m_dlatspacetime[Cols2 - 1];
        }

        for (int colll = cbegin; colll < Cols; colll++)
        {

            if (m_nLen == 0)
            {

                int      col    = 0;
                int      colll2 = colll / 2;
                uint64_t patime = 0;
                if ((colll & 0x01) == 1)
                {
                    col    = (colll - 1) / 2 + Cols2;
                    patime = m_pulsetime[colll2] + (int64_t)(((Rows - 1) * 1.2 + 2) * m_dlatspacetime[colll2 + 1]);
                }
                else
                {
                    col    = colll2;
                    patime = m_pulsetime[colll2] + (int64_t)((Rows - 1) * 0.2 * m_dlatspacetime[colll2 + 1]);
                }
                m_cache.clear();
                rbegin          = 0;
                m_pPositionsArr = new double[Rows * 3];
                m_pReflsArr     = new int[Rows];
                int res         = m_libRef->getXYZScanPoints(iScan, 0, col, Rows, m_pPositionsArr, m_pReflsArr);   //getXYZScanPoints2
                m_cache.resize(Rows);
                m_nLen = 0;
                ScannerInformation sinfo;
                double             lastDist  = -1;
                unsigned           lastCheck = 0;
                bool               bChecked  = false;
                int                nlineID   = (col >= Cols2) ? (col - Cols2) : col;
                unsigned           nBad      = 0;
                for (int nRow = Rows - 1; nRow >= 0; nRow--)
                {
                    int      row   = nRow;
                    uint64_t atime = 0;
                    if ((colll & 0x01) == 1)
                    {
                        row   = Rows - nRow - 1;
                        atime = patime + (int64_t)(row * m_dlatspacetime[colll2 + 1]);
                    }
                    else
                    {
                        atime = patime + (int64_t)((Rows - row - 1) * m_dlatspacetime[colll2 + 1]);
                    }

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

                    double elev3 = 0.0, hor3 = 0.0;
                    if (dist > 0)
                    {
                        hor3  = atan2(y, x);
                        elev3 = acos(z / dist);
                        if (!m_bBinAll)
                        {
                            elev3 = PI + ((((colll + 1) & 0x01) == 0) ? elev3 : -elev3);   //lin
                        }
                    }
                    //uint64_t autotime;
                    //m_libRef->getAutomationTimeOfScanPoint(iScan, row1, col, &autotime); //7

                    double   dtime  = atime * TIME_RATIO_6;   // autotime
                    uint32_t utime1 = (uint32_t)(dtime);
                    dtime -= utime1;

                    time_t rawt = m_rawtime + utime1;
                    gmtime_s(&m_gmtm, &rawt);

                    dtime += m_gmtm.tm_hour * 3600 + m_gmtm.tm_min * 60 + m_gmtm.tm_sec;
                    if (dtime > 86400)
                    {
                        nBad++;
                        dtime = 86400;
                    }

                    m_cache[m_nLen].lineID = nlineID;
                    m_cache[m_nLen].time   = dtime;
                    m_cache[m_nLen].dist   = (float) dist;

                    m_cache[m_nLen].hAngle = (float) hor3;
                    m_cache[m_nLen].vAngle = (float) elev3;   //h--lin

                    m_cache[m_nLen].x           = (float) x;
                    m_cache[m_nLen].y           = (float) y;
                    m_cache[m_nLen].z           = (float) z;
                    m_cache[m_nLen].intensity   = reflect;
                    m_cache[m_nLen].reflectance = zero;
                    m_cache[m_nLen].deviation   = zero;
                    m_nLen++;
                    if (m_filter.bSingleCheck)
                    {
                    }   // if bSingleCheck

                }   //for nRow
                delete[] m_pPositionsArr;
                delete[] m_pReflsArr;
                m_pPositionsArr = NULL;
                m_pReflsArr     = NULL;

                if (m_filter.bBadLineCheck)
                {
                }

                if (m_filter.bEdgeCheck)
                {
                }
            }   // if m_nLen == 0

            for (int iRow = rbegin; iRow < m_nLen; iRow++)
            {

                if (m_cache[iRow].lineID < 0)
                {
                    continue;
                }
                siArray[idx] = m_cache[iRow];
                idx++;
                if (idx >= nCount)
                {   //下一次调用
                    ibegin = iScan;
                    cbegin = colll;
                    rbegin = iRow + 1;
                    idx    = 0;

                    *nRead = nCount;
                    return 0;
                }

            }   //for iRow

            m_nLen = 0;
            m_cache.clear();
            rbegin = 0;
        }   //for colll
        cbegin = 0;

        delete[] m_pulsetime;
        delete[] m_dlatpulsetime;
        delete[] m_dlatspacetime;
        m_pulsetime     = NULL;
        m_dlatpulsetime = NULL;
        m_dlatspacetime = NULL;

        if (m_libRef->unloadScan(iScan) == 0)
        {
        }
    }
    *nRead = idx;
    idx    = 0;
    return -1;   //eof
}
