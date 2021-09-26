#ifndef PRODUCEXTF_H
#define PRODUCEXTF_H
#include "xtfformat.h"

#include <vector>
#include <QFile>
#include <QByteArray>

namespace vsrxtf{

class ProduceXTF //: public QObject
{
    //Q_OBJECT
public:
    explicit ProduceXTF();
    void     saveXTFFileHeader();
    void     saveMBData(const MBINSData& mMbInsData);
    void     saveATTData(const MBINSData& mMbInsData);
    void     saveNAVData(const MBINSData& mMbInsData);
    void     saveSNIPPETData(const MBINSData &mMbInsData);

    void sltinsdata(const ATTData&   rawinsdata);
    void sltmbBathydata(const MBData& rawmbdata);
    QString sltxtfFileName(const QString&, const QString& fileName);
    void sltXTFHeader(const XTFFILEHEADER& xtfHeader);
    void sltCloseFile(bool);    
    void processMbINSdata(ATTData &spanVec1, MBData &mMbVec, ATTData &spanVec2);

    QFile   *mXtfFile = nullptr;
    bool    mInsProflag = true;
private:
    std::vector <ATTData> mSpanVec;
    std::vector <MBData>  mMbVec;
    std::vector <MBINSData>   mMbInsData;
    XTFFILEHEADER mXtfFileHeader;
    int     mMbsecond;
    int     mInsSecond;
    bool    mMbProflag;

};

}

#endif // PRODUCEXTF_H
