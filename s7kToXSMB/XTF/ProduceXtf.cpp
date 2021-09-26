#include "ProduceXtf.h"
#include <QFileInfo>
#include <QDate>
#include <QDebug>
#include <QDir>

enum DataFlag{
    MBDATAFLAG  = 11,
    INSDATAFLAG = 12,
    NAVDATAFLAG = 13,
    SNIPPETFLAG = 14,
};
namespace vsrxtf {
ProduceXTF::ProduceXTF()//QObject *parent) : QObject(parent)
{
    mInsSecond=-1;
    mMbsecond=-1;
    mMbProflag = true;
}

void ProduceXTF::sltinsdata(const ATTData&   rawinsdata)
{
    //    spanVec.push_back(rawinsdata);
    //    if(inssecond!=-1)
    //    {
    //        if(inssecond!=rawinsdata.Second)
    //        {
    //            insproflag=false;
    //            inssecond = rawinsdata.Second;
    //        }
    //    }
    //    else
    //    {
    //        inssecond = rawinsdata.Second;
    //    }
    //    if(insproflag==false && mbproflag==false)
    //    {
    //        processMbINSdata(spanVec,mbVec);
    //    }
}
void ProduceXTF::sltmbBathydata(const MBData& rawmbdata)
{
    //    mbVec.push_back(rawmbdata);
    //    if(mbsecond!=-1)
    //    {
    //        if(mbsecond!=rawmbdata.Second)
    //        {
    //            mbsecond = rawmbdata.Second;
    //            mbproflag=false;
    //        }
    //    }
    //    else
    //    {
    //        mbsecond = rawmbdata.Second;
    //    }
    //    if(insproflag==false && mbproflag==false)
    //    {
    //        processMbINSdata(spanVec,mbVec);
    //    }
}

QString ProduceXTF::sltxtfFileName(const QString& savePath, const QString& fileName)
{    
    QString saveName = fileName + ".xtf";
    QString realName = savePath + "/" + saveName;

    mXtfFile = new QFile(realName);

    mXtfFile->open(QIODevice::WriteOnly | QIODevice::Truncate);
    if (!mXtfFile->isOpen())
    {
        delete mXtfFile;
        mXtfFile = nullptr;
        return QString();
    }
    QByteArray saveNameLatin = saveName.toLatin1();
    memset(mXtfFileHeader.ThisFileName, '\0', 64);
    memcpy(mXtfFileHeader.ThisFileName, saveNameLatin.data(), std::max(64, saveNameLatin.size()));

    saveXTFFileHeader();
    return realName;
}

//! 线性插值 spanVec1 惯导左值  mbVec多波束值 spanvec2 惯导右值，
void ProduceXTF::processMbINSdata( ATTData &spanVec1, MBData &mbVec, ATTData &spanVec2)
{
    MBINSData tempdata;
    tempdata.DataFlag = INSDATAFLAG;
    tempdata.Year = spanVec1.Year;
    tempdata.Month = spanVec1.Month;
    tempdata.Day = spanVec1.Day;
    tempdata.Hour = spanVec1.Hour;
    tempdata.Minute = spanVec1.Minute;
    tempdata.Second = spanVec1.Second;
    tempdata.Milliseconds = spanVec1.Milliseconds;
    tempdata.AttTimeTag = unsigned(spanVec1.dTime * 1000.0 + 0.5) + 24 * 3600 * 1000;
    tempdata.roll = spanVec1.roll;
    tempdata.pitch = spanVec1.pitch;
    tempdata.heading = spanVec1.heading;
    tempdata.heave = spanVec1.heave;
    mMbInsData.push_back(tempdata);
    if (spanVec1.Milliseconds == 0)
    {
        MBINSData tempNavdata;
        tempNavdata.DataFlag = NAVDATAFLAG;
        tempNavdata.Year = spanVec1.Year;
        tempNavdata.Month = spanVec1.Month;
        tempNavdata.Day = spanVec1.Day;
        tempNavdata.Hour = spanVec1.Hour;
        tempNavdata.Minute = spanVec1.Minute;
        tempNavdata.Second = spanVec1.Second;
        tempNavdata.Microseconds = 10 * spanVec1.Milliseconds;                   //??
        tempNavdata.AttTimeTag = unsigned(spanVec1.dTime * 1000.0 + 0.5) + 24 * 3600 * 1000;
        tempNavdata.latitude = spanVec1.latitude;
        tempNavdata.longitude = spanVec1.longitude;
        tempNavdata.heigt = spanVec1.heigt;
        mMbInsData.push_back(tempNavdata);
    }
    int mbjulianday     = mbVec.JulianDay;                  //mbdata.toJulianDay();
    int insjulianday1   = spanVec1.JulianDay;               //insdata1.toJulianDay();
    int insjulianday2   = spanVec2.JulianDay;               //insdata2.toJulianDay();
    double mbtime       = mbVec.dTime;                      //Hour*3600+mbi.Minute*60+mbi.Second+double(mbi.HSeconds)/100.0;
    double instime1     = spanVec1.dTime;                   //Hour*3600+spanj.Minute*60+spanj.Second+double(spanj.Milliseconds)/1000.0;
    double instime2     = spanVec2.dTime;                   //Hour*3600+spanVec.at(j+1).Minute*60+spanVec.at(j+1).Second+double(spanVec.at(j+1).Milliseconds)/1000.0;
    double detaTime1    = (mbjulianday - insjulianday1) * 24 * 3600 + (mbtime - instime1);
    double detaTime2    = (insjulianday2 - insjulianday1) * 24 * 3600 + (instime2 - instime1);

    detaTime1 /= detaTime2;

    double B            = spanVec1.latitude + (detaTime1)*(spanVec2.latitude - spanVec1.latitude);
    double L            = spanVec1.longitude + (detaTime1)*(spanVec2.longitude - spanVec1.longitude);
    double H            = spanVec1.heigt + (detaTime1)*(spanVec2.heigt - spanVec1.heigt);
    double Roll         = spanVec1.roll + (detaTime1)*(spanVec2.roll - spanVec1.roll);
    double Pitch        = spanVec1.pitch + (detaTime1)*(spanVec2.pitch - spanVec1.pitch);
    double Heading      = spanVec1.heading + (detaTime1)*(spanVec2.heading - spanVec1.heading);
    double Heave        = spanVec1.heave + (detaTime1)*(spanVec2.heave - spanVec1.heave);

    unsigned attTimeTag = unsigned(spanVec1.dTime * 1000.0 + 0.5) + 24 * 3600 * 1000;
    unsigned navTimeTag = (spanVec1.Hour * 3600 + spanVec1.Minute * 60 + spanVec1.Second) * 1000 + 24 * 3600 * 1000;

    MBINSData tempMbdata;
    tempMbdata.DataFlag = MBDATAFLAG;
    tempMbdata.Year = mbVec.Year;
    tempMbdata.Month = mbVec.Month;
    tempMbdata.Day = mbVec.Day;
    tempMbdata.Hour = mbVec.Hour;
    tempMbdata.Minute = mbVec.Minute;
    tempMbdata.Second = mbVec.Second;
    tempMbdata.HSeconds = mbVec.HSeconds;
    tempMbdata.JulianDay = mbVec.JulianDay;
    tempMbdata.pingNum = mbVec.pingNum;
    tempMbdata.soundSpeed = mbVec.soundSpeed;
    tempMbdata.mbdata = mbVec.rawdata;
    tempMbdata.AttTimeTag = attTimeTag;
    tempMbdata.NavTimeTag = navTimeTag;
    tempMbdata.latitude = B;
    tempMbdata.longitude = L;
    tempMbdata.heigt = H;
    tempMbdata.roll = Roll;
    tempMbdata.pitch = Pitch;
    tempMbdata.heading = Heading;
    tempMbdata.heave = Heave;
    mMbInsData.push_back(tempMbdata);

//    MBINSData tempSNPdata;
//    tempSNPdata.DataFlag = SNIPPETFLAG;
//    tempSNPdata.Year = mbVec.Year;
//    tempSNPdata.Month = mbVec.Month;
//    tempSNPdata.Day = mbVec.Day;
//    tempSNPdata.Hour = mbVec.Hour;
//    tempSNPdata.Minute = mbVec.Minute;
//    tempSNPdata.Second = mbVec.Second;
//    tempSNPdata.HSeconds = mbVec.HSeconds;
//    tempSNPdata.JulianDay = mbVec.JulianDay;
//    tempSNPdata.pingNum = mbVec.pingNum;
//    tempSNPdata.soundSpeed = mbVec.soundSpeed;
//    tempSNPdata.mbdata = mbVec.rawSNPdata;
//    tempSNPdata.AttTimeTag = attTimeTag;
//    tempSNPdata.NavTimeTag = navTimeTag;
//    tempSNPdata.latitude = B;
//    tempSNPdata.longitude = L;
//    tempSNPdata.heigt = H;
//    tempSNPdata.roll = Roll;
//    tempSNPdata.pitch = Pitch;
//    tempSNPdata.heading = Heading;
//    tempSNPdata.heave = Heave;
//    mbinsData.push_back(tempSNPdata);

    for(int i = 0 ; i < mMbInsData.size() ; i++)
    {
        if (mMbInsData.at(i).DataFlag == MBDATAFLAG)
        {
            saveMBData(mMbInsData.at(i));
        }
        else if (mMbInsData.at(i).DataFlag == INSDATAFLAG)
        {
            saveATTData(mMbInsData.at(i));
        }
        else if (mMbInsData.at(i).DataFlag == NAVDATAFLAG)
        {
            saveNAVData(mMbInsData.at(i));
        }
        else if (mMbInsData.at(i).DataFlag == SNIPPETFLAG)
        {
            saveSNIPPETData(mMbInsData.at(i));
        }
    }
    mMbInsData.clear();
}

void appendChanInfo(QByteArray& bdata , const CHANINFO& cir)
{
    bdata.append(reinterpret_cast<const char*>(&cir.TypeOfChannel), sizeof(cir.TypeOfChannel));
    bdata.append(reinterpret_cast<const char*>(&cir.SubChannelNumber), sizeof(cir.SubChannelNumber));
    bdata.append(reinterpret_cast<const char*>(&cir.CorrectionFlags), sizeof(cir.CorrectionFlags));
    bdata.append(reinterpret_cast<const char*>(&cir.UniPolar), sizeof(cir.UniPolar));
    bdata.append(reinterpret_cast<const char*>(&cir.BytesPerSample), sizeof(cir.BytesPerSample));
    bdata.append(reinterpret_cast<const char*>(&cir.Reserved), sizeof(cir.Reserved));
    bdata.append(reinterpret_cast<const char*>(cir.ChannelName), 16);
    bdata.append(reinterpret_cast<const char*>(&cir.VoltScale), sizeof(cir.VoltScale));
    bdata.append(reinterpret_cast<const char*>(&cir.Frequency), sizeof(cir.Frequency));
    bdata.append(reinterpret_cast<const char*>(&cir.HorizBeamAngle), sizeof(cir.HorizBeamAngle));
    bdata.append(reinterpret_cast<const char*>(&cir.TiltAngle), sizeof(cir.TiltAngle));
    bdata.append(reinterpret_cast<const char*>(&cir.BeamWidth), sizeof(cir.BeamWidth));
    bdata.append(reinterpret_cast<const char*>(&cir.OffsetX), sizeof(cir.OffsetX));
    bdata.append(reinterpret_cast<const char*>(&cir.OffsetY), sizeof(cir.OffsetY));
    bdata.append(reinterpret_cast<const char*>(&cir.OffsetZ), sizeof(cir.OffsetZ));
    bdata.append(reinterpret_cast<const char*>(&cir.OffsetYaw), sizeof(cir.OffsetYaw));
    bdata.append(reinterpret_cast<const char*>(&cir.OffsetPitch), sizeof(cir.OffsetPitch));
    bdata.append(reinterpret_cast<const char*>(&cir.OffsetRoll), sizeof(cir.OffsetRoll));
    bdata.append(reinterpret_cast<const char*>(&cir.BeamsPerArray), sizeof(cir.BeamsPerArray));
    bdata.append(reinterpret_cast<const char*>(cir.padv1r), 54);
}
void copyChanInfo(CHANINFO& cil, const CHANINFO& cir)
{
    cil.TypeOfChannel = cir.TypeOfChannel;
    cil.SubChannelNumber = cir.SubChannelNumber;
    cil.CorrectionFlags = cir.CorrectionFlags;
    cil.UniPolar = cir.UniPolar;
    cil.BytesPerSample = cir.BytesPerSample;
    cil.Reserved = 0;

    {
        memcpy(cil.ChannelName, cir.ChannelName, 16);
    }
    cil.VoltScale = cir.VoltScale;
    cil.Frequency = cir.Frequency;
    cil.HorizBeamAngle = cir.HorizBeamAngle;
    cil.TiltAngle = cir.TiltAngle;
    cil.BeamWidth = cir.BeamWidth;
    cil.OffsetX = cir.OffsetX;
    cil.OffsetY = cir.OffsetY;
    cil.OffsetZ = cir.OffsetZ;
    cil.OffsetYaw = cir.OffsetYaw;
    cil.OffsetPitch = cir.OffsetPitch;
    cil.OffsetRoll = cir.OffsetRoll;
    cil.BeamsPerArray = 0;

    {
        memset(cil.padv1r, '\0', 54);
    }
}
void clearChanInfo(CHANINFO& chi)
{
    chi.TypeOfChannel = 0;
    chi.SubChannelNumber = 0;
    chi.CorrectionFlags = 0;
    chi.UniPolar = 0;
    chi.BytesPerSample = 0;
    chi.Reserved = 0;

    {
        memset(chi.ChannelName, '\0', 16);
    }
    chi.VoltScale = 0;
    chi.Frequency = 0;
    chi.HorizBeamAngle = 0;
    chi.TiltAngle = 0;
    chi.BeamWidth = 0;
    chi.OffsetX = 0;
    chi.OffsetY = 0;
    chi.OffsetZ = 0;
    chi.OffsetYaw = 0;
    chi.OffsetPitch = 0;
    chi.OffsetRoll = 0;
    chi.BeamsPerArray = 0;

    {
        memset(chi.padv1r, '\0', 54);
    }
}
void ProduceXTF::sltXTFHeader(const XTFFILEHEADER& xtfHeader)
{
    mXtfFileHeader.FileFormat = xtfHeader.FileFormat;
    mXtfFileHeader.SystemType = xtfHeader.SystemType;

    {
        memcpy(mXtfFileHeader.RecordingProgramName, xtfHeader.RecordingProgramName, 8);
        memcpy(mXtfFileHeader.RecordingProgramVersion, xtfHeader.RecordingProgramVersion, 8);
        memcpy(mXtfFileHeader.SonarName, xtfHeader.SonarName, 16);
    }
    mXtfFileHeader.SonarType = xtfHeader.SonarType;

    {
        memcpy(mXtfFileHeader.NoteString, xtfHeader.NoteString, 64);
        memcpy(mXtfFileHeader.ThisFileName, xtfHeader.ThisFileName, 64);
    }
    mXtfFileHeader.NavUnits = xtfHeader.NavUnits;
    mXtfFileHeader.NumberOfSonarChannels = xtfHeader.NumberOfSonarChannels;
    mXtfFileHeader.NumberOfBathymetryChannels = xtfHeader.NumberOfBathymetryChannels;
    mXtfFileHeader.NumberOfSnippetChannels = xtfHeader.NumberOfSnippetChannels;
    mXtfFileHeader.NumberOfForwardLookArrays = xtfHeader.NumberOfForwardLookArrays;
    mXtfFileHeader.NumberOfEchoStrengthChannels = xtfHeader.NumberOfEchoStrengthChannels;
    mXtfFileHeader.NumberOfInterferometryChannels = xtfHeader.NumberOfInterferometryChannels;
    mXtfFileHeader.Revpad1 = 0;
    mXtfFileHeader.Revpad2 = 0;
    mXtfFileHeader.ReferencePointHeight = xtfHeader.ReferencePointHeight;

    {
        memcpy(mXtfFileHeader.ProjectionType, xtfHeader.ProjectionType, 12);
        memcpy(mXtfFileHeader.SpheriodType, xtfHeader.SpheriodType, 10);
    }
    mXtfFileHeader.NavigationLatency = xtfHeader.NavigationLatency;
    mXtfFileHeader.OriginX = xtfHeader.OriginX;
    mXtfFileHeader.OriginY = xtfHeader.OriginY;
    mXtfFileHeader.NavOffsetX = xtfHeader.NavOffsetX;
    mXtfFileHeader.NavOffsetY = xtfHeader.NavOffsetY;
    mXtfFileHeader.NavOffsetZ = xtfHeader.NavOffsetZ;
    mXtfFileHeader.NavOffsetYaw = xtfHeader.NavOffsetYaw;

    mXtfFileHeader.MRUOffsetX = xtfHeader.MRUOffsetX;
    mXtfFileHeader.MRUOffsetY = xtfHeader.MRUOffsetY;
    mXtfFileHeader.MRUOffsetZ = xtfHeader.MRUOffsetZ;
    mXtfFileHeader.MRUOffsetRoll = xtfHeader.MRUOffsetRoll;
    mXtfFileHeader.MRUOffsetPitch = xtfHeader.MRUOffsetPitch;
    mXtfFileHeader.MRUOffsetYaw = xtfHeader.MRUOffsetYaw;

    if (xtfHeader.NumberOfSonarChannels != 0)
    {
        copyChanInfo(mXtfFileHeader.ChanInfo[0], xtfHeader.ChanInfo[0]);
        copyChanInfo(mXtfFileHeader.ChanInfo[1], xtfHeader.ChanInfo[1]);

        if (xtfHeader.NumberOfBathymetryChannels != 0)
        {
            copyChanInfo(mXtfFileHeader.ChanInfo[2], xtfHeader.ChanInfo[2]);
        }
    }
    else {
        if (xtfHeader.NumberOfBathymetryChannels != 0)
        {
            copyChanInfo(mXtfFileHeader.ChanInfo[0], xtfHeader.ChanInfo[0]);
        }
    }

    int chanSum = mXtfFileHeader.NumberOfBathymetryChannels + mXtfFileHeader.NumberOfSonarChannels;

    for (int i = chanSum; i < 6; i++)
    {
        clearChanInfo(mXtfFileHeader.ChanInfo[i]);
    }
}

void ProduceXTF::saveXTFFileHeader()
{
    QByteArray bdata;
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.FileFormat), sizeof(mXtfFileHeader.FileFormat));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.SystemType),sizeof(mXtfFileHeader.SystemType));
    bdata.append(reinterpret_cast<const char*>(mXtfFileHeader.RecordingProgramName), 8);
    bdata.append(reinterpret_cast<const char*>(mXtfFileHeader.RecordingProgramVersion), 8);
    bdata.append(reinterpret_cast<const char*>(mXtfFileHeader.SonarName),16);
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.SonarType), sizeof(mXtfFileHeader.SonarType));
    bdata.append(reinterpret_cast<const char*>(mXtfFileHeader.NoteString), 64);
    bdata.append(reinterpret_cast<const char*>(mXtfFileHeader.ThisFileName),64);
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NavUnits), sizeof(mXtfFileHeader.NavUnits));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NumberOfSonarChannels), sizeof(mXtfFileHeader.NumberOfSonarChannels));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NumberOfBathymetryChannels),sizeof(mXtfFileHeader.NumberOfBathymetryChannels));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NumberOfSnippetChannels), sizeof(mXtfFileHeader.NumberOfSnippetChannels));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NumberOfForwardLookArrays), sizeof(mXtfFileHeader.NumberOfForwardLookArrays));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NumberOfEchoStrengthChannels),sizeof(mXtfFileHeader.NumberOfEchoStrengthChannels));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NumberOfInterferometryChannels), sizeof(mXtfFileHeader.NumberOfInterferometryChannels));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.Revpad1), sizeof(mXtfFileHeader.Revpad1));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.Revpad2), sizeof(mXtfFileHeader.Revpad2));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.ReferencePointHeight), sizeof(mXtfFileHeader.ReferencePointHeight));
    bdata.append(reinterpret_cast<const char*>(mXtfFileHeader.ProjectionType), 12);
    bdata.append(reinterpret_cast<const char*>(mXtfFileHeader.SpheriodType),10);
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NavigationLatency), sizeof(mXtfFileHeader.NavigationLatency));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.OriginX), sizeof(mXtfFileHeader.OriginX));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.OriginY),sizeof(mXtfFileHeader.OriginY));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NavOffsetY), sizeof(mXtfFileHeader.NavOffsetY));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NavOffsetX), sizeof(mXtfFileHeader.NavOffsetX));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NavOffsetZ),sizeof(mXtfFileHeader.NavOffsetZ));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.NavOffsetYaw), sizeof(mXtfFileHeader.NavOffsetYaw));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.MRUOffsetY), sizeof(mXtfFileHeader.MRUOffsetY));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.MRUOffsetX), sizeof(mXtfFileHeader.MRUOffsetX));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.MRUOffsetZ),sizeof(mXtfFileHeader.MRUOffsetZ));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.MRUOffsetYaw), sizeof(mXtfFileHeader.MRUOffsetYaw));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.MRUOffsetPitch), sizeof(mXtfFileHeader.MRUOffsetPitch));
    bdata.append(reinterpret_cast<const char*>(&mXtfFileHeader.MRUOffsetRoll), sizeof(mXtfFileHeader.MRUOffsetRoll));
    for(int i=0;i<6;i++)
    {
        appendChanInfo(bdata, mXtfFileHeader.ChanInfo[i]);
    }
    mXtfFile->write(bdata);
}

void  ProduceXTF::saveMBData(const MBINSData& mbinsData)
{
    if(!mXtfFile->isOpen())
        mXtfFile->open(QIODevice::WriteOnly|QIODevice::Append);

    XTFBATHHEADER mbBathy;
    mbBathy.MagicNumber = 0xFACE;
    mbBathy.HeaderType = 65;
    mbBathy.SubChannelNumber = 0;
    mbBathy.NumChansToFollow=0;
    mbBathy.ReservedSpace1[0] = 0;
    mbBathy.ReservedSpace1[1] = 0;
    int byteSize = mbinsData.mbdata.size()+256;
    float group =  (float(byteSize)/64.0f);
    int   group2 = int(byteSize/64);
    if(group==group2)
    {
        mbBathy.NumBytesThisRecord = byteSize;
    } else {
        mbBathy.NumBytesThisRecord = (group2+1)*64;
    }
    int paddedNum =    mbBathy.NumBytesThisRecord-   byteSize;
    mbBathy.Year = mbinsData.Year;
    mbBathy.Month = mbinsData.Month;
    mbBathy.Day = mbinsData.Day;
    mbBathy.Hour = mbinsData.Hour;
    mbBathy.Minute = mbinsData.Minute;
    mbBathy.Second = mbinsData.Second;
    mbBathy.HSeconds = mbinsData.HSeconds;

    mbBathy.JulianDay = mbinsData.JulianDay;
    mbBathy.EventNumber = 0;
    mbBathy.PingNumber = mbinsData.pingNum;
    mbBathy.SoundVelocity = mbinsData.soundSpeed/2;
    mbBathy.OceanTide = 0;
    mbBathy.Reserved2 = 0;
    mbBathy.ConductivityFreq = 0;
    mbBathy.TemperatureFreq=0;
    mbBathy.PressureFreq=0;
    mbBathy.PressureTemp=0;
    mbBathy.Conductivity=0;
    mbBathy.WaterTemperature=0;
    mbBathy.Pressure=0;
    mbBathy.ComputedSoundVelocity=0;
    mbBathy.MagX=0;
    mbBathy.MagY=0;
    mbBathy.MagZ=0;
    mbBathy.AuxVal1=0;
    mbBathy.AuxVal2=0;
    mbBathy.AuxVal3=0;
    mbBathy.AuxVal4=0;
    mbBathy.AuxVal5=0;
    mbBathy.AuxVal6=0;
    mbBathy.SpeedLog=0;
    mbBathy.Turbidity=0;
    mbBathy.ShipSpeed=0;
    mbBathy.ShipGyro = mbinsData.heading;
    mbBathy.ShipYcoordinate = mbinsData.latitude;
    mbBathy.ShipXcoordinate = mbinsData.longitude;
    mbBathy.ShipAltitude=0;
    mbBathy.ShipDepth=0;
    mbBathy.FixTimeHour=0;
    mbBathy.FixTimeMinute=0;
    mbBathy.FixTimeSecond=0;
    mbBathy.FixTimeHsecond=0;
    mbBathy.SensorSpeed=0;
    mbBathy.KP=0;
    mbBathy.SensorYcoordinate=mbinsData.latitude;
    mbBathy.SensorXcoordinate=mbinsData.longitude;
    mbBathy.SonarStatus=0;
    mbBathy.RangeToFish=0;
    mbBathy.BearingToFish=0;
    mbBathy.CableOut=0;
    mbBathy.Layback=0;
    mbBathy.CableTension=0;
    mbBathy.SensorDepth=0;
    mbBathy.SensorPrimaryAltitude=0;
    mbBathy.SensorAuxAltitude=0;
    mbBathy.SensorPitch = mbinsData.pitch;
    mbBathy.SensorRoll = mbinsData.roll;
    mbBathy.SensorHeading = mbinsData.heading;
    mbBathy.Heave = mbinsData.heave;
    mbBathy.Yaw = 0;
    mbBathy.AttitudeTimeTag = mbinsData.AttTimeTag;
    mbBathy.DOT=0;
    mbBathy.NavFixMilliseconds=mbinsData.NavTimeTag;
    mbBathy.ComputerClockHour=0;
    mbBathy.ComputerClockMinute=0;
    mbBathy.ComputerClockSecond=0;
    mbBathy.ComputerClockHsec=0;
    mbBathy.FishPositionDeltaX=0;
    mbBathy.FishPositionDeltaY=0;
    mbBathy.FishPositionErrorCode=0;
    mbBathy.OptionalOffsey=0;
    mbBathy.CableOutHundredths=0;

    {
        memset( mbBathy.ReservedSpace1r, 0, 6);
    }

    QByteArray bdata;
    bdata.append(reinterpret_cast<const char*>(&mbBathy.MagicNumber), sizeof(mbBathy.MagicNumber));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.HeaderType),sizeof(mbBathy.HeaderType));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SubChannelNumber), sizeof(mbBathy.SubChannelNumber));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.NumChansToFollow), sizeof(mbBathy.NumChansToFollow));
    bdata.append(reinterpret_cast<const char*>(mbBathy.ReservedSpace1), 2 * sizeof(uint16_t));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.NumBytesThisRecord), sizeof(mbBathy.NumBytesThisRecord));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Year), sizeof(mbBathy.Year));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Month),sizeof(mbBathy.Month));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Day), sizeof(mbBathy.Day));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Hour), sizeof(mbBathy.Hour));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Minute),sizeof(mbBathy.Minute));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Second), sizeof(mbBathy.Second));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.HSeconds), sizeof(mbBathy.HSeconds));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.JulianDay), sizeof(mbBathy.JulianDay));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.EventNumber),sizeof(mbBathy.EventNumber));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.PingNumber), sizeof(mbBathy.PingNumber));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SoundVelocity), sizeof(mbBathy.SoundVelocity));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.OceanTide),sizeof(mbBathy.OceanTide));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Reserved2), sizeof(mbBathy.Reserved2));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ConductivityFreq), sizeof(mbBathy.ConductivityFreq));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.TemperatureFreq), sizeof(mbBathy.TemperatureFreq));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.PressureFreq),sizeof(mbBathy.PressureFreq));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.PressureTemp),sizeof(mbBathy.PressureTemp));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Conductivity), sizeof(mbBathy.Conductivity));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.WaterTemperature), sizeof(mbBathy.WaterTemperature));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Pressure), sizeof(mbBathy.Pressure));

    bdata.append(reinterpret_cast<const char*>(&mbBathy.ComputedSoundVelocity), sizeof(mbBathy.ComputedSoundVelocity));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.MagX), sizeof(mbBathy.MagX));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.MagY), sizeof(mbBathy.MagY));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.MagZ), sizeof(mbBathy.MagZ));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.AuxVal1), sizeof(mbBathy.AuxVal1));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.AuxVal2), sizeof(mbBathy.AuxVal2));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.AuxVal3), sizeof(mbBathy.AuxVal3));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.AuxVal4), sizeof(mbBathy.AuxVal4));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.AuxVal5), sizeof(mbBathy.AuxVal5));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.AuxVal6), sizeof(mbBathy.AuxVal6));

    bdata.append(reinterpret_cast<const char*>(&mbBathy.SpeedLog), sizeof(mbBathy.SpeedLog));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Turbidity), sizeof(mbBathy.Turbidity));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ShipSpeed), sizeof(mbBathy.ShipSpeed));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ShipGyro), sizeof(mbBathy.ShipGyro));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ShipYcoordinate), sizeof(mbBathy.ShipYcoordinate));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ShipXcoordinate), sizeof(mbBathy.ShipXcoordinate));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ShipAltitude), sizeof(mbBathy.ShipAltitude));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ShipDepth), sizeof(mbBathy.ShipDepth));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.FixTimeHour), sizeof(mbBathy.FixTimeHour));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.FixTimeMinute), sizeof(mbBathy.FixTimeMinute));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.FixTimeSecond), sizeof(mbBathy.FixTimeSecond));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.FixTimeHsecond), sizeof(mbBathy.FixTimeHsecond));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SensorSpeed), sizeof(mbBathy.SensorSpeed));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.KP), sizeof(mbBathy.KP));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SensorYcoordinate), sizeof(mbBathy.SensorYcoordinate));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SensorXcoordinate), sizeof(mbBathy.SensorXcoordinate));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SonarStatus), sizeof(mbBathy.SonarStatus));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.RangeToFish), sizeof(mbBathy.RangeToFish));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.BearingToFish), sizeof(mbBathy.BearingToFish));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.CableOut), sizeof(mbBathy.CableOut));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Layback), sizeof(mbBathy.Layback));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.CableTension), sizeof(mbBathy.CableTension));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SensorDepth), sizeof(mbBathy.SensorDepth));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SensorPrimaryAltitude), sizeof(mbBathy.SensorPrimaryAltitude));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SensorAuxAltitude), sizeof(mbBathy.SensorAuxAltitude));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SensorPitch), sizeof(mbBathy.SensorPitch));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SensorRoll), sizeof(mbBathy.SensorRoll));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.SensorHeading), sizeof(mbBathy.SensorHeading));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Heave), sizeof(mbBathy.Heave));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.Yaw), sizeof(mbBathy.Yaw));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.AttitudeTimeTag), sizeof(mbBathy.AttitudeTimeTag));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.DOT), sizeof(mbBathy.DOT));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.NavFixMilliseconds), sizeof(mbBathy.NavFixMilliseconds));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ComputerClockHour), sizeof(mbBathy.ComputerClockHour));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ComputerClockMinute), sizeof(mbBathy.ComputerClockMinute));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ComputerClockSecond), sizeof(mbBathy.ComputerClockSecond));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.ComputerClockHsec), sizeof(mbBathy.ComputerClockHsec));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.FishPositionDeltaX), sizeof(mbBathy.FishPositionDeltaX));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.FishPositionDeltaY), sizeof(mbBathy.FishPositionDeltaY));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.FishPositionErrorCode), sizeof(mbBathy.FishPositionErrorCode));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.OptionalOffsey), sizeof(mbBathy.OptionalOffsey));
    bdata.append(reinterpret_cast<const char*>(&mbBathy.CableOutHundredths), sizeof(mbBathy.CableOutHundredths));

    {
        bdata.append(reinterpret_cast<const char*>(mbBathy.ReservedSpace1r), 6);
    }
    mXtfFile->write(bdata);
    mXtfFile->write(mbinsData.mbdata);
    QByteArray paddedChar;
    for(int i=0;i<paddedNum;i++)
    {
        char padde = '\0';
        paddedChar.append(padde);
    }
    if(paddedChar.size()!=0)
    {
        mXtfFile->write(paddedChar);
    }
}

void  ProduceXTF::saveATTData(const MBINSData& mbinsData)
{
    if(!mXtfFile->isOpen())
        mXtfFile->open(QIODevice::WriteOnly|QIODevice::Append);

    XTFATTITUDEDATA attidata;
    attidata.MagicNumber = 0xFACE;
    attidata.HeaderType = 3;
    attidata.SubChannelNumber = 0;
    attidata.NumChansToFollow=0;
    attidata.Reserved1[0] = 0;
    attidata.Reserved1[1] = 0;
    attidata.NumBytesThisRecord = 64;
    attidata.Reserved2[0] = 0;
    attidata.Reserved2[1] = 0;
    attidata.EpochMicroseconds = 0;
    attidata.SourceEpoch = 0;
    attidata.Pitch=mbinsData.pitch;
    attidata.Roll=mbinsData.roll;
    attidata.Heave=mbinsData.heave;
    attidata.Yaw = 0;
    attidata.TimeTag = mbinsData.AttTimeTag;
    attidata.Heading = mbinsData.heading;
    attidata.Year = mbinsData.Year;
    attidata.Month=mbinsData.Month;
    attidata.Day=mbinsData.Day;
    attidata.Hour=mbinsData.Hour;
    attidata.Minutes = mbinsData.Minute;
    attidata.Seconds=mbinsData.Second;
    attidata.Milliseconds=mbinsData.Milliseconds;
    attidata.Reserved3[0] = 0;

    QByteArray bdata;
    bdata.append(reinterpret_cast<const char*>(&attidata.MagicNumber), sizeof(attidata.MagicNumber));
    bdata.append(reinterpret_cast<const char*>(&attidata.HeaderType), sizeof(attidata.HeaderType));
    bdata.append(reinterpret_cast<const char*>(&attidata.SubChannelNumber), sizeof(attidata.SubChannelNumber));
    bdata.append(reinterpret_cast<const char*>(&attidata.NumChansToFollow), sizeof(attidata.NumChansToFollow));
    bdata.append(reinterpret_cast<const char*>(attidata.Reserved1), 2 * sizeof(uint16_t));
    bdata.append(reinterpret_cast<const char*>(&attidata.NumBytesThisRecord), sizeof(attidata.NumBytesThisRecord));
    bdata.append(reinterpret_cast<const char*>(attidata.Reserved2), 2 * sizeof(uint32_t));
    bdata.append(reinterpret_cast<const char*>(&attidata.EpochMicroseconds), sizeof(attidata.EpochMicroseconds));
    bdata.append(reinterpret_cast<const char*>(&attidata.SourceEpoch), sizeof(attidata.SourceEpoch));
    bdata.append(reinterpret_cast<const char*>(&attidata.Pitch), sizeof(attidata.Pitch));
    bdata.append(reinterpret_cast<const char*>(&attidata.Roll), sizeof(attidata.Roll));
    bdata.append(reinterpret_cast<const char*>(&attidata.Heave), sizeof(attidata.Heave));
    bdata.append(reinterpret_cast<const char*>(&attidata.Yaw), sizeof(attidata.Yaw));
    bdata.append(reinterpret_cast<const char*>(&attidata.TimeTag), sizeof(attidata.TimeTag));
    bdata.append(reinterpret_cast<const char*>(&attidata.Heading), sizeof(attidata.Heading));
    bdata.append(reinterpret_cast<const char*>(&attidata.Year), sizeof(attidata.Year));
    bdata.append(reinterpret_cast<const char*>(&attidata.Month), sizeof(attidata.Month));
    bdata.append(reinterpret_cast<const char*>(&attidata.Day), sizeof(attidata.Day));
    bdata.append(reinterpret_cast<const char*>(&attidata.Hour), sizeof(attidata.Hour));
    bdata.append(reinterpret_cast<const char*>(&attidata.Minutes), sizeof(attidata.Minutes));
    bdata.append(reinterpret_cast<const char*>(&attidata.Seconds), sizeof(attidata.Seconds));
    bdata.append(reinterpret_cast<const char*>(&attidata.Milliseconds), sizeof(attidata.Milliseconds));
    bdata.append(reinterpret_cast<const char*>(&attidata.Reserved3), sizeof(attidata.Reserved3));
    mXtfFile->write(bdata);
}
void  ProduceXTF::saveNAVData(const MBINSData& mbinsData)
{
    if(!mXtfFile->isOpen())
        mXtfFile->open(QIODevice::WriteOnly|QIODevice::Append);

    XTFPOSRAWNAVIGITION posdata;
    posdata.MagicNumber = 0xFACE;
    posdata.HeaderType = 107;
    posdata.SubChannelNumber = 0;
    posdata.NumChansToFollow=0;
    posdata.pad2r[0]=0;
    posdata.pad2r[1] = 0;
    posdata.NumBytesThisRecord = 64;
    posdata.Year = mbinsData.Year;
    posdata.Month=mbinsData.Month;
    posdata.Day=mbinsData.Day;
    posdata.Hour=mbinsData.Hour;
    posdata.Minutes = mbinsData.Minute;
    posdata.Seconds=mbinsData.Second;
    posdata.MicroSeconds=mbinsData.Microseconds;
    posdata.RawYcoordinate = mbinsData.latitude;
    posdata.RawXcoordinate = mbinsData.longitude;
    posdata.RawAltitude=mbinsData.heigt;
    posdata.Pitch=0;
    posdata.Roll=0;
    posdata.Heave = 0;
    posdata.Heading = 0;
    posdata.pad1 = 0;

    QByteArray bdata;
    bdata.append(reinterpret_cast<const char*>(&posdata.MagicNumber), sizeof(posdata.MagicNumber));
    bdata.append(reinterpret_cast<const char*>(&posdata.HeaderType), sizeof(posdata.HeaderType));
    bdata.append(reinterpret_cast<const char*>(&posdata.SubChannelNumber), sizeof(posdata.SubChannelNumber));
    bdata.append(reinterpret_cast<const char*>(&posdata.NumChansToFollow), sizeof(posdata.NumChansToFollow));
    bdata.append(reinterpret_cast<const char*>(posdata.pad2r), 2 * sizeof(uint16_t));
    bdata.append(reinterpret_cast<const char*>(&posdata.NumBytesThisRecord), sizeof(posdata.NumBytesThisRecord));
    bdata.append(reinterpret_cast<const char*>(&posdata.Year), sizeof(posdata.Year));
    bdata.append(reinterpret_cast<const char*>(&posdata.Month), sizeof(posdata.Month));
    bdata.append(reinterpret_cast<const char*>(&posdata.Day), sizeof(posdata.Day));
    bdata.append(reinterpret_cast<const char*>(&posdata.Hour), sizeof(posdata.Hour));
    bdata.append(reinterpret_cast<const char*>(&posdata.Minutes), sizeof(posdata.Minutes));
    bdata.append(reinterpret_cast<const char*>(&posdata.Seconds), sizeof(posdata.Seconds));
    bdata.append(reinterpret_cast<const char*>(&posdata.MicroSeconds), sizeof(posdata.MicroSeconds));
    bdata.append(reinterpret_cast<const char*>(&posdata.RawYcoordinate), sizeof(posdata.RawYcoordinate));
    bdata.append(reinterpret_cast<const char*>(&posdata.RawXcoordinate), sizeof(posdata.RawXcoordinate));
    bdata.append(reinterpret_cast<const char*>(&posdata.RawAltitude), sizeof(posdata.RawAltitude));
    bdata.append(reinterpret_cast<const char*>(&posdata.Pitch), sizeof(posdata.Pitch));
    bdata.append(reinterpret_cast<const char*>(&posdata.Roll), sizeof(posdata.Roll));
    bdata.append(reinterpret_cast<const char*>(&posdata.Heave), sizeof(posdata.Heave));
    bdata.append(reinterpret_cast<const char*>(&posdata.Heading), sizeof(posdata.Heading));
    bdata.append(reinterpret_cast<const char*>(&posdata.pad1), sizeof(posdata.pad1));
    mXtfFile->write(bdata);
}

void  ProduceXTF::saveSNIPPETData(const MBINSData& mbinsData)
{
    if(!mXtfFile->isOpen())
        mXtfFile->open(QIODevice::WriteOnly|QIODevice::Append);

    XTFPINGHEADER SNIPPERHEADER;
    SNIPPERHEADER.MagicNumber = 0xFACE;
    SNIPPERHEADER.HeaderType = 65;
    SNIPPERHEADER.SubChannelNumber = 0;
    SNIPPERHEADER.NumChansToFollow=0;
    SNIPPERHEADER.ReservedSpace1[0] = 0;
    SNIPPERHEADER.ReservedSpace1[1] = 0;
    int byteSize = mbinsData.mbdata.size()+256;
    float group =  (float(byteSize)/64.0f);
    int   group2 = int(byteSize/64);
    if(group==group2)
    {
        SNIPPERHEADER.NumBytesThisRecord = byteSize;
    } else {
        SNIPPERHEADER.NumBytesThisRecord = (group2+1)*64;
    }
    int paddedNum =    SNIPPERHEADER.NumBytesThisRecord-   byteSize;
    SNIPPERHEADER.Year = mbinsData.Year;
    SNIPPERHEADER.Month = mbinsData.Month;
    SNIPPERHEADER.Day = mbinsData.Day;
    SNIPPERHEADER.Hour = mbinsData.Hour;
    SNIPPERHEADER.Minute = mbinsData.Minute;
    SNIPPERHEADER.Second = mbinsData.Second;
    SNIPPERHEADER.HSeconds = mbinsData.HSeconds;

    SNIPPERHEADER.JulianDay = mbinsData.JulianDay;
    SNIPPERHEADER.EventNumber = 0;
    SNIPPERHEADER.PingNumber = mbinsData.pingNum;
    SNIPPERHEADER.SoundVelocity = mbinsData.soundSpeed/2;
    SNIPPERHEADER.OceanTide = 0;
    SNIPPERHEADER.Reserved2 = 0;
    SNIPPERHEADER.ConductivityFreq = 0;
    SNIPPERHEADER.TemperatureFreq=0;
    SNIPPERHEADER.PressureFreq=0;
    SNIPPERHEADER.PressureTemp=0;
    SNIPPERHEADER.Conductivity=0;
    SNIPPERHEADER.WaterTemperature=0;
    SNIPPERHEADER.Pressure=0;
    SNIPPERHEADER.ComputedSoundVelocity=0;
    SNIPPERHEADER.MagX=0;
    SNIPPERHEADER.MagY=0;
    SNIPPERHEADER.MagZ=0;
    SNIPPERHEADER.AuxVal1=0;
    SNIPPERHEADER.AuxVal2=0;
    SNIPPERHEADER.AuxVal3=0;
    SNIPPERHEADER.AuxVal4=0;
    SNIPPERHEADER.AuxVal5=0;
    SNIPPERHEADER.AuxVal6=0;
    SNIPPERHEADER.SpeedLog=0;
    SNIPPERHEADER.Turbidity=0;
    SNIPPERHEADER.ShipSpeed=0;
    SNIPPERHEADER.ShipGyro = mbinsData.heading;
    SNIPPERHEADER.ShipYcoordinate = mbinsData.latitude;
    SNIPPERHEADER.ShipXcoordinate = mbinsData.longitude;
    SNIPPERHEADER.ShipAltitude=0;
    SNIPPERHEADER.ShipDepth=0;
    SNIPPERHEADER.FixTimeHour=0;
    SNIPPERHEADER.FixTimeMinute=0;
    SNIPPERHEADER.FixTimeSecond=0;
    SNIPPERHEADER.FixTimeHsecond=0;
    SNIPPERHEADER.SensorSpeed=0;
    SNIPPERHEADER.KP=0;
    SNIPPERHEADER.SensorYcoordinate=mbinsData.latitude;
    SNIPPERHEADER.SensorXcoordinate=mbinsData.longitude;
    SNIPPERHEADER.SonarStatus=0;
    SNIPPERHEADER.RangeToFish=0;
    SNIPPERHEADER.BearingToFish=0;
    SNIPPERHEADER.CableOut=0;
    SNIPPERHEADER.Layback=0;
    SNIPPERHEADER.CableTension=0;
    SNIPPERHEADER.SensorDepth=0;
    SNIPPERHEADER.SensorPrimaryAltitude=0;
    SNIPPERHEADER.SensorAuxAltitude=0;
    SNIPPERHEADER.SensorPitch = mbinsData.pitch;
    SNIPPERHEADER.SensorRoll = mbinsData.roll;
    SNIPPERHEADER.SensorHeading = mbinsData.heading;
    SNIPPERHEADER.Heave = mbinsData.heave;
    SNIPPERHEADER.Yaw = 0;
    SNIPPERHEADER.AttitudeTimeTag = mbinsData.AttTimeTag;
    SNIPPERHEADER.DOT=0;
    SNIPPERHEADER.NavFixMilliseconds=mbinsData.NavTimeTag;
    SNIPPERHEADER.ComputerClockHour=0;
    SNIPPERHEADER.ComputerClockMinute=0;
    SNIPPERHEADER.ComputerClockSecond=0;
    SNIPPERHEADER.ComputerClockHsec=0;
    SNIPPERHEADER.FishPositionDeltaX=0;
    SNIPPERHEADER.FishPositionDeltaY=0;
    SNIPPERHEADER.FishPositionErrorCode=0;
    SNIPPERHEADER.OptionalOffsey=0;
    SNIPPERHEADER.CableOutHundredths=0;

    {
        memset( SNIPPERHEADER.ReservedSpace1r, 0, 6);
    }

    QByteArray bdata;
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.MagicNumber), sizeof(SNIPPERHEADER.MagicNumber));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.HeaderType),sizeof(SNIPPERHEADER.HeaderType));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SubChannelNumber), sizeof(SNIPPERHEADER.SubChannelNumber));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.NumChansToFollow), sizeof(SNIPPERHEADER.NumChansToFollow));
    bdata.append(reinterpret_cast<const char*>(SNIPPERHEADER.ReservedSpace1), 2 * sizeof(uint16_t));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.NumBytesThisRecord), sizeof(SNIPPERHEADER.NumBytesThisRecord));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Year), sizeof(SNIPPERHEADER.Year));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Month),sizeof(SNIPPERHEADER.Month));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Day), sizeof(SNIPPERHEADER.Day));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Hour), sizeof(SNIPPERHEADER.Hour));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Minute),sizeof(SNIPPERHEADER.Minute));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Second), sizeof(SNIPPERHEADER.Second));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.HSeconds), sizeof(SNIPPERHEADER.HSeconds));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.JulianDay), sizeof(SNIPPERHEADER.JulianDay));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.EventNumber),sizeof(SNIPPERHEADER.EventNumber));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.PingNumber), sizeof(SNIPPERHEADER.PingNumber));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SoundVelocity), sizeof(SNIPPERHEADER.SoundVelocity));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.OceanTide),sizeof(SNIPPERHEADER.OceanTide));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Reserved2), sizeof(SNIPPERHEADER.Reserved2));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ConductivityFreq), sizeof(SNIPPERHEADER.ConductivityFreq));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.TemperatureFreq), sizeof(SNIPPERHEADER.TemperatureFreq));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.PressureFreq),sizeof(SNIPPERHEADER.PressureFreq));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.PressureTemp),sizeof(SNIPPERHEADER.PressureTemp));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Conductivity), sizeof(SNIPPERHEADER.Conductivity));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.WaterTemperature), sizeof(SNIPPERHEADER.WaterTemperature));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Pressure), sizeof(SNIPPERHEADER.Pressure));

    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ComputedSoundVelocity), sizeof(SNIPPERHEADER.ComputedSoundVelocity));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.MagX), sizeof(SNIPPERHEADER.MagX));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.MagY), sizeof(SNIPPERHEADER.MagY));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.MagZ), sizeof(SNIPPERHEADER.MagZ));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.AuxVal1), sizeof(SNIPPERHEADER.AuxVal1));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.AuxVal2), sizeof(SNIPPERHEADER.AuxVal2));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.AuxVal3), sizeof(SNIPPERHEADER.AuxVal3));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.AuxVal4), sizeof(SNIPPERHEADER.AuxVal4));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.AuxVal5), sizeof(SNIPPERHEADER.AuxVal5));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.AuxVal6), sizeof(SNIPPERHEADER.AuxVal6));

    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SpeedLog), sizeof(SNIPPERHEADER.SpeedLog));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Turbidity), sizeof(SNIPPERHEADER.Turbidity));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ShipSpeed), sizeof(SNIPPERHEADER.ShipSpeed));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ShipGyro), sizeof(SNIPPERHEADER.ShipGyro));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ShipYcoordinate), sizeof(SNIPPERHEADER.ShipYcoordinate));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ShipXcoordinate), sizeof(SNIPPERHEADER.ShipXcoordinate));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ShipAltitude), sizeof(SNIPPERHEADER.ShipAltitude));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ShipDepth), sizeof(SNIPPERHEADER.ShipDepth));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.FixTimeHour), sizeof(SNIPPERHEADER.FixTimeHour));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.FixTimeMinute), sizeof(SNIPPERHEADER.FixTimeMinute));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.FixTimeSecond), sizeof(SNIPPERHEADER.FixTimeSecond));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.FixTimeHsecond), sizeof(SNIPPERHEADER.FixTimeHsecond));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SensorSpeed), sizeof(SNIPPERHEADER.SensorSpeed));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.KP), sizeof(SNIPPERHEADER.KP));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SensorYcoordinate), sizeof(SNIPPERHEADER.SensorYcoordinate));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SensorXcoordinate), sizeof(SNIPPERHEADER.SensorXcoordinate));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SonarStatus), sizeof(SNIPPERHEADER.SonarStatus));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.RangeToFish), sizeof(SNIPPERHEADER.RangeToFish));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.BearingToFish), sizeof(SNIPPERHEADER.BearingToFish));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.CableOut), sizeof(SNIPPERHEADER.CableOut));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Layback), sizeof(SNIPPERHEADER.Layback));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.CableTension), sizeof(SNIPPERHEADER.CableTension));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SensorDepth), sizeof(SNIPPERHEADER.SensorDepth));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SensorPrimaryAltitude), sizeof(SNIPPERHEADER.SensorPrimaryAltitude));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SensorAuxAltitude), sizeof(SNIPPERHEADER.SensorAuxAltitude));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SensorPitch), sizeof(SNIPPERHEADER.SensorPitch));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SensorRoll), sizeof(SNIPPERHEADER.SensorRoll));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.SensorHeading), sizeof(SNIPPERHEADER.SensorHeading));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Heave), sizeof(SNIPPERHEADER.Heave));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.Yaw), sizeof(SNIPPERHEADER.Yaw));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.AttitudeTimeTag), sizeof(SNIPPERHEADER.AttitudeTimeTag));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.DOT), sizeof(SNIPPERHEADER.DOT));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.NavFixMilliseconds), sizeof(SNIPPERHEADER.NavFixMilliseconds));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ComputerClockHour), sizeof(SNIPPERHEADER.ComputerClockHour));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ComputerClockMinute), sizeof(SNIPPERHEADER.ComputerClockMinute));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ComputerClockSecond), sizeof(SNIPPERHEADER.ComputerClockSecond));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.ComputerClockHsec), sizeof(SNIPPERHEADER.ComputerClockHsec));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.FishPositionDeltaX), sizeof(SNIPPERHEADER.FishPositionDeltaX));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.FishPositionDeltaY), sizeof(SNIPPERHEADER.FishPositionDeltaY));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.FishPositionErrorCode), sizeof(SNIPPERHEADER.FishPositionErrorCode));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.OptionalOffsey), sizeof(SNIPPERHEADER.OptionalOffsey));
    bdata.append(reinterpret_cast<const char*>(&SNIPPERHEADER.CableOutHundredths), sizeof(SNIPPERHEADER.CableOutHundredths));

    {
        bdata.append(reinterpret_cast<const char*>(SNIPPERHEADER.ReservedSpace1r), 6);
    }
    mXtfFile->write(bdata);
    mXtfFile->write(mbinsData.mbdata);
    QByteArray paddedChar;
    for(int i=0;i<paddedNum;i++)
    {
        char padde = '\0';
        paddedChar.append(padde);
    }
    if(paddedChar.size()!=0)
    {
        mXtfFile->write(paddedChar);
    }
}

void ProduceXTF::sltCloseFile(bool bclear)
{
    if(mXtfFile)
    {        
        if(mXtfFile->isWritable())
        {
            if(mXtfFile->isOpen())
            {
                mXtfFile->close();
            }
        }
    }
    if (bclear)
    {
        mInsSecond = -1;
        mMbsecond = -1;
        mInsProflag = true;

        mSpanVec.clear();
        mMbVec.clear();
        mMbInsData.clear();
    }

}

}
