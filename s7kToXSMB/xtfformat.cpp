#include "xtfformat.h"
#include <QDebug>
#include <QDateTime>
#include <QtMath>
//#include "comfun.h"
//#include "Constants.h"
namespace vsrxtf {

void XTFFORMAT::parseXTFPingHeader(QByteArray &bdata, const int sizeOff, XTFBATHHEADER &bathydata)
{
    const char * data = bdata.constData() + sizeOff;
    memcpy(&bathydata.Year, data + 14, 2);
    memcpy(&bathydata.Month, data + 16, 1);
    memcpy(&bathydata.Day, data + 17, 1);
    memcpy(&bathydata.Hour, data + 18, 1);
    memcpy(&bathydata.Minute, data + 19, 1);
    memcpy(&bathydata.Second, data + 20, 1);
    memcpy(&bathydata.HSeconds, data + 21, 1);
    memcpy(&bathydata.JulianDay, data + 22, 2);
    memcpy(&bathydata.EventNumber, data + 24, 4);
    memcpy(&bathydata.PingNumber, data + 28, 4);
    memcpy(&bathydata.SoundVelocity, data + 32, 4);
    memcpy(&bathydata.OceanTide, data + 36, 4);
    memcpy(&bathydata.Reserved2, data + 40, 4);
    memcpy(&bathydata.ConductivityFreq, data + 44, 4);
    memcpy(&bathydata.TemperatureFreq, data + 48, 4);
    memcpy(&bathydata.PressureFreq, data + 52, 4);
    memcpy(&bathydata.PressureTemp, data + 56, 4);
    memcpy(&bathydata.Conductivity, data + 60, 4);
    memcpy(&bathydata.WaterTemperature, data + 64, 4);
    memcpy(&bathydata.Pressure, data + 68, 4);
    memcpy(&bathydata.ComputedSoundVelocity, data + 72, 4);
    memcpy(&bathydata.MagX, data + 76, 4);
    memcpy(&bathydata.MagY, data + 80, 4);
    memcpy(&bathydata.MagZ, data + 84, 4);
    memcpy(&bathydata.AuxVal1, data + 88, 4);
    memcpy(&bathydata.AuxVal2, data + 92, 4);
    memcpy(&bathydata.AuxVal3, data + 96, 4);
    memcpy(&bathydata.AuxVal4, data + 100, 4);
    memcpy(&bathydata.AuxVal5, data + 104, 4);
    memcpy(&bathydata.AuxVal6, data + 108, 4);
    memcpy(&bathydata.SpeedLog, data + 112, 4);
    memcpy(&bathydata.Turbidity, data + 116, 4);
    memcpy(&bathydata.ShipSpeed, data + 120, 4);
    memcpy(&bathydata.ShipGyro, data + 124, 4);
    memcpy(&bathydata.ShipYcoordinate, data + 128, 8);
    memcpy(&bathydata.ShipXcoordinate, data + 136, 8);
    memcpy(&bathydata.ShipAltitude, data + 144, 2);
    memcpy(&bathydata.ShipDepth, data + 146, 2);
    memcpy(&bathydata.FixTimeHour, data + 148, 1);
    memcpy(&bathydata.FixTimeMinute, data + 149, 1);
    memcpy(&bathydata.FixTimeSecond, data + 150, 1);
    memcpy(&bathydata.FixTimeHsecond, data + 151, 1);
    memcpy(&bathydata.SensorSpeed, data + 152, 4);
    memcpy(&bathydata.KP, data + 156, 4);
    memcpy(&bathydata.SensorYcoordinate, data + 160, 8);
    memcpy(&bathydata.SensorXcoordinate, data + 168, 8);
    memcpy(&bathydata.SonarStatus, data + 176, 2);
    memcpy(&bathydata.RangeToFish, data + 178, 2);
    memcpy(&bathydata.BearingToFish, data + 180, 2);
    memcpy(&bathydata.CableOut, data + 182, 2);
    memcpy(&bathydata.Layback, data + 184, 4);
    memcpy(&bathydata.CableTension, data + 188, 4);
    memcpy(&bathydata.SensorDepth, data + 192, 4);
    memcpy(&bathydata.SensorPrimaryAltitude, data + 196, 4);
    memcpy(&bathydata.SensorAuxAltitude, data + 200, 4);
    memcpy(&bathydata.SensorPitch, data + 204, 4);
    memcpy(&bathydata.SensorRoll, data + 208, 4);
    memcpy(&bathydata.SensorHeading, data + 212, 4);
    memcpy(&bathydata.Heave, data + 216, 4);
    memcpy(&bathydata.Yaw, data + 220, 4);
    memcpy(&bathydata.AttitudeTimeTag, data + 224, 4);
    memcpy(&bathydata.DOT, data + 228, 4);
    memcpy(&bathydata.NavFixMilliseconds, data + 232, 4);
    memcpy(&bathydata.ComputerClockHour, data + 236, 1);
    memcpy(&bathydata.ComputerClockMinute, data + 237, 1);
    memcpy(&bathydata.ComputerClockSecond, data + 238, 1);
    memcpy(&bathydata.ComputerClockHsec, data + 239, 1);
    memcpy(&bathydata.FishPositionDeltaX, data + 240, 2);
    memcpy(&bathydata.FishPositionDeltaY, data + 242, 2);
    memcpy(&bathydata.FishPositionErrorCode, data + 244, 1);
    memcpy(&bathydata.OptionalOffsey, data + 245, 4);
    memcpy(&bathydata.CableOutHundredths, data + 249, 1);
    memcpy(bathydata.ReservedSpace1r, data + 250, 6);

}

void XTFFORMAT::parseBathyDataToMBStruct(QByteArray &datagram, int sizeOff, SONICRAWSTRUCT &Sodabathydata)
{

    char       *buffer = datagram.data() + sizeOff;
    memcpy(Sodabathydata.packetName, buffer + 0, 4);
    byteChange4((char*)&Sodabathydata.packetSize, buffer + 4);

    int sizeCount = 12;
    //-------------------------------------H0_Section--------------------------------------------//
    // uint16_t  H0_SectionName;                // 'H0'=12360
    sizeCount += 2;
    // uint16_t  H0_SectionSize;                // [bytes] size of this entire section
    sizeCount += 2;
    // char   H0_ModelNumber[12];          // example "2024", unused chars are nulls
    sizeCount += 12;
    // char   H0_SerialNumber[12];         // example "100017", unused chars are nulls
    sizeCount += 12;
    // [seconds] ping 时间整数部分 相对于 0000 hours 1-Jan-1970, integer part//
    uint32_t  H0_TimeSeconds;
    byteChange4((char*)&H0_TimeSeconds, buffer + sizeCount);

    sizeCount += 4;

    // [nanoseconds] ping 时间小数部分 相对于 to 0000 hours 1-Jan-1970, fraction part//
    uint32_t  H0_TimeNanoseconds;
    byteChange4((char*)&H0_TimeNanoseconds, buffer + sizeCount);

    sizeCount += 4;
    qint64  milliseconds = H0_TimeSeconds;
    milliseconds *= 1000;
    milliseconds += (H0_TimeNanoseconds / 1000000);//??
    QDateTime datetime = QDateTime::fromMSecsSinceEpoch(milliseconds);
    datetime = datetime.toUTC();

    Sodabathydata.Year = datetime.date().year();
    Sodabathydata.Month = uint8_t(datetime.date().month());
    Sodabathydata.Day = uint8_t(datetime.date().day());

    Sodabathydata.Hour = datetime.time().hour();
    Sodabathydata.Minute = datetime.time().minute();
    Sodabathydata.Second = datetime.time().second() + double(datetime.time().msec()) / 1000.0;

    //  pingnums since power-up or reboot  //
    byteChange4((char*)&Sodabathydata.pingNum, buffer + sizeCount);
    sizeCount += 4;
    // [seconds] time between most recent two pings//
    byteChange4((char*)&Sodabathydata.pingPeriod, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.soundSpeed, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.frequency, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.txPower, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.txPulseWidth, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.txbeamWidthVert, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.txBeamWidthHoriz, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.txSteeringVert, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.txSteeringHoriz, buffer + sizeCount);
    sizeCount += 4;

    // uint32_t  H0_TxMiscInfo;               // to be determined
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.rxBandWidth, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.rxSampleRate, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.rxRange, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.rxGain, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.rxSpreading, buffer + sizeCount);
    sizeCount += 4;

    byteChange4((char*)&Sodabathydata.rxAbsorption, buffer + sizeCount);
    sizeCount += 4;

    // float  H0_RxMountTilt;              // [radians]
    byteChange4((char*)&Sodabathydata.rxMountTilt, buffer + sizeCount);
    sizeCount += 4;
    // uint32_t  H0_RxMiscInfo;               // to be determined
    sizeCount += 4;
    //uint16_t  H0_reserved;                 // reserved for future use
    sizeCount += 2;

    byteChange2((char*)&Sodabathydata.points, buffer + sizeCount);
    sizeCount += 2;
    //-----------/***      R0_Section    ***/------------//
    uint16_t  R0_SectionName;              // 'R0'=12370
    byteChange2((char*)&R0_SectionName, buffer + sizeCount);

    sizeCount += 2;
    uint16_t  R0_SectionSize;              // [bytes] size of this entire section
    byteChange2((char*)&R0_SectionSize, buffer + sizeCount);
    sizeCount += 2;

    float  R0_ScalingFactor;
    byteChange4((char*)&R0_ScalingFactor, buffer + sizeCount);
    sizeCount += 4;
    //    inparse<<"R0_ScalingFactor:"<<R0_ScalingFactor<<'\n';

    // [seconds two-way] = R0_Range * R0_ScalingFactor
    for (int i = 0; i < Sodabathydata.points; i++)
    {
        uint16_t    _range = 0;
        byteChange2((char*)&_range, buffer + sizeCount);
        sizeCount += 2;
        Sodabathydata.range[i] = _range*R0_ScalingFactor;

    }
    uint16_t  temp;
    temp = Sodabathydata.points & 1;
    if (temp != 0)
    {
        sizeCount += 2;
    }
    //----------------等角模式--------------------//

    uint16_t sectionpart;
    memcpy(&sectionpart, buffer + sizeCount, 2);
    sizeCount += 2;
    if (sectionpart == A0) {              /***      A0_Section  'A0'=12353  ***/

        //                uint16_t  A0_SectionSize;              // [bytes] size of this entire section
        sizeCount += 2;
        float  A0_AngleFirst;               // [radians] angle of first (port side) bathy point, relative to array centerline, AngleFirst < AngleLast
        byteChange4((char*)&A0_AngleFirst, buffer + sizeCount);
        sizeCount += 4;
        float  A0_AngleLast;                // [radians] angle of last (starboard side) bathy point
        byteChange4((char*)&A0_AngleLast, buffer + sizeCount);
        sizeCount += 4;

        //                float  A0_MoreInfo[6];              // several to-be-determined values such as attitude info to assist the GUI display
        for (int i = 0; i < 6; i++)
        {
            sizeCount += 4;
        }

        Sodabathydata.beamModel = EQUALANGLE;
        for (int i = 0; i < Sodabathydata.points; i++)
        {
            double angle = (A0_AngleLast - A0_AngleFirst) / (Sodabathydata.points - 1);
            Sodabathydata.angles[i] = float(A0_AngleFirst + i*angle);
        }

    }
    else if (sectionpart == A2) {      /***      A2_Section   'A2'=12865 ***/
        Sodabathydata.beamModel = EQUALDISTANCE;

        uint16_t  A2_SectionSize;              // [bytes] size of this entire section
        byteChange2((char*)&A2_SectionSize, buffer + sizeCount);
        sizeCount += 2;

        float  A2_AngleFirst;               // [radians] angle of first (port side) bathy point, relative to array centerline, AngleFirst < AngleLast
        byteChange4((char*)&A2_AngleFirst, buffer + sizeCount);
        sizeCount += 4;

        float  A2_ScalingFactor;
        byteChange4((char*)&A2_ScalingFactor, buffer + sizeCount);
        sizeCount += 4;

        for (int i = 0; i < 6; i++)
        {
            sizeCount += 4;
        }

        //-------A2_AngleStep----- [radians] angle[n] = (32-bit sum of A2_AngleStep[0] through A2_AngleStep[n]) * A2_ScalingFactor
        int anglestepSum = 0;
        for (int i = 0; i < Sodabathydata.points; i++)
        {
            uint16_t    _angleStep = 0;
            byteChange2((char*)&_angleStep, buffer + sizeCount);
            sizeCount += 2;

            anglestepSum += _angleStep;
            Sodabathydata.angles[i] = A2_AngleFirst + anglestepSum*A2_ScalingFactor;
        }

        uint16_t  temp1;
        temp1 = Sodabathydata.points & 1;
        if (temp1 != 0)
        {
            sizeCount += 2;
        }
    }


    /***      I1_Section    ***/

    memcpy(&sectionpart, buffer + sizeCount, 2);
    sizeCount += 2;
    unsigned char intensityFlagtemp = INTENSITYNO;
    if (sectionpart == I1)           // 'I1'=12617
    {
        intensityFlagtemp = INTENSITYDO;

        sizeCount += 2;

        byteChange4((char*)&Sodabathydata.InScalFactor, buffer + sizeCount);
        sizeCount += 4;
        
        for (int i = 0; i < Sodabathydata.points; i++)
        {
            uint16_t I1_Intensity = 0;
            byteChange2((char*)&I1_Intensity, buffer + sizeCount);
            sizeCount += 2;
            Sodabathydata.intensity[i] = I1_Intensity*Sodabathydata.InScalFactor;

        }
        uint16_t  temp2;
        temp2 = Sodabathydata.points & 1;
        if (temp2 != 0)
        {
            sizeCount += 2;
        }
    }

    /***      G0_Section    ***/

    //            uint16_t  G0_SectionName;              // 'G0'
    //            memcpy(&G0_SectionName, buffer+sizeCount, 2);
    sizeCount += 2;
    //qDebug()<<"G0_SectionName:"<<G0_SectionName;
    //            uint16_t  G0_SectionSize;              // [bytes] size of this entire section
    //            byteChange2((char*)&G0_SectionSize,buffer+sizeCount,2);

    sizeCount += 2;
    //qDebug()<<"G0_SectionSize:"<<G0_SectionSize;
    //            float  G0_DepthGateMin;             // [seconds two-way]
    //            byteChange4((char*)&G0_DepthGateMin,buffer+sizeCount,4);

    sizeCount += 4;
    //qDebug()<<"G0_DepthGateMin:"<<G0_DepthGateMin;
    //            float  G0_DepthGateMax;             // [seconds two-way]
    //            byteChange4((char*)&G0_DepthGateMax,buffer+sizeCount,4);

    sizeCount += 4;
    //qDebug()<<"G0_DepthGateMax:"<<G0_DepthGateMax;
    //            float  G0_DepthGateSlope;           // [radians]
    //            byteChange4((char*)&G0_DepthGateSlope,buffer+sizeCount,4);

    sizeCount += 4;
    //qDebug()<<"G0_DepthGateSlope:"<<G0_DepthGateSlope;
    /***      G1_Section    // 'G1'***/
    memcpy(&sectionpart, buffer + sizeCount, 2);
    sizeCount += 2;
    if (sectionpart == G1)
    {

        //                uint16_t  G1_SectionSize;              // [bytes] size of this entire section
        //                byteChange2((char*)&G1_SectionSize,buffer+sizeCount );

        sizeCount += 2;

        //                float  G1_ScalingFactor;
        //                byteChange4((char*)&G1_ScalingFactor,buffer+sizeCount,4);

        sizeCount += 4;


        //                QVector <GATE>           G1_Gate;
        for (int i = 0; i < Sodabathydata.points; i++)
        {
            //                    GATE    tempGate;
            //                   tempGate.RangeMin=buffer[sizeCount];
            sizeCount += 1;
            //                  tempGate.RangeMax=buffer[sizeCount];
            sizeCount += 1;
            //                    G1_Gate.append(tempGate);

        }

        uint16_t  temp2;
        temp2 = Sodabathydata.points & 1;
        if (temp2 != 0)
        {
            //                    uint16_t  G1_unused;    // ensure 32-bit section size
            //                    byteChange2((char*)&G1_unused,buffer+sizeCount );
            sizeCount += 2;

        }
    }
    else {
        sizeCount -= 2;
    }

    /***      Q0_Section    ***/

    char  Q0_SectionName[2];              // 'Q0' quality, 4-bit
    memcpy(Q0_SectionName, buffer + sizeCount, 2);
    sizeCount += 2;
    uint16_t  Q0_SectionSize;              // [bytes] size of this entire section
    byteChange2((char*)&Q0_SectionSize, buffer + sizeCount);

    sizeCount += 2;
    uint16_t  groups;
    groups = (Sodabathydata.points + 7) / 8;

    // 8 groups of 4 flags bits (phase detect, magnitude detect, colinearity, brightness), packed left-to-right

    //            for(int i=0;i<groups;i++)
    //            {
    //                uint32_t   _Quality=0;
    //                byteChange4((char*)&_Quality,buffer+sizeCount );
    //                sizeCount +=4;
    //            }
    for (int i = 0; i < Sodabathydata.points / 2; i++)
    {
        //byteChange1(at2,buffer+sizeCount );
        uint8_t _Quality = buffer[sizeCount];
        sizeCount += 1;

        uint8_t tempQ1 = _Quality >> 4;
        uint8_t tempQ2 = _Quality & 0x0F;

        //                qDebug()<<QString("%1").arg(tempQ1,0,16).toUpper();
        //                qDebug()<<QString("%1").arg(tempQ2,0,16).toUpper();
        Sodabathydata.quality[2 * i] = tempQ1;
        Sodabathydata.quality[2 * i + 1] = tempQ2;
    }

}

void XTFFORMAT::printBathyPing(XTFBATHHEADER &bathydata, SONICRAWSTRUCT &Sodabathydata, QTextStream &outTXT)
{
    outTXT << "\nNum Bytes:" << bathydata.NumBytesThisRecord << '\n';
    outTXT << "Heaser Type:" << bathydata.HeaderType << '\n';
    outTXT << "Subchannel Number:" << bathydata.SubChannelNumber << '\n';
    outTXT << "NumChans To Follow:" << bathydata.NumChansToFollow << '\n';
    outTXT << "R2Sonic Number:" << bathydata.Year << "-" << bathydata.JulianDay << "  "
           << bathydata.Month << "/" << bathydata.Day << "  "
           << bathydata.Hour << ":" << bathydata.Minute << ":" << bathydata.Second + double(bathydata.HSeconds) / 100.0 << "  "
           << "ATT Time:" << bathydata.AttitudeTimeTag << "  Nav Time:" << bathydata.NavFixMilliseconds << '\n';
    outTXT << "tide/lat(ship)/long(ship)/lat(sensor)/long(sensor)/shiphead/fishhead:"
           << bathydata.OceanTide << "/";
    outTXT.setRealNumberPrecision(9);
    outTXT << bathydata.ShipYcoordinate << "/" << bathydata.ShipXcoordinate << "/"
           << bathydata.SensorYcoordinate << "/" << bathydata.SensorXcoordinate << "/";
    outTXT.setRealNumberPrecision(6);
    outTXT << bathydata.ShipGyro << "/" << bathydata.SensorHeading << '\n';
    outTXT.setRealNumberPrecision(6);
    outTXT << "Heave/Pitch/Roll/Yaw:" << bathydata.Heave << "/" << bathydata.SensorPitch << "/" << bathydata.SensorRoll << "/" << bathydata.Yaw << '\n';
    outTXT << "Freq Cond/Temp/Pres:" << bathydata.ConductivityFreq << "/" << bathydata.TemperatureFreq << "/" << bathydata.PressureFreq << '\n';
    outTXT << "PresTemp/WatTemp/Conductivity/Pressure:" << bathydata.PressureTemp << "/" << bathydata.WaterTemperature << "/" << bathydata.Conductivity << "/" << bathydata.Pressure << '\n';
    outTXT << "SV/Cable/LayBk/CT/Aux1/2/3/4/5/6:" << bathydata.SoundVelocity << "/" << bathydata.Layback << "/" << bathydata.CableTension << "/"
           << bathydata.AuxVal1 << "/"
           << bathydata.AuxVal2 << "/"
           << bathydata.AuxVal3 << "/"
           << bathydata.AuxVal4 << "/"
           << bathydata.AuxVal5 << "/"
           << bathydata.AuxVal6 << "\n";
    outTXT << "Ship Depth/Altitude/Speed:" << bathydata.ShipDepth << "/" << bathydata.ShipAltitude << "/" << bathydata.ShipSpeed << '\n';
    outTXT << "sensor Depth/Aux Altitude/Speed:" << bathydata.SensorDepth << "/" << bathydata.SensorAuxAltitude << "/" << bathydata.SensorSpeed << '\n';
    outTXT.setRealNumberPrecision(7);
    outTXT << "Time stamp:" << bathydata.Year << "-" << bathydata.JulianDay << "  "
           << bathydata.Hour << ":" << bathydata.Minute << ":"
           << bathydata.Second + double(bathydata.HSeconds) / 100.0 << '\n';
    outTXT << "Ping Number:" << bathydata.PingNumber << '\n';
    outTXT.setRealNumberPrecision(8);
    outTXT << "Sound speed:" << 2 * bathydata.SoundVelocity << '\n';



    outTXT << "Packet Name:" << Sodabathydata.packetName[0]
           << Sodabathydata.packetName[1]
           << Sodabathydata.packetName[2]
           << Sodabathydata.packetName[3] << '\n';
    outTXT << "Packet Size:" << Sodabathydata.packetSize << '\n';
    outTXT << "Ping Time:" << Sodabathydata.Year << "-"
           << Sodabathydata.Month << "-"
           << Sodabathydata.Day << "  "
           << Sodabathydata.Hour << ":"
           << Sodabathydata.Minute << ":"
           << Sodabathydata.Second << '\n';
    outTXT << "Ping Num:" << Sodabathydata.pingNum << '\n';
    outTXT << "Ping Points:" << Sodabathydata.points << '\n';
    outTXT << "Ping Period/sound/frenquency:" << Sodabathydata.pingPeriod << "/"
           << Sodabathydata.soundSpeed << "/"
           << Sodabathydata.frequency << '\n';
    if (Sodabathydata.beamModel == EQUALANGLE)
    {
        outTXT << "Beam Model:" << "Equal ANGLE" << '\n';
    }
    else {
        outTXT << "Beam Model:" << "Equal DISTANCE" << '\n';
    }
    outTXT << "tx power/PW/BWV/BWH/SteerV/SteerH:" << Sodabathydata.txPower << "/ "
           << Sodabathydata.txPulseWidth << "/ "
           << Sodabathydata.txbeamWidthVert << "/ "
           << Sodabathydata.txBeamWidthHoriz << "/ "
           << Sodabathydata.txSteeringVert << "/ "
           << Sodabathydata.txSteeringHoriz << '\n';
    outTXT << "rx BW/SR/R/Gain/Spr/Abs/MT:" << Sodabathydata.rxBandWidth << "/ "
           << Sodabathydata.rxSampleRate << "/ "
           << Sodabathydata.rxRange << "/ "
           << Sodabathydata.rxGain << "/ "
           << Sodabathydata.rxSpreading << "/ "
           << Sodabathydata.rxAbsorption << "/ "
           << Sodabathydata.rxMountTilt << '\n';
    outTXT << "Intensity Scaling Factor:" << Sodabathydata.InScalFactor << '\n';
    outTXT << "\n #   X Angle       TT          Across     Depth  Intensity  Q" << '\n';
    outTXT << "================================================================" << '\n';
    outTXT.setPadChar(' ');
    for (int i = 0; i < Sodabathydata.points; i++)
    {
        float tempX = Sodabathydata.range[i] * Sodabathydata.soundSpeed*sin(Sodabathydata.angles[i])*0.5;
        float tempZ = Sodabathydata.range[i] * Sodabathydata.soundSpeed*cos(Sodabathydata.angles[i])*0.5;
        outTXT.setFieldWidth(3);
        outTXT.setFieldAlignment(QTextStream::AlignRight);
        outTXT << i + 1;

        outTXT.setFieldWidth(8);
        outTXT.setFieldAlignment(QTextStream::AlignRight);
        outTXT << QString::number(Sodabathydata.angles[i] * 180 / M_PI, 'f', 2);

        outTXT.setFieldWidth(14);
        outTXT.setFieldAlignment(QTextStream::AlignRight);
        outTXT << QString::number(Sodabathydata.range[i] / 2, 'f', 9);


        outTXT.setFieldWidth(12);
        outTXT.setFieldAlignment(QTextStream::AlignRight);
        outTXT << QString::number(tempX, 'f', 3);


        outTXT.setFieldWidth(10);
        outTXT.setFieldAlignment(QTextStream::AlignRight);
        outTXT << QString::number(tempZ, 'f', 3);


        outTXT.setFieldWidth(8);
        outTXT.setFieldAlignment(QTextStream::AlignRight);
        outTXT << QString::number(Sodabathydata.intensity[i]);


        outTXT.setFieldWidth(7);
        outTXT.setFieldAlignment(QTextStream::AlignRight);
        outTXT << showbase << uppercasedigits << hex << Sodabathydata.quality[i] << dec;

        outTXT.setFieldWidth(0);
        outTXT.setFieldAlignment(QTextStream::AlignCenter);
        outTXT << '\n';
    }
    outTXT << "================================================================" << '\n';

}

void XTFFORMAT::parseAttitudeToStruct(QByteArray &bdata, XTFATTITUDEDATA &attidata, int sizeOff, QTextStream &outTXT)
{
    const char* data = bdata.constData() + sizeOff;
    memcpy(attidata.Reserved2, data + 14, 8);
    memcpy(&attidata.EpochMicroseconds, data + 22, 4);
    memcpy(&attidata.SourceEpoch, data + 26, 4);
    memcpy(&attidata.Pitch, data + 30, 4);
    memcpy(&attidata.Roll, data + 34, 4);
    memcpy(&attidata.Heave, data + 38, 4);
    memcpy(&attidata.Yaw, data + 42, 4);
    memcpy(&attidata.TimeTag, data + 46, 4);
    memcpy(&attidata.Heading, data + 50, 4);
    memcpy(&attidata.Year, data + 54, 2);
    memcpy(&attidata.Month, data + 56, 1);
    memcpy(&attidata.Day, data + 57, 1);
    memcpy(&attidata.Hour, data + 58, 1);
    memcpy(&attidata.Minutes, data + 59, 1);
    memcpy(&attidata.Seconds, data + 60, 1);
    memcpy(&attidata.Milliseconds, data + 61, 2);
    memcpy(&attidata.Reserved3, data + 63, 1);

    outTXT << "\nNum Bytes:" << attidata.NumBytesThisRecord << '\n';
    outTXT << "Header Type:" << attidata.HeaderType << '\n';
    outTXT << "Subchannel Number:" << attidata.SubChannelNumber << '\n';
    outTXT << "NumChans To Follow:" << attidata.NumChansToFollow << '\n';
    outTXT << "ATT Time:" << attidata.Year << "-"
           << attidata.Month << "-"
           << attidata.Day << "  "
           << attidata.Hour << ":"
           << attidata.Minutes << ":"
           << attidata.Seconds + float(attidata.Milliseconds / 1000.0) << '\n';
    outTXT << "ATT TimeTag/Source Epoch/Epoch Microseconds:" << attidata.TimeTag << "/"
           << attidata.SourceEpoch << "/"
           << attidata.EpochMicroseconds << '\n';
    outTXT << "Heading/Roll/Pitch/Yaw/Heave:" << attidata.Heading << "/"
           << attidata.Roll << "/"
           << attidata.Pitch << "/"
           << attidata.Yaw << "/"
           << attidata.Heave << '\n';
}

void XTFFORMAT::parsePosToStruct(QByteArray &bdata, XTFPOSRAWNAVIGITION &posdata, int sizeOff, QTextStream &outTXT)
{
    const char* data = bdata.constData() + sizeOff;
    memcpy(&posdata.Year, data + 14, 2);
    memcpy(&posdata.Month, data + 16, 1);
    memcpy(&posdata.Day, data + 17, 1);
    memcpy(&posdata.Hour, data + 18, 1);
    memcpy(&posdata.Minutes, data + 19, 1);
    memcpy(&posdata.Seconds, data + 20, 1);
    memcpy(&posdata.MicroSeconds, data + 21, 2);
    memcpy(&posdata.RawYcoordinate, data + 23, 8);
    memcpy(&posdata.RawXcoordinate, data + 31, 8);
    memcpy(&posdata.RawAltitude, data + 39, 8);
    memcpy(&posdata.Pitch, data + 47, 4);
    memcpy(&posdata.Roll, data + 51, 4);
    memcpy(&posdata.Heave, data + 55, 4);
    memcpy(&posdata.Heading, data + 59, 4);
    memcpy(&posdata.pad1, data + 63, 1);

    outTXT << "\nNum Bytes:" << posdata.NumBytesThisRecord << '\n';
    outTXT << "Header Type:" << posdata.HeaderType << '\n';
    outTXT << "Subchannel Number:" << posdata.SubChannelNumber << '\n';
    outTXT << "NumChans To Follow:" << posdata.NumChansToFollow << '\n';
    outTXT << "ATT Time:" << posdata.Year << "-"
           << posdata.Month << "-"
           << posdata.Day << "  "
           << posdata.Hour << ":"
           << posdata.Minutes << ":"
           << posdata.Seconds + float(posdata.MicroSeconds / 10000.0) << '\n';
    outTXT << "Raw Y/X/Altitude:" << posdata.RawYcoordinate << "/"
           << posdata.RawXcoordinate << "/"
           << posdata.RawAltitude << '\n';
    outTXT << "Heading/Roll/Pitch/Heave:" << posdata.Heading << "/"
           << posdata.Roll << "/"
           << posdata.Pitch << "/"
           << posdata.Heave << '\n';
}

void XTFFORMAT::parseXTFFileHeader(QByteArray &bdata, XTFFILEHEADER &xtfHeader, QTextStream &outTXT)
{
    const char* data = bdata.constData();

    memcpy(&xtfHeader.FileFormat, data + 0, 1);
    memcpy(&xtfHeader.SystemType, data + 1, 1);
    memcpy(xtfHeader.RecordingProgramName, data + 2, 8);
    memcpy(xtfHeader.RecordingProgramVersion, data + 10, 8);
    memcpy(xtfHeader.SonarName, data + 18, 16);
    memcpy(&xtfHeader.SonarType, data + 34, 2);
    memcpy(xtfHeader.NoteString, data + 36, 64);
    memcpy(xtfHeader.ThisFileName, data + 100, 64);
    memcpy(&xtfHeader.NavUnits, data + 164, 2);
    memcpy(&xtfHeader.NumberOfSonarChannels, data + 166, 2);
    memcpy(&xtfHeader.NumberOfBathymetryChannels, data + 168, 2);
    memcpy(&xtfHeader.NumberOfSnippetChannels, data + 170, 1);
    memcpy(&xtfHeader.NumberOfForwardLookArrays, data + 171, 1);
    memcpy(&xtfHeader.NumberOfEchoStrengthChannels, data + 172, 2);
    memcpy(&xtfHeader.NumberOfInterferometryChannels, data + 174, 1);
    memcpy(&xtfHeader.Revpad1, data + 175, 1);
    memcpy(&xtfHeader.Revpad2, data + 176, 2);
    memcpy(&xtfHeader.ReferencePointHeight, data + 178, 4);
    memcpy(xtfHeader.ProjectionType, data + 182, 12);
    memcpy(xtfHeader.SpheriodType, data + 194, 10);
    memcpy(&xtfHeader.NavigationLatency, data + 204, 4);
    memcpy(&xtfHeader.OriginY, data + 208, 4);
    memcpy(&xtfHeader.OriginX, data + 212, 4);
    memcpy(&xtfHeader.NavOffsetY, data + 216, 4);
    memcpy(&xtfHeader.NavOffsetX, data + 220, 4);
    memcpy(&xtfHeader.NavOffsetZ, data + 224, 4);
    memcpy(&xtfHeader.NavOffsetYaw, data + 228, 4);
    memcpy(&xtfHeader.MRUOffsetY, data + 232, 4);
    memcpy(&xtfHeader.MRUOffsetX, data + 236, 4);
    memcpy(&xtfHeader.MRUOffsetZ, data + 240, 4);
    memcpy(&xtfHeader.MRUOffsetYaw, data + 244, 4);
    memcpy(&xtfHeader.MRUOffsetPitch, data + 248, 4);
    memcpy(&xtfHeader.MRUOffsetRoll, data + 252, 4);


    outTXT << "SODA dumpXTF utility\n\n";
    outTXT << "File Header\n";
    outTXT << "File Format:" << xtfHeader.FileFormat << '\n';
    outTXT << "System Type:" << xtfHeader.SystemType << '\n';
    outTXT << "Recording Program Name:" << xtfHeader.RecordingProgramName << '\n';
    outTXT << "Recording Program Version:" << xtfHeader.RecordingProgramVersion << '\n';
    outTXT << "Sonar Name:" << xtfHeader.SonarName << '\n';
    outTXT << "Sonar Type:" << xtfHeader.SonarType << " <refer to XTFFILEHEADER>" << '\n';
    outTXT << "Note String:" << xtfHeader.NoteString << '\n';
    outTXT << "This File Name:" << xtfHeader.ThisFileName << '\n';
    outTXT << "Nav Units:" << xtfHeader.NavUnits << " <refer to XTFFILEHEADER>" << '\n';
    outTXT << "Number Of Sonar Channels:" << xtfHeader.NumberOfSonarChannels << '\n';
    outTXT << "Number Of Bathymetry Channels:" << xtfHeader.NumberOfBathymetryChannels << '\n';
    outTXT << "Number Of Snippet Channels:" << xtfHeader.NumberOfSnippetChannels << '\n';
    outTXT << "Number Of Forward Look Arrays:" << xtfHeader.NumberOfForwardLookArrays << '\n';
    outTXT << "Number Of Echo Strength Channels:" << xtfHeader.NumberOfEchoStrengthChannels << '\n';
    outTXT << "Number Of Interferometry Channels:" << xtfHeader.NumberOfInterferometryChannels << '\n';
    outTXT << "Reference Point Height:" << xtfHeader.ReferencePointHeight << '\n';
    outTXT << "Projection Type:" << (char*)(xtfHeader.ProjectionType) << '\n';
    outTXT << "Spheriod Type:" << (char*)xtfHeader.SpheriodType << '\n';
    outTXT << "Navigation Latency:" << xtfHeader.NavigationLatency << '\n';
    outTXT << "Origin X/Y:" << xtfHeader.OriginX << "  " << xtfHeader.OriginY << '\n';
    outTXT << "Nav Offset X/Y/Z/Yaw:" << xtfHeader.NavOffsetX << "/"
           << xtfHeader.NavOffsetY << "/"
           << xtfHeader.NavOffsetZ << "/"
           << xtfHeader.NavOffsetYaw << '\n';
    outTXT << "MRU Offset X/Y/Z/Yaw/Pitch/Roll/:" << xtfHeader.MRUOffsetX << "/"
           << xtfHeader.MRUOffsetY << "/"
           << xtfHeader.MRUOffsetZ << "/"
           << xtfHeader.MRUOffsetYaw << "/"
           << xtfHeader.MRUOffsetPitch << "/"
           << xtfHeader.MRUOffsetRoll << '\n';
    int channelNum = xtfHeader.NumberOfSonarChannels
            + xtfHeader.NumberOfBathymetryChannels
            + xtfHeader.NumberOfSnippetChannels
            + xtfHeader.NumberOfForwardLookArrays
            + xtfHeader.NumberOfEchoStrengthChannels
            + xtfHeader.NumberOfInterferometryChannels;
    outTXT << "\nChaninfo data:\n";
    for (int i = 0; i < channelNum; i++)
    {
        data += i * 128;
        memcpy(&xtfHeader.ChanInfo[i].TypeOfChannel, data + 256, 1);
        memcpy(&xtfHeader.ChanInfo[i].SubChannelNumber, data + 257, 1);
        memcpy(&xtfHeader.ChanInfo[i].CorrectionFlags, data + 258, 2);
        memcpy(&xtfHeader.ChanInfo[i].UniPolar, data + 260, 2);
        memcpy(&xtfHeader.ChanInfo[i].BytesPerSample, data + 262, 2);
        memcpy(&xtfHeader.ChanInfo[i].Reserved, data + 264, 4);
        memcpy(xtfHeader.ChanInfo[i].ChannelName, data + 268, 16);
        memcpy(&xtfHeader.ChanInfo[i].VoltScale, data + 284, 4);
        memcpy(&xtfHeader.ChanInfo[i].Frequency, data + 288, 4);
        memcpy(&xtfHeader.ChanInfo[i].HorizBeamAngle, data + 292, 4);
        memcpy(&xtfHeader.ChanInfo[i].TiltAngle, data + 296, 4);
        memcpy(&xtfHeader.ChanInfo[i].BeamWidth, data + 300, 4);
        memcpy(&xtfHeader.ChanInfo[i].OffsetX, data + 304, 4);
        memcpy(&xtfHeader.ChanInfo[i].OffsetY, data + 308, 4);
        memcpy(&xtfHeader.ChanInfo[i].OffsetZ, data + 312, 4);
        memcpy(&xtfHeader.ChanInfo[i].OffsetYaw, data + 316, 4);
        memcpy(&xtfHeader.ChanInfo[i].OffsetPitch, data + 320, 4);
        memcpy(&xtfHeader.ChanInfo[i].OffsetRoll, data + 324, 4);
        memcpy(&xtfHeader.ChanInfo[i].BeamsPerArray, data + 328, 2);
        memcpy(xtfHeader.ChanInfo[i].padv1r, data + 330, 54);
        outTXT << "Channel #" << i + 1 << ":" << (char*)xtfHeader.ChanInfo[i].ChannelName << "  "
               << "CF:" << xtfHeader.ChanInfo[i].CorrectionFlags << "  "
               << "UniPolar:" << xtfHeader.ChanInfo[i].UniPolar << "  "
               << "Bytes/Sample:" << xtfHeader.ChanInfo[i].BytesPerSample << "  "
               << "Volt:" << xtfHeader.ChanInfo[i].VoltScale << "  "
               << "Freq:" << xtfHeader.ChanInfo[i].Frequency << "  "
               << "HorizBA:" << xtfHeader.ChanInfo[i].HorizBeamAngle << "  "
               << "Tilt:" << xtfHeader.ChanInfo[i].TiltAngle << "  "
               << "BW:" << xtfHeader.ChanInfo[i].BeamWidth << "\n";
        outTXT << "           Sub:" << xtfHeader.ChanInfo[i].SubChannelNumber << "  "
               << "Offset X/Y/Z:"
               << xtfHeader.ChanInfo[i].OffsetX << "/"
               << xtfHeader.ChanInfo[i].OffsetY << "/"
               << xtfHeader.ChanInfo[i].OffsetZ << "  "
               << "Offset Yaw/Pitch/Roll:"
               << xtfHeader.ChanInfo[i].OffsetYaw << "/"
               << xtfHeader.ChanInfo[i].OffsetPitch << "/"
               << xtfHeader.ChanInfo[i].OffsetRoll << '\n';
    }
}

void XTFFORMAT::printXTFPingHeader(XTFPINGHEADER &sonardata, QTextStream &outTXT)
{
    outTXT << "\nNum Bytes:" << sonardata.NumBytesThisRecord << '\n';
    outTXT << "Heaser Type:" << sonardata.HeaderType << '\n';
    outTXT << "Subchannel Number:" << sonardata.SubChannelNumber << '\n';
    outTXT << "NumChans To Follow:" << sonardata.NumChansToFollow << '\n';
    outTXT << "R2Sonic Number:" << sonardata.Year << "-" << sonardata.JulianDay << "  "
           << sonardata.Month << "/" << sonardata.Day << "  "
           << sonardata.Hour << ":" << sonardata.Minute << ":" << sonardata.Second + double(sonardata.HSeconds) / 100.0 << "  "
           << "ATT Time:" << sonardata.AttitudeTimeTag << "  Nav Time:" << sonardata.NavFixMilliseconds << '\n';
    outTXT << "tide/lat(ship)/long(ship)/lat(sensor)/long(sensor)/shiphead/fishhead:"
           << sonardata.OceanTide << "/";
    outTXT.setRealNumberPrecision(9);
    outTXT << sonardata.ShipYcoordinate << "/" << sonardata.ShipXcoordinate << "/"
           << sonardata.SensorYcoordinate << "/" << sonardata.SensorXcoordinate << "/";
    outTXT.setRealNumberPrecision(6);
    outTXT << sonardata.ShipGyro << "/" << sonardata.SensorHeading << '\n';
    outTXT.setRealNumberPrecision(6);
    outTXT << "Heave/Pitch/Roll/Yaw:" << sonardata.Heave << "/" << sonardata.SensorPitch << "/" << sonardata.SensorRoll << "/" << sonardata.Yaw << '\n';
    outTXT << "Freq Cond/Temp/Pres:" << sonardata.ConductivityFreq << "/" << sonardata.TemperatureFreq << "/" << sonardata.PressureFreq << '\n';
    outTXT << "PresTemp/WatTemp/Conductivity/Pressure:" << sonardata.PressureTemp << "/" << sonardata.WaterTemperature << "/" << sonardata.Conductivity << "/" << sonardata.Pressure << '\n';
    outTXT << "SV/Cable/LayBk/CT/Aux1/2/3/4/5/6:" << sonardata.SoundVelocity << "/" << sonardata.Layback << "/" << sonardata.CableTension << "/"
           << sonardata.AuxVal1 << "/"
           << sonardata.AuxVal2 << "/"
           << sonardata.AuxVal3 << "/"
           << sonardata.AuxVal4 << "/"
           << sonardata.AuxVal5 << "/"
           << sonardata.AuxVal6 << "\n";
    outTXT << "Ship Depth/Altitude/Speed:" << sonardata.ShipDepth << "/" << sonardata.ShipAltitude << "/" << sonardata.ShipSpeed << '\n';
    outTXT << "sensor Depth/Aux Altitude/Speed:" << sonardata.SensorDepth << "/" << sonardata.SensorAuxAltitude << "/" << sonardata.SensorSpeed << '\n';
    outTXT.setRealNumberPrecision(7);
    outTXT << "Time stamp:" << sonardata.Year << "-" << sonardata.JulianDay << "  "
           << sonardata.Hour << ":" << sonardata.Minute << ":"
           << sonardata.Second + double(sonardata.HSeconds) / 100.0 << '\n';
    outTXT << "Ping Number:" << sonardata.PingNumber << '\n';
    outTXT.setRealNumberPrecision(8);
    outTXT << "Sound speed:" << 2 * sonardata.SoundVelocity << '\n';

}

void XTFFORMAT::parseSonarChanDataToMBStruct(QByteArray &bdata, int &sizeOff, XTFPINGCHANHEADER *sonarChandata)
{
    const char* data = bdata.constData() + sizeOff;
    for (int chanIndex = 0; chanIndex < 2; chanIndex++)
    {
        memcpy(&sonarChandata[chanIndex].ChannelNumber, data + 0, 2);
        memcpy(&sonarChandata[chanIndex].DownsampleMethod, data + 2, 2);
        memcpy(&sonarChandata[chanIndex].SlantRange, data + 4, 4);
        memcpy(&sonarChandata[chanIndex].GroundRange, data + 8, 4);
        memcpy(&sonarChandata[chanIndex].TimeDelay, data + 12, 4);
        memcpy(&sonarChandata[chanIndex].TimeDuration, data + 16, 4);
        memcpy(&sonarChandata[chanIndex].SecondsPerPing, data + 20, 4);
        memcpy(&sonarChandata[chanIndex].ProcessingFlags, data + 24, 2);
        memcpy(&sonarChandata[chanIndex].Frequency, data + 26, 2);
        memcpy(&sonarChandata[chanIndex].InitialGainCode, data + 28, 2);
        memcpy(&sonarChandata[chanIndex].GainCode, data + 30, 2);
        memcpy(&sonarChandata[chanIndex].BandWidth, data + 32, 2);
        memcpy(&sonarChandata[chanIndex].ContactNumber, data + 34, 4);
        memcpy(&sonarChandata[chanIndex].ContactClassification, data + 38, 2);
        memcpy(&sonarChandata[chanIndex].ContactSubNumber, data + 40, 1);
        memcpy(&sonarChandata[chanIndex].ContactType, data + 41, 1);
        memcpy(&sonarChandata[chanIndex].NumSamples, data + 42, 4);
        memcpy(&sonarChandata[chanIndex].MillivoltScale, data + 46, 2);
        memcpy(&sonarChandata[chanIndex].ContactTimeOffTrack, data + 48, 4);
        memcpy(&sonarChandata[chanIndex].ContactCloseNumber, data + 52, 1);
        memcpy(&sonarChandata[chanIndex].Reserved2, data + 53, 1);
        memcpy(&sonarChandata[chanIndex].FixedVSOP, data + 54, 4);
        memcpy(&sonarChandata[chanIndex].Weight, data + 58, 2);
        memcpy(sonarChandata[chanIndex].ReservedPad1r, data + 60, 4);
        sizeOff += 64;

        {
            if (sonarChandata[chanIndex].SampleByteSizeFlag == 1) {
                sonarChandata[chanIndex].Sample1.resize(sonarChandata[chanIndex].NumSamples);
                memcpy(&sonarChandata[chanIndex].Sample1[0], data + 64, sonarChandata[chanIndex].NumSamples * 1);

            }
            else if (sonarChandata[chanIndex].SampleByteSizeFlag == 2) {
                sonarChandata[chanIndex].Sample2.resize(sonarChandata[chanIndex].NumSamples);
                memcpy(&sonarChandata[chanIndex].Sample2[0], data + 64, sonarChandata[chanIndex].NumSamples * 2);

            }
            else if (sonarChandata[chanIndex].SampleByteSizeFlag == 4) {
                sonarChandata[chanIndex].Sample4.resize(sonarChandata[chanIndex].NumSamples);
                memcpy(&sonarChandata[chanIndex].Sample4[0], data + 64, sonarChandata[chanIndex].NumSamples * 4);

            }
        }
        sizeOff += sonarChandata[chanIndex].SampleByteSizeFlag*sonarChandata[chanIndex].NumSamples;
        data += sizeOff;
    }

}

void XTFFORMAT::printSonarChanData(XTFPINGCHANHEADER *sonarChandata, QTextStream &outTXT)
{
    for (int i = 0; i < 2; i++)
    {
        outTXT << "Channel #" << i << ": Chan Num:" << sonarChandata[i].ChannelNumber << '\n';
        outTXT << "Sample Bytes:" << sonarChandata[i].SampleByteSizeFlag << "  NumSamples:" << sonarChandata[i].NumSamples << "  Downsample Method:" << sonarChandata[i].DownsampleMethod
               << "  Slant Range:" << sonarChandata[i].SlantRange << "  Ground Range:" << sonarChandata[i].GroundRange
               << "  Time Delay:" << sonarChandata[i].TimeDelay << "  Time Duration:" << sonarChandata[i].TimeDuration << '\n';
        outTXT << "Seconds PerPing:" << sonarChandata[i].SecondsPerPing << "  Process Flag:" << sonarChandata[i].ProcessingFlags
               << "  Frequency:" << sonarChandata[i].Frequency << "  Initial GainCode:" << sonarChandata[i].InitialGainCode
               << "  Gain Code:" << sonarChandata[i].GainCode << "  Band Width:" << sonarChandata[i].BandWidth
               << "  Millivolt:" << sonarChandata[i].MillivoltScale << "  Fixed VSOP:" << sonarChandata[i].FixedVSOP
               << "  Weight:" << sonarChandata[i].Weight << '\n';
        outTXT << "Contact Type/Number/Classification/subnumber/timeofftrack/closenumber:" << sonarChandata[i].ContactType << "/"
               << sonarChandata[i].ContactNumber << "/"
               << sonarChandata[i].ContactClassification << "/"
               << sonarChandata[i].ContactSubNumber << "/"
               << sonarChandata[i].ContactTimeOffTrack << "/"
               << sonarChandata[i].ContactCloseNumber << '\n';
    }
    if (sonarChandata[0].NumSamples == sonarChandata[1].NumSamples)
    {
        outTXT << "Num       Port       Starboard\n";
        outTXT << "==============================\n";
        for (int i = 0; i<int(sonarChandata[0].NumSamples); i++)
        {
            if (sonarChandata[0].SampleByteSizeFlag == 1) {
                outTXT.setFieldWidth(4);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << i + 1;

                outTXT.setFieldWidth(10);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[0].Sample1[i]);

                outTXT.setFieldWidth(13);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[1].Sample1[i]);
                outTXT << '\n';

            }
            else if (sonarChandata[0].SampleByteSizeFlag == 2) {
                outTXT.setFieldWidth(4);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << i + 1;

                outTXT.setFieldWidth(10);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[0].Sample2[i]);

                outTXT.setFieldWidth(13);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[1].Sample2[i]);
                outTXT << '\n';

            }
            else if (sonarChandata[0].SampleByteSizeFlag == 4) {
                outTXT.setFieldWidth(4);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << i + 1;

                outTXT.setFieldWidth(10);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[0].Sample4[i]);

                outTXT.setFieldWidth(13);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[1].Sample4[i]);
                outTXT << '\n';

            }
        }
        outTXT << "==============================\n";

    }
    else {
        outTXT << "Num       Port\n";
        outTXT << "===================\n";
        for (int i = 0; i<int(sonarChandata[0].NumSamples); i++)
        {
            if (sonarChandata[0].SampleByteSizeFlag == 1) {
                outTXT.setFieldWidth(4);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << i + 1;

                outTXT.setFieldWidth(10);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[0].Sample1[i]);
                outTXT << '\n';

            }
            else if (sonarChandata[0].SampleByteSizeFlag == 2) {
                outTXT.setFieldWidth(4);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << i + 1;

                outTXT.setFieldWidth(10);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[0].Sample2[i]);
                outTXT << '\n';

            }
            else if (sonarChandata[0].SampleByteSizeFlag == 4) {
                outTXT.setFieldWidth(4);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << i + 1;

                outTXT.setFieldWidth(10);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[0].Sample4[i]);
                outTXT << '\n';

            }
        }
        outTXT << "Num       Starboard\n";
        outTXT << "====================\n";
        for (int i = 0; i<int(sonarChandata[1].NumSamples); i++)
        {
            if (sonarChandata[1].SampleByteSizeFlag == 1) {
                outTXT.setFieldWidth(4);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << i + 1;

                outTXT.setFieldWidth(10);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[1].Sample1[i]);
                outTXT << '\n';
            }
            else if (sonarChandata[1].SampleByteSizeFlag == 2) {
                outTXT.setFieldWidth(4);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << i + 1;

                outTXT.setFieldWidth(10);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[1].Sample2[i]);
                outTXT << '\n';

            }
            else if (sonarChandata[1].SampleByteSizeFlag == 4) {
                outTXT.setFieldWidth(4);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << i + 1;

                outTXT.setFieldWidth(10);
                outTXT.setFieldAlignment(QTextStream::AlignRight);
                outTXT << QString::number(sonarChandata[1].Sample4[i]);
                outTXT << '\n';

            }
        }
        outTXT << "=========================\n";
    }
    outTXT.setFieldWidth(0);

}

void XTFFORMAT::byteChange2(char *xbuffer,char *buffer,int init)
{
    for(int i=0;i<init;i++)
    {
        memcpy(xbuffer+init-1-i,buffer+i,1);
    }
}

void XTFFORMAT::byteChange4(char *xbuffer,char *buffer,int init)
{
    for(int i=0;i<init;i++)
    {
        memcpy(xbuffer+init-1-i,buffer+i,1);
    }
}

}
