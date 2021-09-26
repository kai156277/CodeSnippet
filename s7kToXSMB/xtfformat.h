#ifndef XTFFORMAT_H
#define XTFFORMAT_H

#include <QByteArray>

#include <QTextStream>
#include <vector>
#include "stdint.h"
#include <XTF/XtfDataStructure.h>
/***********************************************************************/
/* Include-file for reading .XTF files.  This file defines the
   structures currently defined in the .XTF file format.
*/
#define FMT_XTF 123 // unique ID for all XTF files.
using namespace xtf;

namespace vsrxtf{

enum XTF_HEADER{
    XTF_HEADER_SONAR = 0, // sidescan and subbottom
    XTF_HEADER_NOTES = 1, // notes - text annotation
    XTF_HEADER_BATHY = 2, // bathymetry (Seabat, Odom)
    XTF_HEADER_ATTITUDE = 3, // TSS or MRU attitude (pitch, roll, heave, yaw)
    XTF_HEADER_FORWARD = 4, // forward-look sonar (polar display)
    XTF_HEADER_ELAC = 5, // Elac multibeam
    XTF_HEADER_RAW_SERIAL = 6, // Raw data from serial port
    XTF_HEADER_EMBED_HEAD = 7, // Embedded header structure
    XTF_HEADER_HIDDEN_SONAR = 8 // hidden (non-displayable) ping
};

enum EqualAngle{
    EQUALANGLE = 101,
    EQUALDISTANCE = 102
};

enum Intensity{
    INTENSITYNO = 0,
    INTENSITYDO = 1
};


// Embedded header record -- occurs mid-file
///////////////////////////////////////////////////////////////////////////////
typedef struct {
    uint16_t MagicNumber;      // Set to 0xFACE
    uint8_t HeaderType;       // 7=embedded header record
    uint8_t SubChannelNumber; //
    uint16_t NumChansToFollow; //
    uint16_t Rpad2r[2];

    uint32_t NumBytesThisRecord; // Total byte count for this embedded record
    // including the 64 front-pad bytes
    uint8_t Rpad1r[50]; // make even to 64 bytes so far
    XTFFILEHEADER NewHeader;

} XTFEMBEDDEDFILEHEADER;

// Annotation record
// An annotation record is a line of text which can be saved to the
// file and is displayed in the "Notes" field on the Isis display.
// This text is displayed during playback.  Additionally, this text
// may be printed in realtime or in playback.  This can be activated
// in the Print Annotation dialog box.
///////////////////////////////////////////////////////////////////////////////
typedef struct {

    uint16_t MagicNumber;      // Set to 0xFACE
    uint8_t HeaderType;       // XTF_HEADER_NOTES (1)
    uint8_t SubChannelNumber;
    uint16_t NumChansToFollow;
    uint16_t ReservedByte2r[2];
    uint32_t NumBytesThisRecord; // Total byte count for this update

    //
    // Date and time of the annotation
    //
    uint16_t  Year;
    uint8_t  Month;
    uint8_t  Day;
    uint8_t  Hour;
    uint8_t  Minute;
    uint8_t  Second;
    uint8_t  ReservedByte1r[35];

    char  NotesText[256-56];

} XTFNOTESHEADER;
typedef struct {

    uint16_t MagicNumber;      // Set to 0xFACE
    uint8_t HeaderType;       // 107
    uint8_t SubChannelNumber;
    uint16_t NumChansToFollow;
    uint16_t pad2r[2];
    uint32_t NumBytesThisRecord; // Total byte count for this update

    //
    // Date and time of the annotation
    //
    uint16_t  Year;
    uint8_t  Month;
    uint8_t  Day;
    uint8_t  Hour;
    uint8_t  Minutes;
    uint8_t  Seconds;
    uint16_t  MicroSeconds;
    double RawYcoordinate;
    double RawXcoordinate;
    double RawAltitude;
    float  Pitch;
    float  Roll;
    float  Heave;
    float  Heading;
    uint8_t   pad1;

} XTFPOSRAWNAVIGITION;


// RAW ASCII data received over serial port
// These packets are stored in the XTF file on a per-serial-port
// basis.  To store the raw ASCII data for a given serial port, add
// the token "{SAVEALL}" to the serial port template.  Use of this
// option is not generally recommended, since Isis already parses the
// data for all usefull information.

///////////////////////////////////////////////////////////////////////////////
typedef struct {

    uint16_t MagicNumber;      // Set to 0xFACE
    uint8_t HeaderType;       // will be XTF_HEADER_RAW_SERIAL (7)
    uint8_t SerialPort;
    uint16_t Repad2r[3];
    uint32_t NumBytesThisRecord; // Total byte count for this update

    //
    // Date and time raw ASCII data was posted to disk
    //
    uint16_t  Year;
    uint8_t  Month;
    uint8_t  Day;
    uint8_t  Hour;
    uint8_t  Minute;
    uint8_t  Second;
    uint8_t  HSeconds;      // hundredth of seconds (0-99)
    uint16_t  JulianDay;     // days since Jan 1.

    uint32_t TimeTag;       // millisecond timer value
    uint16_t  StringSize;    // Number of valid chars in RawAsciiData string
    char  RawAsciiData[64-30]; // will be padded in 64-byte increments to make
    // structure an even multiple of 64 bytes

} XTFRAWSERIALHEADER;

//!
typedef struct{
    unsigned long       ID;                     //Identifier code. SNP0= 0x534E5030
    unsigned short      HeaderSize;             //Header size, bytes.
    unsigned short      DataSize;               //Data size following header, bytes.
    unsigned long       PingNumber;             //Sequential ping number.
    unsigned long       Seconds;                //Time since 00:00:00, 1-Jan-1970
    unsigned long       Millisec;
    unsigned short      Latency;                //Time from ping to output (milliseconds)
    unsigned short      SonarID[2];             //Least significant four bytes of Ethernet address.
    unsigned short      SonarModel;             //Coded model number of sonar.
    unsigned short      Frequency;              //Sonar frequency (kHz).
    unsigned short      SSpeed;                 //Programmed sound velocity (m/sec).
    unsigned short      SampleRate;             //A/D sample rate (samples/sec).
    unsigned short      PingRate;               //Pings per second, 0.001 Hz steps.
    unsigned short      Range;                  //Range setting (meters).
    unsigned short      Power;                  //Power
    unsigned short      Gain;                   //(b15=auto, b14=TVG, b6..0=gain).
    unsigned short      PulseWidth;             //Transmit pulse width (microseconds).
    unsigned short      Spread;                 //TVG spreading, n*log(R), 0.25dB steps.
    unsigned short      Absorb;                 //TVG absorption, dB/km, 1dB steps.
    unsigned short      Proj;                   //b7 = steering, b4..0 = projector type.
    unsigned short      ProjWidth;              //Transmit beam width along track, 0.1 deg steps.
    unsigned short      SpacingNum;             //Receiver beam spacing, numerator, degrees.
    unsigned short      SpacingDen;             //Receiver beam spacing, denominator.
    short               ProjAngle;              //Projector steering, degrees*PKT_STEER_RES
    unsigned short      MinRange;               //Range filter settings
    unsigned short      MaxRange;
    unsigned short      MinDepth;               //Depth filter settings.
    unsigned short      MaxDepth;               //Depth filter settings.
    unsigned short      Filters;                //Enabled filters: b1=depth, b0=range.
    uint8_t             bFlags[2];              //Bits 0 – 11 spare,
    //Bits 12 – 14 snipMode,
    //Bit 15 RollStab. Bit 0: roll stabilization enabled.
    
    short               HeadTemp;               //Head temperature, 0.1C steps.
    unsigned short      BeamCnt;                //number of beams
}SNP0Data;

typedef struct{
    unsigned long   ID;                         //Identifier code. SNP1= 0x534E5031
    unsigned short  HeaderSize;                 //Header size, bytes.
    unsigned short  DataSize;                   //Data size following header, bytes.
    unsigned long   PingNumber;                 //Sequential ping number.
    unsigned short  Beam;                       //Beam number, 0..N-1.
    unsigned short  SnipSamples;                //Snippet size, samples.
    unsigned short  GainStart;                  //16 M Gain at start of snippet, 0.01 dB steps, 0=ignore.
    unsigned short  GainEnd;                    //Gain at end of snippet, 0.01 dB steps, 0=ignore.
    unsigned short  FragOffset;                 //Fragment offset, samples from ping.
    unsigned short  FragSamples;                //Fragment size, samples
}SNP1Data;

// SONICRAWSTRUCT
// customize struct for printing bathy data
typedef struct {
    char   packetName[4];
    unsigned int   packetSize;
    uint16_t  Year;
    uint8_t  Month;
    uint8_t  Day;
    uint8_t  Hour;
    uint8_t  Minute;
    double  Second;
    unsigned int   pingNum;
    float          pingPeriod;
    float          soundSpeed;
    float          frequency;
    float          txPower;
    unsigned short points;
    float          range[1024];
    unsigned short beamModel;//1为等角模式；2为等距模式
    float          angles[1024];
    float          intensity[1024];
    uint32_t       quality[1024];
    float          txPulseWidth;
    float          txbeamWidthVert;
    float          txBeamWidthHoriz;
    float          txSteeringVert;
    float          txSteeringHoriz;
    float          rxBandWidth;
    float          rxSampleRate;
    float          rxRange;                  // [seconds two-way]
    float          rxGain;
    float          rxSpreading;
    float          rxAbsorption;
    float          rxMountTilt;
    float          InScalFactor;
} SONICRAWSTRUCT;

typedef struct{
    uint16_t  Year;          // Computer date when this record was saved
    uint8_t  Month;
    uint8_t  Day;
    uint8_t  Hour;          // Computer time when this record was saved
    uint8_t  Minute;
    uint8_t  Second;
    uint8_t  HSeconds;      // hundredths of seconds (0-99)
    uint16_t  JulianDay;     // Number of days since January 1
    double dTime;
    unsigned int   pingNum;
    float          soundSpeed;
    QByteArray     rawdata;
    QByteArray     rawSNPdata;
} MBData;

typedef struct{
    //DWORD attTimeTag;
    uint16_t  Year;
    uint8_t  Month;
    uint8_t  Day;
    uint8_t  Hour;
    uint8_t  Minute;
    uint8_t  Second;
    uint16_t  Milliseconds;
    uint16_t  JulianDay;     // Number of days since January 1
    double dTime;
    double latitude;
    double longitude;
    double heigt;
    double roll;
    double pitch;
    double heading;
    float  heave;
} ATTData;

typedef struct{
    uint32_t attTimeTag;
    uint16_t  Year;
    uint8_t  Month;
    uint8_t  Day;
    uint8_t  Hour;
    uint8_t  Minute;
    uint8_t  Second;
    uint16_t  Milliseconds;
    double latitude;
    double longitude;
    double heigt;
} NAVData;

typedef struct {
    int   DataFlag;
    uint16_t  Year;          // Computer date when this record was saved
    uint8_t  Month;
    uint8_t  Day;
    uint8_t  Hour;          // Computer time when this record was saved
    uint8_t  Minute;
    uint8_t  Second;
    uint8_t  HSeconds;      // hundredths of seconds (0-99)
    uint32_t AttTimeTag;
    uint32_t NavTimeTag;

    uint16_t  Milliseconds;//10-3
    uint16_t  Microseconds;//10-6
    uint16_t  JulianDay;     // Number of days since January 1
    unsigned int   pingNum;
    float          soundSpeed;
    QByteArray     mbdata;
    double latitude;
    double longitude;
    double heigt;
    double roll;
    double pitch;
    double heading;
    float  heave;
} MBINSData;

const static uint16_t R0 = 12370;
const static uint16_t A0 = 12353;
const static uint16_t A2 = 12865;
const static uint16_t I1 = 12617;
const static uint16_t G1 = 12615;

class XTFFORMAT
{
public:
    static void parseXTFPingHeader(QByteArray &data, int sizeOff ,XTFBATHHEADER &bathydata);
    static void parseBathyDataToMBStruct(QByteArray &datagram, int sizeOff, SONICRAWSTRUCT &Sodabathydata);
    static void printBathyPing(XTFBATHHEADER &bathydata, SONICRAWSTRUCT &Sodabathydata, QTextStream &outTXT);
    static void parseAttitudeToStruct(QByteArray &data, XTFATTITUDEDATA &attidata, int sizeOff, QTextStream &outTXT);
    static void parsePosToStruct(QByteArray &data, XTFPOSRAWNAVIGITION &posdata, int sizeOff, QTextStream &outTXT);
    static void parseXTFFileHeader(QByteArray &data, XTFFILEHEADER &xtfHeader, QTextStream &outTXT);
    static void printXTFPingHeader(XTFPINGHEADER &sonardata, QTextStream &outTXT);
    static void parseSonarChanDataToMBStruct(QByteArray &data, int &sizeOff, XTFPINGCHANHEADER *sonarChandata);
    static void printSonarChanData(XTFPINGCHANHEADER *sonarChandata, QTextStream &outTXT);
    static void byteChange2(char *xbuffer, char *buffer, int init = 2);
    static void byteChange4(char *xbuffer, char *buffer, int init = 4);
};

}

#endif //XTFFORMAT_H
