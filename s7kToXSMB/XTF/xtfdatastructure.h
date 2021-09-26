#ifndef XTFDATASTRUCTURE_H
#define XTFDATASTRUCTURE_H

#include <stdint.h>
#include <vector>

typedef unsigned char       BYTE;   //8
typedef unsigned short      WORD;   //16
typedef unsigned long       DWORD;  //32

namespace xtf{

typedef struct {
    BYTE TypeOfChannel;      // SUBBOTTOM = 0; PORT = 1, STBD = 2, BATHYMETRY = 3;
    BYTE SubChannelNumber;   // 索引号
    WORD CorrectionFlags;    // 1 = 声呐图像存储为倾斜距离,  2 = 为地面距离（改正）
    WORD UniPolar;           // 0 = data is bipolar, 1 =data is unipolar
    WORD BytesPerSample;     // 1 = 8字节数据, 2 = 16字节数据, 4 = 32bit
    DWORD Reserved;          //Reserved
    char ChannelName[16];
    float VoltScale;        // 默认是5
    float Frequency;        // 中心传输频率
    float HorizBeamAngle;   // 通常是1
    float TiltAngle;        // 通常是30
    float BeamWidth;        // 3dB 波束宽度，通常是50度
    float OffsetX;          // 右为正
    float OffsetY;          // 前为正
    float OffsetZ;          // 下为正
    float OffsetYaw;        // 右转为正
    float OffsetPitch;      // 前仰为正
    float OffsetRoll;       // Orientation of positive roll is lean to starboard.
    WORD  BeamsPerArray;    // 未使用
    char padv1r[54];        // 0
} CHANINFO;

///////////////////////////////////////////////////////////////////////////////

typedef struct {    
    BYTE FileFormat;                    // 必须123
    BYTE SystemType;                    // 用于记录该文件的系统类型.
    char RecordingProgramName[8];       // Example: "Isis"
    char RecordingProgramVersion[8];    // Example: "172" for 1.72
    char SonarName[16];                 // 用于访问声纳的服务程序名称. Example: "C31_SERV.EXE"
    WORD SonarType;                     // 声呐类型: 53 = R2Sonic QINSy , SEABAT=8
    char NoteString[64];                // 在声呐设备对话框中输入的信息
    char ThisFileName[64];              // 本文件的名称
    WORD NavUnits;                      // 0 = meters or 3 = degrees
    WORD NumberOfSonarChannels;         // 声呐通道数量   0
    WORD NumberOfBathymetryChannels;    // 测深通道数量   1
    BYTE NumberOfSnippetChannels;       // snippet通道数量
    BYTE NumberOfForwardLookArrays;     //
    WORD NumberOfEchoStrengthChannels;  //
    BYTE NumberOfInterferometryChannels;//
    BYTE Revpad1;                       //预留的
    WORD Revpad2;                       //预留的
    float ReferencePointHeight;         //基准点距水平线的高度（m）

    //! navigetion system parameters

    BYTE     ProjectionType[12];       // 未使用  set 0
    BYTE     SpheriodType[10];         // 未使用  set 0
    long     NavigationLatency;        // 导航系统延时 ms
    float    OriginY;                  // 未使用  set 0
    float    OriginX;                  // 未使用  set 0
    float    NavOffsetY;               // 前为正
    float    NavOffsetX;               // 右为正
    float    NavOffsetZ;               // 下为正
    float    NavOffsetYaw;             // 右转为正
    float    MRUOffsetY;               // 前为正
    float    MRUOffsetX;               // 右为正
    float    MRUOffsetZ;               // 下为正
    float    MRUOffsetYaw;             // 右转为正
    float    MRUOffsetPitch;           // 前仰为正
    float    MRUOffsetRoll;            // Orientation of positive roll is lean to starboard.

    //! ChanInfo

    CHANINFO ChanInfo[6];
} XTFFILEHEADER;

///////////////////////////////////////////////////////////////////////////////
//XTFPINGCHANHEADER is used to hold data that can be unique to each channel from ping to ping.
//One of these headers follows each XTFPINGHEADER, no XTFPINGCHANHEADERS follow a
//XTFBATHYHEADER.
typedef struct {
    WORD  ChannelNumber;        // 0=port (low frequency), 1=stbd (low frequency), 2=port (high frequency), 3=stbd (high frequency)
    WORD  DownsampleMethod;     // 2=MAX, 4=Rms
    float SlantRange;           // 数据倾斜范围  m
    float GroundRange;          // 数据水平范围  m = (SlantRange^2 - Altitude^2)
    float TimeDelay;            // 几乎总是 0 s
    float TimeDuration;         // Amount of time recorded s
    float SecondsPerPing;       // ping间隔 s
    DWORD ProcessingFlags;      // 几乎总是 0       //4=TVG, 8=BAC and GAC, 16=Filter, etc...
    WORD  Frequency;            // 发射频率
    WORD  InitialGainCode;      // Settings as transmitted by sonar
    WORD  GainCode;             // 增益
    WORD  BandWidth;            // 波宽
    DWORD ContactNumber;        // 连接数目
    WORD  ContactClassification;// 连接等级
    BYTE  ContactSubNumber;     // 连接子数
    BYTE  ContactType;          // 连接类型
    DWORD NumSamples;           // 循环次结构的样本数量
    WORD  MillivoltScale;       // 最强电压 mV
    float ContactTimeOffTrack;  // 连接的时间偏移 ms
    BYTE  ContactCloseNumber;   // 连接关闭数目
    BYTE  Reserved2;            // 0
    float FixedVSOP;
    short Weight;
    BYTE  ReservedPad1r[4];     // 0
//????????????????????????????????????????????????????????????????????????????????????????????//
    BYTE  SampleByteSizeFlag;
    std::vector <BYTE>  Sample1;
    std::vector <WORD>  Sample2;
    std::vector <DWORD> Sample4;
//????????????????????????????????????????????????????????????????????????????????????????????//
} XTFPINGCHANHEADER;

///////////////////////////////////////////////////////////////////////////////

// Attitude data packet（ping）, 64 bytes in length.
typedef struct {
    WORD MagicNumber;           // 必须 0xFACE
    BYTE HeaderType;            // 必须 3
    BYTE SubChannelNumber;      // When HeaderType is Bathy, indicates which head
    WORD NumChansToFollow;      // If Sonar Ping, Number of channels to follow
    WORD Reserved1[2];          // 0
    DWORD NumBytesThisRecord;   // 总字节数 256字节
    DWORD Reserved2[2];         // 0
    DWORD EpochMicroseconds;    //          ms
    DWORD SourceEpoch;          // 1970/1/1 second
    float Pitch;                // 前仰为正
    float Roll;                 // Positive value is roll to starboard
    float Heave;                // 上浮为正
    float Yaw;                  // 右转为正
    DWORD TimeTag;              // 系统时间 ms
    float Heading;              // 度
    WORD  Year;
    BYTE  Month;
    BYTE  Day;
    BYTE  Hour;
    BYTE  Minutes;
    BYTE  Seconds;
    WORD  Milliseconds;         // 0 - 999
    BYTE  Reserved3[1];         // 0
} XTFATTITUDEDATA;

///////////////////////////////////////////////////////////////////////////////

typedef struct {
    WORD MagicNumber;           // 必须 0xFACE
    BYTE HeaderType;            // HEADER_SONAR (0), HEADER_BATHY (2), HEADER_ATTITUDE (3)...
    BYTE SubChannelNumber;      // When HeaderType is Bathy, indicates which head; When sonar, which ping of a batch (Klein 5000:0..4)
    WORD NumChansToFollow;      // 如果 Sonar Ping , 为通道数（channels）
    WORD ReservedSpace1[2];     // 0
    DWORD NumBytesThisRecord;   // 总字节数  x64
    WORD  Year;
    BYTE  Month;
    BYTE  Day;
    BYTE  Hour;
    BYTE  Minute;
    BYTE  Second;
    BYTE  HSeconds;             // 百分之一秒
    WORD  JulianDay;            // 儒略日
    DWORD EventNumber;          // 最近记录事件号 nav (0)
    DWORD PingNumber;           // 数据包号.
    float SoundVelocity;        // 声速 m/s 一般750
    float OceanTide;            // [{t}] 海洋潮汐 m
    DWORD Reserved2;            // 0
    float ConductivityFreq;     // [Q] 电导率 Hz
    float TemperatureFreq;      // [b] 温度频率 Hz
    float PressureFreq;         // [0] 压力频率 Hz
    float PressureTemp;         // [;] 压力温度 摄氏度
    float Conductivity;         // [{c}] 导电性 S/m             // [Q] 算得
    float WaterTemperature;     // [{w}] 水温 摄氏度             // [b] 算得
    float Pressure;             // [{p}] 水压 p                   // [0] 算得
    float ComputedSoundVelocity;// 声速 m/s   //[{c}],[{w}],[{p}] using the Chen Millero formula (1977) formula (JASA,62,1129-1135).
    float MagX;                 // [e] X-axis 磁强数据, mgauss
    float MagY;                 // [w] Y-axis 磁强数据, mgauss
    float MagZ;                 // [z] Z-axis 磁强数据, mgauss

    //!Sensors Information.

    float AuxVal1;              // [1] 自定义辅助值
    float AuxVal2;              // [2] 自定义
    float AuxVal3;              // [3] 自定义
    float AuxVal4;              // [4] 自定义
    float AuxVal5;              // [5] 自定义
    float AuxVal6;              // [6] 自定义
    float SpeedLog;             // [s] 计程仪传感器 on towfish - knots. This  isn't fish speed!
    float Turbidity;            // [|] 浑浊度传感器 (0 to +5 volts) stored times 10000
    float ShipSpeed;            // [v] 船速 节
    float ShipGyro;             // [G] 船陀螺仪 角度
    double ShipYcoordinate;     // [y] 纬度
    double ShipXcoordinate;     // [x] 经度
    WORD ShipAltitude;          // 船高度 dm
    WORD ShipDepth;             // 船深度 dm
    BYTE FixTimeHour;           // [H] 导航更新时间间隔
    BYTE FixTimeMinute;         // [I] 导航更新时间间隔
    BYTE FixTimeSecond;         // [S] 导航更新时间间隔
    BYTE FixTimeHsecond;        // 百分之一秒
    float SensorSpeed;          // [V] 速度 节
    float KP;                   // [{K}] 千米管
    double SensorYcoordinate;   // [E] 纬度
    double SensorXcoordinate;   // [N] 经度
    WORD SonarStatus;           // 系统状态
    WORD RangeToFish;           // [?]    dm.  不用
    WORD BearingToFish;         // [>] * 100.  不用
    WORD CableOut;              // [o]     m.
    float Layback;              // [l] 船到鱼的地面距离.   0;
    float CableTension;         // [P] 串口电缆张力    不用
    float SensorDepth;          // [0] 海面到传感器距离
    float SensorPrimaryAltitude;// [7] 传感器到水底距离
    float SensorAuxAltitude;    // [a] 辅助高度
    float SensorPitch;          // [8] Pitch in degrees (nose up +)
    float SensorRoll;           // [9] Roll in degrees (roll to starboard +)
    float SensorHeading;        // [h] Fish heading in degrees
    float Heave;                // heave
    float Yaw;                  // yaw.  (right + )
    DWORD AttitudeTimeTag;      // ms.
    float DOT;                  // 杂项 偏离轨道距离
    DWORD NavFixMilliseconds;   // 杂项 接收惯导时 ms
    BYTE  ComputerClockHour;    // 忽略
    BYTE  ComputerClockMinute;  // 忽略
    BYTE  ComputerClockSecond;  // 忽略
    BYTE  ComputerClockHsec;    // 忽略`
    short FishPositionDeltaX;   // [{DX}]
    short FishPositionDeltaY;   // [{DY}]
    unsigned char FishPositionErrorCode;
    unsigned int OptionalOffsey;
    BYTE CableOutHundredths;
    BYTE ReservedSpace1r[6];
} XTFPINGHEADER, XTFBATHHEADER;




}
#endif // XTFDATASTRUCTURE_H
