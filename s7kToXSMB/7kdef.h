#ifndef K7DEF_H
#define K7DEF_H

#include "stdint.h"

#include <QVector>

enum RecordType : uint32_t
{
    S7K_R1003 = 1003,   // 位置
    S7K_R1008 = 1008,   // 深度
    S7K_R1015 = 1015,   // 姿态
    S7K_R7000 = 7000,   // 7k 声纳设置
    S7K_R7004 = 7004,   // 7k 波束几何形状
    S7K_R7006 = 7006,   // 7k 水深测量数据
    S7K_R7027 = 7027,   // 7k 水深测量原始数据
};

#pragma pack(push)
#pragma pack(1)

typedef struct
{
    uint16_t mProtocolVersion;
    uint16_t mOffset;
    uint32_t mTotal_packets;
    uint16_t mTotal_records;
    uint16_t mTransmissionIdentifier;
    uint32_t mPacketSize;
    uint32_t mTotalSize;
    uint32_t mSequenceNumber;
    uint32_t mDesitinationDeviceIdentifier;
    uint16_t mDestinationEnumerator;
    uint16_t mSourceEnumerator;
    uint32_t mSourceDeviceIdentifier;
} NetWorkFrame;

struct S7KTime
{
    uint16_t mYear;      // 2018
    uint16_t mDay;       // 1 - 366
    float    mSeconds;   // 0.000000-59.999999
    uint8_t  mHours;     // 0-23
    uint8_t  mMinutes;   // 0-59
};

constexpr auto RF_FLAG_ROR = 0x8000;

struct DataRecordFrame
{
    uint16_t mProtocolVersion;
    uint16_t mOffset;
    uint32_t mSyncPattern;
    uint32_t mSize;
    uint32_t mOptinalDataOffset;
    uint32_t mOptinalDataIdentifier;
    S7KTime  mTimeStamp;
    uint16_t mRecrodVersion;
    uint32_t mRecordTypeIdentifier;
    uint32_t mDeviceIdentifier;
    uint32_t mSystemEnumerator;
    uint32_t mReserved1;
    uint16_t mFlags;
    uint16_t mReserved2;
    uint32_t mReserved3;
    uint32_t mTotalRecordsInFragmentdData;
    uint32_t mFragmentNumber;
};

struct R7000RTH
{
    uint64_t mSonarId;                     // Sonar serial number
    uint32_t mPingNumber;                  // Sequential number
    uint16_t mMultipingSequence;           // Flag to indicate multi-ping sequence.  0 or the sequence number of the ping in the multi-ping sequence.
    float    mFrequency;                   // Transmit frequency in Hertz
    float    mSampleRate;                  // Sample rate in Hertz
    float    mReceiverBandwidth;           // Hertz
    float    mTxPulseWidth;                // In seconds
    uint32_t mTxPulseTypeIdentifier;       // 0 – CW , 1 – Linear chirp , 2 – Multi-ping 2 , 3 – Multi-ping 4
    uint32_t mTxPulseEnvelopeIdentifier;   // 0 – Tapered rectangular , 1 – Tukey
    float    mTxPulseEnvelopeParameter;    // Some envelopes don’t use this parameter
    uint32_t mTxPulseReserved;             // Additional pulse information
    float    mMaxPingRate;                 // Maximum ping rate in pings per second
    float    mPingPeriod;                  // Seconds since last ping
    float    mRangeSelection;              // Range selection in meters
    float    mPowerSelection;              // Power selection in dB re 1Pa
    float    mGainSelection;               // Gain selection in dB
    uint32_t mControlFlags;
    uint32_t mProjectorIdentifier;                     // Projector selection
    float    mProjectorBeamSteeringAngleVertical;      // radians
    float    mProjectorBeamSteeringAngleHorizontal;    // radians
    float    mProjectorBeam_3dB_BeamWidthVertical;     // radians
    float    mProjectorBeam_3dB_BeamWidthHorizontal;   // radians
    float    mProjectorBeamFocalPoint;                 // meters
    uint32_t mProjectorBeamWeightingWindowType;        // 0 – Rectangular , 1 – Chebychev
    float    mProjectorBeamWeightingWindowParameter;   // N/A

    // BIT FIELD:
    // Bit 0-3: Pitch stabilization method
    // Bit 4-7: Yaw stabilization method
    // Bit 8-31: Reserved
    uint32_t mTransmitFlags;
    uint32_t mHydrophoneIdentifier;            // Hydrophone selection
    uint32_t mReceiveBeamWeightingWindow;      // 0 – Chebychev , 1 – Kaiser
    float    mReceiveBeamWeightingParameter;   // N/A
    // BIT FIELD:
    // Bit 0: Roll compensation indicator
    //    Bit 1: Reserved
    //    Bit 2: Heave compensation indicator
    //    Bit 3: Reserved
    //    Bit 4-7: Dynamic focusing method
    //    Bit 8-11: Doppler compensation method
    //    Bit 12-15: Match filtering method
    //    Bit 16-19: TVG method
    //    Bit 20-23: Multi-ping mode
    //    0 – No multi-ping
    //    If non-zero, this represents the sequence
    //    number of the ping in the multi-ping
    //    sequence.
    //    Bit 24-31: Reserved
    uint32_t mReceiveFlags;
    float    mReceiveBeamWidth;             // radians
    float    mBottomDetectionFilterInfo1;   // Min range (if range filter is active)
    float    mBottomDetectionFilterInfo2;   // Max range (if range filter is active)
    float    mBottomDetectionFilterInfo3;   // Min depth (if depth filter is active)
    float    mBottomDetectionFilterInfo4;   // Max depth (if depth filter is active)
    float    mAbsorption;                   // Absorption in dB/km
    float    mSoundVelocity;                // Sound velocity in m/s
    float    mSpreading;                    // Spreading loss in dB
    uint16_t mReserved;                     // Reserved
};

struct R7000
{
    DataRecordFrame mDRF;
    R7000RTH        mRTH;
};

struct R7004RTH
{
    uint64_t mSonarId;
    uint32_t mNumOfBeams;
};

struct R7004RD
{
    float mBeamVAngle;
    float mBeamHAngle;
    float mBeamWidthY;
    float mBeamWidthX;
    float mTxDelay;
};

struct R7004
{
    DataRecordFrame mDRF;
    R7004RTH        mRTH;
    float *         mBeamVAngles;
    float *         mBeamHAngles;
    float *         mBeamWidthYs;
    float *         mBeamWidthXs;
    // float *         mTxDelays;
};

struct R7027RTH
{
    uint64_t mSerialId;
    uint32_t mPingNumber;
    uint16_t mMultipingSequence;
    uint32_t mDetectionNums;
    uint32_t mDataSize;
    uint8_t  mDetectionAlgorithm;
    uint32_t mFlags;
    float    mSamplingRate;
    float    mTxAngle;
    float    mAppliedRoll;
    uint32_t mReserved[15];
};

struct R7027RD
{
    uint16_t mBeamDescriptor;
    float    mDetectionPoint;
    float    mRxAngle;
    uint32_t mFlags;
    uint32_t mQuality;
    float    mUncertainty;
    float    mIntensity;
    float    mMinLimit;
    float    mMaxLimit;
};
/*
struct R7027RD
{
    uint16_t mBeamDescriptor;
    float    mDetectionPoint;
    float    mRxAngle;
    uint32_t mFlags;
    uint32_t mQuality;
    float    mUncertainty;
    float    mIntensity;
    float    mMinLimit;
    float    mMaxLimit;
};
*/

struct R7027OD
{
    float   mFrequency;
    double  mLatitude;
    double  mLongitude;
    float   mHeading;
    uint8_t mHeightSource;
    float   mTide;
    float   mRoll;
    float   mPitch;
    float   mHeave;
    float   mVehicleDepth;
};

struct R7027ODBeam
{
    float mDepth;
    float mAlongTrackDistance;
    float mAcrossTrackDistance;
    float mPointingAngle;
    float mAzimuthAngle;
};

struct R7027
{
    DataRecordFrame      mDRF;
    R7027RTH             mRTH;
    QVector<R7027RD>     mRDs;
    R7027OD              mOD;
    QVector<R7027ODBeam> mODBeams;
};

constexpr uint8_t mask_R7027RTH_flags_uncertainty        = 0x0F;   // bit 0-3
constexpr uint8_t bit_R7027RTH_flags_multi_detection     = 0x10;   // bit 4
constexpr uint8_t bit_R7027RTH_flags_has_snippets_detect = 0x40;   // bit 6

constexpr uint32_t mask_R7027RT_flags_priority       = 0x00001E00;   // bit 9-12
constexpr uint32_t mask_R7027RT_flags_quality_type   = 0x000001FC;   // bit 2-8
constexpr uint32_t bit_R7027RT_flags_magnitude       = 0x00000001;   // bit 0
constexpr uint32_t bit_R7027RT_flags_phase           = 0x00000002;   // bit 1
constexpr uint32_t bit_R7027RT_flags_snippets_detect = 0x00004000;   // bit 14
constexpr uint32_t bit_R7027RT_quality_brightness    = 0x00000001;   // bit 0
constexpr uint32_t bit_R7027RT_quality_colinearity   = 0x00000002;   // bit 1
constexpr uint32_t shift_R7027RT_flags_priority      = 9;
constexpr uint32_t shift_R7027RT_flags_quality_type  = 2;

#pragma pack(pop)
#endif   // 7KDEF_H
