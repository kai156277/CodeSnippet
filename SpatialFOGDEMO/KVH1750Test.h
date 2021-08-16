#pragma once
#include <stdint.h>

#pragma pack(push)
#pragma pack(1)
struct KVHMessage
{
    uint32_t header;
    float    xRot;
    float    yRot;
    float    zRot;
    float    xAcc;
    float    yAcc;
    float    zAcc;
    uint8_t  status;
    uint8_t  sequenceNumber;
    uint16_t temperature;
    uint32_t crc;
};
#pragma pack(pop)
static_assert((sizeof(KVHMessage) == 36), "KVHMessage is not 36 Bytes");

uint16_t reversebytes_uint16t(uint16_t value);

uint32_t reversebytes_uint32t(uint32_t value);

float reversebytes_float(float value);

uint64_t reversebytes_uint64t(uint64_t value);

uint32_t crc32_mpeg_2(const uint8_t *data, uint64_t length);

class KVH1750
{
public:
    KVH1750();
    static bool dataDecode(const uint8_t *data, KVHMessage *kvh);
};
