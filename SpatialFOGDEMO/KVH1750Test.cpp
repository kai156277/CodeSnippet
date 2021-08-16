#include "KVH1750Test.h"
#include <memory>

// clang-format off
static const uint8_t test_data_1[] = {
    0xFE, 	 0x81,	  0xFF,   0x55, // header
    0x37,    0xA9,    0x6A,   0x6E, // gyro x
    0x38,    0x58,    0x6C,   0x1F, // gyro y
    0xB7,    0x5B,    0xF8,   0x62,	// gyro z
    0xBF,    0x80,    0x3E,   0x78, // accel x
    0xBB,    0x65,    0x0D,   0x28, // accel y
    0x3B,    0x0A,    0x37,   0xAC, // accel z
    0x77,    						// status
    0x3D,    						// sequence
    0x00,    0x28, 					// temperature
    0x4B,    0xFA,    0x34,   0xD8, // crc
};
// clang-format on

KVH1750::KVH1750()
{
}

bool KVH1750::dataDecode(const uint8_t *data, KVHMessage *kvh)
{
    if (!kvh)
        return false;

    memcpy_s(kvh, 36, data, 36);
    kvh->header      = reversebytes_uint32t(kvh->header);
    kvh->xRot        = reversebytes_float(kvh->xRot);
    kvh->yRot        = reversebytes_float(kvh->yRot);
    kvh->zRot        = reversebytes_float(kvh->zRot);
    kvh->xAcc        = reversebytes_float(kvh->xAcc);
    kvh->yAcc        = reversebytes_float(kvh->yAcc);
    kvh->zAcc        = reversebytes_float(kvh->zAcc);
    kvh->temperature = reversebytes_uint16t(kvh->temperature);
    kvh->crc         = reversebytes_uint32t(kvh->crc);

    if (kvh->header != 0xfe81ff55)
        return false;

    if (kvh->crc != crc32_mpeg_2(data, 32))
        return false;

    return true;
}

uint16_t reversebytes_uint16t(uint16_t value)
{
    return (value & 0x00FFU) << 8 |
           (value & 0xFF00U) >> 8;
}

static void reversebytes_uint32t(const uint32_t *value, uint32_t *r_value)
{
    (*r_value) = reversebytes_uint32t(*value);
}

uint32_t reversebytes_uint32t(uint32_t value)
{
    return (value & 0x000000FFU) << 24 |
           (value & 0x0000FF00U) << 8 |
           (value & 0x00FF0000U) >> 8 |
           (value & 0xFF000000U) >> 24;
}

float reversebytes_float(float value)
{
    float new_value = 0.0;
    reversebytes_uint32t((uint32_t *) &value, (uint32_t *) &new_value);
    return new_value;
}

uint64_t reversebytes_uint64t(uint64_t value)
{

    return (value & 0x00000000000000FFU) << 56 |
           (value & 0x000000000000FF00U) << 40 |
           (value & 0x0000000000FF0000U) << 24 |
           (value & 0x00000000FF000000U) << 8 |
           (value & 0x000000FF00000000U) >> 8 |
           (value & 0x0000FF0000000000U) >> 24 |
           (value & 0x00FF000000000000U) >> 40 |
           (value & 0xFF00000000000000U) >> 56;
}

/******************************************************************************
 * Name:    CRC-32/MPEG-2  x32+x26+x23+x22+x16+x12+x11+x10+x8+x7+x5+x4+x2+x+1
 * Poly:    0x4C11DB7
 * Init:    0xFFFFFFF
 * Refin:   False
 * Refout:  False
 * Xorout:  0x0000000
 * Note:
 *****************************************************************************/
uint32_t crc32_mpeg_2(const uint8_t *data, uint64_t length)
{
    uint8_t  i;
    uint32_t crc = 0xffffffff;   // Initial value
    while (length--)
    {
        crc ^= (uint32_t)(*data++) << 24;   // crc ^=(uint32_t)(*data)<<24; data++;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }
    return crc;
}
