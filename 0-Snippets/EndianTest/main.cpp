#include <QDebug>
#include <QString>
#include <iostream>

using namespace std;

/*
 * 0x1234
 *
 * BigEndian:
 * 0x12 | 0x34
 *
 * Small:
 * 0x34 | 0x12
 */

bool isBigEndian()
{
    union Num
    {
        short a;
        char  b;
    };
    Num num;
    num.a = 0x1234;

    if (0x12 == num.b)
    {
        return true;
    }
    return false;
}

uint16_t reversebytes_uint16t(uint16_t value)
{
    return (value & 0x00FFU) << 8 |
           (value & 0xFF00U) >> 8;
}

uint32_t reversebytes_uint32t(uint32_t value)
{
    return (value & 0x000000FFU) << 24 |
           (value & 0x0000FF00U) << 8 |
           (value & 0x00FF0000U) >> 8 |
           (value & 0xFF000000U) >> 24;
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

void printf_num(uint8_t *data, uint32_t size)
{
    QStringList datas;
    for (int i = 0; i < size; ++i)
    {
        datas.append(QString::number(*(data + i), 16));
    }
    qDebug() << datas.join(' ');
}

int main()
{
    qDebug() << "is BigEndian: " << isBigEndian();
    uint16_t a = 0x1234;
    uint32_t b = 0x12345678;
    uint64_t c = 0x12345678ABCDEF0;
    qDebug() << "SmallEndian: ";
    printf_num((uint8_t *) &a, sizeof(a));
    printf_num((uint8_t *) &b, sizeof(b));
    printf_num((uint8_t *) &c, sizeof(c));
    qDebug() << "BigEndian: ";
    a = reversebytes_uint16t(a);
    b = reversebytes_uint32t(b);
    c = reversebytes_uint64t(c);
    printf_num((uint8_t *) &a, sizeof(a));
    printf_num((uint8_t *) &b, sizeof(b));
    printf_num((uint8_t *) &c, sizeof(c));
    return 0;
}
