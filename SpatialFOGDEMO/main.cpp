#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <type_traits>

#include <QDataStream>
#include <QDebug>
#include <QFile>
#include <QTextStream>

#ifdef _WIN32
#    include <Windows.h>
#else
#    include <unistd.h>
#endif

#include "C/Dynamic/an_packet_protocol.h"
#include "C/Dynamic/rs232/rs232.h"
#include "C/Dynamic/spatial_packets.h"
#include "KVH1750Test.h"

#define M_PI 3.14159265358979323846

#define RADIANS_TO_DEGREES (180.0 / M_PI)

int an_packet_transmit(an_packet_t *an_packet)
{
    an_packet_encode(an_packet);
    return SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

/*
 * This is an example of sending a configuration packet to Spatial.
 *
 * 1. First declare the structure for the packet, in this case filter_options_packet_t.
 * 2. Set all the fields of the packet structure
 * 3. Encode the packet structure into an an_packet_t using the appropriate helper function
 * 4. Send the packet
 * 5. Free the packet
 */
void set_filter_options()
{
    an_packet_t *           an_packet;
    filter_options_packet_t filter_options_packet;

    /* initialise the structure by setting all the fields to zero */
    memset(&filter_options_packet, 0, sizeof(filter_options_packet_t));

    filter_options_packet.permanent                    = TRUE;
    filter_options_packet.vehicle_type                 = vehicle_type_car;
    filter_options_packet.internal_gnss_enabled        = TRUE;
    filter_options_packet.atmospheric_altitude_enabled = TRUE;
    filter_options_packet.velocity_heading_enabled     = TRUE;
    filter_options_packet.reversing_detection_enabled  = TRUE;
    filter_options_packet.motion_analysis_enabled      = TRUE;

    an_packet = encode_filter_options_packet(&filter_options_packet);

    an_packet_transmit(an_packet);

    an_packet_free(&an_packet);
}

// clang-format off
static const uint8_t test_data_1[] = {
    0xFE, 	 0x81,	  0xFF,   0x55,
    0x37,    0xA9,    0x6A,   0x6E,
    0x38,    0x58,    0x6C,   0x1F,
    0xB7,    0x5B,    0xF8,   0x62,
    0xBF,    0x80,    0x3E,   0x78,
    0xBB,    0x65,    0x0D,   0x28,
    0x3B,    0x0A,    0x37,   0xAC,
    0x77,    0x3D,    0x00,   0x28,
    0x4B,    0xFA,    0x34,   0xD8,
};
// clang-format on

#if 0
int main(int argc, char *argv[])
{
    KVHMessage msg;
    qDebug() << KVH1750::dataDecode(test_data_1, &msg);
    qDebug() << "header:" << QString::number(msg.header, 16);
    qDebug() << "x rot:" << msg.xRot;
    qDebug() << "y rot:" << msg.yRot;
    qDebug() << "z rot:" << msg.zRot;
    qDebug() << "x acc:" << msg.xAcc;
    qDebug() << "y acc:" << msg.yAcc;
    qDebug() << "z acc:" << msg.zAcc;
    qDebug() << "status:" << QString::number(msg.status, 16);
    qDebug() << "sn:" << msg.sequenceNumber;
    qDebug() << "temp:" << msg.temperature;
    qDebug() << "crc:" << msg.crc;
    qDebug() << "-------";
}
#endif

int main(int argc, char *argv[])
{
    QString kvh_file = "D:\\SpatialFOG\\KVH1750\\20210514-190000.kvh";
    QFile   kvh(kvh_file);
    if (!kvh.open(QIODevice::ReadOnly))
    {
        qDebug() << "open file faild:" << kvh_file;
    }

    KVHMessage msg;
    qDebug() << "sizeof: " << sizeof(KVHMessage);
    QDataStream bin(&kvh);
    while (!bin.atEnd())
    {
        bin.readRawData((char *) &msg, 36);
        qDebug() << "sn:" << msg.sequenceNumber;
        qDebug() << "temp:" << msg.temperature;
        qDebug() << "status:" << QString::number(msg.status, 2);
        qDebug() << "x rot:" << msg.xRot;
        qDebug() << "y rot:" << msg.yRot;
        qDebug() << "z rot:" << msg.zRot;
        qDebug() << "x acc:" << msg.xAcc;
        qDebug() << "y acc:" << msg.yAcc;
        qDebug() << "z acc:" << msg.zAcc;
        qDebug() << "header:" << QString::number(msg.header, 16);
        qDebug() << "crc:" << msg.crc;
        qDebug() << "-------";
    }
    qDebug() << "END!";

    return 0;
}
#if 0
int main(int argc, char *argv[])
{
    an_decoder_t an_decoder;
    an_packet_t *an_packet;

    system_state_packet_t system_state_packet;
    raw_sensors_packet_t  raw_sensors_packet;

    int bytes_received;

    //    if (argc != 3)
    //    {
    //        printf("Usage - program com_port baud_rate\nExample - packet_example.exe COM1 115200\n");
    //        exit(EXIT_FAILURE);
    //    }

    /* open the com port 921600 */
    char com[] = "COM3";
    if (OpenComport(com, 460800))
    {
        printf("Could not open serial port\n");
        exit(EXIT_FAILURE);
    }

    an_decoder_initialise(&an_decoder);

    while (1)
    {
        if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
        {
            /* increment the decode buffer length by the number of bytes received */
            an_decoder_increment(&an_decoder, bytes_received);

            /* decode all the packets in the buffer */
            while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
            {
                if (an_packet->id == packet_id_system_state) /* system state packet */
                {
                    /* copy all the binary data into the typedef struct for the packet */
                    /* this allows easy access to all the different values             */
                    if (decode_system_state_packet(&system_state_packet, an_packet) == 0)
                    {
                        printf("System State Packet:\n");
                        printf("\tLatitude = %f, Longitude = %f, Height = %f\n", system_state_packet.latitude * RADIANS_TO_DEGREES, system_state_packet.longitude * RADIANS_TO_DEGREES, system_state_packet.height);
                        printf("\tRoll = %f, Pitch = %f, Heading = %f\n", system_state_packet.orientation[0] * RADIANS_TO_DEGREES, system_state_packet.orientation[1] * RADIANS_TO_DEGREES, system_state_packet.orientation[2] * RADIANS_TO_DEGREES);
                    }
                }
                else if (an_packet->id == packet_id_raw_sensors) /* raw sensors packet */
                {
                    /* copy all the binary data into the typedef struct for the packet */
                    /* this allows easy access to all the different values             */
                    if (decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
                    {
                        printf("Raw Sensors Packet:\n");
                        printf("\tAccelerometers X: %f Y: %f Z: %f\n", raw_sensors_packet.accelerometers[0], raw_sensors_packet.accelerometers[1], raw_sensors_packet.accelerometers[2]);
                        printf("\tGyroscopes X: %f Y: %f Z: %f\n", raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES);
                    }
                }
                else
                {
                    printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
                }

                /* Ensure that you free the an_packet when your done with it or you will leak memory */
                an_packet_free(&an_packet);
            }
        }
#    ifdef _WIN32
        Sleep(1000);
#    else
        usleep(10000);
#    endif
    }
}
#    if 0
#    endif
#endif
