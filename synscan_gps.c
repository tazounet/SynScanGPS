// SynScan GPS emulator using a Pico with a GPS
// Copyright (C) 2014-2025 tazounet

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/status_led.h"
#include "minmea.h"

// GPS uart configuration
#define GPS_UART_ID uart0
#define GPS_BAUD_RATE 9600
#define GPS_UART_TX_PIN 0
#define GPS_UART_RX_PIN 1

// SynScan uart configuration
#define SYNSCAN_UART_ID uart1
#define SYNSCAN_BAUD_RATE 4800
#define SYNSCAN_UART_TX_PIN 8
#define SYNSCAN_UART_RX_PIN 9

// LED colors
#define RED_LED PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(0x0f, 0, 0)
#define YELLOW_LED PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(0x0f, 0x50, 0)
#define GREEN_LED PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(0, 0x0f, 0)

// SynScan binary message
// User Position, Velocity & Time II (D1h)
typedef struct synscan_pvt_msg_t
{
    uint16_t week_no;
    uint32_t time_of_week;
    uint32_t date;
    uint32_t time;
    int32_t latitude;
    int32_t longitude;
    int16_t altitude;
    uint16_t heading;
    uint16_t speed;
    uint8_t fix_indicator;
    uint8_t quality_of_fix;
    uint8_t number_of_sv;
    uint8_t number_of_sv_in_fix;
    uint8_t gdop;
    uint8_t pdop;
    uint8_t hdop;
    uint8_t vdop;
    uint8_t tdop;
} __attribute__((packed)) synscan_pvt_msg_t;
// header = 4 bytes
// crc + trailer = 3 bytes
#define SYNSCAN_PVT_MSG_SIZE (4 + sizeof(synscan_pvt_msg_t) + 3)

static void synscan_parse(char c);
static void synscan_build_pvt_msg(synscan_pvt_msg_t *pvt, char *msg);
static void sendAck();
static uint8_t uint2bcd(int val);
static char compute_checksum(char *buf, uint16_t len);
static void gps_parse(char c);

// Global State
bool g_send_synscan_pvt_msg = false;
synscan_pvt_msg_t g_synscan_pvt_msg = {0};
bool g_new_nmea = false;

// Synscan command buffer
#define SYNSCAN_BUFF_SIZE 32
char g_synscan_buffer[SYNSCAN_BUFF_SIZE] = {0};
uint8_t g_synscan_index = 0;

// GPS nmea buffer
#define GPS_BUFF_SIZE MINMEA_MAX_SENTENCE_LENGTH
char g_nmea_buffer[GPS_BUFF_SIZE] = {0};
uint8_t g_nmea_index = 0;

bool colored_status_led_change_color(uint32_t color)
{
    // we need to turn off the LED first to change color
    colored_status_led_set_state(false);
    sleep_us(100);
    return colored_status_led_set_on_with_color(color);
}

uint8_t uint2bcd(int val)
{
    uint8_t ival = val % 100; // Ensure ival is in the range 0-99
    return ((ival / 10) << 4) | (ival % 10);
}

// Compute binary message checksum
static char compute_checksum(char *buf, uint16_t len)
{
    char chksum = 0;

    for (uint16_t i = 0; i < len; i++)
    {
        chksum ^= buf[i];
    }

    return chksum;
}

static void synscan_parse(char c)
{
    // Add byte to buffer
    if (g_synscan_index < SYNSCAN_BUFF_SIZE - 1)
    {
        g_synscan_buffer[g_synscan_index++] = c;
    }
    else
    {
        // Buffer overflow, reset index
        g_synscan_index = 0;
    }

    if (c == '\n')
    {
        // End of command
        if (strncmp(g_synscan_buffer, "%%\xf1\x13\x00\xe2\r\n", g_synscan_index) == 0)
        {
            // No output
            g_send_synscan_pvt_msg = false;

            printf("Received 'no output' command\n");

            // Send ack
            sendAck();
        }
        else if (strncmp(g_synscan_buffer, "%%\xf1\x13\x03\xe1\r\n", g_synscan_index) == 0)
        {
            // Binary PVT output
            g_send_synscan_pvt_msg = true;

            printf("Received 'binary output' command\n");

            // Send ack
            sendAck();
        }
        else
        {
            printf("Received unknown command: ");
            for (uint8_t i = 0; i < g_synscan_index; i++)
            {
                printf("%02x ", g_synscan_buffer[i]);
            }
            printf("\n");
        }

        // Clean buff
        g_synscan_index = 0;
    }
}

static void synscan_build_pvt_msg(synscan_pvt_msg_t *pvt, char *msg)
{
    uint16_t size_for_crc = 4 + sizeof(synscan_pvt_msg_t);

    // header
    msg[0] = '%';
    msg[1] = '%';
    msg[2] = '\xf2';
    msg[3] = '\xd1';

    // copy PVT data
    memcpy(msg + 4, pvt, sizeof(synscan_pvt_msg_t));

    // Checksum
    msg[size_for_crc] = compute_checksum(msg, size_for_crc);

    // Trailer
    msg[size_for_crc + 1] = '\r';
    msg[size_for_crc + 2] = '\n';
}

static void sendAck()
{
    char msg[7] = {'%', '%', '\x06', '\x13', '\x15', '\r', '\n'};
    for (uint16_t i = 0; i < sizeof(msg); i++)
    {
        uart_putc_raw(SYNSCAN_UART_ID, msg[i]);
    }
}

static void gps_parse(char c)
{
    // copy character to nmea buffer
    g_nmea_buffer[g_nmea_index++] = c;

    // if we reach the end of the sentence or buffer is full
    if (c == '\n' || g_nmea_index >= sizeof(g_nmea_buffer) - 2)
    {
        g_nmea_buffer[g_nmea_index] = '\0';

        // call minmea parser
        switch (minmea_sentence_id(g_nmea_buffer, false))
        {
        case MINMEA_SENTENCE_RMC:
        {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, g_nmea_buffer))
            {
                uint8_t tmp_bytes[4];

                g_synscan_pvt_msg.week_no = 0;      // not used
                g_synscan_pvt_msg.time_of_week = 0; // not used

                tmp_bytes[0] = uint2bcd(frame.date.day);
                tmp_bytes[1] = uint2bcd(frame.date.month);
                tmp_bytes[2] = uint2bcd(frame.date.year);
                tmp_bytes[3] = 0;
                memcpy(&(g_synscan_pvt_msg.date), tmp_bytes, 4);

                tmp_bytes[0] = uint2bcd(frame.time.seconds);
                tmp_bytes[1] = uint2bcd(frame.time.minutes);
                tmp_bytes[2] = uint2bcd(frame.time.hours);
                tmp_bytes[3] = 0;
                memcpy(&(g_synscan_pvt_msg.time), tmp_bytes, sizeof(g_synscan_pvt_msg.time));

                float latitude = (minmea_tocoord(&frame.latitude) / 360.0) * 4294967296.0;
                g_synscan_pvt_msg.latitude = (int32_t)latitude;

                float longitude = (minmea_tocoord(&frame.longitude) / 360.0) * 4294967296.0;
                g_synscan_pvt_msg.longitude = (int32_t)longitude;

                g_synscan_pvt_msg.heading = (uint16_t)minmea_tofloat(&frame.course);
                g_synscan_pvt_msg.speed = (uint16_t)minmea_tofloat(&frame.speed);

                // Flag that we have a new NMEA sentence using the RMC sentence
                // shall be only set once per second
                g_new_nmea = true;
            }
            else
            {
                printf("$xxRMC sentence is not parsed\n");
            }
        }
        break;

        case MINMEA_SENTENCE_GGA:
        {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, g_nmea_buffer))
            {
                g_synscan_pvt_msg.altitude = (int16_t)minmea_tofloat(&frame.altitude);
                g_synscan_pvt_msg.number_of_sv = frame.satellites_tracked;
                g_synscan_pvt_msg.number_of_sv_in_fix = frame.satellites_tracked;
                g_synscan_pvt_msg.fix_indicator = 0; // not used
            }
            else
            {
                printf("$xxGGA sentence is not parsed\n");
            }
        }
        break;

        // GSA
        case MINMEA_SENTENCE_GSA:
        {
            struct minmea_sentence_gsa frame;
            if (minmea_parse_gsa(&frame, g_nmea_buffer))
            {
                g_synscan_pvt_msg.pdop = (uint8_t)(minmea_tofloat(&frame.pdop) * 10.0);
                g_synscan_pvt_msg.hdop = (uint8_t)(minmea_tofloat(&frame.hdop) * 10.0);
                g_synscan_pvt_msg.vdop = (uint8_t)(minmea_tofloat(&frame.vdop) * 10.0);
                g_synscan_pvt_msg.quality_of_fix = frame.fix_type - 1; // 0 for no fix, 1 for 2D fix, 2 for 3D fix
            }
            else
            {
                printf("$xxGSA sentence is not parsed\n");
            }
        }
        break;

        case MINMEA_INVALID:
        {
            printf("$xxxxx sentence is not valid\n");
        }
        break;

        case MINMEA_UNKNOWN:
        {
            printf("$xxxxx sentence is unknown\n");
        }
        break;

        default: // ignore other sentences
            break;
        }

        g_nmea_index = 0;
    }
}

int main()
{
    char synscan_pvt_buff[SYNSCAN_PVT_MSG_SIZE] = {0};
    uint16_t synscan_pvt_buff_index = SYNSCAN_PVT_MSG_SIZE;

    stdio_init_all();

    // GPS UART initialization
    gpio_set_function(GPS_UART_TX_PIN, UART_FUNCSEL_NUM(GPS_UART_ID, GPS_UART_TX_PIN));
    gpio_set_function(GPS_UART_RX_PIN, UART_FUNCSEL_NUM(GPS_UART_ID, GPS_UART_RX_PIN));
    uart_init(GPS_UART_ID, GPS_BAUD_RATE);

    // SynScan UART initialization
    gpio_set_function(SYNSCAN_UART_TX_PIN, UART_FUNCSEL_NUM(SYNSCAN_UART_ID, SYNSCAN_UART_TX_PIN));
    gpio_set_function(SYNSCAN_UART_RX_PIN, UART_FUNCSEL_NUM(SYNSCAN_UART_ID, SYNSCAN_UART_RX_PIN));
    uart_init(SYNSCAN_UART_ID, SYNSCAN_BAUD_RATE);

    // LED initialization
    status_led_init();

    // red LED
    colored_status_led_set_on_with_color(RED_LED);

    while (true)
    {
        // GPS UART processing
        while (uart_is_readable(GPS_UART_ID))
        {
            char c = uart_getc(GPS_UART_ID);
            gps_parse(c);
        }

        // SynScan UART processing
        while (uart_is_readable(SYNSCAN_UART_ID))
        {
            char c = uart_getc(SYNSCAN_UART_ID);
            synscan_parse(c);
        }

        // send SynScan PVT message char by char to avoid blocking the loop too long
        if (synscan_pvt_buff_index < SYNSCAN_PVT_MSG_SIZE)
        {
            // Send the next bytes of the PVT message
            for (uint16_t i = 0; (i < 4) && (synscan_pvt_buff_index < SYNSCAN_PVT_MSG_SIZE); i++)
            {
                uart_putc_raw(SYNSCAN_UART_ID, synscan_pvt_buff[synscan_pvt_buff_index++]);
            }
        }

        // check if we have a new NMEA sentence
        if (g_new_nmea)
        {
            // Update the status LED color
            if (g_synscan_pvt_msg.quality_of_fix == 0) // No fix
            {
                colored_status_led_change_color(RED_LED);
            }
            else if (g_synscan_pvt_msg.quality_of_fix == 1) // 2D fix
            {
                colored_status_led_change_color(YELLOW_LED);
            }
            else if (g_synscan_pvt_msg.quality_of_fix == 2) // 3D fix
            {
                colored_status_led_change_color(GREEN_LED);
            }

            // Check if we need to send the SynScan PVT message
            if (g_send_synscan_pvt_msg)
            {
                // Build the PVT message
                synscan_build_pvt_msg(&g_synscan_pvt_msg, synscan_pvt_buff);

                // Reset the index to start sending the message
                synscan_pvt_buff_index = 0;
            }

            // Reset the new NMEA flag
            g_new_nmea = false;
        }
    }

    return 0;
}
