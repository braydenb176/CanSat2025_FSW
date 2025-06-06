#ifndef _LC76G_H_
#define _LC76G_H_

#include "stm32g4xx_hal.h"
#include "uart_interrupt.h"

#define TIMEOUT 1000

// DMA buffer aligned to 4-bytes
#define GPS_DMA_BUFFER_SIZE 128
uint8_t gps_dma_buffer[GPS_DMA_BUFFER_SIZE] __attribute__((aligned(4)));

/* Define GPS commands */
// Checksums calculated using: https://nmeachecksum.eqth.net/

// PAIR messages
static const char LC76_ENABLE_GGA[] = "$PAIR062,0,1*3F";
static const char LC76_DISABLE_GGL[] = "$PAIR062,1,0*3F";
static const char LC76_DISABLE_GSA[] = "$PAIR062,2,0*3C";
static const char LC76_DISABLE_GSV[] = "$PAIR062,3,0*3D";
static const char LC76_DISABLE_RMC[] = "$PAIR062,4,0*3A";
static const char LC76_DISABLE_VTG8[] = "$PAIR062,5,0*3B";

// We need: time, lat, lon, alt, numberOfSats
// Time format: HH:MM:SS
// Degrees in decimal degrees
// Altitude in meters above sea level
typedef struct {
    uint8_t time_H;         // UTC Time
    uint8_t time_M;
    uint8_t time_S;

    double lat;
    double lon;

    double altitude;

    uint8_t num_sat_used;
}LC76G_gps_data;
LC76G_gps_data gps_data;

/* Define functions */
LC76G_gps_data LC76G_read_data();
// void LC76G_Send_Command(char *data);

#endif /* _LC76G_H_ */