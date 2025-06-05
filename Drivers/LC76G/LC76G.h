#ifndef _LC76G_H_
#define _LC76G_H_

#include "stm32g4xx_hal.h"
#include "uart_interrupt.h"

#define TIMEOUT 1000

/* Define GPS commands */

// The following two PAIR sentences need to be revised
// char LC76G_UPDATE_1HZ[] = "$PAIR220,1000*1F";  // Set update rate to 1 second
// char LC76G_BAUD_115200[] = "$PAIR864,0,0,115200";   // 115200 baud

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

/* Define functions */
LC76G_gps_data LC76G_read_data();
// void LC76G_Send_Command(char *data);
// Coordinates LC76G_Google_Coordinates(void);
// GNRMC LC76G_Gat_GNRMC(void);
// void LC76G_Exit_BackupMode(void);

#endif /* _LC76G_H_ */