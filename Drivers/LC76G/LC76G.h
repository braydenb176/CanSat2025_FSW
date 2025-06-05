#ifndef _LC76G_H_
#define _LC76G_H_

#include "stm32g4xx_hal.h"
#include "uart_interrupt.h"

#define TIMEOUT 1000

/* Define GPS commands */

// The following two PAIR sentences need to be revised
char LC76G_UPDATE_1HZ[] = "$PAIR220,1000*1F";  // Set update rate to 1 second
char LC76G_BAUD_115200[] = "$PAIR864,0,0,115200";   // 115200 baud


// Baud rates
#define LC76G_BAUD_4800    "$PAIR864,0,0,4800"    // 4800 baud
#define LC76G_BAUD_9600    "$PAIR864,0,0,9600"    // 9600 baud
#define LC76G_BAUD_19200   "$PAIR864,0,0,19200"   // 19200 baud
#define LC76G_BAUD_38400   "$PAIR864,0,0,38400"   // 38400 baud
#define LC76G_BAUD_57600   "$PAIR864,0,0,57600"   // 57600 baud
#define LC76G_BAUD_115200  "$PAIR864,0,0,115200"  // 115200 baud

// Update rates
#define LC76G_POS_FIX_100MS   "$PAIR050,100"      // 10Hz   - 100ms
#define LC76G_POS_FIX_200MS   "$PAIR050,200"      // 5Hz    - 200ms
#define LC76G_POS_FIX_300MS   "$PAIR050,300"      // 3.33Hz - 300ms
#define LC76G_POS_FIX_400MS   "$PAIR050,400"      // 2.5Hz  - 400ms
#define LC76G_POS_FIX_500MS   "$PAIR050,500"      // 2Hz    - 500ms
#define LC76G_POS_FIX_600MS   "$PAIR050,600"      // 1.67Hz - 600ms
#define LC76G_POS_FIX_700MS   "$PAIR050,700"      // 1.43Hz - 700ms
#define LC76G_POS_FIX_800MS   "$PAIR050,800"      // 1.25Hz - 800ms
#define LC76G_POS_FIX_900MS   "$PAIR050,900"      // 1.11Hz - 900ms
#define LC76G_POS_FIX_1S      "$PAIR050,1000"     // 1Hz    - 1 second

// Startup modes
#define LC76G_HOT_START       "$PAIR004"          // Hot starts  - 1
#define LC76G_WARM_START      "$PAIR005"          // Warm starts - 2
#define LC76G_COLD_START      "$PAIR006"          // Cold starts - 3

// Clear module config
#define LC76G_CONFIG_CLEAR    "$PAIR007"          // Clear system configuration and cold starts

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