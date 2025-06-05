#include "LC76G.h"

void LC76G_init()
{
    /*
    // Disable all other types of NEMA sentences
    
    // Enable GGA sentences
    HAL_UART_Transmit(&huart5,LC76G_ENABLE_GGA_FORMAT, strlen(LC76G_ENABLE_GGA_FORMAT), TIMEOUT);
    HAL_Delay(100);
    */
   
    // Ensure that there is a GPS fix
    char buf[2] = {0};
    uint16_t num_iters = 1;
    while (buf == '\0' || num_iters == 0) {
        // Skip everything before <Quality> field
        HAL_UART_Receive(&huart5, NULL, 41, TIMEOUT);
        HAL_UART_Receive(&huart5, buf, 1, TIMEOUT);
        // Skip everything after <Quality> field
        HAL_UART_Receive(&huart5, NULL, 31, TIMEOUT);
        HAL_Delay(100);

        // If this value overflows then we've been in this loop far too long
        num_iters++;
    }

    //Inform caller that a GPS fix was not acquired
    // if (num_iters == 0)
        // handle error
}

LC76G_gps_data LC76G_read_data()
{
    LC76G_gps_data gps_data;

    // Zero-initalized buffer
    char buf[80] = {0};
    char *endptr;

    /* Format from LC76G:
    $<TalkerID>GGA,<UTC>,<Lat>,<N/S>,<Lon>,<E/W>,<Quality>,<NumSatUsed>,<HDOP>,
    <Alt>,M,<Sep>,M,<DiffAge>,<DiffStation>*<Checksum><CR><LF>

    Example:
    GNGGA,040143.000,3149.334166,N,11706.941670,E,2,36,0.48,61.496,M,-0.335,M,,*58
    */

    HAL_UART_Receive(&huart5, buf, 74, TIMEOUT);
    HAL_UART_Transmit(&huart3,buf, strlen(buf), TIMEOUT);
/*
    // Skip the first three fields of received data ($<TalkerID>GGA,)
    HAL_UART_Receive(&huart5, NULL, 7, TIMEOUT);
    
    // Read UTC data
    HAL_UART_Receive(&huart5, buf, 2, TIMEOUT);
    gps_data.time_H = (uint8_t)(strtol(buf, NULL, 10));
    // Ensure that the conversion worked
    //if (buf == endptr || *endptr != '\0')
        // handle error

    HAL_UART_Receive(&huart5, buf, 2, TIMEOUT);
    gps_data.time_M = (uint8_t)(strtol(buf, NULL, 10));
    HAL_UART_Receive(&huart5, buf, 2, TIMEOUT);
    gps_data.time_S = (uint8_t)(strtol(buf, NULL, 10));

    // Skip comma (,)
    HAL_UART_Receive(&huart5, NULL, 1, TIMEOUT);

    // Read latitude
    HAL_UART_Receive(&huart5, buf, 2, TIMEOUT); // Degrees
    gps_data.lat = 0;
    gps_data.lat += strtol(buf, NULL, 10);
    HAL_UART_Receive(&huart5, buf, 2, TIMEOUT); // Minutes
    gps_data.lat += (double)strtol(buf, NULL, 10) / 60;
    HAL_UART_Receive(&huart5, NULL, 1, TIMEOUT);    // Skip period
    HAL_UART_Receive(&huart5, buf, 6, TIMEOUT); // Decimal minutes
    gps_data.lat += (strtol(buf, NULL) / 1000000) / 60;

    // Skip (,<N/S>,)
    HAL_UART_Receive(&huart5, NULL, 3, TIMEOUT);

    // Read longitude
    HAL_UART_Receive(&huart5, buf, 3, TIMEOUT); // Degrees
    gps_data.lon = 0;
    gps_data.lon += strtol(buf, NULL, 10);
    HAL_UART_Receive(&huart5, buf, 2, TIMEOUT); // Minutes
    gps_data.lon += (double)strtol(buf, NULL, 10) / 60;
    HAL_UART_Receive(&huart5, NULL, 1, TIMEOUT);    // Skip period
    HAL_UART_Receive(&huart5, buf, 6, TIMEOUT); // Decimal minutes
    gps_data.lon += ((double)strtol(buf, NULL) / 1000000) / 60;
    
    // Skip (,<E/W>,)
    HAL_UART_Receive(&huart5, NULL, 3, TIMEOUT);

    // Read <Quality> attribute
    HAL_UART_Receive(&huart5, buf, 1, TIMEOUT);
    //if (buf == '0')
        // alert caller that there is no GPS fix

    // Skip comma (,)
    HAL_UART_Receive(&huart5, NULL, 1, TIMEOUT);

    // Read number of satellites
    HAL_UART_Receive(&huart5, buf, 2, TIMEOUT);
    gps_data.num_sat_used = strtol(buf, NULL, 10);

    // Skip (,<HDOP>,)
    HAL_UART_Receive(&huart5, NULL, 4, TIMEOUT);

    // Read altitude
    HAL_UART_Receive(&huart5, buf, 2, TIMEOUT); // integer portion of value
    gps_data.altitude = 0;
    gps_data.altitude += strtol(buf, NULL, 10);
    HAL_UART_Receive(&huart5, NULL, 1, TIMEOUT);    // Skip period
    HAL_UART_Receive(&huart5, buf, 2, TIMEOUT); // decimal portion of value
    gps_data.altitude += (double)strtol(buf, NULL, 10) / 1000;

    // Skip the rest of the transmission:
    // (,M,<Sep>,M,<DiffAge>,<DiffStation>*<Checksum><CR><LF>)
    HAL_UART_Receive(&huart5, NULL, 18, TIMEOUT);
*/
    return gps_data;
}

/*void LC76G_Send_Command(char *data)
{
    HAL_UART_Transmit(&huart3, &data, strlen(data), TIMEOUT);
    HAL_Delay(10);
}
*/
