#include "LC76G.h"
#include <string.h>
#include <math.h>

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
    while (buf == '\0' || num_iters == 0)
    {
        // Skip everything before <Quality> field
        HAL_UART_Receive(&huart5, NULL, 41, TIMEOUT);
        HAL_UART_Receive(&huart5, buf, 1, TIMEOUT);
        // Skip everything after <Quality> field
        HAL_UART_Receive(&huart5, NULL, 31, TIMEOUT);
        HAL_Delay(100);

        // If this value overflows then we've been in this loop far too long
        num_iters++;
    }

    // Inform caller that a GPS fix was not acquired
    //  if (num_iters == 0)
    //  handle error
}

LC76G_gps_data LC76G_read_data()
{
    LC76G_gps_data gps_data;

    // Zero-initalized buffer
    unsigned char GGA_sentence[128] = {0};

    // Receive GGA sentence from GPS module
    HAL_UART_Receive(&huart5, GGA_sentence, 128, 3000);
    printf("Raw data from GPS module: %s", GGA_sentence);
    HAL_UART_Transmit(&huart3, GGA_sentence, strlen(GGA_sentence), TIMEOUT);

    // GGA format:
    // $<TalkerID>GGA,<UTC>,<Lat>,<N/S>,<Lon>,<E/W>,<Quality>,<NumSatUsed>,
    // <HDOP>,<Alt>,M,<Sep>,M,<DiffAge>,<DiffStation>*<Checksum><CR><LF>

    // Iterate through the comma-delimited string
    char *token = strok(GGA_sentence, ",");
    uint8_t tokenNum = 1;
    while (token != NULL && strchr(token, '$') == NULL) {
        switch(tokenNum) {
            case 1: // <UTC>
                char temp1[3] = {0};
                strncpy(temp1, token, 2);        // Hours
                gps_data.time_H = strtol(temp1, NULL, 10);

                memset(temp1, 0, 3 * sizeof(char));
                strncpy(temp1, token + 2, 2);    // Minutes
                gps_data.time_M = strtol(temp1, NULL, 10);

                memset(temp1, 0, 3 * sizeof(char));
                strncpy(temp1, token + 4, 2);    // Seconds
                gps_data.time_M = strtol(temp1, NULL, 10);
                break;
            case 2: // <Lat>
                char temp2[8] = {0};
                strncpy(temp2, token, 2);        // Degrees
                gps_data.lat = (double)strtol(temp2, NULL, 10);

                memset(temp2, 0, 8 * sizeof(char));
                strncpy(temp2, token + 2, 2);    // Minutes
                gps_data.lat += (double)strtol(temp2, NULL, 10) / 60;

                memset(temp2, 0, 8 * sizeof(char));
                strncpy(temp2, token + 5, 6);    // Decimal minutes
                gps_data.lat += ((double)strtol(temp2, NULL) / pow(10, strlen(temp2))) / 60;
                break;
            case 3: // <N/S>
                // Flip sign of latitude if in southern hemisphere
                if (token[0] == 'S')
                    gps_data.lat = -gps_data.lat;
                break;
            case 4: // <Lon>
                char temp3[8] = {0};
                strncpy(temp3, token, 3);        // Degrees
                gps_data.lon = (double)strtol(temp3, NULL, 10);

                memset(temp3, 0, 8 * sizeof(char));
                strncpy(temp3, token + 1, 2);    // Minutes
                gps_data.lon += (double)strtol(temp3, NULL, 10) / 60;

                memset(temp3, 0, 8 * sizeof(char));
                strncpy(temp3, token + 6, 6);    // Decimal minutes
                gps_data.lon += ((double)strtol(temp3, NULL) / pow(10, strlen(temp3))) / 60;
                break;
            case 5: // <E/W>
                // Flip sign of latitude if in western hemisphere
                if (token[0] == 'W')
                    gps_data.lon = -gps_data.lon;
                break;
            case 7: // <NumSatUsed>
                gps_data.num_sat_used = strtol(token, NULL, 10);
                break;
            case 9: // <Alt>
                gps_data.altitude = strtol(token, NULL, 10);
                break;
            default:
                break;
        }   // End switch statement

        token = strok(NULL, ",");
        tokenNum++;
    }   // End string iteration

    return gps_data;
}

/*void LC76G_Send_Command(char *data)
{
    HAL_UART_Transmit(&huart3, &data, strlen(data), TIMEOUT);
    HAL_Delay(10);
}
*/
