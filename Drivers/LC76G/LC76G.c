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
    char GGA_sentence[128] = {0};

    // Receive GGA sentence from GPS module
    HAL_UART_Receive(&huart5, GGA_sentence, 128, TIMEOUT);
<<<<<<< Updated upstream
    //HAL_UART_Transmit(&huart3, GGA_sentence, strlen(buf), TIMEOUT);
=======
    //printf("Raw data from GPS module: %s", GGA_sentence);
    //HAL_UART_Transmit(&huart3, GGA_sentence, strlen(GGA_sentence), TIMEOUT);
>>>>>>> Stashed changes

    // GGA format:
    // $<TalkerID>GGA,<UTC>,<Lat>,<N/S>,<Lon>,<E/W>,<Quality>,<NumSatUsed>,
    // <HDOP>,<Alt>,M,<Sep>,M,<DiffAge>,<DiffStation>*<Checksum><CR><LF>

    // Iterate through the comma-delimited string
<<<<<<< Updated upstream
    char *token = strok(GGA_sentence, ",");
    uint8_t tokenNum = 0;
=======
    char *token = strtok(GGA_sentence, ",");
    uint8_t tokenNum = 1;
>>>>>>> Stashed changes
    while (token != NULL && strchr(token, '$') == NULL) {
        token = strok(NULL, ",");
        tokenNum++;

        switch(tokenNum) {
            case 1: // <UTC>
                char temp[3] = {0};
                strncpy(temp, token, 2);        // Hours
                gps_data.time_H = strtol(temp, NULL, 10);

                memset(temp, 0, 3 * sizeof(char));
                strncpy(temp, token + 2, 2);    // Minutes
                gps_data.time_M = strtol(temp, NULL, 10);

                memset(temp, 0, 3 * sizeof(char));
                strncpy(temp, token + 4, 2);    // Seconds
                gps_data.time_M = strtol(temp, NULL, 10);
                break;
            case 2: // <Lat>
                char temp[8] = {0};
                strncpy(temp, token, 2);        // Degrees
                gps_data.lat = (double)strtol(temp, NULL, 10);

                memset(temp, 0, 8 * sizeof(char));
                strncpy(temp, token + 2, 2);    // Minutes
                gps_data.lat += (double)strtol(temp, NULL, 10) / 60;

                memset(temp, 0, 8 * sizeof(char));
                strncpy(temp, token + 5, 6);    // Decimal minutes
                gps_data.lat += ((double)strtol(temp, NULL) / pow(10, strlen(temp))) / 60;
                break;
            case 3: // <N/S>
                // Flip sign of latitude if in southern hemisphere
                if (token[0] == 'S')
                    gps_data.lat = -gps_data.lat;
                break;
            case 4: // <Lon>
                char temp[8] = {0};
                strncpy(temp, token, 3);        // Degrees
                gps_data.lon = (double)strtol(temp, NULL, 10);

                memset(temp, 0, 8 * sizeof(char));
                strncpy(temp, token + 1, 2);    // Minutes
                gps_data.lon += (double)strtol(temp, NULL, 10) / 60;

                memset(temp, 0, 8 * sizeof(char));
                strncpy(temp, token + 6, 6);    // Decimal minutes
                gps_data.lon += ((double)strtol(temp, NULL) / pow(10, strlen(temp))) / 60;
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
<<<<<<< Updated upstream
=======

        token = strtok(NULL, ",");
        tokenNum++;
>>>>>>> Stashed changes
    }   // End string iteration

    return gps_data;
}

/*void LC76G_Send_Command(char *data)
{
    HAL_UART_Transmit(&huart3, &data, strlen(data), TIMEOUT);
    HAL_Delay(10);
}
*/
