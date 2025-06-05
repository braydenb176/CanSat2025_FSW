#include "MS5607SPI.h"
#include <math.h>

// Index is by seconds ago the value was calculated
float altitude_history[] = {0, 0, 0};

float max_altitude = 0.0;
float apogee_base_ratio = 0.75; 
float apogee_difference_ratio = 0.00;
float const alt_offset_height = 20.00;

float calibrated_atitude = 0.00;

/*
Units
Pressure - mbars - double
Temperature - Celcius - double
*/

// https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
// The altitude equation is for absolute altitude.
// calibraing : 1 = True, 0 = False

float caluclateAltitude(double pressure, int calibrating){
    float h_meter = 0.3048*((1 - pow((pressure/1013.25), 0.190284))*145366.54);
    if (calibrating == 1){
        // Absolute Altitude of the ground station
        calibrated_atitude = h_meter;
        // Relative Altitude of GCS
        return 0;
    }
    else{
        // Relative Altitude
        altitude_history[3] = altitude_history[2];
        altitude_history[2] = altitude_history[1];
        altitude_history[0] = h_meter - calibrated_atitude;
        find_apogee();
        return altitude_history[0];
    }
}

void find_apogee(){
    if(altitude_history[2] > altitude_history[1] && altitude_history[2] > altitude_history[0]){
        if (max_altitude == 0){
            max_altitude = altitude_history[2];
            apogee_difference_ratio = alt_offset_height / max_altitude;
        }
        else if(max_altitude * (apogee_base_ratio + apogee_difference_ratio) > altitude_history[0]){
            // DO ACTUATOR STUFF
        }
    }
}
