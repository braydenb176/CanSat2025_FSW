#include <stdint.h>
#include "ICM42688PSPI.h"

uint16_t gyro_old_x = 0;
uint16_t gyro_old_y = 0;
uint16_t gyro_old_z = 0;
uint32_t old_time = 0; // Might need to reinitialized to actual start.

void Transfer_Data(uint16_t x, uint16_t y, uint16_t z, uint32_t time){
    gyro_old_x = x;
    gyro_old_y = y;
    gyro_old_z = z;
    old_time = time;
}

uint16_t Get_Accel_X(uint16_t gyro_x, uint32_t time){
    return (gyro_old_x - gyro_x) / (old_time - time);
}

uint16_t Get_Accel_Y(uint16_t gyro_y, uint32_t time){
    return (gyro_old_y - gyro_y) / (old_time - time);
}

uint16_t Get_Accel_Z(uint16_t gyro_z, uint32_t time){
    return (gyro_old_z - gyro_z) / (old_time - time);
}