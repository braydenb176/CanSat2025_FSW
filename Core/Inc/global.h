/*
 * global.h
 *
 *  Created on: Feb 25, 2025
 *      Author: bbevel0133
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#include "stm32g4xx_hal.h"
#include "string.h"

#define STATE_TEXT_LEN 14 //13 max, plus 1 for null char
#define CMD_ECHO_LEN 10

//flags

extern volatile uint8_t transmit_enable;

//structs

typedef struct {
	int16_t  TEAM_ID;
	uint32_t MISSION_TIME; //store as seconds since midnight, decode later
	uint32_t PACKET_COUNT;
	char     MODE;
	char     STATE[STATE_TEXT_LEN];

	float    ALTITUDE;
	float    TEMPERATURE;
	float    PRESSURE;
	float 	 VOLTAGE; //might change to int, can multiply by 10 and store as int

	float    GYRO_R;
	float    GYRO_P;
	float    GYRO_Y;

	float    ACCEL_R;
	float    ACCEL_P;
	float    ACCEL_Y;

	float    MAG_R;
	float    MAG_P;
	float    MAG_Y;

	int16_t  AUTO_GYRO_ROTATION_RATE;

	uint32_t GPS_TIME;
	float    GPS_ALTITUDE;
	float    GPS_LATITUDE;
	float    GPS_LONGITUDE;
	uint8_t  GPS_SATS;

	char     CMD_ECHO[CMD_ECHO_LEN];

	//might add more as needed (optional)
} Mission_Data;

extern Mission_Data global_mission_data;

void init_mission_data(void);

#endif /* INC_GLOBAL_H_ */
