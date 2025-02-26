/*
 * global.c
 *
 *  Created on: Feb 25, 2025
 *      Author: bbevel0133
 */

#include "global.h"

volatile uint8_t transmit_enable = 0;
volatile uint8_t simulation_enable = 0;

Mission_Data global_mission_data = {0};

void init_mission_data(){
	memset(&global_mission_data, 0, sizeof(global_mission_data));

	global_mission_data.TEAM_ID = 3174;
	global_mission_data.MODE = 'F';
	strcpy(global_mission_data.STATE, "LAUNCH_PAD");
}


