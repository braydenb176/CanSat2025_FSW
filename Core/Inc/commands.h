/*
 * commands.h
 *
 *  Created on: Feb 25, 2025
 *      Author: bbevel0133
 */

#ifndef INC_COMMANDS_H_
#define INC_COMMANDS_H_

#include "stm32g4xx_hal.h"
#include "global.h"
#include "string.h"
#include "stdio.h"
#include "ctype.h"

//macro, no function call overhead (kinda over the top but its still an optimization)
#define UTC_TO_INT(time_str) ( \
    ((time_str)[0] - '0') * 10 * 3600 + ((time_str)[1] - '0') * 3600 + \
    ((time_str)[3] - '0') * 10 * 60 + ((time_str)[4] - '0') * 60 + \
    ((time_str)[6] - '0') * 10 + ((time_str)[7] - '0') \
)

#define CMD_HEADER_SIZE 9

//these include the null char, ie TEAM_ID_SIZE is 4 chars plus 1 for null char
#define MAX_CMD_SIZE 5
#define MAX_CX_SIZE 4
#define MAX_ST_SIZE 9
#define MAX_SIM_SIZE 9
#define MAX_SIMP_SIZE 7
#define MAX_MEC_SIZE 9 //defined by team, for now restrict naming convention to 8 chars plus null char, should really be one or no more than 3

typedef enum {
	CMD_OK,
	CMD_INVLD,
	CMD_HEADER_INVLD,
	CMD_CX_ON,
	CMD_CX_OFF,
	CMD_CX_INVLD,
	CMD_ST_GPS,
	CMD_ST_UTC,
	CMD_ST_INVLD,
	CMD_SIM_E,
	CMD_SIM_S,
	CMD_SIM_NE,
	CMD_SIM_F,
	CMD_SIM_INVLD,
	CMD_SIMP_RX,
	CMD_SIMP_INVLD,
	CMD_CAL_RX,
	CMD_CAL_INVLD,
	CMD_MEC_WIRE,
	CMD_MEC_INVLD
} CMD_STATUS;

CMD_STATUS process_command(char* incoming, Mission_Data* mission_data);
void run_command_test_cases(Mission_Data* mission_data);

#endif /* INC_COMMANDS_H_ */
