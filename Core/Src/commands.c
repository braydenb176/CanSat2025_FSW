/*
 * commands.c
 *
 *  Created on: Feb 25, 2025
 *      Author: bbevel0133
 */


#include "commands.h"

static uint8_t sim_enabled = 0;

//static CMD_STATUS trim_xbee_header(const char* incoming, const char* trimmed){
//
//
//	return CMD_OK;
//}

static void get_current_token(const uint8_t token_size, char* token, char** cmd_ptr, Mission_Data* mission_data){
	uint8_t char_cnt = 0;
	for(uint8_t i = 0; i < token_size; i++){
		token[i] = **cmd_ptr;
		(*cmd_ptr)++;
		if(token[i] == ',' || token[i] == '\0'){
			token[i] = '\0';
			break;
		}else{
			char_cnt++;
		}
	}

	//need to make sure cmd_echo is init before this
	strncat(mission_data->CMD_ECHO, token, char_cnt);
}

static uint8_t verify_time_format(const char* time){


	return 1;
}

static CMD_STATUS verify_cmd_header(const char* incoming){
	if(strncmp(incoming, "CMD,3174,", CMD_HEADER_SIZE) != 0){
		return CMD_HEADER_INVLD;
	}

	return CMD_OK;
}

static uint8_t verify_pressure_data(const char* data, float* pressure){
	//make sure every char is a digit, then set pressure to that value
	if(*data == '\0' || data == NULL){
		return 0;
	}

	const char* ptr = data;
	uint8_t decimal_cnt = 0;
	uint32_t before_point = 0;
	uint32_t after_point = 0;

	float decimal_mult = 1.0f;

	while(*ptr){
		if(*ptr == '.'){
			if(decimal_cnt++ == 1){
				return 0; //too many decimal points, return false for unsuccessful
			}
		}else if(!isdigit((uint8_t) *ptr)){
			return 0;
		}else{
			if(decimal_cnt == 1){ //after the decimal
				before_point = before_point * 10 + (*ptr - '0');
			}else{ //before decimal, int
				after_point = after_point * 10 + (*ptr - '0');
				decimal_mult *= 10;
			}
		}
		ptr++;
	}

	*pressure = (float) (before_point + (after_point / decimal_mult));

	return 1;
}

static CMD_STATUS perform_CX(const char* incoming, char* cmd_ptr, Mission_Data* mission_data){
	char on_off[MAX_CX_SIZE];
	get_current_token(MAX_CX_SIZE, on_off, &cmd_ptr, mission_data);

	if(!strcmp(on_off, "ON")){
	    //maybe have global flag? could also do the freertos events/mutexes

		return CMD_CX_ON;
	}else if(!strcmp(on_off, "OFF")){

		return CMD_CX_OFF;
	}else{
	    //invalid, do nothing, or set error code somehow?
		return CMD_CX_INVLD;
	}
}

static CMD_STATUS perform_ST(const char* incoming, char* cmd_ptr, Mission_Data* mission_data){
	char time[MAX_ST_SIZE];
	get_current_token(MAX_ST_SIZE, time, &cmd_ptr, mission_data);

	if(!strcmp(time, "GPS")){
		mission_data->MISSION_TIME = mission_data->GPS_TIME;
		return CMD_ST_GPS;
	}else{
	  //first verify if in right format (not null pointer, colons in right place, all digits)
	  //then use macro, or helper func another file kinda needs to be made for that (including verifying above)
	  if(verify_time_format(time)){
		  mission_data->MISSION_TIME = (uint32_t) UTC_TO_INT(time);
		  return CMD_ST_UTC;
	  }else{
		  return CMD_ST_INVLD;
	  }
	}
}

static CMD_STATUS perform_SIM(const char* incoming, char* cmd_ptr, Mission_Data* mission_data){
	char sim[MAX_SIM_SIZE];
	get_current_token(MAX_SIM_SIZE, sim, &cmd_ptr, mission_data);

	if(!strcmp(sim, "ENABLE")){
		sim_enabled = 1;
		return CMD_SIM_E;
	}else if(!strcmp(sim, "ACTIVATE")){
		if(sim_enabled){
			mission_data->MODE = 'S';
			return CMD_SIM_S;
		}
		return CMD_SIM_NE;
	}else if(!strcmp(sim, "DISABLE")){
		sim_enabled = 0;
		mission_data->MODE = 'F';
		return CMD_SIM_F;
	}else{
		return CMD_SIM_INVLD;
	}
}

static CMD_STATUS perform_SIMP(const char* incoming, char* cmd_ptr, Mission_Data* mission_data){
	char simp[MAX_SIMP_SIZE];
	get_current_token(MAX_SIMP_SIZE, simp, &cmd_ptr, mission_data);

	if(!verify_pressure_data(simp, &mission_data->PRESSURE)){
		return CMD_SIMP_INVLD;
	}
	//mission_data->PRESSURE = (float) (sim_pressure / 1000);
	return CMD_SIMP_RX;
}

static CMD_STATUS perform_CAL(Mission_Data* mission_data){
	mission_data->ALTITUDE = 0;
	return CMD_CAL_RX;
}

static CMD_STATUS perform_MEC(const char* incoming, char* cmd_ptr, Mission_Data* mission_data){
	char mec[MAX_MEC_SIZE];
	get_current_token(MAX_MEC_SIZE, mec, &cmd_ptr, mission_data);

	if(!strcmp(mec, "MEC1")){

	}else if(!strcmp(mec, "MEC2")){

	}else if(!strcmp(mec, "MEC3")){

	}else{
		return CMD_MEC_INVLD; //add the rest of status codes as more mechanisms added
	}

	return 1; //change later, just to get rid of warning
}

CMD_STATUS process_command(char* incoming, Mission_Data* mission_data){
	CMD_STATUS cmd_status;
	//if in debug mode, ignore xbee header trimming. or just comment out i guess
//		char trimmed_cmd_str[32];
//		memset(trimmed_cmd_str, 0);
//		cmd_status = trim_xbee_header(incoming, trimmed_cmd_str);
//
//		if(cmd_status != CMD_OK){
//			return cmd_status;
//		}

	//make sure the beginning is CMD,3174
	cmd_status = verify_cmd_header(incoming);

	if(cmd_status != CMD_OK){
		//uart print error/status?
		return cmd_status;
	}

	char cmd[MAX_CMD_SIZE];
	char* cmd_ptr = incoming + CMD_HEADER_SIZE; //magic numbers, define later todo

	get_current_token(MAX_CMD_SIZE, cmd, &cmd_ptr, mission_data);

	if(!strcmp(cmd, "CX")){ //Payload Telemetry On/Off Command
		cmd_status = perform_CX(incoming, cmd_ptr, mission_data);
	}else if(!strcmp(cmd, "ST")){ //Set Time
		cmd_status = perform_ST(incoming, cmd_ptr, mission_data);
	}else if(!strcmp(cmd, "SIM")){ //Simulation Mode Control Command
		cmd_status = perform_SIM(incoming, cmd_ptr, mission_data);
	}else if(!strcmp(cmd, "SIMP")){ //Simulated Pressure Data (to be used in Simulation Mode only)
		cmd_status = perform_SIMP(incoming, cmd_ptr, mission_data);
	}else if(!strcmp(cmd, "CAL")){ //Calibrate Altitude to Zero
		cmd_status = perform_CAL(mission_data);
	}else if(!strcmp(cmd, "MEC")){ //Mechanism actuation command
		cmd_status = perform_MEC(incoming, cmd_ptr, mission_data);
	}else{
		return CMD_INVLD;
	}

	return cmd_status;
}

//TEST METHODS BELOW

static void test_cmd_header(Mission_Data* mission_data){
	CMD_STATUS cmd_status;
	char test[64];

	//Correct header
	strcpy(test, "CMD,3174,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect beginning of string, 1 char
	strcpy(test, "C,3174,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect beginning of string, 2 chars
	strcpy(test, "CM,3174,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect beginning of string, 3 chars
	strcpy(test, "CDM,3174,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect, No delimiter
	strcpy(test, "CDM3174,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect, No team ID
	strcpy(test, "CMD,,CX,ON");
	cmd_status = verify_cmd_header(test);

	//Incorrect, no rest of message, only header
	strcpy(test, "CMD,3174");
	cmd_status = verify_cmd_header(test);

	cmd_status = cmd_status; //get rid of annoying not used warning
}

void run_command_test_cases(Mission_Data* mission_data){
	test_cmd_header(mission_data);
	//add more as needed, test each command
}





