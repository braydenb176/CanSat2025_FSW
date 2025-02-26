/*
 * commands.c
 *
 *  Created on: Feb 25, 2025
 *      Author: bbevel0133
 */


#include "commands.h"

static uint8_t sim_enabled = 0;

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

static CMD_STATUS verify_team_id(const char* incoming, const uint16_t* team_id){
	char team_ID_arr[TEAM_ID_SIZE];
	int team_ID_int;

	//"CMD,1000," copy substr starting from index 4, retrieve only team ID
	strncpy(team_ID_arr, incoming + TEAM_ID_SIZE-1, TEAM_ID_SIZE-1);
	team_ID_arr[TEAM_ID_SIZE-1] = '\0'; //make sure to add the null char to end the char array

	//convert char array into int, if this entered then an integer was not parsed, id is invalid
	if(sscanf(team_ID_arr, "%d", &team_ID_int) != 1){
		return CMD_ID_INVLD;
	}

	if(team_ID_int != *team_id){
		return CMD_ID_WRONG;
	}

	return CMD_OK;
}

static uint8_t verify_time_format(const char* time){


	return 1;
}

static uint8_t verify_pressure_data(const char* data, uint32_t* pressure){
	//make sure every char is a digit, then set pressure to that value
	if(*data == '\0')
		return 0;

	const char* ptr = data;

	while(*ptr){
		if(!isdigit((uint8_t) *data)){
			return 0;
		}
		ptr++;
	}

	sscanf(data, "%u", (unsigned int*) pressure);

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

	uint32_t sim_pressure;
	if(!verify_pressure_data(simp, &sim_pressure)){
		return CMD_SIMP_INVLD;
	}

	mission_data->PRESSURE = (float) (sim_pressure / 1000);
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
}

CMD_STATUS process_command(char* incoming, Mission_Data* mission_data){

	CMD_STATUS cmd_status = verify_team_id(incoming, (uint16_t*) &mission_data->TEAM_ID);

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





