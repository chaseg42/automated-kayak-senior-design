/*
 * UI.c
 *
 *  Created on: Mar 1, 2026
 *      Author: gattusoc
 */


#include "UI.h"

// Initialize system as disabled
UIdata ui_state = {
    .mode = 0,
    .direction_to_turn = 0,
    .speed = 0,
	.override_speed45 = 0,
	.override_speed135 = 0,
	.override_speed225 = 0,
	.override_speed315 = 0,
    .new_data_flag = 0,
    .rx_data = { 0 }
};

const char* mode_str[] = {"DISABLE", "MOVE", "ANCHOR", "FOLLOW_SHORE", "MOTOR_OVERRIDE"};
const char* dir_str[] = {"LEFT", "RIGHT", "FORWARD", "REVERSE"};
