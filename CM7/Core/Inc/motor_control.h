/*
 * motor_control.h
 *
 *  Created on: May 2, 2026
 *      Author: gattusoc
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "UI.h"
#include "sonar.h"

typedef struct
{
	uint8_t desired_speed_cmd;
	direction_t desired_drive_direction;
	direction_t desired_shore_side;
	direction_t last_drive_direction;
	float move_desired_heading_deg;
	bool move_heading_correction_active;
	float follow_desired_heading_deg;
	bool follow_heading_correction_active;
	double anchor_desired_latitude;
	double anchor_desired_longitude;
	float anchor_desired_heading_deg;
	bool anchor_heading_correction_active;
	bool anchor_position_correction_active;
} MotorControlState;

typedef struct
{
	uint8_t speed_45;
	uint8_t speed_135;
	uint8_t speed_225;
	uint8_t speed_315;
} motor_speed;

void MotorControl_InitState(MotorControlState *state);

void MotorControl_ModeMove(MotorControlState *state, bool mode_entry, bool got_ui_update,
													 const UIdata *ui, bool sonar_data_valid, const Sonar_t *sonar,
											 motor_speed *motor_cmd,
													 bool *mode_entry_out);

void MotorControl_ModeAnchor(MotorControlState *state, bool mode_entry,
									 motor_speed *motor_cmd,
														 bool *mode_entry_out);

void MotorControl_ModeFollowShore(MotorControlState *state, bool mode_entry, bool got_ui_update,
																	const UIdata *ui, bool sonar_data_valid, const Sonar_t *sonar,
												motor_speed *motor_cmd,
																	bool *mode_entry_out);

void MotorControl_ModeOverride(const UIdata *ui,
									 motor_speed *motor_cmd);

void MotorControl_SetOutputs(const motor_speed *motor_cmd);


#endif /* INC_MOTOR_CONTROL_H_ */
