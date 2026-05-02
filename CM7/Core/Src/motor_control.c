/*
 * motor_control.c
 *
 *  Created on: May 2, 2026
 *      Author: gattusoc
 */

#include "motor_control.h"

#include <math.h>

#include "gps.h"
#include "tim.h"

#define MOTOR_PWM_MAX_COUNTS             10000
#define SONAR_OBSTACLE_NEAR_CM           10.0f  // distance threshold for sonar aggressive maneuver
#define SONAR_OBSTACLE_CAUTION_CM        100.0f // distance threshold for sonar softer maneuver
#define MOTOR_TURN_SOFT_DELTA_CMD        40     // change in speed for softer maneuver
#define MOTOR_TURN_HARD_DELTA_CMD        90     // change in speed for aggressive maneuver
#define MOTOR_DEADBAND_MIN_ON_CMD        63U
#define UI_SPEED_MAX_CMD                 100U

#define ANCHOR_HEADING_ON_DEG            15.0f
#define ANCHOR_HEADING_OFF_DEG           5.0f
#define ANCHOR_POSITION_ON_M             2.0f
#define ANCHOR_POSITION_OFF_M            0.5f
#define ANCHOR_POSITION_SPEED_MAX_0_100  80U
#define ANCHOR_POSITION_SPEED_MIN_0_100  20U

#define GPS_METERS_PER_DEG_LAT           111111.0f
#define GPS_DEG_TO_RAD                   0.01745329252f

static uint8_t Motor_ClampSpeedCmd(int32_t speed_cmd)
{
	if (speed_cmd < 0)
	{
		return 0U;
	}
	// Enforce deadband so any nonzero command spins the motor.
	if ((speed_cmd > 0) && (speed_cmd < MOTOR_DEADBAND_MIN_ON_CMD))
	{
		return MOTOR_DEADBAND_MIN_ON_CMD;
	}
	if (speed_cmd > 255)
	{
		return 255U;
	}
	return (uint8_t)speed_cmd;
}

static uint8_t Motor_MapSpeed0_100_to_PWM(uint8_t speed_cmd_0_100)
{
	// Linear map: 0->0, 1->deadband min, 100->255.
	if (speed_cmd_0_100 == 0U)
	{
		return 0U;
	}
	if (speed_cmd_0_100 >= UI_SPEED_MAX_CMD)
	{
		return 255U;
	}

	uint32_t scaled = (uint32_t)(speed_cmd_0_100 - 1U) * (255U - MOTOR_DEADBAND_MIN_ON_CMD);
	uint32_t pwm = MOTOR_DEADBAND_MIN_ON_CMD + (scaled / (UI_SPEED_MAX_CMD - 1U));
	if (pwm > 255U)
	{
		pwm = 255U;
	}
	return (uint8_t)pwm;
}

static void Motor_ComputeQuadSpeed(uint8_t base_speed_cmd, direction_t turn_direction, uint8_t turn_delta_cmd,
										 motor_speed *motor_cmd)
{
	// Forward uses 135/225; reverse uses 45/315 with optional yaw bias.
	int32_t motor_45 = 0;
	int32_t motor_135 = 0;
	int32_t motor_225 = 0;
	int32_t motor_315 = 0;

	if (turn_direction == REVERSE)
	{
		motor_45 = base_speed_cmd;
		motor_315 = base_speed_cmd;
	}
	else
	{
		motor_135 = base_speed_cmd;
		motor_225 = base_speed_cmd;

		if (turn_direction == RIGHT)
		{
			motor_135 -= turn_delta_cmd;
			motor_225 += turn_delta_cmd;
		}
		else if (turn_direction == LEFT)
		{
			motor_135 += turn_delta_cmd;
			motor_225 -= turn_delta_cmd;
		}
	}

	motor_cmd->speed_45 = Motor_ClampSpeedCmd(motor_45);
	motor_cmd->speed_135 = Motor_ClampSpeedCmd(motor_135);
	motor_cmd->speed_225 = Motor_ClampSpeedCmd(motor_225);
	motor_cmd->speed_315 = Motor_ClampSpeedCmd(motor_315);
}

static float GPS_NormalizeHeadingError(float heading_error_deg)
{
	// Wrap to [-180, 180] for consistent yaw error handling.
	while (heading_error_deg > 180.0f)
	{
		heading_error_deg -= 360.0f;
	}
	while (heading_error_deg < -180.0f)
	{
		heading_error_deg += 360.0f;
	}
	return heading_error_deg;
}

static void GPS_CalculateOffsetMeters(double desired_lat, double desired_lon,
																			double current_lat, double current_lon,
																			float *north_m, float *east_m)
{
	// Flat-earth approximation for local offsets.
	float lat_rad = (float)desired_lat * GPS_DEG_TO_RAD;
	float meters_per_deg_lon = GPS_METERS_PER_DEG_LAT * cosf(lat_rad);

	*north_m = (float)((current_lat - desired_lat) * GPS_METERS_PER_DEG_LAT);
	*east_m = (float)((current_lon - desired_lon) * meters_per_deg_lon);
}

static uint8_t Anchor_ComputePositionSpeed(float distance_m)
{
	float distance_span = ANCHOR_POSITION_ON_M - ANCHOR_POSITION_OFF_M;
	float ratio = 1.0f;
	float speed_range = (float)(ANCHOR_POSITION_SPEED_MAX_0_100 - ANCHOR_POSITION_SPEED_MIN_0_100);

	if (distance_span > 0.0f)
	{
		ratio = (distance_m - ANCHOR_POSITION_OFF_M) / distance_span;
	}
	if (ratio < 0.0f)
	{
		ratio = 0.0f;
	}
	else if (ratio > 1.0f)
	{
		ratio = 1.0f;
	}

	uint8_t speed_0_100 = (uint8_t)(ANCHOR_POSITION_SPEED_MIN_0_100 + (ratio * speed_range));
	return Motor_MapSpeed0_100_to_PWM(speed_0_100);
}

void MotorControl_InitState(MotorControlState *state)
{
	if (state == NULL)
	{
		return;
	}

	state->desired_speed_cmd = 0U;
	state->desired_drive_direction = FORWARD;
	state->desired_shore_side = RIGHT;
	state->last_drive_direction = FORWARD;
	state->move_desired_heading_deg = 0.0f;
	state->move_heading_correction_active = false;
	state->anchor_desired_latitude = 0.0;
	state->anchor_desired_longitude = 0.0;
	state->anchor_desired_heading_deg = 0.0f;
	state->anchor_heading_correction_active = false;
	state->anchor_position_correction_active = false;
}

void MotorControl_ModeMove(MotorControlState *state, bool mode_entry, bool got_ui_update,
													 const UIdata *ui, bool sonar_data_valid, const Sonar_t *sonar,
											 motor_speed *motor_cmd,
													 bool *mode_entry_out)
{
	if ((state == NULL) || (ui == NULL) || (sonar == NULL) || (motor_cmd == NULL))
	{
		return;
	}

	if (mode_entry)
	{
		// Capture initial heading so straight-line travel can be stabilized.
		state->desired_speed_cmd = Motor_MapSpeed0_100_to_PWM(ui->speed);
		state->desired_drive_direction = ui->direction_to_turn;
		state->move_desired_heading_deg = (float)GPS_Data.rotation.E;
		state->move_heading_correction_active = false;
		state->last_drive_direction = state->desired_drive_direction;
	}

	if (got_ui_update)
	{
		// Refresh drive intent and reset heading lock on straight direction changes.
		state->desired_speed_cmd = Motor_MapSpeed0_100_to_PWM(ui->speed);
		state->desired_drive_direction = ui->direction_to_turn;
		if (state->desired_drive_direction != state->last_drive_direction)
		{
			if ((state->desired_drive_direction == FORWARD) || (state->desired_drive_direction == REVERSE))
			{
				state->move_desired_heading_deg = (float)GPS_Data.rotation.E;
				state->move_heading_correction_active = false;
			}
			state->last_drive_direction = state->desired_drive_direction;
		}
	}

	Motor_ComputeQuadSpeed(state->desired_speed_cmd, state->desired_drive_direction, 0, motor_cmd);

	// Sonar avoidance overrides heading stabilization in forward travel.
	if ((state->desired_drive_direction != REVERSE) && sonar_data_valid && (sonar->distance <= SONAR_OBSTACLE_NEAR_CM))
	{
		Motor_ComputeQuadSpeed(state->desired_speed_cmd, RIGHT, MOTOR_TURN_HARD_DELTA_CMD, motor_cmd);
	}
	else if ((state->desired_drive_direction != REVERSE) && sonar_data_valid && (sonar->distance <= SONAR_OBSTACLE_CAUTION_CM))
	{
		Motor_ComputeQuadSpeed(state->desired_speed_cmd, RIGHT, MOTOR_TURN_SOFT_DELTA_CMD, motor_cmd);
	}
	else
	{
		// GPS heading correction when traveling straight forward or reverse.
		if ((state->desired_drive_direction == FORWARD) || (state->desired_drive_direction == REVERSE))
		{
			float current_heading_deg = (float)GPS_Data.rotation.E;
			float heading_error_deg = GPS_NormalizeHeadingError(current_heading_deg - state->move_desired_heading_deg);

			if (!state->move_heading_correction_active && (fabsf(heading_error_deg) > ANCHOR_HEADING_ON_DEG))
			{
				state->move_heading_correction_active = true;
			}
			else if (state->move_heading_correction_active && (fabsf(heading_error_deg) < ANCHOR_HEADING_OFF_DEG))
			{
				state->move_heading_correction_active = false;
			}

			if (state->move_heading_correction_active)
			{
				direction_t correction_direction = (heading_error_deg > 0.0f) ? RIGHT : LEFT;
				if (state->desired_drive_direction == FORWARD)
				{
					// Forward: bias 135/225 pair to correct yaw.
					Motor_ComputeQuadSpeed(state->desired_speed_cmd, correction_direction, MOTOR_TURN_SOFT_DELTA_CMD, motor_cmd);
				}
				else
				{
					// Reverse: bias 45/315 pair to correct yaw.
					int32_t motor_45 = motor_cmd->speed_45;
					int32_t motor_315 = motor_cmd->speed_315;

					if (correction_direction == RIGHT)
					{
						motor_45 += MOTOR_TURN_SOFT_DELTA_CMD;
						motor_315 -= MOTOR_TURN_SOFT_DELTA_CMD;
					}
					else
					{
						motor_45 -= MOTOR_TURN_SOFT_DELTA_CMD;
						motor_315 += MOTOR_TURN_SOFT_DELTA_CMD;
					}

					motor_cmd->speed_45 = Motor_ClampSpeedCmd(motor_45);
					motor_cmd->speed_315 = Motor_ClampSpeedCmd(motor_315);
				}
			}
		}
	}

	if (mode_entry_out != NULL)
	{
		*mode_entry_out = false;
	}
}

void MotorControl_ModeAnchor(MotorControlState *state, bool mode_entry,
									 motor_speed *motor_cmd,
														 bool *mode_entry_out)
{
	if ((state == NULL) || (motor_cmd == NULL))
	{
		return;
	}

	if (mode_entry)
	{
		// Snapshot anchor reference on entry.
		state->anchor_desired_latitude = GPS_Data.world_position.N;
		state->anchor_desired_longitude = GPS_Data.world_position.E;
		state->anchor_desired_heading_deg = (float)GPS_Data.rotation.E;
		state->anchor_heading_correction_active = false;
		state->anchor_position_correction_active = false;
	}

	{
		// Priority: heading correction, then position correction.
		float current_heading_deg = (float)GPS_Data.rotation.E;
		float heading_error_deg = GPS_NormalizeHeadingError(current_heading_deg - state->anchor_desired_heading_deg);
		float north_m = 0.0f;
		float east_m = 0.0f;
		GPS_CalculateOffsetMeters(state->anchor_desired_latitude, state->anchor_desired_longitude,
															GPS_Data.world_position.N, GPS_Data.world_position.E,
															&north_m, &east_m);
		float distance_m = sqrtf((north_m * north_m) + (east_m * east_m));
		uint8_t anchor_heading_speed_cmd = Motor_MapSpeed0_100_to_PWM(ANCHOR_POSITION_SPEED_MAX_0_100);
		uint8_t anchor_position_speed_cmd = Anchor_ComputePositionSpeed(distance_m);

		if (!state->anchor_heading_correction_active && (fabsf(heading_error_deg) > ANCHOR_HEADING_ON_DEG))
		{
			state->anchor_heading_correction_active = true;
		}
		else if (state->anchor_heading_correction_active && (fabsf(heading_error_deg) < ANCHOR_HEADING_OFF_DEG))
		{
			state->anchor_heading_correction_active = false;
		}

		if (!state->anchor_position_correction_active && (distance_m > ANCHOR_POSITION_ON_M))
		{
			state->anchor_position_correction_active = true;
		}
		else if (state->anchor_position_correction_active && (distance_m < ANCHOR_POSITION_OFF_M))
		{
			state->anchor_position_correction_active = false;
		}

		if (state->anchor_heading_correction_active)
		{
			// Rotate in place to regain heading.
			if (heading_error_deg > 0.0f)
			{
				motor_cmd->speed_45 = anchor_heading_speed_cmd;
				motor_cmd->speed_225 = anchor_heading_speed_cmd;
			}
			else if (heading_error_deg < 0.0f)
			{
				motor_cmd->speed_135 = anchor_heading_speed_cmd;
				motor_cmd->speed_315 = anchor_heading_speed_cmd;
			}
		}
		else if (state->anchor_position_correction_active)
		{
			// Translate in two steps: lateral then forward/back.
			if (fabsf(east_m) > ANCHOR_POSITION_OFF_M)
			{
				if (east_m > 0.0f)
				{
					motor_cmd->speed_225 = anchor_position_speed_cmd;
					motor_cmd->speed_315 = anchor_position_speed_cmd;
				}
				else
				{
					motor_cmd->speed_45 = anchor_position_speed_cmd;
					motor_cmd->speed_135 = anchor_position_speed_cmd;
				}
			}
			else if (fabsf(north_m) > ANCHOR_POSITION_OFF_M)
			{
				if (north_m > 0.0f)
				{
					motor_cmd->speed_45 = anchor_position_speed_cmd;
					motor_cmd->speed_315 = anchor_position_speed_cmd;
				}
				else
				{
					motor_cmd->speed_135 = anchor_position_speed_cmd;
					motor_cmd->speed_225 = anchor_position_speed_cmd;
				}
			}
		}
	}

	if (mode_entry_out != NULL)
	{
		*mode_entry_out = false;
	}
}

void MotorControl_ModeFollowShore(MotorControlState *state, bool mode_entry, bool got_ui_update,
																	const UIdata *ui, bool sonar_data_valid, const Sonar_t *sonar,
												motor_speed *motor_cmd,
																	bool *mode_entry_out)
{
	if ((state == NULL) || (ui == NULL) || (sonar == NULL) || (motor_cmd == NULL))
	{
		return;
	}

	if (mode_entry)
	{
		// Initialize shoreline tracking intent from UI.
		state->desired_speed_cmd = Motor_MapSpeed0_100_to_PWM(ui->speed);
		state->desired_shore_side = ui->direction_to_turn;
	}

	if (got_ui_update)
	{
		state->desired_speed_cmd = Motor_MapSpeed0_100_to_PWM(ui->speed);
		state->desired_shore_side = ui->direction_to_turn;
	}

	if (sonar_data_valid && (sonar->distance <= SONAR_OBSTACLE_NEAR_CM))
	{
		// Bias away from shore-side obstacle.
		direction_t avoid_direction = (state->desired_shore_side == RIGHT) ? LEFT : RIGHT;
		Motor_ComputeQuadSpeed(state->desired_speed_cmd, avoid_direction, MOTOR_TURN_HARD_DELTA_CMD, motor_cmd);
	}
	else if (sonar_data_valid && (sonar->distance <= SONAR_OBSTACLE_CAUTION_CM))
	{
		direction_t avoid_direction = (state->desired_shore_side == RIGHT) ? LEFT : RIGHT;
		Motor_ComputeQuadSpeed(state->desired_speed_cmd, avoid_direction, MOTOR_TURN_SOFT_DELTA_CMD, motor_cmd);
	}
	else
	{
		// Maintain desired speed with a small shoreline bias.
		Motor_ComputeQuadSpeed(state->desired_speed_cmd, state->desired_shore_side, 20, motor_cmd);
	}

	if (mode_entry_out != NULL)
	{
		*mode_entry_out = false;
	}
}

void MotorControl_ModeOverride(const UIdata *ui,
									 motor_speed *motor_cmd)
{
	if ((ui == NULL) || (motor_cmd == NULL))
	{
		return;
	}

	motor_cmd->speed_45 = Motor_MapSpeed0_100_to_PWM(ui->override_speed45);
	motor_cmd->speed_135 = Motor_MapSpeed0_100_to_PWM(ui->override_speed135);
	motor_cmd->speed_225 = Motor_MapSpeed0_100_to_PWM(ui->override_speed225);
	motor_cmd->speed_315 = Motor_MapSpeed0_100_to_PWM(ui->override_speed315);
}

void MotorControl_SetOutputs(const motor_speed *motor_cmd)
{
	if (motor_cmd == NULL)
	{
		return;
	}

	// Convert 0-255 commands to timer counts.
	uint32_t motor_45_arr = (MOTOR_PWM_MAX_COUNTS * motor_cmd->speed_45) / 255;
	uint32_t motor_135_arr = (MOTOR_PWM_MAX_COUNTS * motor_cmd->speed_135) / 255;
	uint32_t motor_225_arr = (MOTOR_PWM_MAX_COUNTS * motor_cmd->speed_225) / 255;
	uint32_t motor_315_arr = (MOTOR_PWM_MAX_COUNTS * motor_cmd->speed_315) / 255;

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motor_45_arr);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor_135_arr);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, motor_225_arr);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, motor_315_arr);
}


