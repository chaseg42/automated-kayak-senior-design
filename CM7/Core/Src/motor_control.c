/*
 * motor_control.c
 *
 *  Created on: May 2, 2026
 *      Author: gattusoc
 */

#include "motor_control.h"

#include <math.h>

#include "gps.h"
#include "stm32h7xx_hal.h"
#include "radar.h"
#include "tim.h"

#define MOTOR_PWM_MAX_COUNTS             10000
#define SONAR_OBSTACLE_NEAR_CM           100.0f  // distance threshold for sonar aggressive maneuver
#define RADAR_OBSTACLE_NEAR_M            1.0f   // distance threshold for radar aggressive maneuver
#define RADAR_OBSTACLE_CAUTION_M         3.0f   // distance threshold for radar softer maneuver
#define RADAR_FRONT_CONE_DEG             60.0f  // ignore objects outside this forward cone
#define RADAR_STALE_TIMEOUT_MS           2000U  // ignore radar data older than this
#define MOTOR_TURN_SOFT_DELTA_CMD        40     // change in speed for softer maneuver
#define MOTOR_TURN_HARD_DELTA_CMD        90     // change in speed for aggressive maneuver
#define MOTOR_DEADBAND_MIN_ON_CMD        63U
#define UI_SPEED_MAX_CMD                 100U

#define ANCHOR_HEADING_ON_DEG            15.0f
#define ANCHOR_HEADING_OFF_DEG           5.0f
// Hysteresis: activate correction when > ANCHOR_POSITION_ON_M,
// deactivate when < ANCHOR_POSITION_OFF_M
#define ANCHOR_POSITION_ON_M             2.5f
#define ANCHOR_POSITION_OFF_M            2.0f
// Anchor correction speeds (0-100 units). Do NOT exceed 50 per request.
#define ANCHOR_POSITION_SPEED_MAX_0_100  50U
#define ANCHOR_POSITION_SPEED_MIN_0_100  10U
#define ANCHOR_HEADING_SPEED_0_100       50U

#define FOLLOW_SHORE_TARGET_DEPTH_CM     100.0f
#define FOLLOW_SHORE_DEADBAND_CM         10.0f
#define FOLLOW_SHORE_KP_CMD_PER_CM       0.6f
#define FOLLOW_SHORE_MAX_DELTA_CMD       90U

#define ENABLE_HEADING_CORRECTION        0

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

static void Motor_SetAllSpeeds(uint8_t speed_cmd_0_100, motor_speed *motor_cmd)
{
	uint8_t pwm_cmd = Motor_MapSpeed0_100_to_PWM(speed_cmd_0_100);

	motor_cmd->speed_45 = pwm_cmd;
	motor_cmd->speed_135 = pwm_cmd;
	motor_cmd->speed_225 = pwm_cmd;
	motor_cmd->speed_315 = pwm_cmd;
}

static bool Radar_GetAvoidanceCommand(direction_t *avoid_direction, uint8_t *delta_cmd)
{
	if ((avoid_direction == NULL) || (delta_cmd == NULL))
	{
		return false;
	}

	if (!radar_detections.radar_state)
	{
		return false;
	}

	uint32_t age_ms = HAL_GetTick() - radar_last_update_ms;
	if ((radar_last_update_ms == 0U) || (age_ms > RADAR_STALE_TIMEOUT_MS))
	{
		return false;
	}

	float distance_m = radar_detections.distance;
	float angle_deg = radar_detections.angle_deg;

	if ((distance_m <= 0.0f) || (fabsf(angle_deg) > RADAR_FRONT_CONE_DEG))
	{
		return false;
	}

	if (distance_m <= RADAR_OBSTACLE_NEAR_M)
	{
		*delta_cmd = MOTOR_TURN_HARD_DELTA_CMD;
	}
	else if (distance_m <= RADAR_OBSTACLE_CAUTION_M)
	{
		*delta_cmd = MOTOR_TURN_SOFT_DELTA_CMD;
	}
	else
	{
		return false;
	}

	// Turn away from the obstacle based on angle sign.
	*avoid_direction = (angle_deg >= 0.0f) ? LEFT : RIGHT;
	return true;
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

/**
 * Convert world-frame N/E offsets (meters) into the boat's body frame
 * (forward, right) given current heading in degrees (clockwise from north).
 * body_forward positive => ahead; body_right positive => right side.
 */
static void WorldToBody(float north_m, float east_m, float heading_deg,
						float *body_forward_m, float *body_right_m)
{
	float th = heading_deg * GPS_DEG_TO_RAD;
	float c = cosf(th);
	float s = sinf(th);

	// Rotate by -heading: body = R(-theta) * world
	*body_forward_m = north_m * c + east_m * s;
	*body_right_m = -north_m * s + east_m * c;
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
	state->follow_desired_heading_deg = 0.0f;
	state->follow_heading_correction_active = false;
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
		state->desired_speed_cmd = Motor_MapSpeed0_100_to_PWM(ui->speed);
		state->desired_drive_direction = ui->direction_to_turn;
	}

	if (got_ui_update)
	{
		state->desired_speed_cmd = Motor_MapSpeed0_100_to_PWM(ui->speed);
		state->desired_drive_direction = ui->direction_to_turn;
	}

	if (state->desired_speed_cmd == 0U)
	{
		motor_cmd->speed_45 = 0U;
		motor_cmd->speed_135 = 0U;
		motor_cmd->speed_225 = 0U;
		motor_cmd->speed_315 = 0U;
		if (mode_entry_out != NULL)
		{
			*mode_entry_out = false;
		}
		return;
	}

	if (sonar_data_valid && (sonar->distance <= SONAR_OBSTACLE_NEAR_CM))
	{
		uint8_t speed_cmd = Motor_MapSpeed0_100_to_PWM(UI_SPEED_MAX_CMD);
		if (state->desired_drive_direction == FORWARD)
		{
			motor_cmd->speed_45 = 0U;
			motor_cmd->speed_135 = speed_cmd;
			motor_cmd->speed_225 = speed_cmd;
			motor_cmd->speed_315 = 0U;
		}
		else if (state->desired_drive_direction == REVERSE)
		{
			motor_cmd->speed_45 = speed_cmd;
			motor_cmd->speed_135 = 0U;
			motor_cmd->speed_225 = 0U;
			motor_cmd->speed_315 = speed_cmd;
		}
		if (mode_entry_out != NULL)
		{
			*mode_entry_out = false;
		}
		return;
	}

	if (state->desired_drive_direction == FORWARD)
	{
		motor_cmd->speed_45 = 0U;
		motor_cmd->speed_135 = state->desired_speed_cmd;
		motor_cmd->speed_225 = state->desired_speed_cmd;
		motor_cmd->speed_315 = 0U;
	}
	else if (state->desired_drive_direction == REVERSE)
	{
		motor_cmd->speed_45 = state->desired_speed_cmd;
		motor_cmd->speed_135 = 0U;
		motor_cmd->speed_225 = 0U;
		motor_cmd->speed_315 = state->desired_speed_cmd;
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
		state->anchor_desired_latitude = GPS_Data.world_position_avg.N;
		state->anchor_desired_longitude = GPS_Data.world_position_avg.E;
		state->anchor_desired_heading_deg = (float)GPS_Data.rotation.E;
		state->anchor_heading_correction_active = false;
		state->anchor_position_correction_active = false;
	}

	{
		// Zero outputs initially
		motor_cmd->speed_45 = 0U;
		motor_cmd->speed_135 = 0U;
		motor_cmd->speed_225 = 0U;
		motor_cmd->speed_315 = 0U;

		// Priority: heading correction, then position correction.
		float north_m = 0.0f;
		float east_m = 0.0f;
		GPS_CalculateOffsetMeters(state->anchor_desired_latitude, state->anchor_desired_longitude,
													GPS_Data.world_position_avg.N, GPS_Data.world_position_avg.E,
													&north_m, &east_m);
		float distance_m = sqrtf((north_m * north_m) + (east_m * east_m));
		uint8_t anchor_position_speed_cmd = Anchor_ComputePositionSpeed(distance_m);

		state->anchor_position_correction_active = (distance_m > ANCHOR_POSITION_ON_M);

		// Heading correction (hysteresis)
		float current_heading_deg = (float)GPS_Data.rotation.E;
		float heading_error_deg = GPS_NormalizeHeadingError(current_heading_deg - state->anchor_desired_heading_deg);
		uint8_t anchor_heading_speed_cmd = Motor_MapSpeed0_100_to_PWM(ANCHOR_HEADING_SPEED_0_100);

		if (!state->anchor_heading_correction_active && (fabsf(heading_error_deg) > ANCHOR_HEADING_ON_DEG))
		{
			state->anchor_heading_correction_active = true;
		}
		else if (state->anchor_heading_correction_active && (fabsf(heading_error_deg) < ANCHOR_HEADING_OFF_DEG))
		{
			state->anchor_heading_correction_active = false;
		}

		if (state->anchor_heading_correction_active)
		{
			// Rotate in place to regain heading (use mapped speeds)
			if (heading_error_deg > 0.0f)
			{
				// rotate counter-clockwise -> motors 45 + 225
				motor_cmd->speed_45 = anchor_heading_speed_cmd;
				motor_cmd->speed_225 = anchor_heading_speed_cmd;
			}
			else if (heading_error_deg < 0.0f)
			{
				// rotate clockwise -> motors 135 + 315
				motor_cmd->speed_135 = anchor_heading_speed_cmd;
				motor_cmd->speed_315 = anchor_heading_speed_cmd;
			}
		}

		// Position correction only when heading is stable
		if (state->anchor_position_correction_active && !state->anchor_heading_correction_active)
		{
			// Convert world offsets into body frame so position correction aims at world coordinates
			float body_forward_m = 0.0f;
			float body_right_m = 0.0f;
			WorldToBody(-north_m, -east_m, (float)GPS_Data.rotation.E, &body_forward_m, &body_right_m);

			// Lateral correction (body right/left) first
			if (fabsf(body_right_m) > ANCHOR_POSITION_OFF_M)
			{
				if (body_right_m > 0.0f)
				{
					// Need to move right in body frame -> use motors 225 + 315
					motor_cmd->speed_225 = anchor_position_speed_cmd;
					motor_cmd->speed_315 = anchor_position_speed_cmd;
				}
				else
				{
					// Need to move left in body frame -> use motors 45 + 135
					motor_cmd->speed_45 = anchor_position_speed_cmd;
					motor_cmd->speed_135 = anchor_position_speed_cmd;
				}
			}
			else if (fabsf(body_forward_m) > ANCHOR_POSITION_OFF_M)
			{
				if (body_forward_m > 0.0f)
				{
					// Need to move forward in body frame -> use motors 135 + 225
					motor_cmd->speed_135 = anchor_position_speed_cmd;
					motor_cmd->speed_225 = anchor_position_speed_cmd;
				}
				else
				{
					// Need to move backward in body frame -> use motors 45 + 315
					motor_cmd->speed_45 = anchor_position_speed_cmd;
					motor_cmd->speed_315 = anchor_position_speed_cmd;
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
		state->follow_desired_heading_deg = (float)GPS_Data.rotation.E;
		state->follow_heading_correction_active = false;
	}

	if (got_ui_update)
	{
		state->desired_speed_cmd = Motor_MapSpeed0_100_to_PWM(ui->speed);
		state->desired_shore_side = ui->direction_to_turn;
	}

	if (state->desired_speed_cmd == 0U)
	{
		motor_cmd->speed_45 = 0U;
		motor_cmd->speed_135 = 0U;
		motor_cmd->speed_225 = 0U;
		motor_cmd->speed_315 = 0U;
		if (mode_entry_out != NULL)
		{
			*mode_entry_out = false;
		}
		return;
	}

	bool depth_valid = sonar_data_valid && (sonar->distance > 0.0f);
	float depth_error_cm = 0.0f;

	if (depth_valid)
	{
		depth_error_cm = FOLLOW_SHORE_TARGET_DEPTH_CM - sonar->distance;
	}

	// Radar avoidance takes priority over shoreline depth control.
	direction_t radar_avoid_dir = RIGHT;
	uint8_t radar_delta_cmd = 0U;
	if (Radar_GetAvoidanceCommand(&radar_avoid_dir, &radar_delta_cmd))
	{
		Motor_SetAllSpeeds(ui->speed, motor_cmd);
		state->follow_heading_correction_active = false;
	}
	else if (depth_valid && (fabsf(depth_error_cm) > FOLLOW_SHORE_DEADBAND_CM))
	{
		// Depth control: steer toward/away from shore side to maintain target depth.
		direction_t correction_direction = state->desired_shore_side;
		if (depth_error_cm > 0.0f)
		{
			correction_direction = (state->desired_shore_side == RIGHT) ? LEFT : RIGHT;
		}

		float delta_f = fabsf(depth_error_cm) * FOLLOW_SHORE_KP_CMD_PER_CM;
		uint8_t delta_cmd = (uint8_t)delta_f;
		if (delta_cmd > FOLLOW_SHORE_MAX_DELTA_CMD)
		{
			delta_cmd = FOLLOW_SHORE_MAX_DELTA_CMD;
		}
		if (delta_cmd < MOTOR_TURN_SOFT_DELTA_CMD)
		{
			delta_cmd = MOTOR_TURN_SOFT_DELTA_CMD;
		}

		Motor_SetAllSpeeds(ui->speed, motor_cmd);
		state->follow_heading_correction_active = false;
	}
	else
	{
		// Hold heading using GPS when depth is stable or invalid.
#if ENABLE_HEADING_CORRECTION
		float current_heading_deg = (float)GPS_Data.rotation.E;
		float heading_error_deg = GPS_NormalizeHeadingError(current_heading_deg - state->follow_desired_heading_deg);

		if (!state->follow_heading_correction_active && (fabsf(heading_error_deg) > ANCHOR_HEADING_ON_DEG))
		{
			state->follow_heading_correction_active = true;
		}
		else if (state->follow_heading_correction_active && (fabsf(heading_error_deg) < ANCHOR_HEADING_OFF_DEG))
		{
			state->follow_heading_correction_active = false;
		}

		if (state->follow_heading_correction_active)
		{
			direction_t correction_direction = (heading_error_deg > 0.0f) ? RIGHT : LEFT;
			Motor_SetAllSpeeds(ui->speed, motor_cmd);
		}
		else
		{
			Motor_SetAllSpeeds(ui->speed, motor_cmd);
		}
#else
		Motor_SetAllSpeeds(ui->speed, motor_cmd);
		state->follow_heading_correction_active = false;
#endif
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


