/*
 * UI.h
 *
 *  Created on: Mar 1, 2026
 *      Author: gattusoc
 */

 #include "main.h"

#ifndef INC_UI_H_
#define INC_UI_H_

typedef enum {
  MODE_DISABLE = 0, // DISABLE was already declared elsewhere so add MODE_
  MODE_MOVE = 1,
  MODE_ANCHOR = 2,
  MODE_FOLLOW_SHORE = 3,
  MOTOR_OVERRIDE = 4
} operatingMode_t;

typedef enum {
  LEFT = 0,
  RIGHT = 1,
  FORWARD = 2,
  REVERSE = 3
} direction_t;

typedef struct {
  operatingMode_t mode;
  direction_t direction_to_turn;
  uint8_t speed;
  uint8_t override_speed45;
  uint8_t override_speed135;
  uint8_t override_speed225;
  uint8_t override_speed315;
  bool new_data_flag; // TODO turn into a notification?
  uint8_t rx_data[7];
} UIdata;

extern UIdata ui_state;
extern const char* mode_str[];
extern const char* dir_str[];

#endif /* INC_UI_H_ */
