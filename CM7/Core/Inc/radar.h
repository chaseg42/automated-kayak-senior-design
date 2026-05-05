/*
 * radar.h
 *
 *  Created on: May 2, 2026
 *      Author: gattusoc
 */

#ifndef INC_RADAR_H_
#define INC_RADAR_H_

#include <stdbool.h>
#include <stdint.h>
#include "usbd_def.h"

typedef struct
{
    float distance; // meters
    float angle_deg; // degrees
    float quality; // accuracy
    bool radar_state; // ON = TRUE, OFF = FALSE
} RadarData;

extern bool radar_task_update;
extern uint32_t radar_last_update_ms;

// #define RADAR_ID 0x67

extern RadarData radar_detections;
void usb_radar_rx(uint8_t *buf);
void usb_radar_tx_state();

#endif /* INC_RADAR_H_ */
