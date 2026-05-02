/*
 * radar.h
 *
 *  Created on: May 2, 2026
 *      Author: gattusoc
 */

#ifndef INC_RADAR_H_
#define INC_RADAR_H_

typedef struct
{
    float distance; // meters
    float angle_deg; // degrees
    float quality; // accuracy
} RadarDetection;

extern RadarDetection radar_detections[4];

#endif /* INC_RADAR_H_ */
