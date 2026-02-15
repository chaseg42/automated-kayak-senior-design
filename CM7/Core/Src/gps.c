/**
  ******************************************************************************
  * @file           : gps.c
  * @author         : Jack Bauer
  * @version        : 
  * @date           : Feb 6, 2026
  * @brief          : 
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
**/


#include "common.h"
#include "gps.h"
#include <string.h>

void decode_nav(GPSParsedDataStruct *gpds, GPSDataStruct *gds)
{

	// Decode Position
	double pos[3] = { (double)gpds->latitude * 1e-7, (double)gpds->longitude * 1e-7, (double)gpds->height * 1e-3 };

	// Decode Velocity
	double vel[3] = { (double)gpds->velN * 1e-3, (double)gpds->velE * 1e-3, (double)gpds->velD * 1e-3 };

	// Decode Rotation
	double rot[3] = { (double)gpds->pitch * 1e-5, (double)gpds->yaw * 1e-5, (double)gpds->roll * 1e-5 };

	// Decode UTC Date
	word date[3] = { gpds->month, gpds->day, gpds->year };

	// Decode UTC Time
	byte time[3] = { gpds->hour, gpds->min, gpds->sec };

	gds->world_position.N = pos[0];
	gds->world_position.E = pos[1];
	gds->world_position.D = pos[2];
	gds->velocity.N = vel[0];
	gds->velocity.E = vel[1];
	gds->velocity.D = vel[2];
	gds->rotation.N = rot[0];
	gds->rotation.E = rot[1];
	gds->rotation.D = rot[2];
	gds->utc_date.month = date[0];
	gds->utc_date.day = date[1];
	gds->utc_date.year = date[2];
	gds->utc_time.hour = time[0];
	gds->utc_time.min = time[1];
	gds->utc_time.sec = time[2];

	return;
}

void decode_sec(GPSParsedDataStruct *gpds, GPSDataStruct *gds)
{
	memcpy(gds->device_id, gpds->device_id, sizeof(gds->device_id));
	return;
}
