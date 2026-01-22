/**
  ******************************************************************************
  * @file           : gps.h
  * @author         : Jack Bauer
  * @version        : Pre-production v0.0
  * @date           : Jan 21, 2026
  * @brief          : This header defines the struct where all data from the NEO-M8U will be stored
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
**/

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "common.h"

struct
{
	// ID Solution
	byte device_id[5];

	// PVT Solution
	uint32_t iTOW;		// GPS time
	word year;			// UTC
	byte month;			// UTC
	byte day;			// UTC
	byte hour;			// UTC
	byte min;			// UTC
	byte sec;			// UTC
	byte valid;			// validMag | fullyResolved | validTime | validDate
	uint32_t tAcc;		// UTC Time accuracy estimate in ns
	int nano;			// UTC fraction of the second in ns
	byte fixType;		// See Section 32.17.14.1 in the NEO-M8U receiver description
	byte flags;			// ^
	byte flags2;		// ^
	byte numSV;			// Number of satellites used in solution
	int longitude;		// in degrees and scaled down to 1e-7
	int latitude;		// in degrees and scaled down to 1e-7
	int height;			// Height above ellipsoid in mm
	int hMSL;			// Height above mean sea level in mm
	uint32_t hAcc;		// Horizontal accuracy estimate in mm/s
	uint32_t vAcc;		// Vertical accuracy estimate in mm/s
	int velN;			// NED north velocity in mm/s
	int velE;			// NED east velocity in mm/s
	int velD;			// NED down velocity in mm/s
	int gSpeed;			// Ground Speed (2-D) in mm/s
	int headMot;		// Heading of motion (2-D) in deg and scaled down to 1e-5
	uint32_t sAcc;		// Speed accuracy estiamte in mm/s
	uint32_t headAcc;	// Heading accuracy estimate in deg and scaled down to 1e-5
	word pDOP;			// Position DOP
	// Reserved -- 6 bytes //
	int headVeh;		// Heading of vehicle (2-D) in deg
	short magDec;		// Magnetic declination in deg and scaled down to 1e-2
	word magAcc;		// Magnetic declination accuracy in deg and scaled down to 1e-2
}typedef GPS_Data_Struct;


extern GPS_Data_Struct GPS_Data;


#endif /* INC_GPS_H_ */
