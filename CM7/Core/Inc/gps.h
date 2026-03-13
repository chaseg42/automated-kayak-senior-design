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

#include "main.h"

// This struct will hold the data the is parsed from UBX commands
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
	uint32_t sAcc;		// Speed accuracy estimate in mm/s
	uint32_t headAcc;	// Heading accuracy estimate in deg and scaled down to 1e-5
	word pDOP;			// Position DOP
	// Reserved -- 6 bytes //
	int headVeh;		// Heading of vehicle (2-D) in deg
	short magDec;		// Magnetic declination in deg and scaled down to 1e-2
	word magAcc;		// Magnetic declination accuracy in deg and scaled down to 1e-2

	// HNR Solution
	// Mostly same information
	int speed;			// Only used in HNR solution


	// ATT Solution
	byte version;
	int roll;
	int pitch;
	int yaw;
	uint32_t accRoll;
	uint32_t accPitch;
	uint32_t accYaw;

}typedef GPSParsedDataStruct;


// Structs for system integrations
struct
{
	double N;
	double E;
	double D;
}typedef NEDVector3; // Needs refactoring due to rotation addition


struct
{
	word year;
	byte month;
	byte day;
}typedef UTCDate;

struct
{
	byte hour;
	byte min;
	byte sec;
}typedef UTCTime;


struct
{
	NEDVector3 world_position;
	NEDVector3 velocity;
	NEDVector3 rotation;
	UTCDate utc_date;
	UTCTime utc_time;
	byte device_id[5];
}typedef GPSDataStruct;

extern GPSParsedDataStruct GPS_Parsed_Data;
extern GPSDataStruct GPS_Data;

extern byte UART4_rxBuffer[];
extern bool b_rx_transfer_complete;
extern bool b_tx_transfer_complete;

void decode_nav(GPSParsedDataStruct *gpds, GPSDataStruct *gds);
void decode_sec(GPSParsedDataStruct *gpds, GPSDataStruct *gds);

#endif /* INC_GPS_H_ */
