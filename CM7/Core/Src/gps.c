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



#include "gps.h"
#include <string.h>

#define GPS_POSITION_AVG_WINDOW 8U

static double gps_lat_history[GPS_POSITION_AVG_WINDOW];
static double gps_lon_history[GPS_POSITION_AVG_WINDOW];
static uint8_t gps_history_index = 0U;
static uint8_t gps_history_count = 0U;

static void GPS_UpdatePositionAverage(double latitude, double longitude,
                    double *avg_latitude, double *avg_longitude)
{
  double lat_sum = 0.0;
  double lon_sum = 0.0;
  uint8_t i = 0U;

  gps_lat_history[gps_history_index] = latitude;
  gps_lon_history[gps_history_index] = longitude;

  gps_history_index++;
  if (gps_history_index >= GPS_POSITION_AVG_WINDOW)
  {
    gps_history_index = 0U;
  }

  if (gps_history_count < GPS_POSITION_AVG_WINDOW)
  {
    gps_history_count++;
  }

  for (i = 0U; i < gps_history_count; i++)
  {
    lat_sum += gps_lat_history[i];
    lon_sum += gps_lon_history[i];
  }

  if (gps_history_count > 0U)
  {
    *avg_latitude = lat_sum / (double)gps_history_count;
    *avg_longitude = lon_sum / (double)gps_history_count;
  }
  else
  {
    *avg_latitude = latitude;
    *avg_longitude = longitude;
  }
}

GPSParsedDataStruct GPS_Parsed_Data;
GPSDataStruct GPS_Data;

byte UART4_rxBuffer[GPS_RX_BUFFER_SIZE] __attribute__((aligned(UART4_DMA_CACHE_LINE_SIZE))) = {0};
bool b_rx_transfer_complete, b_tx_transfer_complete = false;
uint8_t UART6_txBuffer[ESP32_GPS_TX_LEN] __attribute__((aligned(UART4_DMA_CACHE_LINE_SIZE))) = {0};
volatile bool usart6_tx_complete = true;

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
  GPS_UpdatePositionAverage(gds->world_position.N, gds->world_position.E,
                  &gds->world_position_avg.N, &gds->world_position_avg.E);
  gds->world_position_avg.D = gds->world_position.D;
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


void GPS_PopulateESP32Buffer(GPSDataStruct *gps, uint8_t buf[85])
{
    uint8_t *p = buf;

    // Start byte for framing
    *p++ = 0xAA;

    // world_position
    memcpy(p, &gps->world_position.N, 8); p += 8;
    memcpy(p, &gps->world_position.E, 8); p += 8;
    memcpy(p, &gps->world_position.D, 8); p += 8;

    // velocity
    memcpy(p, &gps->velocity.N, 8); p += 8;
    memcpy(p, &gps->velocity.E, 8); p += 8;
    memcpy(p, &gps->velocity.D, 8); p += 8;

    // rotation (only yaw = .E is used but send all for consistency)
    memcpy(p, &gps->rotation.N, 8); p += 8;
    memcpy(p, &gps->rotation.E, 8); p += 8;
    memcpy(p, &gps->rotation.D, 8); p += 8;

    // utc_date
    memcpy(p, &gps->utc_date.year, 2);  p += 2;
    *p++ = gps->utc_date.month;
    *p++ = gps->utc_date.day;

    // utc_time
    *p++ = gps->utc_time.hour;
    *p++ = gps->utc_time.min;
    *p++ = gps->utc_time.sec;

    // device_id
    memcpy(p, gps->device_id, 5); p += 5;
}