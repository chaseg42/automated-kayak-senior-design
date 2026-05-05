/*
 * radar.c
 *
 *  Created on: May 2, 2026
 *      Author: gattusoc
 * 
 *      Modified: 05/03/2026
 *      Author: Jack Bauer
 *      
 */

 #include "radar.h"
#include "usbd_cdc.h"

 RadarData radar_detections;
 bool radar_task_update;
 uint32_t radar_last_update_ms;


 void usb_radar_rx(USBD_HandleTypeDef *pdev)
 {
	  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	  if(hcdc == NULL) { while(1); }

	  uint8_t *buf = hcdc->RxBuffer;

	int r = sscanf(buf, "%f,%f,%f", &radar_detections.distance, &radar_detections.angle_deg, &radar_detections.quality);

	if (r < 1)
	{
		radar_task_update = false;
		return;
	}

	radar_last_update_ms = HAL_GetTick();

 }

 void usb_radar_tx_state()
 {

    uint8_t buffer[2];

    // Construct the message
	int tx_size = snprintf(buffer, sizeof(buffer), "0x67%x", radar_task_update);

	// Transmit the constructed message
	CDC_Transmit_FS((char *)buffer, tx_size);

	// Clear the buffer
	clear_buffer((byte *)buffer, sizeof(buffer));

 }

