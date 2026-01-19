/**
  ******************************************************************************
  * @file           : main.c
  * @author         : Jack Bauer
  * @version        : Pre-production v0.0
  * @date           : Jan 18, 2026
  * @brief          : Main program file for the purposes of developing the NE0-M8U GPS software driver
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ubx.h"
#include "system_config.h"

int main(void)
{
  // Initialize the STM32 MCU
  system_initialize();

  // Test for ubx message frames
  // (Step through the debugger)
  UBXStatus status;
  UBXFrame_Typedef UBXFrame = FRAME_PREAMBLE;

  // Ignore the syntax errors, they are limitations of the CDT parser.
  status = initialize_ubx_frame(&UBXFrame, (byte *)ubx_tx_poll_ack, NULL);
  status = initialize_ubx_frame(&UBXFrame, (byte *)ubx_tx_poll_id, NULL);
  status = initialize_ubx_frame(&UBXFrame, (byte *)ubx_tx_poll_pvt, NULL);
  status = initialize_ubx_frame(&UBXFrame, (byte *)ubx_tx_poll_test, NULL);

  // Test for custom ubx message frames
  // (Step through the debugger)
  byte value;
  byte *payload = &value;
  *payload = (byte)0xA1010A;

  // *payload = (byte)0xA1;
  word payload_size = sizeof(payload);
  status = initialize_ubx_frame(&UBXFrame, LOG, 0x01, payload_size, payload, 0x01, 0x02);

  if(status == UBX_OK) { }

  while (1)
  {
  }
}
