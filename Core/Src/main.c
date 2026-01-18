/**
  ******************************************************************************
  * @file           : main.c
  * @author         : Jack Bauer
  * @version        : Pre-production v0.0
  * @date           : 1/18/2026
  * @brief          : Main program development file for the purposes of testing systems by Jack
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ubx.h"

int main(void)
{

  // Test print for ubx message frames
  // (Step through the debugger)
  initialize_ubx_messages(ubx_tx_poll_ack, &UBXFrame);
  initialize_ubx_messages(ubx_tx_poll_id, &UBXFrame);
  initialize_ubx_messages(ubx_tx_poll_pvt, &UBXFrame);
  initialize_ubx_messages(ubx_tx_poll_test, &UBXFrame);
  
  // 
  while (1)
  {
  }
}
