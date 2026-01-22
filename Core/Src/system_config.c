/**
  ******************************************************************************
  * @file           : system_config.c
  * @author         : Jack Bauer
  * @version        : Pre-production v0.0
  * @date           : Feb 23, 2024
  * @brief          : User-defined STM32 configruation file
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
 **/


#include "system_config.h"
#include "msoe_stm_clock.h"

void system_initialize( void )
{
	// msoe_clk_setup( SYSTEM_CLOCK_FREQUENCY_MHZ );
	// PWR->SVMCR |= PWR_SVMCR_IO2SV_Msk;	// Set the IO2SV bit. Lets PG[15..2] work

	LCD_IO_Init();
	LCD_clear();

}


