/**
  ******************************************************************************
  * @file           : system_config.h
  * @author         : Jack Bauer
  * @version        : Pre-production v0.0
  * @date           : Feb 5, 2024
  * @brief          : 
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
 **/

#ifndef INC_SYSTEM_CONFIG_H_
#define INC_SYSTEM_CONFIG_H_

#include <stdint.h>
#include <assert.h>
#include "stm32u575xx.h" // Why am I needing to include this
#include "msoe_stm_lcd.h"

#define SYSTEM_CLOCK_FREQUENCY_MHZ 160

// Function Prototypes
void system_initialize( void );


#endif /* INC_SYSTEM_CONFIG_H_ */
