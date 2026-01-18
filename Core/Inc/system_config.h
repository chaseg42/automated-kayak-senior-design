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
  */

#ifndef INC_SYSTEM_CONFIG_H_
#define INC_SYSTEM_CONFIG_H_

#include <stdint.h>
#include <assert.h>
#include "stm32u575xx.h" // Why am I needing to include this

#define SYSTEM_CLOCK_FREQUENCY_MHZ 160

// ENUMERATIONS //
enum gpio_mode
{
/*	For use in setting MODE bit in MODER Register assignment
   	____________________________________________________
 			MODE			|		REGISTER BIT VALUE
 	________________________|___________________________ */
			INPUT 			= 			0B00,
			OUTPUT 			= 			0B01,
			ALTERNATE 		= 			0B10,
			ANALOG 			= 			0B11

}typedef GPIO_Mode_Enum;

enum gpio_type
{
/*	For use in setting GPIO pin bit trait (for example: what type of output for the pin to have).
	________________________________________________________
				TYPE			|		CASE VALUE
	____________________________|___________________________ */
			SET_MODE_REGISTER	=			0,

			SET_ODR_LOW			=			1,
			SET_ODR_HIGH		=			2,

			SET_IDR_PULL_UP		=			3,
			SET_IDR_PULL_DOWN 	=			4,

			SET_IRQ_FALLING_EDGE=			5,
			SET_IRQ_RISING_EDGE	=			6

}typedef GPIO_Type_Enum;


enum gpio_interupt_port
{
/* Ports have explicit interrupt values. Passed in as a shifted value to define what port
 * The interrupt should look for
	____________________________________________________
		GPIO PORT			|		REGISTER BIT VALUE
	________________________|___________________________ */
			A 				= 			0x00,
			B 				=			0x01,
			C 				= 			0x02,
			D 				= 			0x03,
			E 				= 			0x04,
			F 				= 			0x05,
			G 				=			0X06,
			H 				= 			0x07

}typedef GPIO_InteruptPort_Enum;


enum timer_mode
{
	/* How the timer should count and reset
		____________________________________________________________________
					TIMER MODE				|		REGISTER BIT VALUE
		____________________________________|_______________________________ */
			UP_COUNT 						= 			0B00,
			DOWN_COUNT 						= 			0B01,
			CENTER_ALLIGNED_UP 				= 			0B10,
			CENTER_ALLIGNED_DOWN 			= 			0B10,
			CENTER_ALLIGNED_BOTH 			= 			0B11

}typedef TIM_Mode_Enum;

enum
{
	/* PWM Mode 1 is a normal counter (initializes high)
	 * PWN Mode 2 is a counter that is inverse to PWM mode 1 (initializes low)
		____________________________________________________
			PWN MODE			|		REGISTER BIT VALUE
		________________________|___________________________ */
			PWM_1 				= 			0B110,
			PWM_2				= 			0B111,
}typedef PWM_Mode_Enum;


enum motor_state
{

	OFF		   =    0,
	ON	   	   = 	1,
	STEP_CW    =    2,
	STEP_CCW   = 	3

}typedef MOTOR_State_Enum;

// Timer struct
struct timer
{
	TIM_TypeDef *ptimer;
	uint8_t channel;
	TIM_Mode_Enum timer_mode;
	PWM_Mode_Enum pwm_mode;
	float frequency_kHz;
	float scale;
	float duty_cycle;
	uint8_t IRQ_Enable;

}typedef TIM_Struct;


// Function Prototypes
void system_initialize( void );
void initialize_ports( void );
void initialize_timers( void );
int button_read( GPIO_TypeDef *port, int pin );
void gpio_assign( GPIO_TypeDef *port, int pin, GPIO_Type_Enum TYPE, GPIO_Mode_Enum MODE );
void gpio_enable_irq( GPIO_InteruptPort_Enum PORT_VALUE, int pin, IRQn_Type IRQn, GPIO_Type_Enum TYPE, uint32_t priority );
void timer_enable( TIM_Struct *timer );
void timer_update( TIM_TypeDef *timer, uint8_t channel, float frequency_kHz, float scale, float duty_cycle );

// Unused
//void timer_enable( TIM_TypeDef *timer, Timer_Mode TIMER_MODE, PWM_Mode PWM_MODE, float frequency_kHz, float scale, float duty_cycle, int IRQ_enable );
//void ADC_enable( ADC_TypeDef *adc, int enable_interupt ); // Only use ADC4 for now


#endif /* INC_SYSTEM_CONFIG_H_ */
