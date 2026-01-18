/**
  ******************************************************************************
  * @file           : system_config.c
  * @author         : Jack Bauer
  * @version        : Pre-production v0.0
  * @date           : Feb 23, 2024
  * @brief          : Main program file for the purposes of developing the NE0-M8U GPS software driver
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */


#include "system_config.h"
#include "msoe_stm_clock.h"

void system_initialize( void )
{
	msoe_clk_setup( SYSTEM_CLOCK_FREQUENCY_MHZ );
	PWR->SVMCR |= PWR_SVMCR_IO2SV_Msk;	// Set the IO2SV bit. Lets PG[15..2] work
	// initialize_ports();
}


void initialize_timers()
{
	assert("ASSERT: Function Not implemented");
}

void initialize_ports()
{
	// RCC->AHB2ENR1 |= 	0B1000110;  	// Enable GPIO ports clocks
	assert("ASSERT: Function Not implemented");
}


// Nice function for port manipulation
void gpio_assign( GPIO_TypeDef *port, int pin, GPIO_Type_Enum TYPE, GPIO_Mode_Enum MODE )
{
	switch( TYPE )
	{

		// SET MODE REGISTER
		case SET_MODE_REGISTER :
			port->MODER &= ~(0B11 << 2 * pin); // Clear 2 bits at the pin address
			port->MODER |=  (MODE << 2 * pin); // Set pin MODE (passed as enum)
			break;

		// SET OUTPUT DATA REGISTER LOW
		case SET_ODR_LOW :
			port->ODR &= ~(1 << pin);
			break;

		// SET OUTPUT DATA REGISTER HIGH
		case SET_ODR_HIGH :
			port->ODR |= (1 << pin);
			break;

		// SET INPUT DATA REGISTER w/ PULL-UP RESISTOR
		case SET_IDR_PULL_UP :
			port->MODER &= ~(0B11 << 2 * pin); // Clear 2 bits at the pin address
			port->PUPDR |=  (0B01 << 2 * pin); // Instantiate pull-up resistor at input pin
			break;

		// SET INPUT DATA REGISTER w/ PULL-UP RESISTOR
		case SET_IDR_PULL_DOWN :
			port->MODER &= ~(0B11 << 2 * pin); // Clear 2 bits at the pin address
			port->PUPDR |=  (0B10 << 2 * pin); // Instantiate pull-down resistor at input pin
			break;

		default :
			return;
	}
}


// --------------------------------------------------------	//
// @brief
// 			Enable GPIO Interrupts, pass in 0 for default priority.
// 			Refer to stm32u575xx.h for IRQn enum value
// --------------------------------------------------------	//
void gpio_enable_irq( GPIO_InteruptPort_Enum PORT_VALUE, int pin, IRQn_Type IRQn, GPIO_Type_Enum TYPE, uint32_t priority )
{
	switch( TYPE )
	{

		// SET GPIO INTERUPT, Check for falling edge
		case SET_IRQ_FALLING_EDGE :
			EXTI->FTSR1 |= (1 << pin);
			EXTI->EXTICR[(pin / 4)] |= (PORT_VALUE << ((pin % 4) * 8)); 	// EXTICR uses 32 bits per interrupt index. The EXTICR has 4 indexed channels.
			EXTI->IMR1 |= (1 << pin); 			// CPU Mask					// It supports GPIO pins 0 - 16, with each pin having 8 bit locations in its respective channel.
																			// Ex. pin 3 = 3/4 = index 0 -> or'ed with port value (from ref. manuel) -> shifted into bit location 3 * 8 = 24 bit start location
			NVIC_EnableIRQ(IRQn); 				// Enable Interrupt
			NVIC_SetPriority(IRQn, priority); 	// Set priority of interrupt (default 0 == **most important** )
			break;

		// SET GPIO INTERUPT, Check for rising edge
		case SET_IRQ_RISING_EDGE :
			EXTI->RTSR1 |= (1 << pin);
			EXTI->EXTICR[(pin / 4)] |= (PORT_VALUE << ((pin % 4) * 8)); 	// EXTICR uses 32 bits per interrupt channel. The EXTICR has 4 of these indexed channels
			EXTI->IMR1 |= (1 << pin); 			// CPU Mask					// and supports GPIO pins 0 - 16, with each pin having 8 bit locations in its respective channel.
																			// Ex: pin 3 = 3/4 = index 0 or'ed with port value (from ref. manuel) shifted into bit location 3 * 8 = 24 bit start location
			NVIC_EnableIRQ(IRQn); 				// Enable Interrupt.
			NVIC_SetPriority(IRQn, priority); 	// Set priority of interrupt (default 0 == **most important** )
			break;

		default :
			return;
	}
}


int button_read( GPIO_TypeDef *port, int pin )
{
	return !(port->IDR & (1 << pin));
}


// ONLY ENABLE INTERUPT ON TIM4 FOR NOW!! Everything else is configurable
void timer_enable( TIM_Struct *timer )
{
	float freq = timer->frequency_kHz * 10000;

	/* Calculating ARR

	 	 T = ( Pre_scale / F ) * ( ARR + 1 )
	 	 Timer = ( Pre_scale / ( Input Frequency * Scaler ) ) * ( ARR + 1 )
	 	 ARR = (Timer / Pre_scale) * (Input Frequency) - 1
	 	 ARR = ( 1 / Frequency_wanted * Pre_Scale )) * (SYSTEM_CLOCK_FREQUENCY) - 1

	*/

	timer->ptimer->CR1 |= ( 1 << 7 ); 	// Buffer the ARR and set timer count type
	timer->ptimer->ARR  = ((( 1 / (freq * timer->scale ))) * (SYSTEM_CLOCK_FREQUENCY_MHZ * 1000000) - 1); 	// See formula above


	switch(timer->channel)
	{
		case 1:
			timer->ptimer->CCER |= ( 1 << 4*(timer->channel - 1) );				// Enable CCR1
			timer->ptimer->CCMR1 |= ( timer->pwm_mode << 4 ) | ( 1 << 3 );		// Set PWM_MODE | Pre-load CCR1
			timer->ptimer->CCR1 = ( timer->ptimer->ARR ) * timer->duty_cycle;	// CCR1 Duty Cycle
			break;

		case 2:
			timer->ptimer->CCER |= ( 1 << 4*(timer->channel - 1) );				// Enable CCR2
			timer->ptimer->CCMR1 |= ( timer->pwm_mode << 12 ) | ( 1 << 11 );		// Set PWM_MODE | Pre-load CCR2
			timer->ptimer->CCR2 = ( timer->ptimer->ARR ) * timer->duty_cycle;	// CCR2 Duty Cycle
			break;

		case 3:
			timer->ptimer->CCER |= ( 1 << 4*(timer->channel - 1) );				// Enable CCR3
			timer->ptimer->CCMR2 |= ( timer->pwm_mode << 4 ) | ( 1 << 3 );		// Set PWM_MODE | Pre-load CCR3
			timer->ptimer->CCR3 = ( timer->ptimer->ARR ) * timer->duty_cycle;	// CCR3 Duty Cycle
			break;

		case 4:
			timer->ptimer->CCER |= ( 1 << 4*(timer->channel - 1) );				// Enable CCR4
			timer->ptimer->CCMR2 |= ( timer->pwm_mode << 12 ) | ( 1 << 11 );		// Set PWM_MODE | Pre-load CCR4
			timer->ptimer->CCR4 = ( timer->ptimer->ARR ) * timer->duty_cycle;	// CCR4 Duty Cycle
			break;

		default :
			break;
	}

	// Enable Interrupt on Timer 4 Update event *Add more interupt events here if desired*
	if(timer->IRQ_Enable)
	{
		timer->ptimer->DIER |= 0B1;
	}

	timer->ptimer->EGR |= 1;	// Trigger timer update
	timer->ptimer->CR1 |= 1;	// Enable Timer
}


// Make sure to update on correct timer channel
// TODO: This function incorrectly assumes the timer is configured to generate PWM
void timer_update( TIM_TypeDef *timer, uint8_t channel, float frequency_kHz, float scale, float duty_cycle )
{
	float freq = frequency_kHz * 10000;

	timer->ARR = ((( 1 / (freq * scale ))) * ( SYSTEM_CLOCK_FREQUENCY_MHZ * 1000000 ) - 1); // Refer to timer_enable for formula

	switch(channel)
	{
		case 1:
			timer->CCR1 = ( timer->ARR ) * duty_cycle;		// CCR1 Duty Cycle
			break;

		case 2:
			timer->CCR2 = ( timer->ARR ) * duty_cycle;		// CCR2 Duty Cycle
			break;

		case 3:
			timer->CCR3 = ( timer->ARR ) * duty_cycle;		// CCR3 Duty Cycle
			break;

		case 4:
			timer->CCR4 = ( timer->ARR ) * duty_cycle;		// CCR4 Duty Cycle
			break;

		default :
			break;
	}

}


