/**
 * File name: main.c
 *
 * Created on: 10/15/2025
 * Author: Chase Gattuso
 * 
 * Code to use for motor controller test.
 * Outputs a 20 KHz PWM signal with duty cycle controllable
 * through analog input.
 *
 * Necessary Connections: 
 * 	PD11 - ADC4_IN15: Analog input to control duty cycle
 *  PA5 - AF1 TIM2_CH1: PWM output
 *	All LCD connections
 */

#include "stm32u5xx.h"
#include "main.h"

// Output PWM at at 20 kHz assuming 160 MHz clock
#define T2_PSC 0
#define T2_ARR 8000

// Read ADC at 1 kHz assuming 160 MHz clock
#define T3_PSC 159
#define T3_ARR 1000

#define ADC_RES_MAX 255 // Highest result possible from ADC at current resolution (8 bits)

// Function Declarations
void clock_init(void);
void LCD_init(void);
void GPIO_init(void);
void adc4_init(void);
void tim2_init(void);
void tim3_init(void);

// Global variables for ISR
bool ADC_newval_flag = false;
uint8_t ADC_value = 0;


int main(void)
{
    // Setup
	msoe_clk_setup(160); // uncomment if higher speed desired
	clock_init();
	LCD_init();
    GPIO_init();
    adc4_init();
    tim2_init();
    tim3_init();
    
    float duty_cycle = 0.0; // Duty cycle of PWM output as a float

	for (;;)
	{
        // Calculate PWM output on new ADC value
        if (ADC_newval_flag == true) {
            duty_cycle = (float)(ADC_RES_MAX - ADC_value) / ADC_RES_MAX; // Calculate inverted duty cycle
            TIM2->CCR1 = T2_ARR * duty_cycle; // Set new duty cycle
            LCD_print_udec3(1, 0, ADC_value); // print ADC value to LCD
            LCD_print_udec3(4, 0, (uint8_t)(duty_cycle*100)); // Convert float to percentage before printing
        }
	}
}


/**
 * Function name: clock_init
 * Purpose: Setup clock on STM32U575 for GPIO and timers
 */
void clock_init(void) {
    //RCC->AHB3ENR |= RCC_AHB3ENR_PWREN;  // enable clock for PWR module
	RCC->AHB2ENR1 |= 0x000000FF; // enable GPIOa - h clocks
	PWR->SVMCR |= (1 << 29); // set IO2SV bit to allow GPIOG use
    RCC->APB1ENR1 |= (1 << 0); // enable TIM2 clock
    RCC->APB1ENR1 |= (1 << 1); // enable TIM3 clock
}

/**
 * Function name: LCD_init
 * Purpose: Setup, clear LCD, and print unchanging strings
 */
void LCD_init(void) {
	LCD_IO_Init();
	LCD_clear();
    
    // Print unchanging LCD strings
    LCD_print_str(0,0,"ADC Raw Value: ");
    LCD_print_str(3,0,"PWM Duty Cycle: ");
    LCD_print_str(4,3,"%");
}

/**
 * Function name: GPIO_init
 * Purpose: Setup GPIO pins to be used in program
 *  PB8 - AF1 TIM_CH1: PWM output 
 */
void GPIO_init(void) {
    // PA5 PWM GPIO setup
    GPIOA->MODER &= ~(0b01 << 10); // set PA5 to alternate function
	GPIOA->AFR[0] |= (0b0001 << 20); // select alternate function 1 for TIM2_CH1
}

/**
 * Function name: adc4_init
 * Purpose: initialize ADC4 for use in reading input voltage
 * Settings:
 *  Channel: 15
 *  Conversion mode: single
 *  Sample time: 39.5 clocks
 *  Resolution: 8 bit
 *  Interrupt: Using ADC4_IRQn
 */
void adc4_init(void) {
    // Clocks
    RCC->AHB3ENR |= (1 << 5); // enable ADC4 bus clock
    RCC->CCIPR3 |= (0b101 << 12); // setup MISK as kernel clock source

    // Power setup
	PWR->SVMCR |= (1 << 30);  // set ASV bit to remove Vdda power isolation
	// turn on ADC4 voltage regulator
	ADC4->ISR |= (1 << 12);
	ADC4->CR |= (1 << 28);
	while(!(ADC4->ISR & (1 << 12))); // Wait for LDORDY to be reset

	// Perform calibration
	ADC4->CR |= (1 << 31);  // initiate calibration
	while(ADC4->CR & (1 << 31)); // Wait until complete

    // Initialization
	ADC4->SMPR1 |= (0b101 << 0);  // sample time 39.5 clocks
	ADC4->CHSELR |= (1 << 15);  // enable channel 15
	ADC4->CFGR1 |= (0b10 << 2);  // set 8-bit resolution
    //ADC4->CFGR1 |= (1 << 12); // store new value on overrun
	// Single conversion enabled by default
    ADC4->IER |= (1 << 2);  // enable EOC interrupt

    // Enable ADC4
	ADC4->CR |= (1 << 0);  // enable ADC4
    // PD11 (IN15) needs GPIO MODER defaults to 0x11 which selects analog mode
    NVIC_EnableIRQ(ADC4_IRQn); // NVIC interrupt init for ADC4
}

/**
 * Function name: tim2_init
 * Purpose: initialize tim2 to drive PWM
 * Settings:
 *  Channel: 1 (PA5)
 *  Frequency: 20 kHZ
 *  Interrupt: None
 */
void tim2_init(void)
{
    TIM2->PSC = T2_PSC; // Set PSC value
    TIM2->ARR = T2_ARR; // Set ARR value
    TIM2->CCR1 = T2_ARR; // Initial duty cycle at 50% on CH1

    TIM2->CCMR1 |= ((0b0110 << 4) | (1 << 3));  // PWM1 mode, enable preload reg. - CC1
    TIM2->CCER |= (1 << 0);  // enable output on pin for CC1
    TIM2->CR1 |= (1 << 7);  // set ARPE bit - enable auto-reload preload
    TIM2->EGR |= (1 << 0);  // update timer registers
    TIM2->CR1 |= (1 << 0);  // enable timer

}

/**
 * Function name: tim3_init
 * Purpose: initialize tim3 to start AD conversions
 * Settings:
 *  Channel: None
 *  Frequency: 1 kHz
 *  Interrupt: TIM3_IRQn
 */
void tim3_init(void) {
    TIM3->PSC = T3_PSC; // Set PSC value
	TIM3->ARR = T3_ARR; // Set ARR value

    TIM3->CR1 |= (1 << 7); // set ARPE bit - enable auto-reload preload
	TIM3->EGR |= (1 << 0); // update timer registers
	TIM3->CR1 |= (1 << 2); // interrupt only on over/underflow, also UDIS=0
	TIM3->DIER |= (1 << 0); // enable update interrupt
	TIM3->SR &= ~(1 << 0); // clear pending bit
	TIM3->CR1 |= (1 << 0); // enable timer3
    NVIC_EnableIRQ(TIM3_IRQn);
}


/**
 * Function name: ADC4_IRQHandler
 * Purpose: Interrupt to read new ADC4 conversions
 */
void ADC4_IRQHandler(void) {
	ADC_value = ADC4->DR;
	ADC4->ISR |= (1 << 2); //write 1 to EOC to clear flag
	ADC_newval_flag = true; // Indicate new conversion ready to main
}

/**
 * Function name: TIM3_IRQHandler
 * Purpose: Interrupt to initialize a new ADC4 conversion
 */
void TIM3_IRQHandler(void)
{
  // NVIC_ClearPendingIRQ(TIM2_IRQn);
	TIM3->SR &= ~(1 << 0);  // clear bit by writing '0' to it
	ADC4->CR |= (1 << 2); // Begin new ADC conversion
}
