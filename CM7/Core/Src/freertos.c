/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gps.h"
#include "ubx.h"
#include <stdbool.h>

// TODO: Consider splitting tasks into subsystem task files to reduce file complexity

#define THREAD_STACK_SIZE 512 // In increments of 128
#define GPS_RX_BUFFER_SIZE 256

/*****************************************
 *
 * 				DEBUG
 *
 *****************************************/
osTimerId_t HeartbeatTimerHandle;
const osTimerAttr_t HeartbeatTimer_attributes = {
  .name = "HeartbeatTimer"
};

/*****************************************
 *
 * 				SONAR
 *
 *****************************************/
osThreadId_t SonarTaskHandle;
const osThreadAttr_t SonarTask_attributes = {
                                              .name = "SonarTask",
                                              .stack_size = THREAD_STACK_SIZE,
                                              .priority = (osPriority_t) osPriorityNormal,
                                            };

/*****************************************
 *
 * 				GPS
 *
 *****************************************/
osThreadId_t GPSTaskHandle;
const osThreadAttr_t GPSTask_attributes = {
                                            .name = "GPSTask",
                                            .stack_size = THREAD_STACK_SIZE,
                                            .priority = (osPriority_t) osPriorityNormal,
                                          };
/*****************************************
 *
 * 				GPS Globals
 *
 *****************************************/
byte UART4_rxBuffer[GPS_RX_BUFFER_SIZE] = {0};
GPSParsedDataStruct GPS_Parsed_Data;
GPSDataStruct GPS_Data;
bool b_rx_transfer_complete, b_tx_transfer_complete = false;


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void HeartbeatCallback(void *argument);
void StartSonarTask(void *argument);
void GPSTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {

	// Initialization

	// Mutexes

	// Semaphores

	// Timers
	HeartbeatTimerHandle = osTimerNew(HeartbeatCallback, osTimerPeriodic, NULL, &HeartbeatTimer_attributes);

	// Queues

	// Threads
//	SonarTaskHandle = osThreadNew(StartSonarTask, NULL, &SonarTask_attributes);
	GPSTaskHandle = osThreadNew(GPSTask, NULL, &GPSTask_attributes);


	// Events

}


/* HeartbeatCallback function */
void HeartbeatCallback(void *argument)
{
  /* USER CODE BEGIN HeartbeatCallback */
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  /* USER CODE END HeartbeatCallback */
}


/*****************************************
 *
 * 				SONAR
 *
 *****************************************/
osThreadId_t SonarTaskHandle;
/* USER CODE BEGIN Header_StartSonarTask */
/**
  * @brief  Function implementing the SonarTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSonarTask */
void StartSonarTask(void *argument)
{
  /* USER CODE BEGIN StartSonarTask */
  /* Infinite loop */
//	for(;;)
//	{
//		// TODO move each task to its own file
////		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
////		osDelay(500);
//	}
  /* USER CODE END StartSonarTask */
}


/*****************************************
 *
 * 				GPS
 *
 *****************************************/
/* USER CODE BEGIN Header_GPSTask */
/**
  * @brief  Function implementing the GPSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_GPSTask */
void GPSTask(void *argument)
{
//	byte UART4_rxBuffer[256] = {0};
	UBXFrame_Typedef UBXFrame;
	UBXStatus ubx_status;
	HAL_StatusTypeDef uart4_status;
//	GPS_Data_Struct GPS_Data;

	// Note: DMA transferring does not currently work. This does, however.
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_rxBuffer, GPS_RX_BUFFER_SIZE); // Initialize UART4 to use the RX interrupt

	// TODO: Divide poll_time by 10 for HNR => 10 Hz, otherwise, 1 Hz is default. Waiting for kayak state implementation to include this
	TickType_t poll_time = configTICK_RATE_HZ;
	TickType_t current_ticks = 0;

	// Goal: We want to request GPS data continuously when the receiver is ready.
	// TODO: Indicate to the user when an error occurs in the rx/tx loop
	while(1)
	{

		TickType_t ticks = xTaskGetTickCount();

		uart4_status = HAL_UART_Transmit_DMA(&huart4, ubx_tx_poll_id, sizeof(ubx_tx_poll_id));
		if(uart4_status == HAL_ERROR || uart4_status == HAL_TIMEOUT) { continue; } // Bail

		while(!b_tx_transfer_complete);
		b_tx_transfer_complete = false;

		uart4_status = HAL_UART_Transmit_DMA(&huart4, ubx_tx_poll_pvt, sizeof(ubx_tx_poll_pvt));
		if(uart4_status == HAL_ERROR || uart4_status == HAL_TIMEOUT) { continue; } // Bail

		while(!b_tx_transfer_complete);
		b_tx_transfer_complete = false;

		while(!b_rx_transfer_complete); // Wait until RX transmission is not busy
		b_rx_transfer_complete = false;

		ubx_status = parse_rx_buffer_to_ubx_frame(&UBXFrame);
		if(ubx_status != UBX_OK) { continue; } // Bail

		// Decode information
		GPS_Data = decode_nav(&GPS_Parsed_Data);

		// TODO: Once we integrate the task where this information is used, use a notification here to indicate that this task has concluded.


		// osDelay(1000);
		while((current_ticks - ticks) < poll_time) { current_ticks = xTaskGetTickCount(); }
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4)
	{
		b_tx_transfer_complete = true;
	}
}



// Handle data reception in this callback instead of the UART4 IRQ
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//	if(huart->Instance == UART4)
//	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_rxBuffer, GPS_RX_BUFFER_SIZE); // Re-enable the interrupt
		b_rx_transfer_complete = true;
//	}
}

