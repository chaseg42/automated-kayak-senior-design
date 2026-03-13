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

#define GPS_RX_BUFFER_SIZE 256

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/*****************************************
 *
 * 				GPS Globals
 *
 *****************************************/
byte UART4_rxBuffer[GPS_RX_BUFFER_SIZE] = {0};
GPSParsedDataStruct GPS_Parsed_Data;
GPSDataStruct GPS_Data;
bool b_rx_transfer_complete, b_tx_transfer_complete = false;

/* USER CODE END Variables */
/* Definitions for SonarTask */
osThreadId_t SonarTaskHandle;
const osThreadAttr_t SonarTask_attributes = {
  .name = "SonarTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorControlTas */
osThreadId_t MotorControlTasHandle;
const osThreadAttr_t MotorControlTas_attributes = {
  .name = "MotorControlTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DetermineStateT */
osThreadId_t DetermineStateTHandle;
const osThreadAttr_t DetermineStateT_attributes = {
  .name = "DetermineStateT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GPSTask */
osThreadId_t GPSTaskHandle;
const osThreadAttr_t GPSTask_attributes = {
  .name = "GPSTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for HeartbeatTimer */
osTimerId_t HeartbeatTimerHandle;
const osTimerAttr_t HeartbeatTimer_attributes = {
  .name = "HeartbeatTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSonarTask(void *argument);
void StartMotorControlTask(void *argument);
void StartDetermineStateTask(void *argument);
void StartGPSTask(void *argument);
void HeartbeatCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of HeartbeatTimer */
  HeartbeatTimerHandle = osTimerNew(HeartbeatCallback, osTimerPeriodic, NULL, &HeartbeatTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SonarTask */
  SonarTaskHandle = osThreadNew(StartSonarTask, NULL, &SonarTask_attributes);

  /* creation of MotorControlTas */
  MotorControlTasHandle = osThreadNew(StartMotorControlTask, NULL, &MotorControlTas_attributes);

  /* creation of DetermineStateT */
  DetermineStateTHandle = osThreadNew(StartDetermineStateTask, NULL, &DetermineStateT_attributes);

  /* creation of GPSTask */
  GPSTaskHandle = osThreadNew(StartGPSTask, NULL, &GPSTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

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

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
* @brief Function implementing the MotorControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControlTask */
void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorControlTask */
}

/* USER CODE BEGIN Header_StartDetermineStateTask */
/**
* @brief Function implementing the DetermineStateT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDetermineStateTask */
void StartDetermineStateTask(void *argument)
{
  /* USER CODE BEGIN StartDetermineStateTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDetermineStateTask */
}

/* USER CODE BEGIN Header_StartGPSTask */
/**
* @brief Function implementing the GPSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGPSTask */
void StartGPSTask(void *argument)
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

	//		uart4_status = HAL_UART_Transmit_DMA(&huart4, ubx_tx_poll_id, sizeof(ubx_tx_poll_id));
	//		if(uart4_status == HAL_ERROR || uart4_status == HAL_TIMEOUT) { continue; } // Bail
	//
	//		while(!b_tx_transfer_complete);
	//		b_tx_transfer_complete = false;

			uart4_status = HAL_UART_Transmit_DMA(&huart4, ubx_tx_poll_pvt, sizeof(ubx_tx_poll_pvt));
			if(uart4_status == HAL_ERROR || uart4_status == HAL_TIMEOUT) { continue; } // Bail

			while(!b_tx_transfer_complete);
			b_tx_transfer_complete = false;

			while(!b_rx_transfer_complete); // Wait until RX transmission is not busy
			b_rx_transfer_complete = false;

			ubx_status = parse_rx_buffer_to_ubx_frame(&UBXFrame);
			if(ubx_status != UBX_OK) { continue; } // Bail

			// Decode information
			decode_nav(&GPS_Parsed_Data, &GPS_Data);

			// TODO: Once we integrate the task where this information is used, use a notification here to indicate that this task has concluded.

			while((current_ticks - ticks) < poll_time) { current_ticks = xTaskGetTickCount(); }
		}
}

/* HeartbeatCallback function */
void HeartbeatCallback(void *argument)
{
  /* USER CODE BEGIN HeartbeatCallback */

  /* USER CODE END HeartbeatCallback */
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
	if(huart->Instance == UART4)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_rxBuffer, GPS_RX_BUFFER_SIZE); // Re-enable the interrupt
		b_rx_transfer_complete = true;
	}
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

