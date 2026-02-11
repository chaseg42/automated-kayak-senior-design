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
#include "usb_device.h"
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

osThreadId_t SerialTaskHandle;
const osThreadAttr_t SerialTask_attributes = {
                                            .name = "SerialTask",
                                            .stack_size = THREAD_STACK_SIZE*16,
                                            .priority = (osPriority_t) osPriorityNormal,
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
void SerialTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{

	// Initialization

	// Mutexes

	// Semaphores

	// Timers
	HeartbeatTimerHandle = osTimerNew(HeartbeatCallback, osTimerPeriodic, NULL, &HeartbeatTimer_attributes);

	// Queues

	// Threads
//	SonarTaskHandle = osThreadNew(StartSonarTask, NULL, &SonarTask_attributes);
	GPSTaskHandle = osThreadNew(GPSTask, NULL, &GPSTask_attributes);


	SerialTaskHandle = osThreadNew(SerialTask, NULL, &SerialTask_attributes);


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
		GPS_Data = decode_nav(&GPS_Parsed_Data);

		// TODO: Once we integrate the task where this information is used, use a notification here to indicate that this task has concluded.


		// osDelay(1000);
		while((current_ticks - ticks) < poll_time) { current_ticks = xTaskGetTickCount(); }
	}
}

#define GPS_TIME GPS_Data.utc_time.hour, GPS_Data.utc_time.min, GPS_Data.utc_time.sec
#define GPS_DATE GPS_Data.utc_date.month, GPS_Data.utc_date.day, GPS_Data.utc_date.year
#define GPS_VEL	 GPS_Data.velocity.N, GPS_Data.velocity.E, GPS_Data.velocity.D
#define GPS_POS	 GPS_Data.world_position.N, GPS_Data.world_position.E, GPS_Data.world_position.D
char *message[] = {"Date: %i/%i/%i\r\n", "Time: %i:%i:%i\r\n", "Position = {N: %.2lf, E: %.2lf, D: %.2lf}\r\n", "Velocity = {N: %.2lf, E: %.2lf, D: %.2lf}\r\n"};
char *divider = "========================================\r\n";

//static void usb_tx(char *message, word size);

void SerialTask(void *argument)
{
//	vTaskDelete( NULL );
	TickType_t poll_time = configTICK_RATE_HZ * 2;
	TickType_t current_ticks = 0;
	// __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
	MX_USB_DEVICE_Init(); //

	while(1)
	{
		TickType_t ticks = xTaskGetTickCount();


		word size = strlen(message[0]) + 1; // Add 1 byte due to word in dataw[2]
		char b0[size];

		word dataw[] = {GPS_DATE, GPS_TIME};

		sprintf(b0, message[0], dataw[0], dataw[1], dataw[2]);
		CDC_Transmit_FS(b0, sizeof(b0));
		fflush(stdout);
		osDelay(200);

		size = strlen(message[1]);
		char b1[size];
		sprintf(b1, message[1], dataw[3], dataw[4], dataw[5]);
		CDC_Transmit_FS(b1, sizeof(b1));
		osDelay(200);

		float dataf[] = {GPS_POS, GPS_VEL};

		size = strlen(message[2]) + 4; // Not sure why yet the + 4 offset is needed here to capture the newline characters.
		char b2[size];
		sprintf(b2, message[2], dataf[0], dataf[1], dataf[2]);
		CDC_Transmit_FS(b2, sizeof(b2));
		osDelay(200);

		size = strlen(message[3]);
		char b3[size];
		sprintf(b3, message[3], dataf[3], dataf[4], dataf[5]);
		CDC_Transmit_FS(b3, sizeof(b3));
		osDelay(200);

		CDC_Transmit_FS(divider, strlen(divider));

		while((current_ticks - ticks) < poll_time) { current_ticks = xTaskGetTickCount(); }
	}

}


//static void usb_tx(char *buffer, word size)
//{
//	CDC_Transmit_FS(buffer, sizeof(buffer)); // Transmit over USB
//}


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

