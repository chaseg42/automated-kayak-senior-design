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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "sonar.h"
#include "tim.h"
#include "UI.h"
#include "dma.h"
#include "gps.h"
#include "ubx.h"
#include "queue.h"
#include "motor_control.h"
#include "radar.h"
#include "usbd_def.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_LOOP_DELAY_MS              100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

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
/* Definitions for RadarTask */
osThreadId_t RadarTaskHandle;
const osThreadAttr_t RadarTask_attributes = {
  .name = "RadarTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sonarQueue */
osMessageQueueId_t sonarQueueHandle;
const osMessageQueueAttr_t sonarQueue_attributes = {
  .name = "sonarQueue"
};
/* Definitions for UIQueue */
osMessageQueueId_t UIQueueHandle;
const osMessageQueueAttr_t UIQueue_attributes = {
  .name = "UIQueue"
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
void StartRadarTask(void *argument);
void HeartbeatCallback(void *argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* Create the queue(s) */
  /* creation of sonarQueue */
  sonarQueueHandle = osMessageQueueNew (1, sizeof(Sonar_t), &sonarQueue_attributes);

  /* creation of UIQueue */
  UIQueueHandle = osMessageQueueNew (1, sizeof(UIdata), &UIQueue_attributes);

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

  /* creation of RadarTask */
  RadarTaskHandle = osThreadNew(StartRadarTask, NULL, &RadarTask_attributes);

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
  
  // Begin waiting for end of data to fire interrupt
  HAL_UART_Receive_IT(&huart5, sonar5.rx_data, 4);
  //HAL_UART_Receive_IT(&huart7, sonar7.rx_data, 4);

  osTimerStart(HeartbeatTimerHandle, 500); // TODO move timer start to a task that makes more sense
                                           // Create generic init task?

  /* Infinite loop */
  for(;;)
  {
    uint8_t command = 0x55; // DFRobot sonar part requires 0x55 to be recieved before responding with data

    // Transmit to all sonar sensors
    HAL_UART_Transmit(&huart5, &command, 1, 10);
    //HAL_UART_Transmit(&huart7, &command, 1, 10);

    if (sonar5.new_distance_flag == true)
    {
        sonar5.new_distance_flag = false;
        xQueueOverwrite((QueueHandle_t)sonarQueueHandle, &sonar5);
    }
    //if (sonar7.new_distance_flag == true)
    //{
    //  sonar7.new_distance_flag = false;
    //  xQueueOverwrite((QueueHandle_t)sonarQueueHandle, &sonar7);
    //}
    
    osDelay(250);
  }
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
  UIdata latest_ui = ui_state;
  Sonar_t latest_sonar = sonar5;
  bool sonar_data_valid = false;

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); // Motor relay off

  operatingMode_t current_mode = MODE_DISABLE;
  bool mode_entry = true;
  bool startup_steps_done = false;

  MotorControlState motor_state;
  MotorControl_InitState(&motor_state);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 45 degree
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 135 degree
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // 225 degree
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // 315 degree

  /* Infinite loop */
  for(;;)
  {
    bool got_ui_update = false;

    if (xQueueReceive((QueueHandle_t)UIQueueHandle, &latest_ui, 0) == pdPASS)
    {
      got_ui_update = true;
    }

    if (xQueueReceive((QueueHandle_t)sonarQueueHandle, &latest_sonar, 0) == pdPASS)
    {
      sonar_data_valid = true;
    }

    if ((operatingMode_t)latest_ui.mode != current_mode)
    {
      current_mode = (operatingMode_t)latest_ui.mode;
      mode_entry = true;
    }
    else if (got_ui_update)
    {
      mode_entry = true;
    }

    motor_speed motor_cmd = {0};

    switch (current_mode)
    {
      case MODE_DISABLE:
        if (mode_entry)
        {
          // startup idle section
          if (!startup_steps_done)
          {
            // TODO Set up DMA on ethernet port
            startup_steps_done = true;
          }
          motor_state.desired_speed_cmd = 0;
          mode_entry = false;
        }
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); // Motor relay off
        motor_cmd.speed_45 = 0;
        motor_cmd.speed_135 = 0;
        motor_cmd.speed_225 = 0;
        motor_cmd.speed_315 = 0;
        break;

      case MODE_MOVE:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); // Motor relay on
        MotorControl_ModeMove(&motor_state, mode_entry, got_ui_update, &latest_ui,
                              sonar_data_valid, &latest_sonar,
                              &motor_cmd,
                              &mode_entry);
        break;

      case MODE_ANCHOR:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); // Motor relay on
        MotorControl_ModeAnchor(&motor_state, mode_entry,
                                &motor_cmd,
                                &mode_entry);
        break;

      case MODE_FOLLOW_SHORE:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); // Motor relay on
        MotorControl_ModeFollowShore(&motor_state, mode_entry, got_ui_update, &latest_ui,
                                     sonar_data_valid, &latest_sonar,
                                     &motor_cmd,
                                     &mode_entry);
        break;
      case MOTOR_OVERRIDE:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); // Motor relay on
        MotorControl_ModeOverride(&latest_ui,
                                  &motor_cmd);
        break;
      default:
        current_mode = MODE_DISABLE;
        mode_entry = true;
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); // Motor relay off
        motor_cmd.speed_45 = 0;
        motor_cmd.speed_135 = 0;
        motor_cmd.speed_225 = 0;
        motor_cmd.speed_315 = 0;
        break;
    }

      MotorControl_SetOutputs(&motor_cmd);

    osDelay(MOTOR_LOOP_DELAY_MS);
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

  if (HAL_UART_Receive_IT(&huart6, ui_state.rx_data, 8) != HAL_OK) {
	  while(1);
  }
  /* Infinite loop */
  for(;;)
  {
    if (ui_state.new_data_flag == true) // TODO use a notification instead
    {
      ui_state.new_data_flag = false;
      xQueueOverwrite((QueueHandle_t)UIQueueHandle, &ui_state);
    }
    osDelay(100);
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
  /* USER CODE BEGIN StartGPSTask */

  //	byte UART4_rxBuffer[256] = {0};
		UBXFrame_Typedef UBXFrame;
		UBXStatus ubx_status;
		HAL_StatusTypeDef uart4_status;
	//	GPS_Data_Struct GPS_Data;

    // Note: DMA requires cache maintenance on H7 when D-Cache is enabled.
    SCB_CleanDCache_by_Addr((uint32_t *)UART4_rxBuffer, UART4_DMA_CACHE_ALIGN_UP(GPS_RX_BUFFER_SIZE));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_rxBuffer, GPS_RX_BUFFER_SIZE); // Initialize UART4 to use the RX interrupt

		// TODO: Divide poll_time by 10 for HNR => 10 Hz, otherwise, 1 Hz is default. Waiting for kayak state implementation to include this
		TickType_t poll_time = configTICK_RATE_HZ;
		TickType_t current_ticks = 0;

		// Goal: We want to request GPS data continuously when the receiver is ready.
		// TODO: Indicate to the user when an error occurs in the rx/tx loop

  /* Infinite loop */
  for(;;)
  {
      TickType_t ticks = xTaskGetTickCount();

      uart4_status = HAL_UART_Transmit_DMA(&huart4, ubx_tx_poll_pvt, sizeof(ubx_tx_poll_pvt));
      if(uart4_status == HAL_ERROR || uart4_status == HAL_TIMEOUT) { continue; } // Bail

      xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);

      SCB_InvalidateDCache_by_Addr((uint32_t *)UART4_rxBuffer, UART4_DMA_CACHE_ALIGN_UP(GPS_RX_BUFFER_SIZE));
      ubx_status = parse_rx_buffer_to_ubx_frame(&UBXFrame);
      if(ubx_status != UBX_OK) { continue; } // Bail

      // Decode position/velocity
      decode_nav(&GPS_Parsed_Data, &GPS_Data);

      uart4_status = HAL_UART_Transmit_DMA(&huart4, ubx_tx_poll_att, sizeof(ubx_tx_poll_att));
      if(uart4_status == HAL_ERROR || uart4_status == HAL_TIMEOUT) { continue; } // Bail

      xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);

      SCB_InvalidateDCache_by_Addr((uint32_t *)UART4_rxBuffer, UART4_DMA_CACHE_ALIGN_UP(GPS_RX_BUFFER_SIZE));
      ubx_status = parse_rx_buffer_to_ubx_frame(&UBXFrame);
      if(ubx_status != UBX_OK) { continue; } // Bail

      // Decode rotation using latest ATT fields
      decode_nav(&GPS_Parsed_Data, &GPS_Data);

      if (usart6_tx_complete)
      {
        usart6_tx_complete = false;
        GPS_PopulateESP32Buffer(&GPS_Data, UART6_txBuffer);
        SCB_CleanDCache_by_Addr((uint32_t *)UART6_txBuffer, UART4_DMA_CACHE_ALIGN_UP(ESP32_GPS_TX_LEN));

        // Send send to ESP32
        HAL_UART_Transmit_DMA(&huart6, UART6_txBuffer, ESP32_GPS_TX_LEN);
      }

			// TODO: Once we integrate the task where this information is used, use a notification here to indicate that this task has concluded.

			while((current_ticks - ticks) < poll_time) { current_ticks = xTaskGetTickCount(); }

  }
  /* USER CODE END StartGPSTask */
}

/* USER CODE BEGIN Header_StartRadarTask */
/**
* @brief Function implementing the RadarTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRadarTask */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern bool radar_task_update;
void StartRadarTask(void *argument)
{
  /* USER CODE BEGIN StartRadarTask */
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
  /* Infinite loop */
  for(;;)
  {

	  // Loop for rx the radar
	  // Wait for signal from USB buffer callback
	  // Extract buffer
	  // No data, we can wait for UI here

	  if(!radar_task_update)
	  {
		  if(radar_detections.radar_state == 1)
		  {
			  radar_task_update = true;
		  }
	  }

	  if(radar_task_update)
	  {
		  usb_radar_rx(&hUsbDeviceFS);
	  }

	  usb_radar_tx_state();

	  osDelay(100);


  }
  /* USER CODE END StartRadarTask */
}

/* HeartbeatCallback function */
void HeartbeatCallback(void *argument)
{
  /* USER CODE BEGIN HeartbeatCallback */
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  /* USER CODE END HeartbeatCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // breakpoint here
    while(1);
}
/* USER CODE END Application */

