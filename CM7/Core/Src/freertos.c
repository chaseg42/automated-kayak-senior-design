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
#include "adc.h"
#include "UI.h"
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
  
  // Begin waiting for end of data to fire interrupt
  HAL_UART_Receive_IT(&huart5, sonar5.rx_data, 4);
  HAL_UART_Receive_IT(&huart7, sonar7.rx_data, 4);

  osTimerStart(HeartbeatTimerHandle, 500); // TODO move timer start to a task that makes more sense
                                           // Create generic init task?

  /* Infinite loop */
  for(;;)
  {
    uint8_t command = 0x55; // DFRobot sonar part requires 0x55 to be recieved before responding with data

    // Transmit to all sonar sensors
    HAL_UART_Transmit(&huart5, &command, 1, 10);
    HAL_UART_Transmit(&huart7, &command, 1, 10);

    if (sonar5.new_distance_flag == true)
    {
        sonar5.new_distance_flag = false;

        // TODO send the data somewhere meaningful
    }
    if (sonar7.new_distance_flag == true)
    {
    	sonar7.new_distance_flag = false;
    }
    
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
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 20); // TODO Use an interrupt instead
    uint16_t adc_result = HAL_ADC_GetValue(&hadc3);
    float new_arr = 10000 * ((65535 - adc_result) / 65535.0f); // 10000 is ARR, 16 bit adc
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, new_arr);

    osDelay(100);
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

  osDelay(6000); // Wait for USB to setup

  if (HAL_UART_Receive_IT(&huart6, ui_state.rx_data, 3) != HAL_OK) {
	  while(1);
  }
  /* Infinite loop */
  for(;;)
  {
    if (ui_state.new_data_flag == true) // TODO use a notification instead
    {
      ui_state.new_data_flag = false;
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGPSTask */
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

