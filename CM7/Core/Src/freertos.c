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
#include "queue.h"
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_PWM_MAX_COUNTS             10000
#define MOTOR_LOOP_DELAY_MS              100
#define SONAR_OBSTACLE_NEAR_CM           10.0f  // distance threshold for sonar aggressive maneuver
#define SONAR_OBSTACLE_CAUTION_CM        100.0f // distance threshold for sonar softer maneuver
#define MOTOR_TURN_SOFT_DELTA_CMD        40     // change in speed for softer maneuver
#define MOTOR_TURN_HARD_DELTA_CMD        90     // change in speed for aggressive maneuver

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
static uint8_t Motor_ClampSpeedCmd(int32_t speed_cmd);
static void Motor_ComputeQuadSpeed(uint8_t base_speed_cmd, direction_t turn_direction, uint8_t turn_delta_cmd,
                                   uint8_t *motor_45_speed_cmd, uint8_t *motor_135_speed_cmd,
                                   uint8_t *motor_225_speed_cmd, uint8_t *motor_315_speed_cmd);
static void Motor_SetOutputs(uint8_t motor_45_speed_cmd, uint8_t motor_135_speed_cmd,
                             uint8_t motor_225_speed_cmd, uint8_t motor_315_speed_cmd);

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
        xQueueOverwrite((QueueHandle_t)sonarQueueHandle, &sonar5);
    }
    if (sonar7.new_distance_flag == true)
    {
        sonar7.new_distance_flag = false;
        xQueueOverwrite((QueueHandle_t)sonarQueueHandle, &sonar7);
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
  UIdata latest_ui = ui_state;
  Sonar_t latest_sonar = sonar5;
  bool sonar_data_valid = false;

  operatingMode_t current_mode = MODE_DISABLE;
  bool mode_entry = true;
  bool startup_steps_done = false;

  uint8_t desired_speed_cmd = 0U;
  direction_t desired_drive_direction = FORWARD;
  direction_t desired_shore_side = RIGHT;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

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

    uint8_t motor_45_speed_cmd = 0;
    uint8_t motor_135_speed_cmd = 0;
    uint8_t motor_225_speed_cmd = 0;
    uint8_t motor_315_speed_cmd = 0;

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
          desired_speed_cmd = 0;
          mode_entry = false;
        }
        motor_45_speed_cmd = 0;
        motor_135_speed_cmd = 0;
        motor_225_speed_cmd = 0;
        motor_315_speed_cmd = 0;
        break;

      case MODE_MOVE:
        if (mode_entry)
        {
          // read desired speed from notification
          desired_speed_cmd = latest_ui.speed;
          desired_drive_direction = latest_ui.direction_to_turn;
          // TODO get GPS heading notification and save desired heading
          mode_entry = false;
        }

        if (got_ui_update)
        {
          desired_speed_cmd = latest_ui.speed;
          desired_drive_direction = latest_ui.direction_to_turn;
        }

        Motor_ComputeQuadSpeed(desired_speed_cmd, desired_drive_direction, 0,
                   &motor_45_speed_cmd, &motor_135_speed_cmd,
                   &motor_225_speed_cmd, &motor_315_speed_cmd);

        if ((desired_drive_direction != REVERSE) && sonar_data_valid && (latest_sonar.distance <= SONAR_OBSTACLE_NEAR_CM))
        {
          // object within 10 cm -> aggressive avoidance turn
          Motor_ComputeQuadSpeed(desired_speed_cmd, RIGHT, MOTOR_TURN_HARD_DELTA_CMD,
                                 &motor_45_speed_cmd, &motor_135_speed_cmd,
                                 &motor_225_speed_cmd, &motor_315_speed_cmd);
        }
        else if ((desired_drive_direction != REVERSE) && sonar_data_valid && (latest_sonar.distance <= SONAR_OBSTACLE_CAUTION_CM))
        {
          // object within 1 meter -> softer correction
          Motor_ComputeQuadSpeed(desired_speed_cmd, RIGHT, MOTOR_TURN_SOFT_DELTA_CMD,
                                 &motor_45_speed_cmd, &motor_135_speed_cmd,
                                 &motor_225_speed_cmd, &motor_315_speed_cmd);
        }
        else
        {
          // TODO if current heading is >=15 deg away from desired compute heading correction
          // TODO do something if radar detects obstacle in path
        }
        break;

      case MODE_ANCHOR:
        if (mode_entry)
        {
          // TODO wait for GPS position + heading notification and save desired position and heading
          mode_entry = false;
        }

        // TODO:
        //  Check new heading and correct when >=15 deg away from desired
        //  Check new location and correct when >=1 meter away from desired
        motor_45_speed_cmd = 0;
        motor_135_speed_cmd = 0;
        motor_225_speed_cmd = 0;
        motor_315_speed_cmd = 0;
        break;

      case MODE_FOLLOW_SHORE:
        if (mode_entry)
        {
          desired_speed_cmd = latest_ui.speed;
          desired_shore_side = latest_ui.direction_to_turn;
          // TODO wait for radar data to save shoreline
          mode_entry = false;
        }

        if (got_ui_update)
        {
          desired_speed_cmd = latest_ui.speed;
          desired_shore_side = latest_ui.direction_to_turn;
        }

        if (sonar_data_valid && (latest_sonar.distance <= SONAR_OBSTACLE_NEAR_CM))
        {
          direction_t avoid_direction = (desired_shore_side == RIGHT) ? LEFT : RIGHT;
          Motor_ComputeQuadSpeed(desired_speed_cmd, avoid_direction, MOTOR_TURN_HARD_DELTA_CMD,
                                 &motor_45_speed_cmd, &motor_135_speed_cmd,
                                 &motor_225_speed_cmd, &motor_315_speed_cmd);
        }
        else if (sonar_data_valid && (latest_sonar.distance <= SONAR_OBSTACLE_CAUTION_CM))
        {
          direction_t avoid_direction = (desired_shore_side == RIGHT) ? LEFT : RIGHT;
          Motor_ComputeQuadSpeed(desired_speed_cmd, avoid_direction, MOTOR_TURN_SOFT_DELTA_CMD,
                                 &motor_45_speed_cmd, &motor_135_speed_cmd,
                                 &motor_225_speed_cmd, &motor_315_speed_cmd);
        }
        else
        {
          // no immediate obstacle so keep desired speed with slight shoreline bias
          Motor_ComputeQuadSpeed(desired_speed_cmd, desired_shore_side, 20,
                                 &motor_45_speed_cmd, &motor_135_speed_cmd,
                                 &motor_225_speed_cmd, &motor_315_speed_cmd);

          // TODO follow shoreline using radar data
          // TODO use heading error >=15 deg to compute correction
        }
        break;

      default:
        current_mode = MODE_DISABLE;
        mode_entry = true;
        motor_45_speed_cmd = 0;
        motor_135_speed_cmd = 0;
        motor_225_speed_cmd = 0;
        motor_315_speed_cmd = 0;
        break;
    }

      Motor_SetOutputs(motor_45_speed_cmd, motor_135_speed_cmd,
               motor_225_speed_cmd, motor_315_speed_cmd);

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

		// Note: DMA transferring does not currently work. This does, however.
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
static uint8_t Motor_ClampSpeedCmd(int32_t speed_cmd)
{
  if (speed_cmd < 0)
  {
    return 0U;
  }
  if (speed_cmd > 255)
  {
    return 255U;
  }
  return (uint8_t)speed_cmd;
}

static void Motor_ComputeQuadSpeed(uint8_t base_speed_cmd, direction_t turn_direction, uint8_t turn_delta_cmd,
                                   uint8_t *motor_45_speed_cmd, uint8_t *motor_135_speed_cmd,
                                   uint8_t *motor_225_speed_cmd, uint8_t *motor_315_speed_cmd)
{
  int32_t motor_45 = 0;
  int32_t motor_135 = 0;
  int32_t motor_225 = 0;
  int32_t motor_315 = 0;

  // front of kayak is 0 deg
  // 45 and 315 are front thrusters so keep them off during forward and turns
  // 135 and 225 handle forward movement and yaw while moving forward
  if (turn_direction == REVERSE)
  {
    motor_45 = base_speed_cmd;
    motor_315 = base_speed_cmd;
    // TODO gps correction in reverse if needed
  }
  else
  {
    motor_135 = base_speed_cmd;
    motor_225 = base_speed_cmd;

    // for right yaw while moving forward push 225 more and 135 less
    if (turn_direction == RIGHT)
    {
      motor_135 -= turn_delta_cmd;
      motor_225 += turn_delta_cmd;
    }
    else if (turn_direction == LEFT)
    {
      motor_135 += turn_delta_cmd;
      motor_225 -= turn_delta_cmd;
    }
  }

  *motor_45_speed_cmd = Motor_ClampSpeedCmd(motor_45);
  *motor_135_speed_cmd = Motor_ClampSpeedCmd(motor_135);
  *motor_225_speed_cmd = Motor_ClampSpeedCmd(motor_225);
  *motor_315_speed_cmd = Motor_ClampSpeedCmd(motor_315);
}

static void Motor_SetOutputs(uint8_t motor_45_speed_cmd, uint8_t motor_135_speed_cmd,
                             uint8_t motor_225_speed_cmd, uint8_t motor_315_speed_cmd)
{
  uint32_t motor_45_arr = (MOTOR_PWM_MAX_COUNTS * motor_45_speed_cmd) / 255;
  uint32_t motor_135_arr = (MOTOR_PWM_MAX_COUNTS * motor_135_speed_cmd) / 255;
  uint32_t motor_225_arr = (MOTOR_PWM_MAX_COUNTS * motor_225_speed_cmd) / 255;
  uint32_t motor_315_arr = (MOTOR_PWM_MAX_COUNTS * motor_315_speed_cmd) / 255;

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motor_45_arr);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor_135_arr);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, motor_225_arr);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, motor_315_arr);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // breakpoint here
    while(1);
}
/* USER CODE END Application */

