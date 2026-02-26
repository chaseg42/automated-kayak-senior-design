/*
 * sonar.c
 *
 *  Created on: Feb 25, 2026
 *      Author: gattusoc
 */


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"
#include "sonar.h"

Sonar_t sonar5 = {
                  .rx_data = {0},
                  .distance = 0.0,
                  .new_distance_flag = false
};
Sonar_t sonar7 = {
                  .rx_data = {0},
                  .distance = 0.0,
                  .new_distance_flag = false
};

bool Sonar_uartToDistance(UART_HandleTypeDef *huart, Sonar_t *sonar) {
  bool retval = false;
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
  if (sonar->rx_data[0] == 0xff)
  {
    if (sonar->rx_data[3] == (uint8_t)(sonar->rx_data[0] + sonar->rx_data[1] + sonar->rx_data[2]))
    {
      sonar->distance = ((sonar->rx_data[1] << 8) + sonar->rx_data[2]) / 10.0f;
      sonar->new_distance_flag = true;
      retval = true; // Success
    }
  }
  HAL_UART_Receive_IT(huart, sonar->rx_data, 4);
  return retval;
}
