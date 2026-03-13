/*
 * sonar.h
 *
 *  Created on: Feb 25, 2026
 *      Author: gattusoc
 */

#ifndef INC_SONAR_H_
#define INC_SONAR_H_

#include "main.h"


typedef struct {
    uint8_t rx_data[4];
    float distance;
    bool new_distance_flag;
} Sonar_t;

extern Sonar_t sonar5; // Sonar on UART5
extern Sonar_t sonar7; // Sonar on UART7

bool Sonar_uartToDistance(UART_HandleTypeDef *huart, Sonar_t *sonar);

#endif /* INC_SONAR_H_ */
