/*
 * carHw.h
 *
 *  Created on: 10 feb. 2019
 *      Author: erlin
 */

#ifndef CARHW_H_
#define CARHW_H_

#include "main.h"
#include "cmsis_os.h"
UART_HandleTypeDef huart1;

struct Command {
	char startMarker;
	int command;
	char delimiter;
	int param;
	char endMarker;
};
int minAngleDistance;
int maxAngleDistance;

void runCarHw(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart3);

#endif /* CARHW_H_ */
