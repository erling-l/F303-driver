/*
 * driver.cpp
 *
 *  Created on: 10 feb. 2019
 *      Author: erlin
 */
#include "driver.h"
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

void runDriver() {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  osDelay(600);
}
