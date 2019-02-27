/*
 * messageQueue.h
 *
 *  Created on: 22 feb. 2019
 *      Author: erlin
 */

#ifndef MESSAGEQUEUE_H_
#define MESSAGEQUEUE_H_
#include "stm32f3xx_hal.h"
extern osMessageQId osQueue;
extern uint32_t ProducerValue, ConsumerValue;
extern __IO uint32_t ProducerErrors, ConsumerErrors;
// extern BSP_LED_On();
// extern BSP_LED_Toggle();

#endif /* DRIVER_H_ */
