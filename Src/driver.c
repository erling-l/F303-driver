/*
 * driver.cpp
 *
 *  Created on: 10 feb. 2019
 *      Author: erlin
 */
#include "driver.h"
#include "main.h"
#include "carHw.h"
#include "extEnvironment.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
extern obstacle *obstacleList[];
extern obstacle *obstacleList[40];
extern int firstObject;
extern obstacle *leftWall;
extern obstacle *rightWall;
extern int previousDistance;
extern int minAngleDistance;
extern int maxAngleDistance;
extern long power;
extern long steeringAngle;
long previousPower;


void runDriver() {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	if (obstacleList[firstObject]->angle > 80 && obstacleList[firstObject]->angle < 100){
		if (obstacleList[firstObject]->distance < 100){ // Stop and back out of trouble
			power = -50;
			steeringAngle = 127;
		}else {
			power =50;
			previousPower = power;
			steeringAngle=0;
		}
		if (leftWall->distance < rightWall->distance){
			power = previousPower;
			steeringAngle=5;
		}
		if (leftWall->distance > rightWall->distance){
			power = previousPower;
			steeringAngle=-5;
		}

	}
	//	  osDelay(600);
}
