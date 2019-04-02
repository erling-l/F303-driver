/*
 * extEnvironment.h
 *
 *  Created on: 10 feb. 2019
 *      Author: erlin
 */

#ifndef EXTENVIRONMENT_H_
#define EXTENVIRONMENT_H_
#include <stdbool.h>
typedef struct  {
	long distance;
	long previousDistance;
}Wall;

//typedef struct  {
//	long distanceToLeftWall; // not used
//	long distanceToRightWall;  // not used
//	long angleToLeftCorner;
//	long angleToRightCorner;
//	long distance;
//	long previousDistance;
//	int angle;
//}obstacle;
//typedef struct  {
//	long speed;
//	long directionX;
//	long directionY;
//	long directionZ;
//	bool left;
//	bool right;
//	bool front;
//	bool back;
//
//} car;
//typedef struct  {
//	long timestamp;
//	car myCar;
//	obstacle leftWall;
//	obstacle rightWall;
//	obstacle obstacle1[5];
//}timeFrame;
typedef struct  {
	long timestamp;
	long speed;
	long directionX;
	long directionY;
	long directionZ;
	long distance;
	int angle;
	int direction;
}obstacle;
obstacle *leftWall;
obstacle *rightWall;
int previousDistance;
int obstacles;
void runExtEnvironment(void);
char *strtok(char *str, const char *delim);

#endif /* EXTENVIRONMENT_H_ */
