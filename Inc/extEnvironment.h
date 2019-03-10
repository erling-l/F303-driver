/*
 * extEnvironment.h
 *
 *  Created on: 10 feb. 2019
 *      Author: erlin
 */

#ifndef EXTENVIRONMENT_H_
#define EXTENVIRONMENT_H_
#include <stdbool.h>
struct leftWall {
	long distance;
	long previousDistance;
};
struct rightWall {
	long distance;
	long previousDistance;
};
struct obstacle1 {
	long distance;
	long previousDistance;
	long distanceToLeftWall;
	long distanceToRightWall;
	long angleToLeftCorner;
	long angleToRightCorner;
};
struct car {
	long speed;
	long directionX;
	long directionY;
	long directionZ;
	bool left;
	bool right;
	bool front;
	bool back;

};
void runExtEnvironment(void);
char *strtok(char *str, const char *delim);

#endif /* EXTENVIRONMENT_H_ */
