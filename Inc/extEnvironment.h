/*
 * extEnvironment.h
 *
 *  Created on: 10 feb. 2019
 *      Author: erlin
 */

#ifndef EXTENVIRONMENT_H_
#define EXTENVIRONMENT_H_
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
	long angleToLeftWall;
	long angleToRightWall;
};
struct car {
	long speed;
	long directionX;
	long directionY;
	long directionZ;
	_Bool left;
	_Bool right;
	_Bool front;
	_Bool back;

};
void runExtEnvironment(void);

#endif /* EXTENVIRONMENT_H_ */
