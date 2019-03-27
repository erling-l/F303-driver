/*
 * extEnvironment.cpp
 *
 *  Created on: 10 feb. 2019
 *      Author: erlin
 *      http://www.nada.kth.se/kurser/master/intro/libc/libc_7.html
 */
#include <QMC5883L.h>
#include "extEnvironment.h"
#include <malloc.h>
#include <stdbool.h>
#include <string.h>
char receivedChars[100];
int param1;
int param2;
int param3;
int firstObject = 0;
int lastObject = 0;
char parameterType[2];
extern uint8_t receivedParam[];
uint8_t *receivedParamPtr;
extern uint8_t *receivedParamStartPtr;
extern uint8_t *receivedParamEndPtr;
obstacle *obstacleList[40];
bool newData = false;
void readEnvironment();
void readWithStartEndMarkers();
void parseData(char *strPar);
int atoi();

void runExtEnvironment() {
	readWithStartEndMarkers();
	parseData((char *) &receivedParam);
	readEnvironment();
}
void digitalCompass(){
	QMC5883L_Initialize(MODE_CONTROL_CONTINUOUS, OUTPUT_DATA_RATE_200HZ, FULL_SCALE_2G, OVER_SAMPLE_RATIO_128);

	_qmc5883l_status status = QMC5883L_DataIsReady();
	uint8_t registerValue = QMC5883L_Read_Reg(QMC5883L_STATUS);
	QMC5883L_Read_Data(&X,&Y,&Z); // (-32768 / +32768)
	int16_t temperature = QMC5883L_Read_Temperature();
	printf("MagX %d MagY %d MagZ %d\n",X,Y,Z);
	printf("MagXmin %f MagYmin %f \n",Xmin,Ymin);
	printf("MagXmax %f MagYmax %f \n",Xmax,Ymax);
	printf("Temperature %d \n",temperature);
	printf("Status %d \n",status);
	printf("RegisterValue %d \n",registerValue);
}
void readEnvironment(){

if (parameterType[0] == 'D') {
	obstacleList[firstObject]->distance = param1;
	obstacleList[firstObject]->angle = param2;

//	firstObject += 1;
	if (firstObject >= 40){
//		firstObject =0;
	}
	if (firstObject >= lastObject){
//		lastObject +=1;
		if (lastObject >= 40){
//			lastObject = 0;
		}
	}
} else if (parameterType[0] == 'T'){

}

}
void readWithStartEndMarkers() {
	static bool recvInProgress = false;
	static int numChars;
	static int ndx = 0;
	char startMarker = '<';
	char endMarker = '>';
	char rc;

	if (receivedParamPtr <= receivedParamEndPtr) {
		rc = *receivedParamPtr++;

		if (recvInProgress == true) {
			if (rc != endMarker) {
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= numChars) {
					ndx = numChars - 1;
				}
			}
			else {
				receivedChars[ndx] = '\0'; // terminate the string
				recvInProgress = false;
				ndx = 0;
				newData = true;
			}
		}

		else if (rc == startMarker) {
			recvInProgress = true;
		}
	}
}
void parseData(char *strPar) {
	char str[100] = "<T,123,234,345,>";
	const char s[] = ",";
	char *token;
//	char parameterType[2];
	strcpy(str, strPar);
//	strcpy(str, (char *) receivedParam);
	parameterType[0] = str[0];
	/* get the first token */
	token = strtok(str, s);
	if (token != NULL){
		token = strtok(NULL, s);
		/* walk through other tokens */
		if( token != NULL ) {
			//			printf( " %s\n", token );
			param1 =atoi(token);
			printf( " %i\n", param1);
			token = strtok(NULL, s);
		}
		if( token != NULL ) {
			param2 =atoi(token);
			token = strtok(NULL, s);
		}
		if( token != NULL ) {
			param3 =atoi(token);
			token = strtok(NULL, s);
		}

	}

}
