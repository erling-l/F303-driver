/*
 * 
 * 
 * ” Car control”
 * Implementerad på en Arduino Uno
 * Utveckla en app som autonomt kan köra en liten 4hjulig bil i ett rum och undvika alla hinder
 * i dess väg, samt att kunna köra en rundbana känna igen banan och optimera kommande varv på banan.
 * Fas 1: att genom kommandon över bluetooth starta stoppa backa och styra höger eller
 * vänster med variabel hastighet och variabelt styrutslag.
 * Fas 2: Integrera avståndssensorer(1-6) i kontrolloopen så bilen stannar eller väjer för hinder.
 * Fas 3: Integrera kompass och tachometer så att riktning och disans loggas
 * Fas 5: Integrera en 2:a processor med större beräkningskapacitet och minne.
 * Fas 6: Utveckla algoritmer för att välja väg baserat på Robot SM:s regler för
 * Folkrace Standard max vikt 1000 gram storlek bredd x längd x höjd 150 x 200 x 150
 * ej (1:87 1000 30 x 276 x 52* )
 * Roboten skall kunna ta sig runt banan på mindre än 3 minuter.
 * Banans underlag är mestadels matt svart. Sargerna är vita och c:a 80% av klassens maxhöjd.
 * Banans bredd varierar, men är håller generellt en bredd på 5*(klassens bredd). Dock alltid
 * minst 3*(klassens bredd).Banan kan ha enklare hinder, t.ex. små kullar och pelare vid sargen.
 * Pelarnas bredd är minst halva klassens bredd. Färger på banan kan även skilja sig på vissa
 * ställen. Ju mindre klassen är desto snällare kommer banan att vara vad det gäller kullar och
 * markfrigång. Kullarnas maximala lutning är 25° och höjd maximalt 1.5*(klassens höjd). Minsta
 * rekommenderade markfrigång är 10% av klassens höjd.
 * Det finns två markeringar på banan (en startlinje och en mittlinje). Färgen på är
 * ospecificerad och kan variera.
 * innerradie min 75 mm ytterradie 875 mm
 * 2018-07-06 Fas 1 Klar
 * Testat styrning via bluetooth fungerar ok vändradie ca 100 cm (behöver halveras)
 * motorstyrning fungerar dock lite ryckig behver förbättras
 * Android app fungerar men krashar behöver rättas
 * 2019-03-22
 * Adderat kraftfullare processor STM32F303RE och en kompass QMS5883L 
 * och integrerat kommunikation
 * mellan processorerna.
 * 2019-03-28
 * Anslutit blutooth modul för debugging.Skrivit om readspeed så att den 
 * förhoppningsvis fungerar.
 * 2019-03-29
 * Debuggat kompassen ej fått den att fungera. verkar finnas på I2C bussen men får inga data från den.
 * Städat upp car_control och börjat skriva på driver.
 * 
 */
#include <OneWire.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
//int minPos = 88;   // variable to store the servo min position 0 - 179
//int maxPos = 89; // variable to store the servo max position  1 - 180
int minPos = 0;   // variable to store the servo min position 0 - 179
int maxPos = 180; // variable to store the servo max position  1 - 180
int pos = minPos;      // variable to store the servo position
int servoStep = 2;      // variable to store the servo step size 1 - 179
boolean servoUp;    // boolean to store the servo direction

// constants won't change. Used here to set a pin number :
const int backwardPin = 11;     // Bakåt Hög signal för bakåt OC2A
const int forwardPin =  3;     // Framåt Hög signal för framåt OC2B
const int rightPin =    4;     // Höger Hög signal för höger
const int leftPin =     5;     // Vänster Hög signal för vänster
const int selForwardRightPin =    6;     // Select VL53L0x fram höger
const int selForwardLeftPin =     7;     // Select VL53L0x fram vänster
const int selBackwardRightPin =   8;     // Select VL53L0x bak höger
const int selBackwardLeftPin =    9;     // Select VL53L0x bak vänster
const int servoPin =             10;     // VL53L0x Servocontrol +- 45 degrees forward
const int tachoPinA             = 2;     // Taco pulses 16 pulser/varv ca 8,8 mm/puls från 
                                         // vänster framhjul
const int tachoPinB             = 12;    // Taco pulses 16 pulser/varv ca 8,8 mm/puls från 
                                         // vänster framhjul
const int startPin = 13;                 // Enable start when high
const int compassXPin =     A0;     // Compass x analog value
const int compassYPin =     A1;     // Compass y analog
const int statusLedPin =    A2;     // Status led
const int LedSteady =       1;     // Status led lit
const int LedFlasingFast =  2;     // Status led blinkar snabbt
const int LedFlashingSlow = 3;     // Status led blinkar långsamt
const long interval1 = 2;           // interval at which to update motor(milliseconds)
const long intervalDistanceMeasurment = 60;  // interval at which to upate distance reading 
                                              // (milliseconds)min 30
const long intervalServoControl = 30;

// Variables will change :
int carSpeed = 0;             // Backwards or Forward speed -127 to 128
int steering = 0;             // Steering angle -127 to 128
int speedProcent = 0;
int steerProcent = 0;
int compassXvalue = 0;
int compassYvalue = 0;
int compass = 0;             // 0 to 359 degrees where 0 is travelling north
int rawCompass = 0;
int upLimit = 0;
int a_Curve = 0;
int b_Curve = 0;
int lowLimit = 0;
int crossZero = 0;
int compDisp = 0;
int spread = 0;
int scale = 0;
int oldSensorValueA = 0;
int oldSensorValueB = 0;
int oldAB = 0;
int sensorCount = 0; // Up equals forward down equals backward

//char commandMotor = 's';
//char commandSteering = 't';
bool startstate = false;


// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillisPower = 0;        // will store last time speed
unsigned long previousMillisDistance = 0;        // will store last time distance
unsigned long previousMillisSpeed = 0;        // will store last time distance
unsigned long previousMillisSteer = 0;    // will store last time steer was updated
unsigned long previousMillisServo = 0;

const byte numChars = 32;
int lf = 10;
int cr = 13;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
char messageFromPC[numChars] = {"s"};
int integerFromPC = 0;
int integer2FromPC = 0;
int integer3FromPC = 1;
int integer4FromPC = 179;

boolean newData = false;


void setup() {
  // put your setup code here, to run once:

  pinMode(tachoPinA, INPUT_PULLUP);
  pinMode(tachoPinB, INPUT_PULLUP);
  pinMode(startPin, INPUT);
  //  pinMode(compassXPin, INPUT);
  //  pinMode(compassYPin, INPUT);
  pinMode(servoPin, OUTPUT);

  digitalWrite(forwardPin, LOW);
  digitalWrite(backwardPin, LOW);
  digitalWrite(rightPin, LOW);
  digitalWrite(leftPin, LOW);
  digitalWrite(selForwardRightPin, HIGH);
  digitalWrite(selForwardLeftPin, LOW);
  digitalWrite(selBackwardRightPin, LOW);
  digitalWrite(selBackwardLeftPin, LOW);
  digitalWrite(statusLedPin, LOW);           // Led släckt
  digitalWrite(tachoPinA, HIGH);
  digitalWrite(tachoPinB, HIGH);
  digitalWrite(startPin, LOW);

  pinMode(forwardPin, OUTPUT);
  pinMode(backwardPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  pinMode(selForwardRightPin, OUTPUT);
  pinMode(selForwardLeftPin, OUTPUT);
  pinMode(selBackwardRightPin, OUTPUT);
  pinMode(selBackwardLeftPin, OUTPUT);
  pinMode(statusLedPin, OUTPUT);

  if (digitalRead(startPin) == LOW) {
    startstate = false;
  } else {
    startstate = true;
  }

  Serial.begin(9600);         // Connected to Bluetooth module HC-06 with a levelshifter
  // to convert 5v out from Arduino to 3,3 v in on HC-06
  // AT+BAUD4 equals 9600 Baud(default for HC 06) Can be set to 115200 with AT+BAUD8
  if (Serial.available()) {
    Serial.read();
  }
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
//  Serial.println("Car_control Version 0.0.1 2019-02-15 ");
//  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
//    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // power
//  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));

  myservo.attach(servoPin);  // attaches the servo on servoPin to the servo object
  //  callibrateCompass();
}

//============

void loop() {
  // put your main code here, to run repeatedly:
  // command interpreter
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
//    showParsedData();
    newData = false;
  }
  // command interpreter
  if (digitalRead(startPin) == HIGH) { // do nothing if not started
    startstate = true;
    carSpeed = integerFromPC;
    steering = integer2FromPC;
    minPos = integer3FromPC;
    maxPos = integer4FromPC;
    motor(carSpeed);
    steer(steering);
    readDistance(1);
    controlServo();
    readSpeed(0);
  }
}
void lampOn(int  pinNumber) {
  digitalWrite(pinNumber, HIGH);
}
void lampOff(int pinNumber) {
  digitalWrite(pinNumber, LOW);
}
void motor(int localSpeed) {
  unsigned long currentMillis = millis();
  //  char test;
  if (currentMillis - previousMillisPower >= interval1) {
    // save the last time you checked
    previousMillisPower = currentMillis;
    if (speedProcent <= 100) {
      speedProcent += 1;
    } else {
      speedProcent = 0;
    }
    // if localSpeed is 0 do nothing if positive activate motor forward if negative motor backwards
    if (localSpeed < 0) {
      if (localSpeed + speedProcent < 0) {
        digitalWrite(forwardPin, LOW);
        delayMicroseconds(1000);
        digitalWrite(backwardPin, HIGH);
        //      Serial.print(millis());
        //      Serial.print("   ");
        //      Serial.println(previousMillisPower);
      } else {
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, LOW);

      }
    } else if ( localSpeed > 0 ) {
      if (localSpeed - speedProcent > 0) {
        digitalWrite(backwardPin, LOW);
        delayMicroseconds(1000);
        digitalWrite(forwardPin, HIGH);
      } else {
        digitalWrite(backwardPin, LOW);
        digitalWrite(forwardPin, LOW);
      }
    }  else {
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, LOW);
    }
  }
}
void steer(int localSteer) {
  unsigned long currentMillis = millis();
  //  char test;
  if (currentMillis - previousMillisSteer >= interval1) {
    // save the last time you checked
    previousMillisSteer = currentMillis;
    if (speedProcent <= 100) {
      speedProcent += 1;
    } else {
      speedProcent = 0;
    }    if (localSteer < 0) {
      if (localSteer + steerProcent < 0) {
        digitalWrite(leftPin, LOW);
        delayMicroseconds(1000);
        digitalWrite(rightPin, HIGH);
      }  else {
        digitalWrite(leftPin, LOW);
        digitalWrite(rightPin, LOW);
      }
    } else if ( localSteer > 0 ) {
      if (localSteer - steerProcent > 0) {
        digitalWrite(rightPin, LOW);
        delayMicroseconds(1000);
        digitalWrite(leftPin, HIGH);
      } else {
        digitalWrite(leftPin, LOW);
        digitalWrite(rightPin, LOW);
      }
    } else {
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
    }
  }
}


void readDistance(int sensor) { // Read distance from sensor
  unsigned long currentMillis = millis();
  digitalWrite(selForwardRightPin, HIGH); // Sensor 1
  if (currentMillis - previousMillisDistance >= intervalDistanceMeasurment) {
    // save the last time distance was read
    previousMillisDistance = currentMillis;
    VL53L0X_RangingMeasurementData_t measure;

    //Serial.print("Read meas.. ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      //    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
      Serial.print("<D,"); Serial.print(measure.RangeMilliMeter); Serial.print(","); 
      Serial.print(pos); Serial.print(","); Serial.print(servoUp); Serial.println(",>");
    } else {
      //    Serial.println(" out of range ");
      Serial.print("<D,700,"); Serial.print(pos); Serial.print(","); Serial.print(servoUp); 
      Serial.println(",>");
    }

  }
}
/*
 * Read speed 
 * 2 phase signal
 * phase A   ___|---|______|---|___
 * phase B   -|___|---|__|---|___|---
 * Ahead     -----------
 * Back                 ------------
 * AB transitions forward 01,00,10,11,01 etc
 * variable AB =           1  0  2  3
 * AB transitions back    01,11,10,00,01 etc
 * variable AB =           1  3  2  0
 */
int readSpeed(int  pinNumber) { // Reads weel tachometer
  int sensorValueA = digitalRead(tachoPinA);
  int sensorValueB = digitalRead(tachoPinB);
  switch (oldAB) {
  case 0: {
      if ( sensorValueA == 1) {
          oldAB = 2;
          sensorCount += 1;
        }
        if ( sensorValueA == 1) {
          oldAB = 1;
          sensorCount -= 1;
        }
      }
        break;
    case 1: {
        if ( sensorValueA == 1) {
          oldAB = 3;
          sensorCount -= 1;
        }
        if ( sensorValueB == 0) {
          oldAB = 0;
          sensorCount += 1;
        }
      }
        break;
    case 2: {
        if ( sensorValueA == 0) {
          oldAB = 0;
          sensorCount -= 1;
        }
        if ( sensorValueA == 1) {
          oldAB = 3;
          sensorCount += 1;
        }
      }
        break;
    case 3: {
        if ( sensorValueA == 0) {
          oldAB = 1;
          sensorCount += 1;
        }
        if ( sensorValueB == 0) {
          oldAB = 2;
          sensorCount -= 1;
        }
      }
        break;
  }
/*
 * Send values
 */
   unsigned long currentMillis = millis();
  if (currentMillis - previousMillisSpeed >= intervalDistanceMeasurment) {
    // save the last time distance was read
    previousMillisSpeed = currentMillis;
  Serial.print("<");
  Serial.print("S,");    // Speed
  Serial.print(sensorValueA);
  Serial.print(",");
  Serial.print(sensorValueB);
  Serial.print(",");
  Serial.print(sensorCount);
  Serial.print(",");
  Serial.print(">");
  return sensorCount;
  }
}

//============
/*
 * Receive with end markers
 */
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

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

//============
/*
 * Parse data
 */
void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  integerFromPC = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ",");       // this continues where the previous call left off
  integer2FromPC = atoi(strtokIndx);    // convert this part to a integer
  strtokIndx = strtok(NULL, ",");       // this continues where the previous call left off
  integer3FromPC = atoi(strtokIndx);    // convert this part to a integer
  strtokIndx = strtok(NULL, ",");       // this continues where the previous call left off
  integer4FromPC = atoi(strtokIndx);    // convert this part to a integer

}

//============

void showParsedData() {
  //    Serial.print("Message ");
  //    Serial.println(messageFromPC);
  Serial.print(" Speed ");
  Serial.print(integerFromPC);
  //    Serial.write(cr);
  //    Serial.write(lf);
  Serial.print(" Steer ");
  Serial.println(integer2FromPC);
}
/*
 * Control servo
 */
void controlServo() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisServo >= intervalServoControl) {
    // save the last time servo was read
    previousMillisServo = currentMillis;
    if (servoUp) {
      if (pos <= maxPos) {
        pos += servoStep;           // goes from minPos degrees to maxPos degrees
        // in steps of 1- 179 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        // waits 15ms for the servo to reach the position
      } else {
        servoUp = false;
      }
    } else {
      if (pos > minPos) {
        pos -= servoStep;           // goes from minPos degrees to maxPos degrees
        // in steps of 1 - 179 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        // waits 15ms for the servo to reach the position
      } else {
        servoUp = true;
      }
    }
  }
}
