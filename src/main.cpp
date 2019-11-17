#include <Arduino.h>
#ifdef DEBUG
//Servo library for generating test signal
#include <Servo.h>
#endif

#include "MotorController.h"

//servo inputs
//number of input pins
#define INPINCNT sizeof(inPinNo)/sizeof(inPinNo[0])

#ifdef BOARD_NANO

//throttle
#define INPTHRPIN 2
//steering wheel
#define INPTURNPIN 3

//PWM outputs
/* Nano PWM pin: 3, 5, 6, 9, 10, 11 */
#define ENGFWDPIN 11
#define ENGBCKPIN 10
#define RIGHTPIN	6
#define LEFTPIN	5

#define BAUDRATE 9600

#ifdef DEBUG
//#define TESTOUTPIN 9
#endif

#endif

#ifdef BOARD_NODEMCU
/*
* nodemcu pins:
* D0 - gpio16
* D1 - gpio5
* D2 - gpio4
* D3 - gpio0
* D4 - gpio2
* D5 - gpio14
* D6 - gpio12
* D8 - gpio15
*/

//throttle
#define INPTHRPIN 14
//steering wheel
#define INPTURNPIN 12

//PWM outputs
#define ENGFWDPIN 5
#define ENGBCKPIN 4
#define RIGHTPIN	0
#define LEFTPIN	2

#define BAUDRATE 115200

#ifdef DEBUG
//#define TESTOUTPIN 15
#endif

#endif

#ifdef DEBUG
#define DEBUGOUT(msg) Serial.print(msg)
#define DEBUGNUM(num, base) Serial.print(num, base)
#else
#define DEBUGOUT(msg)
#define DEBUGNUM(num, base)
#endif

#ifdef TESTOUTPIN
Servo testout;
#endif

uint8_t inPinNo[] = {INPTHRPIN, INPTURNPIN};
int16_t inPulseTime[INPINCNT]; //length of the throttle pulse [us], -1 = invalid
unsigned long inPulseStart[INPINCNT]; //begin of pulse micros()
uint8_t inPinVal[INPINCNT];
struct Engines {
	MotorController* drive;
	MotorController* stwheel;
};
union Motors {
	Engines engines;
	MotorController* list[];
};

int16_t servoPulseMin=1000;
int16_t servoPulseMax=2000;
int16_t servoDeadZone=50;
int16_t servoTrim[INPINCNT]; //pulse time for neutral position

Motors motors;

void setup() {
	//configure input pins as inputs and read initial values
	for (uint8_t i=0; i<INPINCNT; i++) {
		pinMode(inPinNo[i], INPUT);
		inPinVal[i] = digitalRead(inPinNo[i]);
		inPulseTime[i] = -1;
		inPulseStart[i] = 0;
		servoTrim[i] = (servoPulseMax + servoPulseMin)/2;
	}
	motors.engines.drive = new MotorController(ENGFWDPIN, ENGBCKPIN, -1U);
	motors.engines.stwheel = new MotorController(LEFTPIN, RIGHTPIN, -1U);
#ifdef DEBUG
	Serial.begin(BAUDRATE);
#endif
	DEBUGOUT("\ndebug mode inputs:");
	DEBUGOUT(INPINCNT);
	DEBUGOUT("\ntrim:");
	DEBUGOUT(servoTrim[0]);
	DEBUGOUT(",");
	DEBUGOUT(servoTrim[1]);
	DEBUGOUT("\n");
#ifdef TESTOUTPIN
	pinMode(TESTOUTPIN, OUTPUT);
	testout.attach(TESTOUTPIN);
#endif
}


#ifdef TESTOUTPIN
float duty = 0.0;
float step = 0.05;
unsigned long lastms = 0;
unsigned long loopcnt = 0;
#endif

unsigned long nowus = 0;

void loop() {
	nowus = micros();
	for (uint8_t i=0; i<INPINCNT; i++) {
		uint8_t newPinVal = digitalRead(inPinNo[i]);
		if (newPinVal == HIGH) {
			if (inPinVal[i] == LOW) {
				//Low to High transition
				inPulseStart[i] = nowus;
			}
		} else {
			if (inPinVal[i] == HIGH) {
				//High to Low transition
				inPulseTime[i] = nowus - inPulseStart[i];
				inPulseStart[i] = nowus;
				if ((inPulseTime[i] > servoPulseMax) || (inPulseTime[i] < servoPulseMin)) {
					inPulseTime[i] = -1;
				} else {
					int engInt = inPulseTime[i] - servoTrim[i];
					float engVal = 0;
					if (abs(engInt) > servoDeadZone) {
						engVal = (float)(engInt) / (float)(servoPulseMax - servoPulseMin) *2.0 ;
					}
					
					motors.list[i]->set(engVal);
					DEBUGOUT(i);
					DEBUGOUT(" ");
					DEBUGOUT(inPulseTime[i]);
					DEBUGOUT("-");
					DEBUGOUT(servoTrim[i]);
					DEBUGOUT("=");
					DEBUGOUT(engVal);
					DEBUGOUT("\n");
				}
			} 
		}
		inPinVal[i] = newPinVal;
		if (inPulseStart[i] < nowus - 21000) {
			//now pulse in last 20ms period
			inPulseTime[i] = -1;
		}
	}
#ifdef TESTOUTPIN
	if (millis() >= lastms + 20) {
		loopcnt = 0;
		duty+=step;
		if (duty >1 or duty <0) {
			step = -step;
			duty+=step;
		}
		analogWrite(ENGFWDPIN, duty * PWMRANGE);
		testout.write(180*duty);
		lastms = millis();
	}
	loopcnt++;
#endif
}

