#include <Arduino.h>
#ifdef DEBUG
//Servo library for generating test signal
#include <Servo.h>
#endif

#include "MotorController.h"
#include "OneDirMotorController.h"

//servo inputs
//number of input pins
#define INPINCNT sizeof(inPinNo)/sizeof(inPinNo[0])

#ifdef BOARD_NANO

//throttle
#define INPTHRPIN 2
//steering wheel
#define INPTURNPIN 3
//Lights
#define INPLIGHTPIN 4

//PWM outputs
/* PWM pins:
 * - Nano (atmega 328), Decimila (atmega 168), atmega8: 
 *   ||= analogWrite\\pin =|= m168p pin#/name =||
 *   || 3 | 25 / PC2 ||
 *   || 5 | 27 / PC4 ||
 *   || 6 | 28 / PC5 ||
 *   || 9 | 15 / PB1 ||
 *   || 10 | 16 / PB2 ||
 *   || 11 | 17 / PB3 ||
 */
#define ENGFWDPIN 11
#define ENGBCKPIN 10
#define RIGHTPIN	6
#define LEFTPIN	5
#define LIGHTPIN 9

#define BAUDRATE 9600

#ifdef DEBUG
//#define TESTOUTPIN 9
#endif

#endif

#ifdef BOARD_NODEMCU
/*
* nodemcu pins:
* D0 - gpio16 - output PWM Lights
* D1 - gpio5  - output PWM engine 
* D2 - gpio4  - output PWM engine
* D3 - gpio0  - output PWM steering
* D4 - gpio2  - output PWM steering
* D5 - gpio14 - input (servo pulses) throtle
* D6 - gpio12 - input (servo pulses) steering
* D7 - gpio13 - input (servo pulses) lights
* D8 - gpio15 - 
*/

//throttle
#define INPTHRPIN 14
//steering wheel
#define INPTURNPIN 12
//Lights
#define INPLIGHTPIN 13

//PWM outputs
#define ENGFWDPIN 5
#define ENGBCKPIN 4
#define RIGHTPIN	0
#define LEFTPIN	2
#define LIGHTPIN 16

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

uint8_t inPinNo[] = {INPTHRPIN, INPTURNPIN, INPLIGHTPIN};
int16_t inPulseTime[INPINCNT]; //length of the throttle pulse [us], -1 = invalid
unsigned long inPulseStart[INPINCNT]; //begin of pulse micros()
uint8_t inPinVal[INPINCNT];

MotorController* motors[3];

int16_t servoPulseMin=1000;
int16_t servoPulseMax=2000;
int16_t servoDeadZone=50;
int16_t servoTrim[INPINCNT]; //pulse time for neutral position

void setup() {
	//configure input pins as inputs and read initial values
	for (uint8_t pinIdx=0; pinIdx<INPINCNT; pinIdx++) {
		pinMode(inPinNo[pinIdx], INPUT);
		inPinVal[pinIdx] = digitalRead(inPinNo[pinIdx]);
		inPulseTime[pinIdx] = -1;
		inPulseStart[pinIdx] = 0;
		servoTrim[pinIdx] = (servoPulseMax + servoPulseMin)/2;
	}
	motors[0] = new MotorController(ENGFWDPIN, ENGBCKPIN, -1U);
	motors[1] = new MotorController(LEFTPIN, RIGHTPIN, -1U);
	motors[2] = new OneDirMotorController(LIGHTPIN, 0.8);
#ifdef DEBUG
	Serial.begin(BAUDRATE);
#endif
	DEBUGOUT("debug mode inputs:");
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
	DEBUGOUT("t ");
	DEBUGOUT(nowus);
	DEBUGOUT("\n");
	nowus = micros();
	for (uint8_t pinIdx=0; pinIdx<INPINCNT; pinIdx++) {
		uint8_t newPinVal = digitalRead(inPinNo[pinIdx]);
		if (newPinVal == HIGH) {
			if (inPinVal[pinIdx] == LOW) {
				//Low to High transition
				inPulseStart[pinIdx] = nowus;
				DEBUGOUT(pinIdx);
				DEBUGOUT(" . ");
				DEBUGOUT(inPulseStart[pinIdx]);
				DEBUGOUT("\n");
			}
		} else {
			if (inPinVal[pinIdx] == HIGH) {
				//High to Low transition
				inPulseTime[pinIdx] = nowus - inPulseStart[pinIdx];
				DEBUGOUT(pinIdx);
				DEBUGOUT(" _ ");
				DEBUGOUT(inPulseStart[pinIdx]);
				DEBUGOUT(" ");
				DEBUGOUT(inPulseTime[pinIdx]);
				inPulseStart[pinIdx] = nowus;
				//DEBUGOUT("-");
				//DEBUGOUT(servoTrim[pinIdx]);
				//DEBUGOUT("=");
				//DEBUGOUT(engVal);
				DEBUGOUT("\n");
				if ((inPulseTime[pinIdx] > servoPulseMax) || (inPulseTime[pinIdx] < servoPulseMin)) {
					inPulseTime[pinIdx] = -1;
				} else {
					int engInt = inPulseTime[pinIdx] - servoTrim[pinIdx];
					float engVal = 0;
					if (abs(engInt) > servoDeadZone) {
						engVal = (float)(engInt) / (float)(servoPulseMax - servoPulseMin) *2.0 ;
					}
					
					motors[pinIdx]->set(engVal);
				}
			} 
		}
		inPinVal[pinIdx] = newPinVal;
		if (inPulseStart[pinIdx] < nowus - 21000) {
			//now pulse in last 20ms period
			inPulseTime[pinIdx] = -1;
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

