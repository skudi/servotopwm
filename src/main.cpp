#include <Arduino.h>

//define pin for test output for test signal generator
//#define TESTOUTPIN 15

#ifdef TESTOUTPIN
//Servo library for generating test signal
#include <Servo.h>
#endif

#include "MotorControllerInterface.h"
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

#endif //BOARD_NANO

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

MotorControllerInterface* motors[3];

int16_t servoPulseMin=1000;
int16_t servoPulseMax=2000;
int16_t servoDeadZone=50;
int16_t servoTrim[INPINCNT]; //pulse time for neutral position

void setup() {
	Serial.begin(BAUDRATE);
	Serial.print("\n\ninitialize\n\nmode inputs:");
	Serial.print(INPINCNT);
	Serial.print("\n");
	//configure input pins as inputs and read initial values
	for (uint8_t pinIdx=0; pinIdx<INPINCNT; pinIdx++) {
		pinMode(inPinNo[pinIdx], INPUT);
		inPinVal[pinIdx] = digitalRead(inPinNo[pinIdx]);
		inPulseTime[pinIdx] = -1;
		inPulseStart[pinIdx] = 0;
		servoTrim[pinIdx] = (servoPulseMax + servoPulseMin)/2;
	}
	motors[0] = new MotorController(ENGFWDPIN, ENGBCKPIN, false);
	motors[1] = new MotorController(LEFTPIN, RIGHTPIN, false);
	motors[2] = new OneDirMotorController(LIGHTPIN);
	Serial.print("Num, Digital.write, trim\n");
	for (uint8_t pinIdx=0; pinIdx<INPINCNT; pinIdx++) {
		Serial.print(pinIdx);
		Serial.print(", ");
		Serial.print(inPinNo[pinIdx]);
		Serial.print(", ");
		Serial.print(servoTrim[pinIdx]);
		Serial.print("\n");
	}
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
	for (uint8_t pinIdx=0; pinIdx<INPINCNT; pinIdx++) {
		uint8_t newPinVal = digitalRead(inPinNo[pinIdx]);
		if (newPinVal == HIGH) {
			if (inPinVal[pinIdx] == LOW) {
				//Low to High transition
				inPulseStart[pinIdx] = nowus;
				DEBUGOUT("\n");
				DEBUGOUT(pinIdx);
				DEBUGOUT(" . ");
				DEBUGOUT(inPulseStart[pinIdx]);
			}
		} else {
			if (inPinVal[pinIdx] == HIGH) {
				//High to Low transition
				inPulseTime[pinIdx] = nowus - inPulseStart[pinIdx];
				DEBUGOUT("\n");
				DEBUGOUT(pinIdx);
				DEBUGOUT(" _ ");
				DEBUGOUT(nowus);
				DEBUGOUT(" ");
				DEBUGOUT(inPulseTime[pinIdx]);
				inPulseStart[pinIdx] = nowus;
				//DEBUGOUT("-");
				//DEBUGOUT(servoTrim[pinIdx]);
				//DEBUGOUT("=");
				//DEBUGOUT(engVal);
				if ((inPulseTime[pinIdx] > 2700) || (inPulseTime[pinIdx] < 300)) {
					inPulseTime[pinIdx] = -1;
				} else {
					int engInt = inPulseTime[pinIdx] - servoTrim[pinIdx];
					float engVal = 0;
					if (abs(engInt) > servoDeadZone) {
						engVal = (float)(engInt) / (float)(servoPulseMax - servoPulseMin) *2.0 ;
					}
					DEBUGOUT("\n");
					DEBUGOUT(pinIdx);
					DEBUGOUT(" s ");
					DEBUGOUT(engVal);
					DEBUGOUT("\n");
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
		testout.write(180*duty);
		lastms = millis();
	}
	loopcnt++;
#endif
}

