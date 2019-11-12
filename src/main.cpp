#include <Arduino.h>
#include <Servo.h>
#include "MotorController.h"

/* Nano PWM pin: 3, 5, 6, 9, 10, 11 */
#define ENGFWDPIN 11
#define ENGBCKPIN 10

#define RIGHTPIN	5
#define LEFTPIN	5

void setup() {
	Serial.begin(9600);
	pinMode(ENGFWDPIN, OUTPUT);
	Serial.println("ready");
}

float duty = 0.0;
float step = 0.07;
unsigned long lastms = 0;

void loop() {
	//if (millis() > lastms + 10) {
		duty+=step;
		if (duty >1 or duty <0) {
			step = -step;
			duty+=step;
		}
		analogWrite(ENGFWDPIN, duty * PWMRANGE);
		//lastms = millis();
	//}
}

