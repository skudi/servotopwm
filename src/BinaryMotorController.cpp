#include "BinaryMotorController.h"

BinaryMotorController::BinaryMotorController(unsigned int pin1, unsigned int pin2, bool pin1IsDirection) 
	: MotorController(pin1, pin2, pin1IsDirection) {
}

BinaryMotorController::BinaryMotorController(unsigned int forwardsPin, unsigned int backwardsPin, unsigned int enablePin)
	: MotorController(forwardsPin, backwardsPin, enablePin) {
}

void BinaryMotorController::set(double power){
	if (power > 0) {
		MotorController::set(1.0);
	} else if (power < 0) {
		MotorController::set(-1.0);
	} else {
		MotorController::set(0.0);
	}
}

