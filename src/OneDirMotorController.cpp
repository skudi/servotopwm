#include "OneDirMotorController.h"

OneDirMotorController::OneDirMotorController(unsigned int pin1) 
	: MotorController(pin1, pin1, false) {
}

OneDirMotorController::OneDirMotorController(unsigned int pin1, float cutOff)
	: MotorController(pin1, pin1, false) {
		this->cutOff = cutOff;
}

void OneDirMotorController::set(double power){
	power = (power + 1.0) / 2.0;
	if (power > this->cutOff) {
		power = this->cutOff;
	}
	MotorController::set(power);
}


