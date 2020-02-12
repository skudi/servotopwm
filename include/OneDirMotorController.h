#include "MotorController.h"

class OneDirMotorController : public MotorController {
	protected:
		float cutOff = 1.0;
  public:
    OneDirMotorController(unsigned int pin1);
    OneDirMotorController(unsigned int pin1, float cutoff);

    virtual void set(double power);
};

