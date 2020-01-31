#include "MotorController.h"

class BinaryMotorController : public MotorController {
  public:
    BinaryMotorController(unsigned int pin1, unsigned int pin2, bool pin1IsDirection);
    BinaryMotorController(unsigned int forwardsPin, unsigned int backwardsPin, unsigned int enablePin);

    virtual void set(double power);
};
