#pragma once

// Abstract base "interface" class
// to allow mocking out the hardware
// interface to allow the StepperMotor
// class to be used for tests
#include <stdint.h>

namespace mgo {

enum class PinState {
    high,
    low
};

// concrete classes can initialise / terminate
// the GPIO library in their ctors / dtors

class IGpio {
public:
    // Support for stepper motors:
    virtual int addMotor(int stepPin, int reversePin, int enablePin) = 0;
    virtual void setStepPin(int motor, PinState) = 0;
    virtual void setReversePin(int motor, PinState) = 0;
    virtual void setEnablePin(int motor, PinState) = 0;
    // Support for rotary encoder:
    virtual void setRotaryEncoderCallback(
        int pinA,
        int pinB,
        void (*callback)(int, int, uint32_t, void*),
        void* userData)
        = 0;
    // General:
    virtual void delayMicroSeconds(long) = 0;
    virtual uint32_t getTick() = 0;
};

} // end namespace
