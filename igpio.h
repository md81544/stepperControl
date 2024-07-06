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
    virtual void setRotaryEncoderCallback(
        int pinA,
        int pinB,
        void (*callback)(int, int, uint32_t, void*),
        void* userData)
        = 0;
    virtual void setLinearScaleAxis1Callback(
        int pinA,
        int pinB,
        void (*callback)(int, int, uint32_t, void*),
        void* userData)
        = 0;
    // General:
    virtual void delayMicroSeconds(long) = 0;
    virtual uint32_t getTick() = 0;
    // These functions are for TESTING/FAKE only (to make the mocked linear scale appear to be
    // working) and should be a no-op on non-fake IGpio-derived classes:
    virtual void scaleGoToPosition(int32_t step) = 0;
    virtual void scaleSetSpeedStepsPerSec(double speed) = 0;
    virtual void scaleStop() = 0;
};

} // end namespace
