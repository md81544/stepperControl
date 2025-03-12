#pragma once

// Concrete implementation of the IGpio ABC

#include "igpio.h"
#include "pigpio.h"

#include <stdexcept>
#include <vector>

namespace mgo {

class GpioException : public std::runtime_error {
public:
    explicit GpioException(const std::string& message)
        : std::runtime_error(message)
    {
    }
};

class Gpio final : public IGpio {
public:
    Gpio();
    virtual ~Gpio();
    int addMotor(int stepPin, int reversePin, int enablePin) override;
    void setStepPin(int motor, PinState) override;
    void setReversePin(int motor, PinState) override;
    void setEnablePin(int motor, PinState) override;
    void setRotaryEncoderCallback(
        int pinA,
        int pinB,
        void (*callback)(int, int, uint32_t, void*),
        void* userData) override;
    void setLinearScaleAxis1Callback(
        int pinA,
        int pinB,
        void (*callback)(int, int, uint32_t, void*),
        void* userData) override;
    void delayMicroSeconds(long usecs) override;
    uint32_t getTick() override;

private:
    // The following scale* functions are deliberately no-op as they are only used in the
    // mock gpio class to simulate the scale moving.
    void scaleGoToPositionMm(double) override {};
    void scaleSetSpeedStepsPerSec(double) override {};
    void scaleStop() override {};
    void
    setCallback(int pinA, int pinB, void (*callback)(int, int, uint32_t, void*), void* userData);
    std::vector<int> m_stepPins;
    std::vector<int> m_reversePins;
    std::vector<int> m_enablePins;
    int m_pinA { 0 };
    int m_pinB { 0 };
};

} // end namespace
