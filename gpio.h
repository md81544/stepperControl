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
    void setEncoderCallback(
        int pinA,
        int pinB,
        void (*callback)(int, int, uint32_t, void*),
        void* userData) override;
    void delayMicroSeconds(long usecs) override;
    uint32_t getTick() override;

private:
    std::vector<int> m_stepPins;
    std::vector<int> m_reversePins;
    std::vector<int> m_enablePins;
    int m_pinA { 0 };
    int m_pinB { 0 };
};

} // end namespace
