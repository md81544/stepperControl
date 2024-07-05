#ifndef FAKE

#include "gpio.h"

namespace mgo {

Gpio::Gpio()
{
    if (gpioInitialise() < 0) {
        throw GpioException("Could not initialise pigpio library");
    }
}

Gpio::~Gpio()
{
    if (m_pinA != 0) {
        gpioSetAlertFuncEx(m_pinA, nullptr, this);
    }
    if (m_pinB != 0) {
        gpioSetAlertFuncEx(m_pinB, nullptr, this);
    }
    gpioTerminate();
}

int Gpio::addMotor(int stepPin, int reversePin, int enablePin)
{
    m_stepPins.push_back(stepPin);
    m_reversePins.push_back(reversePin);
    m_enablePins.push_back(enablePin);
    return static_cast<int>(m_stepPins.size() - 1);
}

void Gpio::setStepPin(int motor, PinState state)
{
    gpioWrite(m_stepPins.at(motor), state == PinState::high ? 1 : 0);
}

void Gpio::setReversePin(int motor, PinState state)
{
    gpioWrite(m_reversePins.at(motor), state == PinState::high ? 1 : 0);
}

void Gpio::setEnablePin(int motor, PinState state)
{
    gpioWrite(m_enablePins.at(motor), state == PinState::high ? 1 : 0);
}

void Gpio::setRotaryEncoderCallback(
    int pinA,
    int pinB,
    void (*callback)(int, int, uint32_t, void*),
    void* user)
{
    m_pinA = pinA;
    m_pinB = pinB;
    // Set up callbacks for rotary encoder
    gpioSetMode(pinA, PI_INPUT);
    gpioSetMode(pinB, PI_INPUT);
    // pull up is needed as encoder common is grounded
    gpioSetPullUpDown(pinA, PI_PUD_UP);
    gpioSetPullUpDown(pinB, PI_PUD_UP);
    // monitor encoder level changes
    gpioSetAlertFuncEx(pinA, callback, user);
    gpioSetAlertFuncEx(pinB, callback, user);
}

void Gpio::delayMicroSeconds(long usecs)
{
    gpioDelay(usecs);
}

uint32_t Gpio::getTick()
{
    return gpioTick();
}

} // end namespace

#endif
