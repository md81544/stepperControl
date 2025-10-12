#include "steppermotor.h"

#include "igpio.h"

#include <cmath>
#include <cstdint>
#include <pthread.h>

namespace mgo {

StepperMotor::StepperMotor(
    IGpio& gpio,
    int stepPin,
    int reversePin,
    int enablePin,
    long stepsPerRevolution,
    double conversionFactor,
    double maxRpm,
    double rampingSpeed,
    bool usingMockLinearScale, /* = true */
    uint32_t mockLinearScaleStepsPerMm /* = 200 */)
    : m_gpio(gpio)
    , m_stepsPerRevolution(stepsPerRevolution)
    , m_conversionFactor(conversionFactor)
    , m_maxRpm(maxRpm)
    , m_rampingSpeed(rampingSpeed / 1000.0)
    , m_usingMockLinearScale(usingMockLinearScale)
    , m_mockLinearScaleStepsPerMm(mockLinearScaleStepsPerMm)

{
    if (m_rampingSpeed < 0.0) {
        m_rampingSpeed = 0.0;
    }
    if (m_rampingSpeed > 0.1) {
        m_rampingSpeed = 0.1;
    }
    if (m_rampingSpeed == 0.0) {
        m_useRamping = false;
    }
    m_motorNumber = m_gpio.addMotor(stepPin, reversePin, enablePin);
    // Ensure we start off with the right direction
    m_gpio.setReversePin(
        m_motorNumber, m_direction == Direction::forward ? PinState::low : PinState::high);

    // Start the thread
    std::thread t([&]() {
        Direction oldDirection = Direction::forward;
        for (;;) {
            int delay;
            uint32_t startTick = m_gpio.getTick();
            { // scope for lock_guard
                // When in this scope we can assume all member
                // variables can be written and read from freely
                // without them being changed externally. Other
                // member functions must lock the mutex m_mtx
                // before changing any member variables.
                std::lock_guard<std::mutex> mtx(m_mtx);
                if (m_terminateThread) {
                    break;
                }
                if (m_stop || m_rpm == 0.0) {
                    m_targetStep = long(m_currentReportedStep);
                    m_stop = false;
                    m_busy = false;
                } else if (!m_busy && m_synchronise && m_synchroniseMotor) {
                    synchronise();
                }
                if (m_targetStep != m_currentReportedStep) {
                    if (oldDirection != m_direction) {
                        // Ensure direction pin is set correctly
                        m_gpio.setReversePin(
                            m_motorNumber,
                            m_direction == Direction::forward ? PinState::low : PinState::high);
                        oldDirection = m_direction;
                    }
                    // Do step pulse
                    m_gpio.setStepPin(m_motorNumber, PinState::high);
                    // Spin for first delay amount:
                    while (m_gpio.getTick() - startTick < static_cast<uint32_t>(m_delay))
                        ;
                    m_gpio.setStepPin(m_motorNumber, PinState::low);
                    if (m_targetStep < m_currentReportedStep) {
                        // we only start counting steps once the slack of the backlash
                        // compensation has been taken up
                        if (m_backlashPosition == 0) {
                            --m_currentReportedStep;
                        } else {
                            --m_backlashPosition;
                        }
                        --m_currentActualStep; // This is only used for testing
                    } else {
                        if (m_backlashPosition == m_backlashSize) {
                            ++m_currentReportedStep;
                        } else {
                            ++m_backlashPosition;
                        }
                        ++m_currentActualStep; // This is only used for testing
                    }
                    if (m_currentReportedStep == m_targetStep) {
                        m_busy = false;
                    }
                }
                delay = m_delay; // remember delay value while in mtx scope
            } // scope for lock_guard

            if (m_busy && m_useRamping && m_rampedRpm < m_rpm) {
                m_rampedRpm += m_rampingSpeed;
                if (m_rampedRpm > m_rpm) {
                    m_rampedRpm = m_rpm;
                }
                m_delay = calculateDelayValue(m_rampedRpm);
            }

            // We always perform the second delay regardless of
            // whether we're stepping, to give the main thread a
            // chance to grab the mutex if needed

            // Spin until we get to the right time
            while (m_gpio.getTick() - startTick < static_cast<uint32_t>(2 * delay))
                ;
        }
    } // thread end
    );
    // Store the thread's handle in m_thread
    m_thread.swap(t);
    // Set the thread's scheduling to be real-time; cannot do this portably:
    sched_param schedParams;
    schedParams.sched_priority = ::sched_get_priority_max(SCHED_RR);
    ::pthread_setschedparam(m_thread.native_handle(), SCHED_RR, &schedParams);
}

StepperMotor::~StepperMotor()
{
    m_terminateThread = true;
    m_thread.join();
}

void StepperMotor::goToStep(long step, bool noLock)
{
    if (!noLock) {
        std::lock_guard<std::mutex> mtx(m_mtx);
    }
    if (m_busy) {
        // We ignore any request to go to a location if
        // we are already stepping. The client code can
        // issue a stop if needed before changing target
        // step location.
        return;
    }
    if (m_currentReportedStep == step) {
        return;
    }
    m_direction = Direction::forward;
    if (step < m_currentReportedStep) {
        m_direction = Direction::reverse;
    }
    m_busy = true;
    m_stop = false;
    m_targetStep = step;
    if (m_usingMockLinearScale) {
#ifdef FAKE
        // This is solely to drive the mock linear scale in step with this motor
        m_gpio.scaleGoToPositionMm(step * m_conversionFactor);
        double motorSpeedMmPerSec = getSpeed() / 60.0;
        // Say we are moving at 0.5 mm/sec then steps per sec should be 200 * 0.5
        // So motorSpeed * stepsPerMm.
        m_gpio.scaleSetSpeedStepsPerSec(motorSpeedMmPerSec * m_mockLinearScaleStepsPerMm);
#endif
    }
}

void StepperMotor::stop()
{
    {
        std::lock_guard<std::mutex> mtx(m_mtx);
        m_stop = true;
    }
    // Wait before returning to allow the stop to be acted on
    // by the stepper thread
    m_gpio.delayMicroSeconds(50'000);
}

void StepperMotor::setRpm(double rpm, bool noLock)
{
    if (rpm < 0) {
        rpm = 0;
    }
    if (rpm > m_maxRpm) {
        rpm = m_maxRpm;
    }
    // NB m_delay (in µsecs) is used TWICE per thread loop
    if (!noLock) {
        std::lock_guard<std::mutex> mtx(m_mtx);
    }
    if (rpm < 0.00001) {
        // Arbitrarily anything lower than this
        // and we assune zero was required
        m_rpm = 0;
        m_rampedRpm = 0;
        m_stop = true;
        return;
    }
    if (m_useRamping && rpm > m_rpm) {
        // We don't start ramping from zero
        m_rampedRpm = m_maxRpm * 0.4;
        if (m_rampedRpm > rpm) {
            m_rampedRpm = rpm;
        }
    }
    m_rpm = rpm;
    m_delay = calculateDelayValue(m_rpm);
#ifdef FAKE
    double motorSpeedMmPerSec = getSpeed() / 60.0;
    m_gpio.scaleSetSpeedStepsPerSec(motorSpeedMmPerSec * m_mockLinearScaleStepsPerMm);
#endif
}

void StepperMotor::setSpeed(double speed, bool noLock)
{
    const double rpm = std::abs(speed / m_conversionFactor / m_stepsPerRevolution);
    setRpm(rpm, noLock);
}

double StepperMotor::getRpm()
{
    return m_rpm;
}

double StepperMotor::getSpeed() const
{
    return std::abs(m_rpm * m_stepsPerRevolution * m_conversionFactor);
}

double StepperMotor::getMaxRpm()
{
    return m_maxRpm;
}

int StepperMotor::getDelay()
{
    std::lock_guard<std::mutex> mtx(m_mtx);
    return m_delay;
}

long StepperMotor::getCurrentStep() const
{
    return m_currentReportedStep;
}

long StepperMotor::getTargetStep() const
{
    return m_targetStep;
}

void StepperMotor::zeroPosition()
{
    {
        std::lock_guard<std::mutex> mtx(m_mtx);
        m_currentReportedStep = 0L;
        m_targetStep = 0L;
    }
    stop();
}

void StepperMotor::wait()
{
    while (m_busy) {
        m_gpio.delayMicroSeconds(10'000);
    }
}

bool StepperMotor::isRunning() const
{
    return m_busy;
}

Direction StepperMotor::getDirection() const
{
    return m_direction;
}

double StepperMotor::getPosition() const
{
    return m_currentReportedStep * m_conversionFactor;
}

double StepperMotor::getPosition(long step) const
{
    return step * m_conversionFactor;
}

void StepperMotor::goToPosition(double mm, bool noLock)
{
    goToStep(mm / m_conversionFactor, noLock);
}

void StepperMotor::setPosition(double mm)
{
    {
        std::lock_guard<std::mutex> mtx(m_mtx);
        long step = mm / m_conversionFactor;
        m_currentReportedStep = step;
        m_targetStep = step;
    }
    stop();
}

void StepperMotor::setBacklashCompensation(unsigned int compensation, unsigned int currentPosition)
{
    std::lock_guard<std::mutex> mtx(m_mtx);
    m_backlashSize = compensation;
    if (currentPosition > compensation) {
        m_backlashPosition = compensation;
    } else {
        m_backlashPosition = currentPosition;
    }
}

long StepperMotor::getCurrentStepWithoutBacklashCompensation() const
{
    return m_currentActualStep;
}

double StepperMotor::getRampedSpeed()
{
    return std::abs(m_rampedRpm * m_stepsPerRevolution * m_conversionFactor);
}

double StepperMotor::getConversionFactor() const
{
    return m_conversionFactor;
}

double StepperMotor::calculateDelayValue(double rpm)
{
    int delay = std::round(500'000.0 / (static_cast<double>(m_stepsPerRevolution) * (rpm / 60.0)));
    if (delay < 10) {
        // Avoid very small delay values as the stepper
        // motor won't be able to keep up
        delay = 10;
    }
    return delay;
}

void StepperMotor::enableRamping(bool flag)
{
    std::lock_guard<std::mutex> mtx(m_mtx);
    if (m_rampingSpeed != 0.0) {
        m_useRamping = flag;
    }
}

void StepperMotor::synchroniseOn(
    const StepperMotor* other,
    std::function<double(double, double)> func,
    bool useZeroAsSyncStartPos // = false
)
{
    std::lock_guard<std::mutex> mtx(m_mtx);
    m_synchronise = true;
    m_synchroniseMotor = other;
    m_synchroniseFunction = func;
    m_useZeroAsSyncStartPos = useZeroAsSyncStartPos;
}

void StepperMotor::synchroniseOff()
{
    std::lock_guard<std::mutex> mtx(m_mtx);
    m_synchronise = false;
    m_syncFirstCall = true;
}

void StepperMotor::synchronise()
{
    // If we know the speed of the other motor, and can
    // determine the delta of its position from last time,
    // we can set our speed as a ratio of the other's speed
    // to our distance to be moved.

    if (m_syncFirstCall) {
        m_syncFirstCall = false;
        m_syncOtherStartPos = m_synchroniseMotor->getPosition();
        if (m_useZeroAsSyncStartPos) {
            m_syncStartPos = 0.0;
        } else {
            m_syncStartPos = getPosition();
        }
        return;
    }
    double otherCurrentPos = m_synchroniseMotor->getPosition();
    double otherPositionDelta = otherCurrentPos - m_syncOtherStartPos;
    double newPosDelta = m_synchroniseFunction(otherPositionDelta, otherCurrentPos);
    // We set the speed higher than it needs to be to ensure
    // that this motor can keep up with the synced motor
    double otherSpeed = 10 * m_synchroniseMotor->getSpeed();
    double deltaRatio = newPosDelta;
    if (otherPositionDelta != 0.0) {
        // otherPositionDelta shouldn't ever be zero, but we check
        // nevertheless to avoid divide by zero
        deltaRatio /= otherPositionDelta;
    }
    setSpeed(otherSpeed * std::abs(deltaRatio) + 10.0, true);
    goToPosition(m_syncStartPos + newPosDelta, true);
}

} // end namespace
