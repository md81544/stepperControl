#include "steppermotor.h"
#include "igpio.h"
#include "../log.h"

#include <cstdint>
#include <cmath>

namespace mgo
{

StepperMotor::StepperMotor(
    IGpio& gpio,
    int stepPin,
    int reversePin,
    int enablePin,
    long stepsPerRevolution,
    double conversionFactor,
    double maxRpm
    )
    :   m_gpio( gpio ),
        m_stepsPerRevolution( stepsPerRevolution ),
        m_conversionFactor( conversionFactor ),
        m_maxRpm( maxRpm )
{
    m_motorNumber = m_gpio.addMotor( stepPin, reversePin, enablePin );
    // Ensure we start off with the right direction
    m_gpio.setReversePin(
        m_motorNumber,
        m_direction == Direction::forward ?
            PinState::low : PinState::high
        );

    // Start the thread
    std::thread t( [&]()
    {
        Direction oldDirection = Direction::forward;
        for(;;)
        {
            int delay;
            uint32_t startTick = m_gpio.getTick();
            {   // scope for lock_guard
                // When in this scope we can assume all member
                // variables can be written and read from freely
                // without them being changed externally. Other
                // member functions must lock the mutex m_mtx
                // before changing any member variables.
                std::lock_guard<std::mutex> mtx( m_mtx );
                if( m_terminateThread)
                {
                    break;
                }
                if ( m_stop )
                {
                    m_targetStep = long( m_currentStep );
                    m_stop = false;
                    m_busy = false;
                }
                if( m_targetStep != m_currentStep )
                {
                    if ( oldDirection != m_direction )
                    {
                        // Ensure direction pin is set correctly
                        m_gpio.setReversePin(
                            m_motorNumber,
                            m_direction == Direction::forward ?
                                PinState::low : PinState::high
                            );
                        oldDirection = m_direction;
                    }
                    // Do step pulse
                    m_gpio.setStepPin( m_motorNumber, PinState::high );
                    m_gpio.delayMicroSeconds( m_delay );
                    m_gpio.setStepPin( m_motorNumber, PinState::low );
                    // Second part of the delay is at the end of the loop
                    if ( m_targetStep < m_currentStep )
                    {
                        --m_currentStep;
                    }
                    else
                    {
                        ++m_currentStep;
                    }
                    if ( m_currentStep == m_targetStep )
                    {
                        m_busy = false;
                    }
                }
                delay = m_delay; // remember delay value while in mtx scope
            } // scope for lock_guard
            // We always perform the second delay regardless of
            // whether we're stepping, to give the main thread a
            // chance to grab the mutex if needed

            // Spin until we get to the right time
            while( m_gpio.getTick() - startTick < static_cast<uint32_t>( 2 * delay ) );
        }
    } // thread end
    );
    // Store the thread's handle in m_thread
    m_thread.swap(t);
}


StepperMotor::~StepperMotor()
{
    m_terminateThread = true;
    m_thread.join();
}


void StepperMotor::goToStep( long step )
{
    std::lock_guard<std::mutex> mtx( m_mtx );
    if ( m_busy )
    {
        // We ignore any request to go to a location if
        // we are already stepping. The client code can
        // issue a stop if needed before changing target
        // step location.
        return;
    }
    if ( m_currentStep == step )
    {
        return;
    }
    m_direction = Direction::forward;
    if ( step < m_currentStep )
    {
        m_direction = Direction::reverse;
    }
    m_busy = true;
    m_targetStep = step;
}

void StepperMotor::stop()
{
    std::lock_guard<std::mutex> mtx( m_mtx );
    m_stop = true;
}

void StepperMotor::setRpm( double rpm )
{
    if( rpm < 0 ) rpm = 0;
    if( rpm > m_maxRpm ) rpm = m_maxRpm;
    // NB m_delay (in µsecs) is used TWICE per thread loop
    std::lock_guard<std::mutex> mtx( m_mtx );
    if ( rpm < 0.1 )
    {
        // Arbitrarily anything lower than this
        // and we stop
        m_rpm = 0;
        m_stop = true;
        return;
    }
    m_rpm = rpm;
    m_delay = std::round( 500'000.0 /
            ( static_cast<double>( m_stepsPerRevolution ) * ( rpm / 60.0 ) )
        );
    if ( m_delay < 10 )
    {
        // Avoid very small delay values as the stepper
        // motor won't be able to keep up
        m_delay = 10;
    }
}

double StepperMotor::getRpm()
{
    return m_rpm;
}

double StepperMotor::getMaxRpm()
{
    return m_maxRpm;
}

int StepperMotor::getDelay()
{
    std::lock_guard<std::mutex> mtx( m_mtx );
    return m_delay;
}

long StepperMotor::getCurrentStep() const
{
    return m_currentStep;
}

void StepperMotor::zeroPosition()
{
    std::lock_guard<std::mutex> mtx( m_mtx );
    m_currentStep = 0;
}

void StepperMotor::wait()
{
    while ( m_busy )
    {
        m_gpio.delayMicroSeconds( 10'000 );
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
    return m_currentStep * m_conversionFactor;
}

double StepperMotor::getPosition( long step) const
{
    return step * m_conversionFactor;
}

} // end namespace
