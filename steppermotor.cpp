#include "steppermotor.h"

#include "igpio.h"

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
                    m_targetStep = long( m_currentReportedStep );
                    m_stop = false;
                    m_busy = false;
                }
                if( m_targetStep != m_currentReportedStep )
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
                    // Spin for first delay amount:
                    while( m_gpio.getTick() - startTick < static_cast<uint32_t>( m_delay ) );
                    m_gpio.setStepPin( m_motorNumber, PinState::low );
                    if ( m_targetStep < m_currentReportedStep )
                    {
                        // we only start counting steps once the slack of the backlash
                        // compensation has been taken up
                        if( m_backlashPosition == 0 )
                        {
                            --m_currentReportedStep;
                        }
                        else
                        {
                            --m_backlashPosition;
                        }
                        --m_currentActualStep; // This is only used for testing
                    }
                    else
                    {
                        if( m_backlashPosition == m_backlashSize )
                        {
                            ++m_currentReportedStep;
                        }
                        else
                        {
                            ++m_backlashPosition;
                        }
                        ++m_currentActualStep; // This is only used for testing
                    }
                    if ( m_currentReportedStep == m_targetStep )
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
    if ( m_currentReportedStep == step )
    {
        return;
    }
    m_direction = Direction::forward;
    if ( step < m_currentReportedStep )
    {
        m_direction = Direction::reverse;
    }
    m_busy = true;
    m_targetStep = step;
}

void StepperMotor::stop()
{
    {
        std::lock_guard<std::mutex> mtx( m_mtx );
        m_stop = true;
    }
    // Wait before returning to allow the stop to be acted on
    // by the stepper thread
    m_gpio.delayMicroSeconds( 50'000 );
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

void StepperMotor::setSpeed( double speed)
{
    double rpm = std::abs( speed / m_conversionFactor / m_stepsPerRevolution );
    setRpm( rpm );
}

double StepperMotor::getRpm()
{
    return m_rpm;
}

double StepperMotor::getSpeed()
{
    return std::abs( m_rpm * m_stepsPerRevolution * m_conversionFactor );
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
    return m_currentReportedStep;
}

long StepperMotor::getTargetStep() const
{
    return m_targetStep;
}

void StepperMotor::zeroPosition()
{
    {
        std::lock_guard<std::mutex> mtx( m_mtx );
        m_currentReportedStep = 0L;
        m_targetStep = 0L;
    }
    stop();
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
    return m_currentReportedStep * m_conversionFactor;
}

double StepperMotor::getPosition( long step) const
{
    return step * m_conversionFactor;
}

void StepperMotor::setPosition( double mm )
{
    goToStep( mm / m_conversionFactor );
}

void StepperMotor::setBacklashCompensation(
    unsigned int compensation,
    unsigned int currentPosition
    )
{
    std::lock_guard<std::mutex> mtx( m_mtx );
    m_backlashSize = compensation;
    if( currentPosition > compensation )
    {
        m_backlashPosition = compensation;
    }
    else
    {
        m_backlashPosition = currentPosition;
    }
}

long StepperMotor::getCurrentStepWithoutBacklashCompensation() const
{
    return m_currentActualStep;
}

} // end namespace
