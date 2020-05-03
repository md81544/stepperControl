#pragma once

#include <atomic>
#include <mutex>
#include <thread>

namespace mgo
{

class IGpio;

enum class Direction
{
    forward,
    reverse
};

class StepperMotor
{
public:
    StepperMotor(
        IGpio&  gpio,
        int     stepPin,
        int     reversePin,
        int     enablePin,
        long    stepsPerRevolution,
        double  conversionFactor
        );
    ~StepperMotor();
    bool isRunning() const;
    Direction getDirection() const;
    long getCurrentStep() const;
    // Call the current step position zero
    void zeroPosition();
    // Go to a specific step
    void goToStep( long step );
    // Set motor speed
    void setRpm( double rpm );
    // Get current delay value. Can't think of a
    // purpose for this apart from unit testing :)
    int getDelay();
    // Stop any motion
    void stop();
    // Block until the current operation completes
    void wait();
    // Return position in real-world (but arbitrary) units,
    // for example mm along an axis. Simply returns the
    // supplied conversion factor multiplied by current step
    double getPosition() const;
    // Do conversion on a value which isn't the current step:
    double getPosition( long step ) const;
private:
    int m_motorNumber{ 0 };
    IGpio& m_gpio;
    long m_stepsPerRevolution;
    std::thread m_thread;
    std::atomic<bool>      m_terminateThread{ false };
    std::atomic<long>      m_targetStep{ 0 };
    std::atomic<bool>      m_busy{ false };
    std::atomic<long>      m_currentStep{ 0 };
    std::atomic<bool>      m_stop{ false };
    std::atomic<int>       m_delay{ 500 }; // µsecs
    std::atomic<Direction> m_direction{ Direction::forward };
    // lock should be taken before any code outside the
    // background thread changes any member variables
    std::mutex  m_mtx;
    double m_conversionFactor;
};

} // end namespace

