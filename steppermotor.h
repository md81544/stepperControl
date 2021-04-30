#pragma once

#include <atomic>
#include <functional>
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
        double  conversionFactor,
        double  maxRpm
        );
    ~StepperMotor();
    bool isRunning() const;
    Direction getDirection() const;
    long getCurrentStep() const;
    long getTargetStep() const;
    // Call the current step position zero
    void zeroPosition();
    // Go to a specific step
    void goToStep( long step, bool noLock = false );
    // Set motor speed in RPM (i.e. doesn't care about gearing)
    void setRpm( double rpm, bool noLock = false );
    // Set motor speed in units/min, (i.e. after gearing)
    void setSpeed( double speed, bool noLock = false );
    // Get motor speed as set with setRpm()
    double getRpm();
    // Get motor speed as set with setRpm(), but in units/min
    double getSpeed() const;
    // Get speed limit as set by ctor
    double getMaxRpm();
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
    // Go to position in UNITS not steps
    void goToPosition( double mm, bool noLock = false );
    // Set current position (don't move) in UNITS not steps
    void setPosition( double mm );
    // Set backlash compensation. The number of steps the system has should be
    // measured. currentPosition should be either zero if slack has been taken
    // up moving negatively, or equal to steps if slack has been taken up by moving
    // in the positive direction of the axis ( i.e. outwards on X or Z ). If this is
    // never called then the motor works as if there is no backlash at all.
    void setBacklashCompensation( unsigned int steps, unsigned int currentPosition );
    // Only really needed for testing:
    long getCurrentStepWithoutBacklashCompensation() const;
    // Also only needed for testing:
    double getRampedSpeed();

    double getConversionFactor() const;
    void enableRamping( bool flag );
    // Synchronise this motor with another. The function object will be called
    // with the *delta* of the other motor's position since start. We return
    // our new expected delta. Speed is handled automatically by the motor object
    // and will track the "other" motor's speed, proportionally.
    void synchroniseOn(
        const StepperMotor * const other,
        std::function<double( double posDelta, double pos )> func
        );
    void synchroniseOff();
private:
    // Internal, only to be used from within an existing lock scope:
    void synchronise();
    int m_motorNumber{ 0 };
    IGpio& m_gpio;
    long m_stepsPerRevolution;
    std::thread m_thread;
    std::atomic<bool>      m_terminateThread{ false };
    std::atomic<long>      m_targetStep{ 0 };
    std::atomic<bool>      m_busy{ false };
    // What step the motor reports that it is on (this is adjusted for backlash):
    std::atomic<long>      m_currentReportedStep{ 0 };
    // Actual current step regardless of backlash compensation (only used for testing):
    std::atomic<long>      m_currentActualStep{ 0 };
    std::atomic<bool>      m_stop{ false };
    std::atomic<int>       m_delay{ 500 }; // µsecs
    std::atomic<Direction> m_direction{ Direction::forward };
    // lock should be taken before any code outside the
    // background thread changes any member variables
    std::mutex  m_mtx;
    double m_conversionFactor;
    double m_rpm{ 0.0 };
    double m_rampedRpm{ 0.0 };
    double m_maxRpm;
    unsigned int m_backlashSize{ 0 };
    unsigned int m_backlashPosition{ 0 };
    bool m_useRamping{ true };
    double calculateDelayValue( double rpm );
    bool m_synchronise{ false };
    const StepperMotor* m_synchroniseMotor{ nullptr };
    std::function<double( double, double )> m_synchroniseFunction;

    bool    m_syncFirstCall{ true };
    double  m_syncOtherStartPos;
    double  m_syncStartPos;
};

} // end namespace

