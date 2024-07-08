#pragma once

// Concrete implementation of the IGpio ABC,
// but mocked. Includes ability to write
// diags messages.

#include "../configreader.h"
#include "igpio.h"

#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace mgo {

class MockGpio final : public IGpio {
public:
    explicit MockGpio(bool printDiags, const IConfigReader& config)
        : m_print(printDiags)
        , m_config(config)
    {
        print() << "Initialising GPIO library\n";
    }

    virtual ~MockGpio()
    {
        print() << "Terminating GPIO library\n";
        m_terminate = true;
        if (m_rotaryEncoderCallbackerThread.joinable()) {
            print() << "Waiting for rotary encoder callbacker thread to terminate\n";
            m_rotaryEncoderCallbackerThread.join();
        }
        if (m_linearScaleAxis1CallbackerThread.joinable()) {
            print() << "Waiting for linear scale callbacker thread to terminate\n";
            m_linearScaleAxis1CallbackerThread.join();
        }
    }

    int addMotor(int, int, int) override
    {
        return m_motorCount++;
    }

    void setStepPin(int motor, PinState state) override
    {
        if (state == PinState::high) {
            print() << "Setting step pin HIGH for motor " << motor << "\n";
        } else {
            print() << "Setting step pin LOW for motor " << motor << "\n";
        }
    }

    void setReversePin(int motor, PinState state) override
    {
        if (state == PinState::high) {
            print() << "Setting reverse pin HIGH for motor " << motor << "\n";
        } else {
            print() << "Setting reverse pin LOW for motor " << motor << "\n";
        }
    }

    void setEnablePin(int motor, PinState state) override
    {
        if (state == PinState::high) {
            print() << "Setting enable pin HIGH for motor " << motor << "\n";
        } else {
            print() << "Setting enable pin LOW for motor " << motor << "\n";
        }
    }

    uint32_t getTick() override
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    void stopRotaryEncoderCallback()
    {
        // class-specific function to allow us to stop the
        // callbacker manually
        print() << "Stopping callbacker\n";
        m_terminate = true;
    }

    void scaleGoToPositionMm(double mm) override
    {
        m_targetPosition = mm * m_config.readLong("LinearScaleAxis1StepsPerMM", 200);
    }

    void scaleSetSpeedStepsPerSec(double speed) override
    {
        // Note the final multiplication by a "fudge factor" to account for delays
        // introduced by the rest of the code. As this is for testing/faking it's not
        // really important, but it just brings the results from the mock linear scale
        // closer to the dead reckoning values.
        m_microsecsPerStep = ((1.0 / speed) * 1'000'000) * 0.855;
    }

    void scaleStop() override
    {
        m_targetPosition = m_stepCount;
    }

    void setRotaryEncoderCallback(
        int pinA,
        int pinB,
        void (*callback)(int, int, uint32_t, void*),
        void* userData) override
    {
        auto t = std::thread([=]() {
            using namespace std::chrono;
            for (;;) {
                try {
                    if (m_terminate) {
                        break;
                    }
                    callback(pinA, 1, getTick(), userData);
                    if (m_terminate) {
                        break;
                    }
                    callback(pinB, 1, getTick(), userData);
                    if (m_terminate) {
                        break;
                    }
                    callback(pinA, 0, getTick(), userData);
                    if (m_terminate) {
                        break;
                    }
                    callback(pinB, 0, getTick(), userData);
                    std::this_thread::sleep_for(
                        microseconds(m_config.readLong("MockRotaryEncoderDelayMicroseconds", 250)));
                } catch (const std::exception& e) {
                    print() << e.what() << std::endl;
                    break;
                }
            }
            print() << "Rotary encoder callbacker thread ended\n";
        });
        m_rotaryEncoderCallbackerThread.swap(t);
    }

    void setLinearScaleAxis1Callback(
        int pinA,
        int pinB,
        void (*callback)(int, int, uint32_t, void*),
        void* userData) override
    {
        auto t = std::thread([=]() {
            using namespace std::chrono;
            for (;;) {
                if (m_terminate) {
                    break;
                }
                // Each pin change (which doesn't replicate the previous state) counts as a step.
                // We keep track of steps internally in the mock (the real object leaves its owner
                // to do that) so we can make the mock 'move' programmatically.
                try {
                    if (m_stepCount < m_targetPosition) {
                        callback(pinA, 1, getTick(), userData);
                        if (++m_stepCount == m_targetPosition) {
                            continue;
                        }
                        if (m_terminate) {
                            break;
                        }
                        std::this_thread::sleep_for(microseconds(m_microsecsPerStep));
                        callback(pinB, 1, getTick(), userData);
                        if (++m_stepCount == m_targetPosition) {
                            continue;
                        }
                        if (m_terminate) {
                            break;
                        }
                        std::this_thread::sleep_for(microseconds(m_microsecsPerStep));
                        callback(pinA, 0, getTick(), userData);
                        if (++m_stepCount == m_targetPosition) {
                            continue;
                        }
                        if (m_terminate) {
                            break;
                        }
                        std::this_thread::sleep_for(microseconds(m_microsecsPerStep));
                        callback(pinB, 0, getTick(), userData);
                        if (++m_stepCount == m_targetPosition) {
                            continue;
                        }
                        if (m_terminate) {
                            break;
                        }
                        std::this_thread::sleep_for(microseconds(m_microsecsPerStep));
                    } else if (m_stepCount > m_targetPosition) {
                        callback(pinB, 0, getTick(), userData);
                        if (--m_stepCount == m_targetPosition) {
                            continue;
                        }
                        if (m_terminate) {
                            break;
                        }
                        std::this_thread::sleep_for(microseconds(m_microsecsPerStep));
                        callback(pinA, 0, getTick(), userData);
                        if (--m_stepCount == m_targetPosition) {
                            continue;
                        }
                        if (m_terminate) {
                            break;
                        }
                        std::this_thread::sleep_for(microseconds(m_microsecsPerStep));
                        callback(pinB, 1, getTick(), userData);
                        if (--m_stepCount == m_targetPosition) {
                            continue;
                        }
                        if (m_terminate) {
                            break;
                        }
                        std::this_thread::sleep_for(microseconds(m_microsecsPerStep));
                        callback(pinA, 1, getTick(), userData);
                        if (--m_stepCount == m_targetPosition) {
                            continue;
                        }
                        if (m_terminate) {
                            break;
                        }
                        std::this_thread::sleep_for(microseconds(m_microsecsPerStep));
                    } else {
                        // Not moving so small-ish sleep just to avoid spinning
                        std::this_thread::sleep_for(milliseconds(50));
                        if (m_terminate) {
                            break;
                        }
                    }
                } catch (const std::exception& e) {
                    print() << e.what() << std::endl;
                    break;
                }
            }
            print() << "Linear scale axis 1 callbacker thread ended\n";
        });
        m_linearScaleAxis1CallbackerThread.swap(t);
    }

    void delayMicroSeconds(long usecs) override
    {
        print() << "Sleeping for " << usecs << " usecs\n";
        usleep(usecs);
    }

private:
    std::atomic<bool> m_terminate { false };
    std::atomic<double> m_targetPosition { 0.0 };
    std::atomic<double> m_microsecsPerStep { 0.01 };
    std::thread m_rotaryEncoderCallbackerThread;
    std::thread m_linearScaleAxis1CallbackerThread;
    int32_t m_stepCount {
        0
    }; // We keep count of steps in this mock to allow us to move to a specific location for testing

    int m_motorCount { 0 };

    bool m_print;
    const IConfigReader& m_config;
    std::fstream m_devNull;
    std::ostream& print()
    {
        if (m_print) {
            return std::cout;
        }
        return m_devNull;
    }
};

} // end namespace
