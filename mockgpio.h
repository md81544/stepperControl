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
        if (m_callbacker.joinable()) {
            print() << "Waiting for callbacker thread to terminate\n";
            m_callbacker.join();
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

    void setEncoderCallback(
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
            print() << "Callbacker thread ended\n";
        });
        m_callbacker.swap(t);
    }

    void delayMicroSeconds(long usecs) override
    {
        print() << "Sleeping for " << usecs << " usecs\n";
        usleep(usecs);
    }

private:
    std::atomic<bool> m_terminate { false };
    std::thread m_callbacker;

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
