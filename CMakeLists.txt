cmake_minimum_required(VERSION 3.20)

project(stepperControl LANGUAGES CXX)

# Consumers which only use MockGpio (e.g. a debug build on a PC) should set
# this OFF, so pigpio isn't needed:
#   set(STEPPERCONTROL_USE_PIGPIO OFF)   (before FetchContent_MakeAvailable)
option(STEPPERCONTROL_USE_PIGPIO "Build mgo::Gpio, which needs the pigpio library" ON)

find_package(Threads REQUIRED)

add_library(stepperControl src/steppermotor.cpp)
add_library(stepperControl::stepperControl ALIAS stepperControl)

# Headers are included as "stepperControl/steppermotor.h" etc
target_include_directories(stepperControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(stepperControl PUBLIC cxx_std_20)
target_compile_options(stepperControl PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(stepperControl PUBLIC Threads::Threads)

# Raspberry Pi 32-bit needs the atomic library (for std::atomic<double> etc)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
	target_link_libraries(stepperControl PUBLIC atomic)
endif()

if(STEPPERCONTROL_USE_PIGPIO)
	find_path(PIGPIO_INCLUDE_DIR pigpio.h)
	find_library(PIGPIO_LIBRARY pigpio)
	if(NOT PIGPIO_INCLUDE_DIR OR NOT PIGPIO_LIBRARY)
		message(FATAL_ERROR
			"pigpio not found. It is only available on a Raspberry Pi; on other "
			"machines set STEPPERCONTROL_USE_PIGPIO=OFF and use mgo::MockGpio")
	endif()
	target_sources(stepperControl PRIVATE src/gpio.cpp)
	target_include_directories(stepperControl PUBLIC ${PIGPIO_INCLUDE_DIR})
	target_link_libraries(stepperControl PUBLIC ${PIGPIO_LIBRARY})
endif()
