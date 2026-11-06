# stepperControl

Drives stepper motors from a Raspberry Pi's GPIO pins (via
[pigpio](https://abyz.me.uk/rpi/pigpio/)), with a background real-time thread
per motor. `mgo::MockGpio` stands in for the hardware so that code using this
library can be run and tested on a PC.

## Using it from CMake

```cmake
include(FetchContent)
FetchContent_Declare(stepperControl
    GIT_REPOSITORY https://github.com/md81544/stepperControl.git
    GIT_TAG        v1.0.0
)
# Only if you won't use mgo::Gpio (e.g. a mocked build on a PC):
set(STEPPERCONTROL_USE_PIGPIO OFF)
FetchContent_MakeAvailable(stepperControl)

target_link_libraries(myApp PRIVATE stepperControl::stepperControl)
```

Then:

```cpp
#include "stepperControl/steppermotor.h"
#include "stepperControl/gpio.h"     // real hardware, or
#include "stepperControl/mockgpio.h" // mocked
```

C++20 is required.

### Working on this library alongside a project that uses it

Point CMake at a local checkout instead of the pinned version:

    cmake -B build -DFETCHCONTENT_SOURCE_DIR_STEPPERCONTROL=/path/to/stepperControl

Once you're happy with the changes, commit and tag them here, then update
`GIT_TAG` in the project.
