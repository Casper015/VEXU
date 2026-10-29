#include "vex.h"

using namespace vex;

// Diagnostic replacement for A_Testing/src/main.cpp, not an additional source.
// Skip vexcodeInit(): do not register motor callbacks or start the old display.
int main() {
  wait(200, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("IMU14 PROBE v1 - turn robot by hand");
  Controller1.Screen.clearScreen();

  if (TestInertial.installed()) {
    TestInertial.calibrate();
    const uint32_t started = Brain.Timer.time();
    while (TestInertial.installed() && TestInertial.isCalibrating() &&
           static_cast<uint32_t>(Brain.Timer.time()) - started < 5000UL) {
      wait(20, msec);
    }
    if (TestInertial.installed() && !TestInertial.isCalibrating()) {
      TestInertial.resetRotation();
    }
  }

  unsigned long frame = 0;
  while (true) {
    const int online = TestInertial.installed() ? 1 : 0;
    const int calibrating = TestInertial.isCalibrating() ? 1 : 0;
    const double angle = TestInertial.rotation(degrees);
    const double heading = TestInertial.heading(degrees);

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("N:%04lu  installed:%d  calibrating:%d   ",
                       frame % 10000UL, online, calibrating);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("rotation:%12.1f deg   ", angle);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("heading: %12.1f deg   ", heading);

    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("N:%04lu OK:%d C:%d   ",
                             frame % 10000UL, online, calibrating);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("I:%9.1f deg    ", angle);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("H:%9.1f deg    ", heading);

    ++frame;
    wait(200, msec);
  }
}
