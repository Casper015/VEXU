#include "vex.h"

using namespace vex;

// ---------------------------------------------------------------------------
// Routes. Add more functions like this one and pick which one autonomous()
// calls. drivePID() is in cm (relative), turnPID() is an absolute heading in
// degrees where 0 is the way the robot faced when autonomous started.
// ---------------------------------------------------------------------------

static void testRoute() {
  drivePID(25);
  turnPID(90);
  drivePID(10);
  drivePID(-10);
  turnPID(-90);
  drivePID(10);
  turnPID(90);
  turnPID(-90);
  drivePID(25);
}

void autonomous(void) {
  // If the field starts autonomous before the inertial sensor has finished
  // calibrating, wait for it so the heading is valid.
  while (InertialSensor.isCalibrating()) {
    wait(20, msec);
  }

  // Hold the lift where it is so it cannot sag while driving.
  Lift.stop(hold);

  // Heading 0 = the direction the robot faces when autonomous starts.
  resetHeading();

  testRoute();
}
