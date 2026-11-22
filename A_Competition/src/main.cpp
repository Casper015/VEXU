/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       casper                                                    */
/*    Created:      11/6/2026, 9:38:20 AM                                     */
/*    Description:  V5 competition program (base: 5_Utilities)                */
/*                                                                            */
/*    Where things are:                                                       */
/*      config.h          every tunable number                                */
/*      robot-config.cpp  ports and devices                                   */
/*      drive.cpp         turnPID() / drivePID()                              */
/*      auton.cpp         autonomous routes                                   */
/*      main.cpp          competition setup and driver control                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Calibrate sensors and start the screen thread.
  vexcodeInit();
  // Set up the cup orienter (Optical Sensor + cascade motor, A button).
  cupInit();
  while (InertialSensor.isCalibrating()) {
    wait(20, msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

// Joystick value with the deadband applied.
static int stickValue(int raw) {
  return abs(raw) < Config::joystickDeadband ? 0 : raw;
}

// Drive commands after the acceleration limit, remembered between loops.
static double forwardCmd = 0;
static double turnCmd = 0;

// Left stick up/down = forward/back, left stick left/right = turn.
// The feel (curve, top speed, acceleration) is set in config.h.
static void driveFromSticks() {
  const double loopSec = Config::driverLoopMs / 1000.0;

  double forwardTarget =
      curveStick(stickValue(Controller1.Axis3.position()), Config::forwardCurve) *
      Config::forwardMaxPercent / 100.0;
  double turnTarget =
      curveStick(stickValue(Controller1.Axis4.position()), Config::turnCurve) *
      Config::turnMaxPercent / 100.0;

  forwardCmd = slewToward(forwardCmd, forwardTarget,
                          Config::forwardAccelPerSec * loopSec,
                          Config::forwardDecelPerSec * loopSec);
  turnCmd = slewToward(turnCmd, turnTarget,
                       Config::turnAccelPerSec * loopSec,
                       Config::turnDecelPerSec * loopSec);

  LeftDrive.spin(forward, forwardCmd + turnCmd, percent);
  RightDrive.spin(forward, forwardCmd - turnCmd, percent);
}

// TEST ONLY: run the autonomous route once from driver control.
// Press Down at any time to cancel it.
static void runAutoTest() {
  stopDrive();
  Lift.stop(hold);
  Intake.stop(coast);
  Claw.stop(hold);
  CascadeMotor.stop(hold);
  forwardCmd = 0;
  turnCmd = 0;

  allowDriveAbort(true);
  autonomous();
  allowDriveAbort(false);

  waitUntil(!Controller1.ButtonUp.pressing()); // no repeat if still held
}

// Run a mechanism while one of two buttons is held, stop when neither is.
// Works for a single motor or a motor_group.
template <class Mechanism>
static void holdToRun(Mechanism& mechanism, bool forwardHeld, bool reverseHeld,
                      double speedPercent, brakeType stopMode) {
  if (forwardHeld && !reverseHeld) {
    mechanism.spin(forward, speedPercent, percent);
  } else if (reverseHeld && !forwardHeld) {
    mechanism.spin(reverse, speedPercent, percent);
  } else {
    mechanism.stop(stopMode);
  }
}

void usercontrol(void) {
  forwardCmd = 0;
  turnCmd = 0;

  while (true) {
    if (Config::enableAutoTestButton && Controller1.ButtonUp.pressing()) {
      runAutoTest();
      continue;
    }

    driveFromSticks();

    // Right stick up/down = lift up/down; released = hold position.
    int liftInput = stickValue(Controller1.Axis2.position());
    if (liftInput != 0) {
      Lift.spin(forward, liftInput * Config::liftMaxPercent / 100.0, percent);
    } else {
      Lift.stop(hold);
    }

    // R1 = intake in, R2 = intake out.
    holdToRun(Intake, Controller1.ButtonR1.pressing(),
              Controller1.ButtonR2.pressing(), Config::intakeSpeedPercent, coast);

    // L1 = claw close, L2 = claw open; released = hold the grip.
    holdToRun(Claw, Controller1.ButtonL1.pressing(),
              Controller1.ButtonL2.pressing(), Config::clawSpeedPercent, hold);

    // X = cascade forward, B = cascade reverse; released = hold position.
    // Skipped while the cup orienter (A button) is turning it.
    if (!cupBusy()) {
      holdToRun(CascadeMotor, Controller1.ButtonX.pressing(),
                Controller1.ButtonB.pressing(), Config::cascadeSpeedPercent, hold);
    }

    wait(Config::driverLoopMs, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
