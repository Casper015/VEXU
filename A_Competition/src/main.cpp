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

// See controller.h: proof on the Brain screen that driver control is running.
volatile unsigned long driverLoopCount = 0;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Calibrate sensors and start the screen thread.
  vexcodeInit();
  // Set up the cup orienter (Optical Sensor + cascade motor, Right arrow).
  cupInit();
  while (InertialSensor.isCalibrating()) {
    wait(20, msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

// Drive commands after the acceleration limit, remembered between loops.
static double forwardCmd = 0;
static double turnCmd = 0;

// Left stick up/down = forward/back, left stick left/right = turn.
// The feel (deadband, curve, top speed, acceleration) is set in config.h and
// the math is in joystick.cpp.
static void driveFromSticks() {
  const double loopSec = Config::driverLoopMs / 1000.0;

  double forwardTarget =
      shapeStick(Controller1.Axis3.position(), Config::joystickDeadband,
                 Config::forwardCurve, Config::forwardMaxPercent);
  double turnTarget =
      shapeStick(Controller1.Axis4.position(), Config::turnDeadband,
                 Config::turnCurve, Config::turnMaxPercent);

  forwardCmd = slewToward(forwardCmd, forwardTarget,
                          Config::forwardAccelPerSec * loopSec,
                          Config::forwardDecelPerSec * loopSec);
  turnCmd = slewToward(turnCmd, turnTarget,
                       Config::turnAccelPerSec * loopSec,
                       Config::turnDecelPerSec * loopSec);

  double leftCmd = 0;
  double rightCmd = 0;
  mixArcade(forwardCmd, turnCmd, leftCmd, rightCmd);

  LeftDrive.spin(forward, leftCmd, percent);
  RightDrive.spin(forward, rightCmd, percent);
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
    driverLoopCount = driverLoopCount + 1;

    if (Config::enableAutoTestButton && Controller1.ButtonUp.pressing()) {
      runAutoTest();
      continue;
    }

    driveFromSticks();

    // Right stick up/down = lift up/down; released = hold position.
    double liftCmd =
        shapeStick(Controller1.Axis2.position(), Config::joystickDeadband,
                   Config::liftCurve, Config::liftMaxPercent);
    if (liftCmd != 0) {
      Lift.spin(forward, liftCmd, percent);
    } else {
      Lift.stop(hold);
    }

    // A = intake forward, Y = intake reverse.
    holdToRun(Intake, Controller1.ButtonA.pressing(),
              Controller1.ButtonY.pressing(), Config::intakeSpeedPercent, coast);

    // L1 = claw close, L2 = claw open; released = hold the grip.
    holdToRun(Claw, Controller1.ButtonL1.pressing(),
              Controller1.ButtonL2.pressing(), Config::clawSpeedPercent, hold);

    // B = cascade forward, X = cascade reverse; released = hold position.
    // Skipped while the cup orienter (Right arrow) is turning it.
    if (!cupBusy()) {
      holdToRun(CascadeMotor, Controller1.ButtonB.pressing(),
                Controller1.ButtonX.pressing(), Config::cascadeSpeedPercent, hold);
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
