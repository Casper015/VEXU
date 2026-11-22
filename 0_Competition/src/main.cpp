/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       casper                                                    */
/*    Created:      11/6/2026, 9:38:20 AM                                     */
/*    Description:  V5 competition program (base: 5_Utilities)                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

/*---------------------------------------------------------------------------*/
/*                         Drive / turn helpers (PID)                        */
/*---------------------------------------------------------------------------*/

// Per-side compensation so the robot drives straight.
const double leftRatio = 1.0;
const double rightRatio = 1.0;

// PID gains taken from pid (1).h: kP = 0.98, kD = 0.67, turn limited to 15%,
// drive limited to 50%. That code computed D as (error change per 10 ms loop)
// and drive error in cm; calculatePID() uses change per ms and drive error in
// rotation-sensor degrees, so the gains are scaled below to behave the same.
const double srcKp = 0.70;
const double srcKd = 0.67;
const double srcLoopMs = 10;
const double turnMaxOutput = 15;
const double driveMaxOutput = 50;

// TEST ONLY: press Up during driver control to run autonomous().
// Set to false before a real competition.
const bool enableAutoTestButton = true;

// Intake speed while A (in) or Y (out) is held (percent).
const double intakeSpeedPercent = 100;

// Lift speed for L1/L2 and for liftTo() in autonomous (percent).
const double liftSpeedPercent = 100;
// liftTo() counts the lift as "there" when BOTH motors are this close
// (degrees) to the target.
const double liftToleranceDegrees = 5;

// Claw top speed with the right stick pushed all the way (percent).
const double clawSpeedPercent = 60;

// Cascade speed for the automatic B step (percent). 100 = as fast as the
// motor's cartridge allows. Lower it if the step overshoots its angle.
const double cascadeStepSpeedPercent = 100;
// Cascade speed while X is held (percent). Slower, for fine control.
const double cascadeManualSpeedPercent = 40;
// One tap of B turns the cascade this many degrees.
const double cascadeStepDegrees = 180;
// Motor turns per one turn of the cascade. 1.0 = motor drives it directly.
// With gears, e.g. motor 12-tooth driving a 36-tooth gear, use 36.0 / 12.0.
const double cascadeGearRatio = 1.0;

// Joystick values below this are treated as 0 (stops drift when released).
const int joystickDeadband = 5;

// The old Drivetrain object is gone (drive is now two 3-motor groups), so
// stopping the drive means stopping both sides.
void stopDrive() {
  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

bool setupTurn(pid& controller, double targetAngle, double uncertainty = 2);
bool setupDrive(pid& controller, double targetDistance,
                double uncertainty_cm = 2.0);

// Turn to an ABSOLUTE heading in degrees (0 = heading at autonomous start,
// positive = right). Blocks until settled or timeout.
void turnPID(double angleDeg, uint32_t timeoutMs = 0) {
  if (timeoutMs == 0) {
    // 15% max turn speed is slow, so allow ~25 ms per degree of turning.
    double turnDeg = fabs(angleDeg - InertialSensor.rotation(degrees));
    timeoutMs = 1000 + (uint32_t)(turnDeg * 25);
  }

  pid turnCtrl{};
  turnCtrl.kp = srcKp;              // 0.98
  turnCtrl.kd = srcKd * srcLoopMs;  // 6.7
  turnCtrl.maxOutput = turnMaxOutput;

  uint32_t startMs = Brain.Timer.time();
  while (!setupTurn(turnCtrl, angleDeg, 1)) {
    if (Brain.Timer.time() - startMs > timeoutMs) break;
    wait(10, msec);
  }
  stopDrive();
}

// Drive straight a RELATIVE distance in cm (negative = backward). Both sides
// get the same speed (no heading correction). Blocks until settled or timeout.
void drivePID(double distanceCm, uint32_t timeoutMs = 0) {
  if (timeoutMs == 0) {
    timeoutMs = 1500 + (uint32_t)(fabs(distanceCm) * 20); // ~20 ms per cm
  }

  pid driveCtrl{};
  const double cmPerDegree = degree_to_cm(1);           // ~0.0443 cm
  driveCtrl.kp = srcKp * cmPerDegree;                   // ~0.0434
  driveCtrl.kd = srcKd * srcLoopMs * cmPerDegree;       // ~0.297
  driveCtrl.maxOutput = driveMaxOutput;

  // Show what this drive was asked to do (controller row 3), so you can check
  // the robot is running the program you just downloaded.
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Drive %+6.1fcm     ", distanceCm);

  RotationSensor.resetPosition();
  double targetDeg = cm_to_degree(distanceCm);
  uint32_t startMs = Brain.Timer.time();
  while (!setupDrive(driveCtrl, targetDeg, 1)) {
    if (Brain.Timer.time() - startMs > timeoutMs) break;
    wait(10, msec);
  }
  stopDrive();
}

// Autonomous: move the lift to a height in lift-motor degrees and hold there.
//   0 = where the lift was when the program started (start it at the bottom).
//   To find a height: raise the lift with L1, read "Lift:" on the controller.
//   liftTo(300);         wait until the lift reaches 300, then continue
//   liftTo(300, false);  start the lift and continue right away (e.g. drive
//                        while it rises)
//   liftTo(0);           back down to the bottom
void liftTo(double liftDegrees, bool waitForIt = true,
            uint32_t timeoutMs = 2000) {
  // Send EACH lift motor to the target on its own, so both get there.
  LiftMotorA.spinToPosition(liftDegrees, degrees, liftSpeedPercent,
                            velocityUnits::pct, false);
  LiftMotorB.spinToPosition(liftDegrees, degrees, liftSpeedPercent,
                            velocityUnits::pct, false);
  if (!waitForIt) return;

  // Done only when BOTH motors are within liftToleranceDegrees of the target.
  uint32_t startMs = Brain.Timer.time();
  bool timedOut = false;
  while (true) {
    double errA = fabs(liftDegrees - LiftMotorA.position(degrees));
    double errB = fabs(liftDegrees - LiftMotorB.position(degrees));
    if (errA <= liftToleranceDegrees && errB <= liftToleranceDegrees) break;
    if (Brain.Timer.time() - startMs > timeoutMs) {
      timedOut = true;  // stuck: give up and hold, so auto can carry on
      break;
    }
    wait(10, msec);
  }
  Lift.stop(hold);

  // Controller row 3: target, where each motor ended up, "!" = ran out of time.
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("L%4.0f A%4.0f B%4.0f%s", liftDegrees,
                           LiftMotorA.position(degrees),
                           LiftMotorB.position(degrees), timedOut ? "!" : " ");
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Calibrate sensors and start the screen thread.
  vexcodeInit();
  while (InertialSensor.isCalibrating()) {
    wait(20, msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // Heading 0 = the direction the robot faces when autonomous starts.
  InertialSensor.setRotation(0, degrees);
  wait(3000, msec);
  liftTo(300);  // example: raise the lift to 300 degrees (see liftTo above)
  Piston.set(true);
  drivePID(100);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*------------------------ ---------------------------------------------------*/

// ---- ONE-MOTOR TEST ---------------------------------------------------------
// true: driver control spins ONLY testMotor with the left stick (up/down);
// every other motor stays off. Set to false to drive normally again.
// To test the next motor, change testMotor and testMotorName below.
const bool motorTestMode = false;
motor& testMotor = LeftMotorC;
const char* testMotorName = "LeftMotorC";

void runMotorTest() {
  int input = Controller1.Axis3.position();
  if (abs(input) < joystickDeadband) input = 0;

  if (input == 0) {
    testMotor.stop(coast);
  } else {
    testMotor.spin(forward, input, percent);
  }

  // Row 1 belongs to controller_screen(), so use rows 2 and 3.
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("%-10s P%-2d %-4s ", testMotorName,
                           (int)testMotor.index() + 1,
                           testMotor.installed() ? "OK" : "NONE");
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("cmd%4d real%4d   ", input,
                           (int)testMotor.velocity(percent));
}

void usercontrol(void) {
  if (motorTestMode) {
    Controller1.Screen.clearScreen();
    while (true) {
      runMotorTest();
      wait(20, msec);
    }
  }

  // Cascade button state (see the cascade block in the loop).
  bool clampR1WasPressed = false;   // R1 state last loop (to catch a new tap)
  bool cascadeBWasPressed = false;  // B state last loop (to catch a new tap)
  bool cascadeXSpinning = false;    // true while X is spinning it
  double cascadeTarget = CascadeMotor.position(degrees);

  while (true) {
    // Up button: run the autonomous route once (joystick is ignored meanwhile).
    if (enableAutoTestButton && Controller1.ButtonUp.pressing()) {
      stopDrive();
      autonomous();
      waitUntil(!Controller1.ButtonUp.pressing()); // no repeat if still held
      continue;
    }

    // Left stick up/down = forward/back, left stick left/right = turn.
    int forwardInput = Controller1.Axis3.position();
    int turnInput = Controller1.Axis4.position();
    int clawInput = Controller1.Axis2.position();
    if (abs(forwardInput) < joystickDeadband) forwardInput = 0;
    if (abs(turnInput) < joystickDeadband) turnInput = 0;
    if (abs(clawInput) < joystickDeadband) clawInput = 0;

    // Stick right = turn right: left side faster, right side slower.
    LeftDrive.spin(forward, forwardInput + turnInput, percent);
    RightDrive.spin(forward, forwardInput - turnInput, percent);

    // Lift: hold L1 = forward, hold L2 = reverse. Released, it holds in place.
    if (Controller1.ButtonL1.pressing()) {
      Lift.spin(forward, liftSpeedPercent, percent);
    } else if (Controller1.ButtonL2.pressing()) {
      Lift.spin(reverse, liftSpeedPercent, percent);
    } else {
      Lift.stop(hold);
    }

    // Claw: right stick up/down. Speed follows how far the stick is pushed,
    // up to clawSpeedPercent. Released, it holds its grip.
    if (clawInput == 0) {
      Claw.stop(hold);
    } else {
      Claw.spin(forward, clawInput * clawSpeedPercent / 100, percent);
    }

    // Pneumatic clamp (three-wire A): each tap of R1 switches it
    // clamp -> release -> clamp ... Holding R1 counts as one tap.
    {
      bool r1Now = Controller1.ButtonR1.pressing();
      if (r1Now && !clampR1WasPressed) {
        Piston.set(!Piston.value());
      }
      clampR1WasPressed = r1Now;
    }

    // Cascade:
    //   B tap  -> turn forward exactly cascadeStepDegrees, fast, and stop.
    //             Each press adds one more step.
    //   X hold -> spin the opposite way (reverse), slowly, until released.
    {
      bool bNow = Controller1.ButtonB.pressing();
      bool xNow = Controller1.ButtonX.pressing();

      if (xNow) {
        if (!cascadeXSpinning) {
          CascadeMotor.spin(reverse, cascadeManualSpeedPercent, percent);
          cascadeXSpinning = true;
        }
      } else {
        if (cascadeXSpinning) {
          // X released: stop here, and count the next B step from here.
          CascadeMotor.stop(hold);
          cascadeTarget = CascadeMotor.position(degrees);
          cascadeXSpinning = false;
        }
        if (bNow && !cascadeBWasPressed) {
          // Fresh B press (not held from the last loop): one step forward.
          cascadeTarget += cascadeStepDegrees * cascadeGearRatio;
          CascadeMotor.spinToPosition(cascadeTarget, degrees,
                                      cascadeStepSpeedPercent, velocityUnits::pct,
                                      false);
        }
      }
      cascadeBWasPressed = bNow;
    }

    // Intake: hold A to pull in, hold Y to push out (clear a jam),
    // release to stop.
    if (Controller1.ButtonA.pressing()) {
      Intake.spin(forward, intakeSpeedPercent, percent);
    } else if (Controller1.ButtonY.pressing()) {
      Intake.spin(reverse, intakeSpeedPercent, percent);
    } else {
      Intake.stop(coast);
    }

    wait(20, msec);
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

/*---------------------------------------------------------------------------*/
/*                     One PID step (non-blocking helpers)                   */
/*---------------------------------------------------------------------------*/

bool setupTurn(pid& controller, double targetAngle, double uncertainty) {
  double currentAngle = InertialSensor.rotation(degrees);
  double error = targetAngle - currentAngle;

  if (hasSettled(controller, error, uncertainty, 50)) {
    stopDrive();
    return true;
  }

  double speed = calculatePID(controller, targetAngle, currentAngle);
  // Turn in place: positive speed = turn right.
  LeftDrive.spin(forward, speed, velocityUnits::pct);
  RightDrive.spin(forward, -speed, velocityUnits::pct);

  return false;
}

bool setupDrive(pid& controller, double targetDistance,
                double uncertainty_cm) {
  double currentDistance = RotationSensor.position(degrees);
  double uncertainty_degree = cm_to_degree(uncertainty_cm);
  double distanceError = targetDistance - currentDistance;

  if (hasSettled(controller, distanceError, uncertainty_degree, 150)) {
    stopDrive();
    return true;
  }

  double speed = calculatePID(controller, targetDistance, currentDistance);

  // Straight: both sides get the same speed. leftRatio/rightRatio can trim
  // one side if the robot still drifts.
  double leftSpeed = clampP(speed * leftRatio,
                            -controller.maxOutput, controller.maxOutput);
  double rightSpeed = clampP(speed * rightRatio,
                             -controller.maxOutput, controller.maxOutput);

  LeftDrive.spin(forward, leftSpeed, velocityUnits::pct);
  RightDrive.spin(forward, rightSpeed, velocityUnits::pct);

  return false;
}
