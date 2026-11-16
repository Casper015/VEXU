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

// Heading hold while driving straight.
const double headingKp = 1;
const double maxCorrection = 8.0;

// PID gains taken from pid (1).h: kP = 0.98, kD = 0.67, turn limited to 15%,
// drive limited to 50%. That code computed D as (error change per 10 ms loop)
// and drive error in cm; calculatePID() uses change per ms and drive error in
// rotation-sensor degrees, so the gains are scaled below to behave the same.
const double srcKp = 0.98;
const double srcKd = 0.67;
const double srcLoopMs = 10;
const double turnMaxOutput = 15;
const double driveMaxOutput = 50;

// TEST ONLY: press Up during driver control to run autonomous().
// Set to false before a real competition.
const bool enableAutoTestButton = true;

// Joystick values below this are treated as 0 (stops drift when released).
const int joystickDeadband = 5;

// Absolute heading (degrees) the robot should hold. turnPID() sets it,
// drivePID() keeps it while driving.
double targetHeading = 0;

bool setupTurn(pid& controller, double targetAngle, double uncertainty = 2);
bool setupDrive(pid& controller, double targetDistance, double targetHeading,
                double uncertainty_cm = 2.0);

// Turn to an ABSOLUTE heading in degrees (0 = heading at autonomous start,
// positive = right). Blocks until settled or timeout.
void turnPID(double angleDeg, uint32_t timeoutMs = 0) {
  if (timeoutMs == 0) {
    // 15% max turn speed is slow, so allow ~25 ms per degree of turning.
    double turnDeg = fabs(angleDeg - TestInertial.rotation(degrees));
    timeoutMs = 1000 + (uint32_t)(turnDeg * 25);
  }

  pid turnCtrl{};
  turnCtrl.kp = srcKp;              // 0.98
  turnCtrl.kd = srcKd * srcLoopMs;  // 6.7
  turnCtrl.maxOutput = turnMaxOutput;

  targetHeading = angleDeg;
  uint32_t startMs = Brain.Timer.time();
  while (!setupTurn(turnCtrl, angleDeg, 1)) {
    if (Brain.Timer.time() - startMs > timeoutMs) break;
    wait(10, msec);
  }
  Drivetrain.stop(brake);
}

// Drive a RELATIVE distance in cm (negative = backward), holding the current
// target heading. Blocks until settled or timeout.
void drivePID(double distanceCm, uint32_t timeoutMs = 0) {
  if (timeoutMs == 0) {
    timeoutMs = 1500 + (uint32_t)(fabs(distanceCm) * 20); // ~20 ms per cm
  }

  pid driveCtrl{};
  const double cmPerDegree = degree_to_cm(1);           // ~0.0443 cm
  driveCtrl.kp = srcKp * cmPerDegree;                   // ~0.0434
  driveCtrl.kd = srcKd * srcLoopMs * cmPerDegree;       // ~0.297
  driveCtrl.maxOutput = driveMaxOutput;

  Rotation2.resetPosition();
  double targetDeg = cm_to_degree(distanceCm);
  uint32_t startMs = Brain.Timer.time();
  while (!setupDrive(driveCtrl, targetDeg, targetHeading, 1)) {
    if (Brain.Timer.time() - startMs > timeoutMs) break;
    wait(10, msec);
  }
  Drivetrain.stop(brake);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Calibrate sensors, bind L1/L2 (arm), R1/R2 (lift), X/Y (intake), B (carousel), start screen thread.
  vexcodeInit();
  while (TestInertial.isCalibrating()) {
    wait(20, msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // Heading 0 = the direction the robot faces when autonomous starts.
  TestInertial.setRotation(0, degrees);
  targetHeading = 0;

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

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (true) {
    // Up button: run the autonomous route once (joystick is ignored meanwhile).
    if (enableAutoTestButton && Controller1.ButtonUp.pressing()) {
      Drivetrain.stop(brake);
      autonomous();
      waitUntil(!Controller1.ButtonUp.pressing()); // no repeat if still held
      continue;
    }

    // Single-stick arcade: left stick up/down = forward/back,
    // left stick left/right = turn.
    int forwardInput = Controller1.Axis3.position();
    int turnInput = Controller1.Axis4.position();
    if (abs(forwardInput) < joystickDeadband) forwardInput = 0;
    if (abs(turnInput) < joystickDeadband) turnInput = 0;

    LeftDriveSmart.spin(forward, forwardInput + turnInput, percent);
    RightDriveSmart.spin(forward, forwardInput - turnInput, percent);

    // Arm (L1/L2), lift (R1/R2), intake (X/Y) and carousel (B) are handled by the button callbacks
    // registered in vexcodeInit().

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
  double currentAngle = TestInertial.rotation(degrees);
  double error = targetAngle - currentAngle;

  if (hasSettled(controller, error, uncertainty, 50)) {
    Drivetrain.stop(brake);
    return true;
  }

  double speed = calculatePID(controller, targetAngle, currentAngle);
  Drivetrain.turn(right, speed, velocityUnits::pct);

  return false;
}

bool setupDrive(pid& controller, double targetDistance, double targetHeading,
                double uncertainty_cm) {
  double currentDistance = Rotation2.position(degrees);
  double uncertainty_degree = cm_to_degree(uncertainty_cm);
  double distanceError = targetDistance - currentDistance;

  if (hasSettled(controller, distanceError, uncertainty_degree, 150)) {
    Drivetrain.stop(brake);
    return true;
  }

  double speed = calculatePID(controller, targetDistance, currentDistance);

  double headingError = targetHeading - TestInertial.rotation(degrees);
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;

  double correction =
      clampP(headingKp * headingError, -maxCorrection, maxCorrection);

  // Apply each side's compensation to the combined drive and heading command.
  double leftSpeed = clampP((speed + correction) * leftRatio,
                            -controller.maxOutput, controller.maxOutput);
  double rightSpeed = clampP((speed - correction) * rightRatio,
                             -controller.maxOutput, controller.maxOutput);

  LeftDriveSmart.spin(forward, leftSpeed, velocityUnits::pct);
  RightDriveSmart.spin(forward, rightSpeed, velocityUnits::pct);

  return false;
}
