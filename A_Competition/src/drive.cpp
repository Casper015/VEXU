#include "vex.h"

using namespace vex;

// Absolute heading (degrees) the robot should hold. turnPID() sets it,
// drivePID() keeps it while driving.
static double targetHeading = 0;

// Auto-test cancel state, see allowDriveAbort() in drive.h.
static bool abortAllowed = false;
static bool aborted = false;

static bool abortRequested() {
  if (abortAllowed && Controller1.ButtonDown.pressing()) aborted = true;
  return aborted;
}

void allowDriveAbort(bool allowed) {
  abortAllowed = allowed;
  aborted = false;
}

void stopDrive() {
  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

void resetHeading() {
  InertialSensor.setRotation(0, degrees);
  targetHeading = 0;
}

// One step of the turn loop. Returns true once the turn has settled.
static bool turnStep(pid& ctrl, double targetAngle) {
  double currentAngle = InertialSensor.rotation(degrees);
  double error = targetAngle - currentAngle;

  if (hasSettled(ctrl, error, Config::turnToleranceDeg, Config::turnSettleMs)) {
    stopDrive();
    return true;
  }

  // Positive speed turns right: left side forward, right side backward.
  double speed = calculatePID(ctrl, targetAngle, currentAngle);
  LeftDrive.spin(forward, speed, velocityUnits::pct);
  RightDrive.spin(forward, -speed, velocityUnits::pct);

  return false;
}

// One step of the straight-drive loop. targetDistance is in rotation-sensor
// degrees. Returns true once the drive has settled.
static bool driveStep(pid& ctrl, double targetDistance) {
  double currentDistance = RotationSensor.position(degrees);
  double distanceError = targetDistance - currentDistance;

  if (hasSettled(ctrl, distanceError, cm_to_degree(Config::driveToleranceCm),
                 Config::driveSettleMs)) {
    stopDrive();
    return true;
  }

  double speed = calculatePID(ctrl, targetDistance, currentDistance);

  double headingError = targetHeading - InertialSensor.rotation(degrees);
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;

  double correction = clampP(Config::headingKp * headingError,
                             -Config::maxCorrection, Config::maxCorrection);

  // Apply each side's compensation to the combined drive and heading command.
  double leftSpeed = clampP((speed + correction) * Config::leftRatio,
                            -ctrl.maxOutput, ctrl.maxOutput);
  double rightSpeed = clampP((speed - correction) * Config::rightRatio,
                             -ctrl.maxOutput, ctrl.maxOutput);

  LeftDrive.spin(forward, leftSpeed, velocityUnits::pct);
  RightDrive.spin(forward, rightSpeed, velocityUnits::pct);

  return false;
}

void turnPID(double angleDeg, uint32_t timeoutMs) {
  if (aborted) return;

  if (timeoutMs == 0) {
    double turnDeg = fabs(angleDeg - InertialSensor.rotation(degrees));
    timeoutMs = Config::turnTimeoutBaseMs +
                (uint32_t)(turnDeg * Config::turnTimeoutMsPerDeg);
  }

  pid turnCtrl{};
  turnCtrl.kp = Config::srcKp;                      // 0.98
  turnCtrl.kd = Config::srcKd * Config::srcLoopMs;  // 6.7
  turnCtrl.maxOutput = Config::turnMaxOutput;

  targetHeading = angleDeg;
  uint32_t startMs = Brain.Timer.time();
  while (!turnStep(turnCtrl, angleDeg)) {
    if (abortRequested()) break;
    if (Brain.Timer.time() - startMs > timeoutMs) break;
    wait(10, msec);
  }
  stopDrive();
}

void drivePID(double distanceCm, uint32_t timeoutMs) {
  if (aborted) return;

  if (timeoutMs == 0) {
    timeoutMs = Config::driveTimeoutBaseMs +
                (uint32_t)(fabs(distanceCm) * Config::driveTimeoutMsPerCm);
  }

  pid driveCtrl{};
  const double cmPerDegree = degree_to_cm(1);                             // ~0.0443 cm
  driveCtrl.kp = Config::srcKp * cmPerDegree;                             // ~0.0434
  driveCtrl.kd = Config::srcKd * Config::srcLoopMs * cmPerDegree;         // ~0.297
  driveCtrl.maxOutput = Config::driveMaxOutput;

  RotationSensor.resetPosition();
  double targetDeg = cm_to_degree(distanceCm);
  uint32_t startMs = Brain.Timer.time();
  while (!driveStep(driveCtrl, targetDeg)) {
    if (abortRequested()) break;
    if (Brain.Timer.time() - startMs > timeoutMs) break;
    wait(10, msec);
  }
  stopDrive();
}
