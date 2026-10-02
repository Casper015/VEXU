#include "vex.h"

using namespace vex;

bool setupTurn(pid& controller, double targetAngle, double uncertainty = 1);
bool setupDrive(pid& controller, double targetDistance, double targetHeading, double uncertainty_cm = 1.0);


const double leftRatio = 0.95;
const double rightRatio = 1.0;

const double headingKp = 0.5;
const double maxCorrection = 8.0;

int main() {
  vexcodeInit();
  while (TestInertial.isCalibrating()) {
    wait(20, msec);
  }

  double howFarCm = 50.0;

  double target_inertial = 90;
  double target_rotation2 = cm_to_degree(howFarCm);

  // First test: use P control for the 90-degree right turn.
  pid turnPID{};
  turnPID.kp = 0.65;
  turnPID.kd = 4.15;
  turnPID.maxOutput = 50;

  pid drivePID{};
  drivePID.kp = 0.06;
  drivePID.kd = 1;
  drivePID.maxOutput = 50;

  int step = 3;

  TestInertial.setRotation(0, degrees);
  double driveHeading = TestInertial.rotation(degrees);

  while(step <= 4){
    switch (step) {
    case 1:
      if (setupTurn(turnPID, target_inertial, 1)) {
        resetPID(turnPID);
        step++;
        wait(5000, msec); 
      }
      break;
    case 2:
      if (setupTurn(turnPID, 0, 1)) {
        resetPID(drivePID);
        Rotation2.resetPosition();
        driveHeading = TestInertial.rotation(degrees);
        step++;
        wait(5000, msec); 
      }
      break;
    case 3:
      if (setupDrive(drivePID, target_rotation2, driveHeading, 1)) {
        resetPID(drivePID);
        step++;
        wait(5000, msec); 
      }
      break;
    case 4:
      if (setupDrive(drivePID, 0, driveHeading, 1)) {
        step++;
        wait(5000, msec); 
      }
      break;
    }
  wait(20, msec);
  }

  Drivetrain.stop(brake);
  return 0;
}

bool setupTurn(pid& controller, double targetAngle, 
  double uncertainty) {
    
    double currentAngle = TestInertial.rotation(degrees);
    double error = targetAngle - currentAngle;
    
    if (hasSettled(controller, error, uncertainty, 150)) {
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
    double DistanceError = targetDistance - currentDistance; 

    if (hasSettled(controller, DistanceError, uncertainty_degree, 150)) {
      Drivetrain.stop(brake); 
      return true;
    }

    double speed = calculatePID(controller, targetDistance, currentDistance);

    double headingError = targetHeading - TestInertial.rotation(degrees);

    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;
    
    

    double correction = clampP(headingKp * headingError, 0, maxCorrection);

    // Apply each side's compensation to the combined drive and heading command.
    double leftSpeed = clampP((speed + correction) * leftRatio, 0, controller.maxOutput);
    double rightSpeed = clampP((speed - correction) * rightRatio, 0, controller.maxOutput);

    LeftDriveSmart.spin(forward, leftSpeed, velocityUnits::pct);
    RightDriveSmart.spin(forward, rightSpeed, velocityUnits::pct);

    return false;
}

