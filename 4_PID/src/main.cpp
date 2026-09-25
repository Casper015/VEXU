#include "vex.h"

using namespace vex;

bool setupTurn(pid& controller, double targetAngle, double uncertainty = 2.0);
bool setupDrive(pid& controller, double targetDistance, double uncertainty = 2.0);

int main() {
  vexcodeInit();
  while (TestInertial.isCalibrating()) {
    wait(20, msec);
  }

  double target_inertial = 90;
  double target_rotation2 = 3600;

  // First test: use P control for the 90-degree right turn.
  pid turnPID{};
  turnPID.kp = 0.65;
  turnPID.kd = 5;
  turnPID.maxOutput = 50;

  pid drivePID{};
  drivePID.kp = 0.12;
  drivePID.kd = 0.8;
  drivePID.maxOutput = 60;

  int step = 1;

  TestInertial.setRotation(0, degrees);

  while(step <= 4){
    switch (step) {
    case 1:
      if (setupTurn(turnPID, target_inertial, 2)) {
        resetPID(turnPID);
        step++;
        wait(5000, msec); 
      }
      break;
    case 2:
      if (setupTurn(turnPID, 0, 2)) {
        resetPID(drivePID);
        Rotation2.resetPosition();
        step++;
        wait(5000, msec); 
      }
      break;
    case 3:
      if (setupDrive(drivePID, target_rotation2, 10)) {
        resetPID(drivePID);
        step++;
        wait(5000, msec); 
      }
      break;
    case 4:
      if (setupDrive(drivePID, 0, 10)) {
        step++;
        wait(5000, msec); 
      }
      break;
    }
  wait(50, msec);
  }

  Drivetrain.stop(brake);
  return 0;
}

bool setupTurn(pid& controller, double targetAngle, 
  double uncertainty) {
    
    double currentAngle = TestInertial.rotation(degrees);
    double error = targetAngle - currentAngle;
    controller_screen();
    if (fabs(error) <= uncertainty) {
      Drivetrain.stop(brake); 
      return true;
    }

    double speed = calculatePID(controller, targetAngle, currentAngle);
    if (speed >= 0) {
      Drivetrain.turn(right, speed, velocityUnits::pct);
    } else {
      Drivetrain.turn(left, -speed, velocityUnits::pct);
    }
    
    return false;
}

bool setupDrive(pid& controller, double targetDistance, 
  double uncertainty) {
    
    double currentDistance = Rotation2.position(degrees);
    double error = targetDistance - currentDistance; 
    controller_screen();
    if (fabs(error) <= uncertainty) {
      Drivetrain.stop(brake); 
      return true;
    }

    double speed = calculatePID(controller, targetDistance, currentDistance);
    if (speed >= 0) {
      Drivetrain.drive(forward, speed, velocityUnits::pct);
    } else {
      Drivetrain.drive(reverse, -speed, velocityUnits::pct);
    }
    return false;
}